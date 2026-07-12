/** @file ww_autotune.cpp
 * @brief Wall wetting (X-Tau) table autotune from wideband AFR feedback.
 *
 * The X-Tau model parameters have distinct, observable signatures in the AFR
 * trace that follows a throttle tip-in:
 *  - The magnitude of the initial lean/rich excursion is set by the "added to
 *    wall" coefficient (X): a lean peak means X is too small (under-compensation).
 *  - The error remaining in the tail of the window is set by the "removed from
 *    wall" coefficient (Y): a lean tail means the real film time constant is
 *    longer than modelled (Y too large), a rich tail the opposite.
 *
 * The wideband reading lags the injection event it measures by the exhaust
 * transport time plus the sensor response time - the same order of magnitude
 * as the transient being measured. Every AFR sample is therefore compared
 * against the AFR target from (delay) ago via a short history ring buffer
 * (de-lag). No per-RPM wideband delay curve exists in the tune, so the delay
 * is modelled as a fixed sensor term plus a transport term proportional to
 * 1/RPM; the constants below are the single place to re-point if a proper
 * delay curve is ever added.
 *
 * Learning is deliberately conservative:
 *  - Only accelerating tip-ins are learned. Deceleration uses the fixed
 *    decelAmount and is not touched.
 *  - A long list of gates must hold for the full observation window (warm
 *    engine, wideband active, closed-loop trim near neutral, no cuts/launch/
 *    ASE/WUE, RPM below the AE taper region). Any gate failure aborts the event.
 *  - One cell (nearest to the tip-in operating point) is adjusted per event,
 *    by a bounded step, and the total movement per cell per drive cycle is
 *    clamped to configPage2.wallWettingFuel table counts (the authority knob).
 *  - Changes are only queued for EEPROM persistence once the engine stops.
 */
#include "globals.h"
#include "ww_autotune.h"
#include "storage.h"
#include "units.h"
#include "load_source.h"
#include "table3d_interpolate.h"

// ============================ Tuning constants ============================

// --- Wideband delay model (replace with a tune curve if one is added) ---
/** Sensor + controller response portion of the wideband delay (ms) */
static constexpr uint16_t WW_LEARN_LAG_FIXED_MS = 80U;
/** Exhaust transport portion: delay_ms = this / RPM (100ms @ 3000rpm) */
static constexpr uint32_t WW_LEARN_LAG_TRANSPORT_MS_RPM = 300000UL;

// --- Observation window (all in 30Hz ticks) ---
/** History depth. Bounds the maximum compensable wideband delay (~766ms) */
static constexpr uint8_t WW_LEARN_RING_SIZE = 24U;
static constexpr uint8_t WW_LEARN_MAX_DELAY_TICKS = WW_LEARN_RING_SIZE - 1U;
/** Event ages (1-based) attributed to the initial excursion -> X table */
static constexpr uint8_t WW_LEARN_PEAK_START = 2U;  // ~66ms
static constexpr uint8_t WW_LEARN_PEAK_END = 9U;    // ~300ms
/** Event ages attributed to the tail -> Y table */
static constexpr uint8_t WW_LEARN_TAIL_START = 12U; // ~400ms
static constexpr uint8_t WW_LEARN_TAIL_END = 30U;   // ~1s
/** Ticks during which samples are marked as belonging to the event */
static constexpr uint8_t WW_LEARN_CAPTURE_TICKS = WW_LEARN_TAIL_END;

// --- Acceptance / adjustment ---
/** Ignore mean AFR errors inside this band (AFR x10, i.e. 3 == 0.3 AFR) */
static constexpr uint8_t WW_LEARN_DEADBAND = 3U;
/** Abort learning if closed loop EGO trim strays further than this from 100% */
static constexpr uint8_t WW_LEARN_EGO_BAND = 7U;
/** Minimum de-lagged samples required in each phase to trust the averages */
static constexpr uint8_t WW_LEARN_MIN_PEAK_SAMPLES = 4U;
static constexpr uint8_t WW_LEARN_MIN_TAIL_SAMPLES = 6U;
/** Maximum table counts a cell may move per event */
static constexpr uint8_t WW_LEARN_MAX_STEP = 3U;
/** Plausibility clamps for the learned coefficients (0-255 == 0-100%) */
static constexpr uint8_t WW_LEARN_X_MIN = 0U;
static constexpr uint8_t WW_LEARN_X_MAX = 200U; // ~78% deposited: beyond this something else is wrong
static constexpr uint8_t WW_LEARN_Y_MIN = 4U;   // film time constant ceiling ~2s
static constexpr uint8_t WW_LEARN_Y_MAX = 230U;
/** Wideband reading sanity band (AFR x10) */
static constexpr uint8_t WW_LEARN_O2_MIN = 70U;
static constexpr uint8_t WW_LEARN_O2_MAX = 220U;
/** Don't learn until this many seconds after cranking, even if ego_sdelay is shorter */
static constexpr uint8_t WW_LEARN_MIN_RUN_SECS = 30U;
/** Minimum battery voltage (x10) for the injectors to be in their linear region */
static constexpr uint8_t WW_LEARN_MIN_BATTERY10 = 105U;

static constexpr uint8_t WW_TABLE_DIM = 8U;
static constexpr uint8_t WW_TABLE_CELLS = WW_TABLE_DIM * WW_TABLE_DIM;

// ============================== Module state ==============================

/** One 30Hz history sample used for de-lagging the wideband reading */
struct WwLearnSample {
  uint8_t afrTarget; ///< currentStatus.afrTarget when the sample was taken
  uint8_t eventAge;  ///< 0 = no event in progress, else 1-based ticks since event start
  bool gatesOk;      ///< All learning gates held when the sample was taken
};

static WwLearnSample ringBuffer[WW_LEARN_RING_SIZE];
static uint8_t ringHead = 0;  ///< Next slot to be written
static uint8_t ringFilled = 0;

static struct {
  bool active;
  bool aborted;
  uint8_t age;          ///< Ticks since the event started (1-based)
  uint8_t cellIndex;    ///< Linear index into the table value arrays
  bool dotWentQuiet;    ///< The triggering DOT dropped back below threshold (retrigger detection)
  int16_t peakErrSum;   ///< Sum of de-lagged AFR errors (x10) in the peak phase
  uint8_t peakErrCount;
  int16_t tailErrSum;
  uint8_t tailErrCount;
} event;

/** Ticks to wait after an event ends before another may start.
 * Sized so every ring slot is rewritten, preventing stale event marks from
 * being attributed to the next event. */
static uint8_t cooldownTicks = 0;

/** Per-cell movement so far this drive cycle, for the authority clamp */
static int8_t addTableDelta[WW_TABLE_CELLS];
static int8_t removeTableDelta[WW_TABLE_CELLS];

static bool tablesDirty = false;
static uint16_t learnedEventCount = 0;

// ============================== Small helpers =============================

static inline uint16_t absI16(int16_t value) {
  return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static inline uint8_t clampU8(int16_t value, uint8_t lo, uint8_t hi) {
  if (value < (int16_t)lo) { return lo; }
  if (value > (int16_t)hi) { return hi; }
  return (uint8_t)value;
}

/** @brief Find the memory index of the axis entry nearest to a compressed lookup value.
 * @note Table axes are stored compressed (RPM/100, load/2) in descending order. */
static uint8_t nearestAxisIndex(const table3d_axis_t *axis, uint8_t compressedValue) {
  uint8_t best = 0;
  uint8_t bestDist = UINT8_MAX;
  for (uint8_t i = 0; i < WW_TABLE_DIM; i++) {
    const uint8_t dist = (axis[i] > compressedValue) ? (axis[i] - compressedValue) : (compressedValue - axis[i]);
    if (dist < bestDist) { bestDist = dist; best = i; }
  }
  return best;
}

/** @brief Linear index into a table3d8 value array for the cell at the given axis memory indices.
 * Value rows follow the Y axis memory order; columns are reversed vs the X axis memory order. */
static inline uint8_t cellValueIndex(uint8_t xAxisMemIdx, uint8_t yAxisMemIdx) {
  return (uint8_t)((yAxisMemIdx * WW_TABLE_DIM) + (WW_TABLE_DIM - 1U - xAxisMemIdx));
}

/** @brief The wideband delay at the current RPM, in 30Hz ticks */
static uint8_t currentDelayTicks(void) {
  uint32_t delayMs = WW_LEARN_LAG_FIXED_MS;
  if (currentStatus.RPM > 0U) {
    delayMs += WW_LEARN_LAG_TRANSPORT_MS_RPM / currentStatus.RPM;
  }
  // ticks = ms * 30 / 1000, rounded
  uint32_t ticks = ((delayMs * 3U) + 50U) / 100U;
  if (ticks < 1U) { ticks = 1U; }
  if (ticks > WW_LEARN_MAX_DELAY_TICKS) { ticks = WW_LEARN_MAX_DELAY_TICKS; }
  return (uint8_t)ticks;
}

// ================================= Gates ==================================

/** @brief All conditions that must hold for learning to be trustworthy */
static bool learnGatesOk(void) {
  if (configPage2.aeMode != AE_MODE_WALL_WETTING) { return false; }
  if (configPage2.wallWettingFuel == 0U) { return false; } // Learning disabled
  if (configPage6.egoType != EGO_TYPE_WIDE) { return false; }
  if (currentStatus.rotationStatus != EngineRotationStatus::Running) { return false; }

  const uint8_t minRunSecs = (configPage6.ego_sdelay > WW_LEARN_MIN_RUN_SECS) ? configPage6.ego_sdelay : WW_LEARN_MIN_RUN_SECS;
  if (currentStatus.runSecs < minRunSecs) { return false; }

  // Fully warm only: both the closed loop coolant threshold and the AE cold
  // multiplier taper must be out of the picture, otherwise cold film dynamics
  // would be learned into tables that have no temperature axis.
  if (currentStatus.coolant < temperatureRemoveOffset(configPage6.egoTemp)) { return false; }
  if (currentStatus.coolant < temperatureRemoveOffset(configPage2.aeColdTaperMax)) { return false; }

  if (currentStatus.RPM < RPM_COARSE.toUser(configPage6.egoRPM)) { return false; }
  // Learn only below the AE RPM taper, where the computed correction is
  // applied at full strength (above it the observed error no longer maps 1:1
  // onto the table coefficients).
  if ((configPage2.aeTaperMax > configPage2.aeTaperMin)
    && (currentStatus.RPM >= RPM_COARSE.toUser(configPage2.aeTaperMin))) { return false; }

  // No other fuel corrections in flux
  if (currentStatus.isDFCOActive || currentStatus.aseIsActive || currentStatus.wueIsActive) { return false; }
  if (currentStatus.launchingHard || currentStatus.launchingSoft) { return false; }
  if (currentStatus.flatShiftingHard || currentStatus.flatShiftSoftCut) { return false; }
  if (currentStatus.nitrousActive || currentStatus.stagingActive) { return false; }
  if (absI16((int16_t)currentStatus.egoCorrection - 100) > WW_LEARN_EGO_BAND) { return false; }

  if (currentStatus.battery10 < WW_LEARN_MIN_BATTERY10) { return false; }
  if ((currentStatus.O2 < WW_LEARN_O2_MIN) || (currentStatus.O2 > WW_LEARN_O2_MAX)) { return false; }

  return true;
}

// ============================ Table adjustment ============================

/** @brief Move one cell by delta, honouring the plausibility clamps and the
 * per-drive-cycle authority. Returns true if the stored value changed. */
static bool adjustCell(table3d8RpmLoad &table, int8_t *sessionDelta, uint8_t cellIdx,
                       int8_t delta, uint8_t minValue, uint8_t maxValue) {
  const int16_t authority = (int16_t)((configPage2.wallWettingFuel > 100U) ? 100U : configPage2.wallWettingFuel);

  int16_t newSessionDelta = (int16_t)sessionDelta[cellIdx] + delta;
  if (newSessionDelta > authority) { newSessionDelta = authority; }
  if (newSessionDelta < -authority) { newSessionDelta = -authority; }

  int16_t effectiveDelta = newSessionDelta - sessionDelta[cellIdx];
  const uint8_t oldValue = table.values.values[cellIdx];
  const uint8_t newValue = clampU8((int16_t)oldValue + effectiveDelta, minValue, maxValue);
  effectiveDelta = (int16_t)newValue - (int16_t)oldValue;
  if (effectiveDelta == 0) { return false; }

  table.values.values[cellIdx] = newValue;
  sessionDelta[cellIdx] = (int8_t)(sessionDelta[cellIdx] + effectiveDelta);
  invalidate_cache(&table.get_value_cache);
  return true;
}

/** @brief Convert a mean AFR error (x10) into a bounded table step */
static int8_t errorToStep(int16_t meanError) {
  const uint16_t magnitude = absI16(meanError);
  if (magnitude <= WW_LEARN_DEADBAND) { return 0; }
  uint16_t step = 1U + ((magnitude - WW_LEARN_DEADBAND) / 8U);
  if (step > WW_LEARN_MAX_STEP) { step = WW_LEARN_MAX_STEP; }
  return (meanError > 0) ? (int8_t)step : (int8_t)(-(int8_t)step);
}

/** @brief Turn a completed event's error averages into table updates */
static void finalizeEvent(void) {
  bool changed = false;

  if (event.peakErrCount >= WW_LEARN_MIN_PEAK_SAMPLES) {
    // Lean peak (positive error) right after tip-in: the model under-estimates
    // how much fuel the wall steals -> increase the "added to wall" coefficient.
    const int8_t step = errorToStep(event.peakErrSum / (int16_t)event.peakErrCount);
    if (step != 0) {
      changed |= adjustCell(wallWettingAddTable, addTableDelta, event.cellIndex,
                            step, WW_LEARN_X_MIN, WW_LEARN_X_MAX);
    }
  }

  if (event.tailErrCount >= WW_LEARN_MIN_TAIL_SAMPLES) {
    // Lean tail: the real film keeps stealing fuel longer than modelled, i.e.
    // the real evaporation is slower -> decrease the "removed from wall"
    // coefficient. Rich tail -> increase it.
    const int8_t step = errorToStep(event.tailErrSum / (int16_t)event.tailErrCount);
    if (step != 0) {
      changed |= adjustCell(wallWettingRemoveTable, removeTableDelta, event.cellIndex,
                            (int8_t)-step, WW_LEARN_Y_MIN, WW_LEARN_Y_MAX);
    }
  }

  if (changed) {
    tablesDirty = true;
    learnedEventCount++;
  }
}

// ============================ Baseline seeding ============================

/** @brief A table needs seeding when its axis is not strictly descending in
 * memory (fresh/erased page reads as all 0x00 or all 0xFF) or when every value
 * is identical (axes present but content never tuned). */
static bool tableNeedsSeed(const table3d8RpmLoad &table) {
  if (table.axisX.axis[0] <= table.axisX.axis[WW_TABLE_DIM - 1U]) { return true; }
  if (table.axisY.axis[0] <= table.axisY.axis[WW_TABLE_DIM - 1U]) { return true; }

  const uint8_t first = table.values.values[0];
  for (uint8_t i = 1U; i < WW_TABLE_CELLS; i++) {
    if (table.values.values[i] != first) { return false; }
  }
  return true;
}

bool wwSeedBaselineTables(bool force) {
  if (!force && !tableNeedsSeed(wallWettingAddTable) && !tableNeedsSeed(wallWettingRemoveTable)) {
    return false;
  }

  // --- RPM axis (compressed /100), denser at low RPM where wall wetting bites ---
  // Fractions of the rev limit in 64ths; bin 0 is fixed at 500rpm.
  static const uint8_t rpmFrac64[WW_TABLE_DIM] = { 0U, 13U, 19U, 26U, 35U, 45U, 55U, 64U };
  const uint8_t revLimit = (configPage4.HardRevLim < 40U) ? 60U : configPage4.HardRevLim; // default 6000 if unset
  uint8_t rpmBins[WW_TABLE_DIM];
  rpmBins[0] = 5U;
  for (uint8_t i = 1U; i < WW_TABLE_DIM; i++) {
    uint8_t bin = (uint8_t)(((uint16_t)revLimit * rpmFrac64[i]) / 64U);
    if (bin <= rpmBins[i - 1U]) { bin = rpmBins[i - 1U] + 1U; }
    rpmBins[i] = bin;
  }

  // --- Load axis (compressed /2), linear from light load to the sensor/algorithm max ---
  uint16_t loadMax = 100U;
  if (configPage2.fuelAlgorithm != LOAD_SOURCE_TPS) {
    loadMax = configPage2.mapMax;
    if (loadMax < 100U) { loadMax = 100U; }
    if (loadMax > 240U) { loadMax = 240U; }
  }
  uint8_t loadBins[WW_TABLE_DIM];
  for (uint8_t i = 0U; i < WW_TABLE_DIM; i++) {
    const uint16_t user = 20U + ((loadMax - 20U) * i) / (WW_TABLE_DIM - 1U);
    uint8_t bin = (uint8_t)(user / 2U);
    if ((i > 0U) && (bin <= loadBins[i - 1U])) { bin = loadBins[i - 1U] + 1U; }
    loadBins[i] = bin;
  }

  // Axes are stored descending in memory: axis[0] = largest
  for (uint8_t m = 0U; m < WW_TABLE_DIM; m++) {
    wallWettingAddTable.axisX.axis[m] = rpmBins[WW_TABLE_DIM - 1U - m];
    wallWettingAddTable.axisY.axis[m] = loadBins[WW_TABLE_DIM - 1U - m];
    wallWettingRemoveTable.axisX.axis[m] = rpmBins[WW_TABLE_DIM - 1U - m];
    wallWettingRemoveTable.axisY.axis[m] = loadBins[WW_TABLE_DIM - 1U - m];
  }

  // --- Values ---
  // X ("added to wall", 0-255 == 0-100%): the deposited fraction grows with
  // load (higher port pressure & injected mass -> more film) and shrinks with
  // RPM (higher air velocity & port temperature strip the spray less time to
  // settle). Warm-engine port injection literature sits around 10-30%:
  // X spans ~10% (low load) to ~30% (full load) at low RPM, halved at redline.
  //
  // Y ("removed from wall" per 33ms update): the evaporation fraction grows
  // with RPM (airflow & heat) and shrinks slightly with load (higher manifold
  // pressure slows evaporation). Y = 8..34 maps to a film time constant of
  // roughly 1s at idle down to 0.25s at the limiter - the classic X-Tau range
  // for a warm port-injected engine.
  for (uint8_t loadIdx = 0U; loadIdx < WW_TABLE_DIM; loadIdx++) {
    for (uint8_t rpmIdx = 0U; rpmIdx < WW_TABLE_DIM; rpmIdx++) {
      uint16_t x = 26U + ((51U * loadIdx) / (WW_TABLE_DIM - 1U));
      x = (x * (64U - ((32U * rpmIdx) / (WW_TABLE_DIM - 1U)))) / 64U;
      if (configPage2.injType == INJ_TYPE_TBODY) {
        x = (x * 3U) / 2U; // Throttle body: the whole manifold is wetted path
      }
      if (configPage2.strokes == 1U) {
        x = (x * 3U) / 4U; // Two-stroke: shorter tract, crankcase-scavenged
      }

      int16_t y = (int16_t)(8U + ((26U * rpmIdx) / (WW_TABLE_DIM - 1U)))
                - (int16_t)((4U * loadIdx) / (WW_TABLE_DIM - 1U));

      const uint8_t cell = cellValueIndex(WW_TABLE_DIM - 1U - rpmIdx, WW_TABLE_DIM - 1U - loadIdx);
      wallWettingAddTable.values.values[cell] = clampU8((int16_t)x, WW_LEARN_X_MIN, WW_LEARN_X_MAX);
      wallWettingRemoveTable.values.values[cell] = clampU8(y, WW_LEARN_Y_MIN, WW_LEARN_Y_MAX);
    }
  }

  invalidate_cache(&wallWettingAddTable.get_value_cache);
  invalidate_cache(&wallWettingRemoveTable.get_value_cache);
  return true;
}

// ============================== Public API ================================

void wwAutotuneInit(void) {
  ringHead = 0;
  ringFilled = 0;
  cooldownTicks = 0;
  event.active = false;
  learnedEventCount = 0;
  tablesDirty = false;
  for (uint8_t i = 0U; i < WW_TABLE_CELLS; i++) {
    addTableDelta[i] = 0;
    removeTableDelta[i] = 0;
  }

  if (wwSeedBaselineTables(false)) {
    // Persist the baseline so TunerStudio sees the same tables we run on
    setEepromWritePending(true);
  }
}

void wwAutotuneUpdate(void) {
  // Queue the learned tables for storage once the engine stops. The main loop
  // performs the actual write when serial is idle (existing pending-write path).
  if (currentStatus.rotationStatus == EngineRotationStatus::Stopped) {
    if (tablesDirty) {
      setEepromWritePending(true);
      tablesDirty = false;
    }
    // Drop any in-flight event: its remaining samples are meaningless now
    event.active = false;
    cooldownTicks = WW_LEARN_RING_SIZE;
    return;
  }

  const bool gatesOk = learnGatesOk();
  const bool accelDemand = (currentStatus.tpsDOT > (int16_t)configPage2.taeThresh)
                        || (currentStatus.mapDOT > (int16_t)configPage2.maeThresh);
  const bool decelDemand = (currentStatus.tpsDOT < -(int16_t)configPage2.taeThresh)
                        || (currentStatus.mapDOT < -(int16_t)configPage2.maeThresh);

  // --- Event state machine ---
  if (!event.active) {
    if (cooldownTicks > 0U) {
      cooldownTicks--;
    } else if (gatesOk && accelDemand && !decelDemand && (ringFilled >= WW_LEARN_MAX_DELAY_TICKS)) {
      // Rising edge of an accelerating transient: anchor the event to the
      // cell at the tip-in operating point
      const uint8_t rpmComp = (uint8_t)((currentStatus.RPM + 50U) / 100U);
      const uint8_t loadComp = (uint8_t)((currentStatus.fuelLoad + 1U) / 2U);
      const uint8_t xIdx = nearestAxisIndex(wallWettingAddTable.axisX.axis, rpmComp);
      const uint8_t yIdx = nearestAxisIndex(wallWettingAddTable.axisY.axis, loadComp);
      event.active = true;
      event.aborted = false;
      event.age = 1;
      event.cellIndex = cellValueIndex(xIdx, yIdx);
      event.dotWentQuiet = false;
      event.peakErrSum = 0;
      event.peakErrCount = 0;
      event.tailErrSum = 0;
      event.tailErrCount = 0;
    }
  } else {
    event.age++;

    if (!event.aborted) {
      // Any gate failure while the event is being observed poisons the data
      if (!gatesOk) { event.aborted = true; }
      // A lift-off mid-window empties the film we are trying to characterise
      if (decelDemand || currentStatus.isDeceleratingTPS) { event.aborted = true; }
      // A second tip-in stacked on the first makes the phases ambiguous
      if (event.age <= WW_LEARN_CAPTURE_TICKS) {
        if (!accelDemand) { event.dotWentQuiet = true; }
        else if (event.dotWentQuiet) { event.aborted = true; }
      }
    }

    // Event ends once every capture-phase sample has had time to be observed
    // through the longest possible wideband delay
    if (event.age > (uint8_t)(WW_LEARN_CAPTURE_TICKS + WW_LEARN_MAX_DELAY_TICKS)) {
      if (!event.aborted) { finalizeEvent(); }
      event.active = false;
      cooldownTicks = WW_LEARN_RING_SIZE;
    }
  }

  // --- Record the current sample for later de-lagged comparison ---
  const bool capturing = event.active && !event.aborted && (event.age <= WW_LEARN_CAPTURE_TICKS);
  ringBuffer[ringHead].afrTarget = currentStatus.afrTarget;
  ringBuffer[ringHead].eventAge = capturing ? event.age : 0U;
  ringBuffer[ringHead].gatesOk = gatesOk;

  // --- De-lagged analysis: attribute the current wideband reading to the
  // sample taken one (modelled) wideband delay ago ---
  if (event.active && !event.aborted && (ringFilled >= WW_LEARN_MAX_DELAY_TICKS)) {
    const uint8_t delayTicks = currentDelayTicks();
    const WwLearnSample &delayed = ringBuffer[(ringHead + WW_LEARN_RING_SIZE - delayTicks) % WW_LEARN_RING_SIZE];
    if ((delayed.eventAge > 0U) && delayed.gatesOk) {
      const int16_t afrError = (int16_t)currentStatus.O2 - (int16_t)delayed.afrTarget; // + == lean
      if ((delayed.eventAge >= WW_LEARN_PEAK_START) && (delayed.eventAge <= WW_LEARN_PEAK_END)) {
        event.peakErrSum += afrError;
        event.peakErrCount++;
      } else if ((delayed.eventAge >= WW_LEARN_TAIL_START) && (delayed.eventAge <= WW_LEARN_TAIL_END)) {
        event.tailErrSum += afrError;
        event.tailErrCount++;
      } else {
        // Between the phases: transition region, intentionally ignored
      }
    }
  }

  ringHead = (ringHead + 1U) % WW_LEARN_RING_SIZE;
  if (ringFilled < WW_LEARN_RING_SIZE) { ringFilled++; }
}

uint16_t wwAutotuneLearnedEventCount(void) {
  return learnedEventCount;
}
