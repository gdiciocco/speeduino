/** @file seq_trim_autotune.cpp
 * @brief Wideband-feedback learner for 6x6 sequential fuel trim tables.
 *
 * The learner is intentionally a slow adaptive LUT, not an AFR controller.
 * Each accepted observation is attributed through the selected AFR channel's
 * bilinearly interpolated RPM/load delay map to
 * the four cells that produced it. Their bilinear membership weights sum to
 * one. Signed evidence is retained per cell across later visits to other
 * RPM/load regions; it is reset only at a new drive cycle or when that trim's
 * learning interpretation changes.
 */
#include "seq_trim_autotune.h"

#include "afr_delay.h"
#include "globals.h"
#include "storage.h"
#include "table3d_interpolate.h"
#include "units.h"

#if defined(CORE_AVR) && !defined(UNIT_TEST)

// The dense learner state needs 1.4 kB with eight injector channels. That is
// safe on the supported 32-bit targets, but would make existing ATmega2560
// eight-channel builds exceed their entire 8 kB SRAM budget. Keep the schema
// portable while failing closed on AVR instead of destabilising the ECU.
namespace {
SeqTrimAutotuneDiagnostics unsupportedDiag;
}

void seqTrimAutotuneInit(void)
{
  unsupportedDiag = SeqTrimAutotuneDiagnostics();
  unsupportedDiag.gateBits = SEQ_TRIM_GATE_DISABLED | SEQ_TRIM_GATE_CONFIG;
}

void seqTrimAutotuneUpdate(void)
{
}

const SeqTrimAutotuneDiagnostics& seqTrimAutotuneDiag(void)
{
  return unsupportedDiag;
}

#else

namespace {

constexpr uint8_t TABLE_DIM = 6U;
constexpr uint8_t TABLE_CELLS = TABLE_DIM * TABLE_DIM;
constexpr uint8_t RING_SIZE = 48U;
constexpr uint8_t MAX_DELAY_TICKS = RING_SIZE - 1U;
constexpr uint8_t AFR_MIN = 70U;
constexpr uint8_t AFR_MAX = 220U;
constexpr uint8_t MIN_RUN_SECONDS = 30U;
constexpr uint8_t MIN_BATTERY10 = 105U;
constexpr int16_t MAX_TPS_DOT = 50; //5 percent/s, raw resolution 0.1
constexpr int16_t MAX_MAP_DOT = 50; //5 kPa/s, raw resolution 0.1
constexpr int16_t MAX_EGO_CORRECTION = 3; //Do not learn through material global EGO correction
constexpr int16_t MAX_ERROR_TENTHS_PERCENT = 200; //20 percent
constexpr uint8_t WEIGHT_SCALE = 64U;
constexpr uint32_t TICKS_PER_MINUTE = 30UL * 60UL;
constexpr uint8_t RAW_TRIM_MIN = 78U;  //-50 percent with the -128 translation
constexpr uint8_t RAW_TRIM_MAX = 178U; //+50 percent

constexpr uint8_t MODE_MASK = 0x03U;
constexpr uint8_t AFR2_MASK = 0x04U;
constexpr uint8_t RESISTANCE_SHIFT = 3U;
constexpr uint8_t RESISTANCE_MASK = 0x07U;
constexpr uint8_t OBJECTIVE_TARGET_MASK = 0x01U;
constexpr uint8_t PERSISTENCE_SHIFT = 1U;
constexpr uint8_t PERSISTENCE_MASK = 0x03U;

enum PersistencePolicy : uint8_t {
  PERSIST_RAM_ONLY = 0U,
  PERSIST_ENGINE_STOP = 1U,
  PERSIST_PERIODIC = 2U,
};

struct HistorySample {
  uint16_t rpm;
  uint16_t load;
  uint8_t afrTarget;
  uint8_t afrDelayTicks[2];
  bool eligible;
};

struct WeightedCell {
  uint8_t index;
  uint8_t weight;
};

HistorySample history[RING_SIZE];
uint8_t historyHead = 0U;
uint8_t historyFilled = 0U;
uint16_t stableTicks = 0U;

// Weighted error integral for each cell. This is the retained evidence: moving
// elsewhere in the map does not clear it.
int32_t cellAccumulator[INJ_CHANNELS][TABLE_CELLS];
uint8_t sessionBaseline[INJ_CHANNELS][TABLE_CELLS];
uint8_t previousConfig[INJ_CHANNELS];
uint8_t previousObjective = 0U;

bool sessionActive = false;
bool tablesDirty = false;
uint32_t saveDelayTicks = 0U;
SeqTrimAutotuneDiagnostics diag;

uint16_t absoluteI16(int16_t value)
{
  return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

int16_t clampError(int32_t value)
{
  if (value > MAX_ERROR_TENTHS_PERCENT) { return MAX_ERROR_TENTHS_PERCENT; }
  if (value < -MAX_ERROR_TENTHS_PERCENT) { return -MAX_ERROR_TENTHS_PERCENT; }
  return (int16_t)value;
}

uint8_t trimMode(uint8_t trim)
{
  const uint8_t mode = configPage15.seqTrimAutotuneConfig[trim] & MODE_MASK;
  return (mode <= SEQ_TRIM_AUTOTUNE_LEARN) ? mode : (uint8_t)SEQ_TRIM_AUTOTUNE_OFF;
}

bool trimUsesAfr2(uint8_t trim)
{
  return (configPage15.seqTrimAutotuneConfig[trim] & AFR2_MASK) != 0U;
}

uint8_t configuredTrimCount(void)
{
  uint8_t count = configPage2.nCylinders;
  if (count > (uint8_t)INJ_CHANNELS) { count = (uint8_t)INJ_CHANNELS; }
  return count;
}

uint8_t activeTrimMask(bool learnOnly)
{
  uint8_t mask = 0U;
  const uint8_t count = configuredTrimCount();
  for (uint8_t trim = 0U; trim < count; trim++) {
    const uint8_t mode = trimMode(trim);
    if ((mode != SEQ_TRIM_AUTOTUNE_OFF) && (!learnOnly || (mode == SEQ_TRIM_AUTOTUNE_LEARN))) {
      mask |= (uint8_t)(1U << trim);
    }
  }
  return mask;
}

bool validAfr(uint8_t afr)
{
  return (afr >= AFR_MIN) && (afr <= AFR_MAX);
}

uint8_t persistencePolicy(void)
{
  const uint8_t policy = (configPage15.seqTrimAutotuneFlags >> PERSISTENCE_SHIFT) & PERSISTENCE_MASK;
  return (policy <= PERSIST_PERIODIC) ? policy : (uint8_t)PERSIST_ENGINE_STOP;
}

bool targetObjective(void)
{
  return (configPage15.seqTrimAutotuneFlags & OBJECTIVE_TARGET_MASK) != 0U;
}

uint16_t resistanceSeconds(uint8_t trim)
{
  const uint8_t preset = (configPage15.seqTrimAutotuneConfig[trim] >> RESISTANCE_SHIFT) & RESISTANCE_MASK;
  return (uint16_t)(4U << preset); //4, 8, ... 512 seconds at one-percent error
}

void clearHistory(void)
{
  historyHead = 0U;
  historyFilled = 0U;
  stableTicks = 0U;
  for (uint8_t index = 0U; index < RING_SIZE; index++) {
    history[index] = HistorySample{0U, 0U, 0U, {0U, 0U}, false};
  }
}

void clearTrimEvidence(uint8_t trim)
{
  for (uint8_t cell = 0U; cell < TABLE_CELLS; cell++) {
    cellAccumulator[trim][cell] = 0;
  }
}

void resetEvidence(void)
{
  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    clearTrimEvidence(trim);
  }
}

void beginDriveCycle(void)
{
  resetEvidence();
  clearHistory();
  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    for (uint8_t cell = 0U; cell < TABLE_CELLS; cell++) {
      sessionBaseline[trim][cell] = trimTables[trim].values.values[cell];
    }
  }
  tablesDirty = false;
  saveDelayTicks = 0U;
  sessionActive = true;
}

void restoreRamOnlyTables(void)
{
  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    bool changed = false;
    for (uint8_t cell = 0U; cell < TABLE_CELLS; cell++) {
      if (trimTables[trim].values.values[cell] != sessionBaseline[trim][cell]) {
        trimTables[trim].values.values[cell] = sessionBaseline[trim][cell];
        changed = true;
      }
    }
    if (changed) { invalidate_cache(&trimTables[trim].get_value_cache); }
  }
}

void endDriveCycle(void)
{
  if (tablesDirty) {
    if (persistencePolicy() == PERSIST_RAM_ONLY) {
      restoreRamOnlyTables();
    } else {
      setEepromWritePending(true);
    }
  }
  tablesDirty = false;
  saveDelayTicks = 0U;
  sessionActive = false;
  clearHistory();
}

void handleConfigurationChanges(void)
{
  const uint8_t objective = configPage15.seqTrimAutotuneFlags & OBJECTIVE_TARGET_MASK;
  if (objective != previousObjective) {
    resetEvidence();
    clearHistory();
    previousObjective = objective;
  }

  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    if (configPage15.seqTrimAutotuneConfig[trim] != previousConfig[trim]) {
      clearTrimEvidence(trim);
      previousConfig[trim] = configPage15.seqTrimAutotuneConfig[trim];
    }
  }
}

uint16_t commonGateBits(void)
{
  uint16_t bits = 0U;
  diag.activeMask = activeTrimMask(false);
  diag.learnMask = activeTrimMask(true);

  if ((diag.activeMask == 0U) || !configPage6.fuelTrimEnabled) { bits |= SEQ_TRIM_GATE_DISABLED; }
  if ((configPage2.injLayout != INJ_SEQUENTIAL) || (configPage2.nCylinders > (uint8_t)INJ_CHANNELS)) { bits |= SEQ_TRIM_GATE_LAYOUT; }
  if (configPage6.egoType != EGO_TYPE_WIDE) { bits |= SEQ_TRIM_GATE_WIDEBAND; }
  if (currentStatus.rotationStatus != EngineRotationStatus::Running) { bits |= SEQ_TRIM_GATE_NOT_RUNNING; }

  const uint8_t minRunSecs = (configPage6.ego_sdelay > MIN_RUN_SECONDS) ? configPage6.ego_sdelay : MIN_RUN_SECONDS;
  if (currentStatus.runSecs < minRunSecs) { bits |= SEQ_TRIM_GATE_RUN_TIME; }
  if (currentStatus.coolant < temperatureRemoveOffset(configPage15.seqTrimAutotuneMinClt)) { bits |= SEQ_TRIM_GATE_COLD; }

  const uint16_t minRpm = (uint16_t)configPage15.seqTrimAutotuneMinRpmDiv100 * 100U;
  const uint16_t maxRpm = (uint16_t)configPage15.seqTrimAutotuneMaxRpmDiv100 * 100U;
  if ((minRpm >= maxRpm) || (currentStatus.RPM < minRpm) || (currentStatus.RPM > maxRpm)) { bits |= SEQ_TRIM_GATE_RPM; }

  const uint16_t minLoad = (uint16_t)configPage15.seqTrimAutotuneMinLoadDiv2 * 2U;
  const uint16_t maxLoad = (uint16_t)configPage15.seqTrimAutotuneMaxLoadDiv2 * 2U;
  if ((minLoad >= maxLoad) || (currentStatus.fuelLoad < minLoad) || (currentStatus.fuelLoad > maxLoad)) { bits |= SEQ_TRIM_GATE_LOAD; }

  if ((absoluteI16(currentStatus.tpsDOT) > MAX_TPS_DOT) || (absoluteI16(currentStatus.mapDOT) > MAX_MAP_DOT)) { bits |= SEQ_TRIM_GATE_TRANSIENT; }
  if (currentStatus.isDFCOActive || currentStatus.aseIsActive || currentStatus.wueIsActive
      || currentStatus.isAcceleratingTPS || currentStatus.isDeceleratingTPS
      || (absoluteI16((int16_t)currentStatus.egoCorrection - 100) > MAX_EGO_CORRECTION)) {
    bits |= SEQ_TRIM_GATE_FUEL_TRANSIENT;
  }
  if (currentStatus.launchingHard || currentStatus.launchingSoft || currentStatus.flatShiftingHard
      || currentStatus.flatShiftSoftCut || currentStatus.nitrousActive || currentStatus.stagingActive) { bits |= SEQ_TRIM_GATE_MOTORSPORT; }
  if (currentStatus.engineProtect.rpm || currentStatus.engineProtect.boostCut || currentStatus.engineProtect.oil
      || currentStatus.engineProtect.afr || currentStatus.engineProtect.coolant) { bits |= SEQ_TRIM_GATE_PROTECTION; }
  if (currentStatus.battery10 < MIN_BATTERY10) { bits |= SEQ_TRIM_GATE_BATTERY; }

  if (targetObjective()) {
    if (!validAfr(currentStatus.afrTarget)) { bits |= SEQ_TRIM_GATE_NO_REFERENCE; }
    const uint8_t count = configuredTrimCount();
    for (uint8_t trim = 0U; trim < count; trim++) {
      if ((trimMode(trim) != SEQ_TRIM_AUTOTUNE_OFF)
          && !validAfr(trimUsesAfr2(trim) ? currentStatus.O2_2 : currentStatus.O2)) {
        bits |= SEQ_TRIM_GATE_AFR_RANGE;
      }
    }
  } else {
    //Cylinder balance is identifiable only with both independent feedback signals.
    if (!validAfr(currentStatus.O2) || !validAfr(currentStatus.O2_2)) {
      bits |= SEQ_TRIM_GATE_NO_REFERENCE;
    }
  }

  if (((configPage15.seqTrimAutotuneFlags >> PERSISTENCE_SHIFT) & PERSISTENCE_MASK) == 3U) { bits |= SEQ_TRIM_GATE_CONFIG; }
  return bits;
}

const HistorySample* dueHistorySample(uint8_t channel)
{
  uint8_t available = historyFilled;
  if (available > MAX_DELAY_TICKS) { available = MAX_DELAY_TICKS; }
  for (uint8_t age = 1U; age <= available; age++) {
    const uint8_t index = (uint8_t)((historyHead + RING_SIZE - age) % RING_SIZE);
    if (history[index].afrDelayTicks[channel] == age) { return &history[index]; }
  }
  return nullptr;
}

void axisMembership(const table3d_axis_t *axis, uint16_t value, uint16_t factor,
                    uint8_t &first, uint8_t &second, uint8_t &firstWeight)
{
  const uint16_t top = (uint16_t)axis[0] * factor;
  const uint16_t bottom = (uint16_t)axis[TABLE_DIM - 1U] * factor;
  if (value >= top) {
    first = second = 0U;
    firstWeight = WEIGHT_SCALE;
    return;
  }
  if (value <= bottom) {
    first = second = TABLE_DIM - 1U;
    firstWeight = WEIGHT_SCALE;
    return;
  }

  for (uint8_t index = 0U; index < TABLE_DIM - 1U; index++) {
    const uint16_t high = (uint16_t)axis[index] * factor;
    const uint16_t low = (uint16_t)axis[index + 1U] * factor;
    if ((value <= high) && (value >= low) && (high > low)) {
      first = index;
      second = index + 1U;
      firstWeight = (uint8_t)(((uint32_t)(value - low) * WEIGHT_SCALE + ((high - low) / 2U)) / (high - low));
      return;
    }
  }

  first = second = TABLE_DIM - 1U;
  firstWeight = WEIGHT_SCALE;
}

uint8_t valueIndex(uint8_t xMemoryIndex, uint8_t yMemoryIndex)
{
  return (uint8_t)(yMemoryIndex * TABLE_DIM + (TABLE_DIM - 1U - xMemoryIndex));
}

void weightedCells(const trimTable3d &table, uint16_t rpm, uint16_t load, WeightedCell (&cells)[4])
{
  uint8_t x0, x1, wx0;
  uint8_t y0, y1, wy0;
  axisMembership(table.axisX.axis, rpm, 100U, x0, x1, wx0);
  axisMembership(table.axisY.axis, load, 2U, y0, y1, wy0);
  const uint8_t wx1 = WEIGHT_SCALE - wx0;
  const uint8_t wy1 = WEIGHT_SCALE - wy0;

  cells[0] = WeightedCell{valueIndex(x0, y0), (uint8_t)(((uint16_t)wx0 * wy0) / WEIGHT_SCALE)};
  cells[1] = WeightedCell{valueIndex(x1, y0), (uint8_t)(((uint16_t)wx1 * wy0) / WEIGHT_SCALE)};
  cells[2] = WeightedCell{valueIndex(x0, y1), (uint8_t)(((uint16_t)wx0 * wy1) / WEIGHT_SCALE)};
  const uint8_t assigned = (uint8_t)(cells[0].weight + cells[1].weight + cells[2].weight);
  cells[3] = WeightedCell{valueIndex(x1, y1), (uint8_t)(WEIGHT_SCALE - assigned)};
}

int16_t feedbackError(uint8_t trim, const HistorySample &sample)
{
  const uint8_t measured = trimUsesAfr2(trim) ? currentStatus.O2_2 : currentStatus.O2;
  uint8_t reference = sample.afrTarget;
  if (!targetObjective()) {
    reference = (uint8_t)(((uint16_t)currentStatus.O2 + currentStatus.O2_2 + 1U) / 2U);
  }
  if (!validAfr(measured) || !validAfr(reference)) { return 0; }
  return clampError(((int32_t)measured - reference) * 1000L / reference);
}

bool updateCell(uint8_t trim, uint8_t cell, uint8_t weight, int16_t error)
{
  if ((weight == 0U) || (configPage15.seqTrimAutotuneAuthority[trim] == 0U)) { return false; }

  const uint16_t resistance = resistanceSeconds(trim);
  const int32_t threshold = (int32_t)resistance * 30L * 10L * WEIGHT_SCALE;
  int32_t accumulated = cellAccumulator[trim][cell] + (int32_t)error * weight;
  if (accumulated > threshold) { accumulated = threshold; }
  if (accumulated < -threshold) { accumulated = -threshold; }
  cellAccumulator[trim][cell] = accumulated;

  int8_t delta = 0;
  if (accumulated >= threshold) { delta = 1; }
  else if (accumulated <= -threshold) { delta = -1; }
  if (delta == 0) { return false; }

  uint8_t authority = configPage15.seqTrimAutotuneAuthority[trim];
  if (authority > 50U) { authority = 50U; }
  int16_t lower = (int16_t)sessionBaseline[trim][cell] - authority;
  int16_t upper = (int16_t)sessionBaseline[trim][cell] + authority;
  if (lower < RAW_TRIM_MIN) { lower = RAW_TRIM_MIN; }
  if (upper > RAW_TRIM_MAX) { upper = RAW_TRIM_MAX; }

  const uint8_t oldValue = trimTables[trim].values.values[cell];
  const int16_t proposed = (int16_t)oldValue + delta;
  if ((proposed < lower) || (proposed > upper)) {
    cellAccumulator[trim][cell] = 0; //anti-windup at the authority boundary
    diag.authorityMask |= (uint8_t)(1U << trim);
    return false;
  }

  trimTables[trim].values.values[cell] = (uint8_t)proposed;
  invalidate_cache(&trimTables[trim].get_value_cache);
  cellAccumulator[trim][cell] -= (delta > 0) ? threshold : -threshold;
  if (!tablesDirty) { saveDelayTicks = 0U; }
  tablesDirty = true;
  if (diag.cellUpdates < UINT16_MAX) { diag.cellUpdates++; }
  diag.lastTrim = trim;
  diag.lastCell = cell;
  diag.lastDelta = delta;
  return true;
}

void processDelayedSample(const HistorySample &sample, uint8_t trim)
{
  if (!sample.eligible) { return; }
  if (!targetObjective() && (!validAfr(currentStatus.O2) || !validAfr(currentStatus.O2_2))) { return; }
  const uint8_t mode = trimMode(trim);
  if (mode == SEQ_TRIM_AUTOTUNE_OFF) { return; }
  if (targetObjective() && !validAfr(trimUsesAfr2(trim) ? currentStatus.O2_2 : currentStatus.O2)) { return; }

  const int16_t error = feedbackError(trim, sample);
  diag.lastErrorTenthsPercent = error;
  if (diag.acceptedSamples < UINT16_MAX) { diag.acceptedSamples++; }
  if ((mode != SEQ_TRIM_AUTOTUNE_LEARN) || (absoluteI16(error) <= configPage15.seqTrimAutotuneDeadband)) { return; }

  bool learned = false;
  WeightedCell cells[4];
  weightedCells(trimTables[trim], sample.rpm, sample.load, cells);
  for (uint8_t slot = 0U; slot < 4U; slot++) {
    learned |= updateCell(trim, cells[slot].index, cells[slot].weight, error);
  }
  if (learned) { diag.state = SEQ_TRIM_STATE_LEARNING; }
}

void updatePersistence(void)
{
  diag.secondsToNextSave = 0U;
  if ((persistencePolicy() == PERSIST_PERIODIC) && (configPage15.seqTrimAutotuneSavePeriod > 0U) && tablesDirty) {
    saveDelayTicks++;
    const uint32_t period = (uint32_t)configPage15.seqTrimAutotuneSavePeriod * TICKS_PER_MINUTE;
    if (saveDelayTicks >= period) {
      setEepromWritePending(true);
      tablesDirty = false;
      saveDelayTicks = 0U;
    } else {
      const uint32_t remaining = (period - saveDelayTicks) / 30U;
      diag.secondsToNextSave = (remaining > UINT16_MAX) ? UINT16_MAX : (uint16_t)remaining;
    }
  } else {
    saveDelayTicks = 0U;
  }
}

} //namespace

void seqTrimAutotuneInit(void)
{
  diag = SeqTrimAutotuneDiagnostics();
  resetEvidence();
  clearHistory();
  sessionActive = false;
  tablesDirty = false;
  saveDelayTicks = 0U;
  previousObjective = configPage15.seqTrimAutotuneFlags & OBJECTIVE_TARGET_MASK;
  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    previousConfig[trim] = configPage15.seqTrimAutotuneConfig[trim];
  }
}

void seqTrimAutotuneUpdate(void)
{
  handleConfigurationChanges();

  if (currentStatus.rotationStatus == EngineRotationStatus::Stopped) {
    if (sessionActive) { endDriveCycle(); }
    diag.gateBits = commonGateBits();
    diag.state = (diag.activeMask == 0U) ? SEQ_TRIM_STATE_DISABLED : SEQ_TRIM_STATE_BLOCKED;
    diag.secondsToNextSave = 0U;
    return;
  }
  if ((currentStatus.rotationStatus == EngineRotationStatus::Running) && !sessionActive) { beginDriveCycle(); }

  diag.authorityMask = 0U;
  diag.gateBits = commonGateBits();
  const bool eligibleNow = (diag.gateBits == 0U);
  if (eligibleNow) {
    if (stableTicks < UINT16_MAX) { stableTicks++; }
  } else {
    stableTicks = 0U;
  }
  const uint16_t requiredStableTicks = (uint16_t)configPage15.seqTrimAutotuneStableTime * 3U;
  const bool stableNow = eligibleNow && (stableTicks >= requiredStableTicks);

  const uint16_t currentLoad = (uint16_t)currentStatus.fuelLoad;
  history[historyHead] = HistorySample{
      currentStatus.RPM,
      currentLoad,
      currentStatus.afrTarget,
      {
        afrDelayTicks30Hz(AFR_DELAY_CHANNEL_1, currentStatus.RPM, currentLoad, MAX_DELAY_TICKS),
        afrDelayTicks30Hz(AFR_DELAY_CHANNEL_2, currentStatus.RPM, currentLoad, MAX_DELAY_TICKS),
      },
      stableNow
  };

  diag.state = (diag.activeMask == 0U) ? SEQ_TRIM_STATE_DISABLED
             : (diag.gateBits != 0U) ? SEQ_TRIM_STATE_BLOCKED
             : !stableNow ? SEQ_TRIM_STATE_STABILISING
             : (diag.learnMask != 0U) ? SEQ_TRIM_STATE_LEARNING
             : SEQ_TRIM_STATE_OBSERVING;

  //Both ends of the causal pair must be acceptable: the delayed operating
  //point that produced the exhaust gas and the current wideband measurement.
  if (sessionActive && stableNow) {
    //Different sensors may have different physical locations and response
    //times. Process one causal history point per active trim.
    const uint8_t count = configuredTrimCount();
    for (uint8_t trim = 0U; trim < count; trim++) {
      if (trimMode(trim) == SEQ_TRIM_AUTOTUNE_OFF) { continue; }
      const uint8_t channel = trimUsesAfr2(trim) ? AFR_DELAY_CHANNEL_2 : AFR_DELAY_CHANNEL_1;
      const HistorySample *sample = dueHistorySample(channel);
      if (sample != nullptr) { processDelayedSample(*sample, trim); }
    }
  }

  historyHead = (uint8_t)((historyHead + 1U) % RING_SIZE);
  if (historyFilled < RING_SIZE) { historyFilled++; }
  if (diag.authorityMask != 0U) { diag.state = SEQ_TRIM_STATE_AUTHORITY; }
  updatePersistence();
}

const SeqTrimAutotuneDiagnostics& seqTrimAutotuneDiag(void)
{
  return diag;
}

#if defined(UNIT_TEST)
int32_t seqTrimAutotuneCellAccumulator(uint8_t trim, uint8_t cell)
{
  if ((trim >= (uint8_t)INJ_CHANNELS) || (cell >= TABLE_CELLS)) { return 0; }
  return cellAccumulator[trim][cell];
}
#endif

#endif //CORE_AVR production stub / full learner
