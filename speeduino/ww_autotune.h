/** @file ww_autotune.h
 * @brief Wall wetting (X-Tau) table autotune from wideband AFR feedback.
 *
 * Learns the wall wetting "added to wall" (X) and "removed from wall" (Y)
 * coefficient tables by observing the de-lagged AFR error following throttle
 * tip-in transients. See ww_autotune.cpp for the algorithm description.
 *
 * Enabled when configPage2.aeMode == AE_MODE_WALL_WETTING and
 * configPage2.wallWettingFuel > 0. The wallWettingFuel value doubles as the
 * learning authority: the maximum number of table counts a single cell may be
 * moved away from its value at boot, per drive cycle.
 *
 * While the engine runs, learned changes are additionally persisted to EEPROM
 * every configPage9.wwLearnSavePeriod minutes (0 = only at engine stop), so
 * an unexpected power loss cannot discard a drive's worth of learning.
 */
#ifndef WW_AUTOTUNE_H
#define WW_AUTOTUNE_H

#include <stdint.h>

// --- Learning gate bits (see WwAutotuneDiagnostics::gateBits). 0 == gate passes ---
constexpr uint16_t WW_GATE_DISABLED    = (1U << 0);  ///< aeMode is not wall wetting, or authority (wallWettingFuel) is 0
constexpr uint16_t WW_GATE_NO_WIDEBAND = (1U << 1);  ///< EGO sensor is not a wideband
constexpr uint16_t WW_GATE_NOT_RUNNING = (1U << 2);  ///< Engine not in the Running state
constexpr uint16_t WW_GATE_RUN_TIME    = (1U << 3);  ///< Too soon after start (ego_sdelay / 30s minimum)
constexpr uint16_t WW_GATE_COLD        = (1U << 4);  ///< Coolant below the EGO or AE cold-taper threshold
constexpr uint16_t WW_GATE_RPM_LOW     = (1U << 5);  ///< RPM below the EGO active RPM
constexpr uint16_t WW_GATE_RPM_TAPER   = (1U << 6);  ///< RPM in/above the AE RPM taper region
constexpr uint16_t WW_GATE_FUEL_CUT    = (1U << 7);  ///< DFCO/ASE/WUE active
constexpr uint16_t WW_GATE_MOTORSPORT  = (1U << 8);  ///< Launch/flat shift/nitrous/staging active
constexpr uint16_t WW_GATE_EGO_TRIM    = (1U << 9);  ///< Closed loop trim not near neutral
constexpr uint16_t WW_GATE_BATTERY     = (1U << 10); ///< Battery voltage too low for linear injectors
constexpr uint16_t WW_GATE_O2_RANGE    = (1U << 11); ///< Wideband reading outside the plausible band

// --- Learner state (see WwAutotuneDiagnostics::state) ---
constexpr uint8_t WW_STATE_DISABLED  = 0U; ///< Autotune off (WW_GATE_DISABLED set)
constexpr uint8_t WW_STATE_BLOCKED   = 1U; ///< Enabled but at least one other gate is failing
constexpr uint8_t WW_STATE_ARMED     = 2U; ///< Gates pass, waiting for a tip-in
constexpr uint8_t WW_STATE_COOLDOWN  = 3U; ///< Post-event cooldown (history ring flush)
constexpr uint8_t WW_STATE_MEASURING = 4U; ///< Tip-in event being observed
constexpr uint8_t WW_STATE_DISCARDED = 5U; ///< Event aborted, waiting out the window

// --- Abort reasons (see WwAutotuneDiagnostics::lastAbortReason) ---
constexpr uint8_t WW_ABORT_NONE           = 0U;
constexpr uint8_t WW_ABORT_GATE_DROPPED   = 1U; ///< A learning gate failed mid-window
constexpr uint8_t WW_ABORT_DECELERATION   = 2U; ///< Lift-off mid-window emptied the film
constexpr uint8_t WW_ABORT_RETRIGGER      = 3U; ///< Second tip-in stacked on the first
constexpr uint8_t WW_ABORT_ENGINE_STOPPED = 4U; ///< Engine stopped mid-window

/** @brief Live diagnostics of the autotune, exposed as TS output channels (offsets 142-156) */
struct WwAutotuneDiagnostics {
  uint16_t gateBits;          ///< Currently failing learning gates (0 = all pass)
  uint8_t state;              ///< Learner state (WW_STATE_*)
  uint8_t lastAbortReason;    ///< Why the most recent event was discarded (WW_ABORT_*)
  uint16_t eventsCompleted;   ///< Events that survived the full observation window
  uint16_t eventsLearned;     ///< Completed events that moved at least one table cell
  uint16_t eventsAborted;     ///< Events discarded mid-window
  uint8_t lastCellIndex;      ///< Value-array index of the last learned cell (255 = none yet)
  uint8_t lastAddValue;       ///< Add table value at that cell after the adjustment
  uint8_t lastRemoveValue;    ///< Remove table value at that cell after the adjustment
  uint16_t secondsToNextSave; ///< Countdown to the next periodic EEPROM save (0 = none scheduled)
};

/** @brief Initialise the autotune state. Call once at startup, after the tune has been loaded.
 *
 * Seeds physically plausible baseline values into the wall wetting tables if
 * they are empty/invalid (fresh page, never tuned).
 */
void wwAutotuneInit(void);

/** @brief Run one autotune step. Call at 30Hz (BIT_TIMER_30HZ), after corrections have run. */
void wwAutotuneUpdate(void);

/** @brief Seed baseline wall wetting tables derived from the engine parameters.
 *
 * @param force When false, tables are only seeded if their current content is
 * empty or degenerate (axis not strictly increasing). When true, always seeds.
 * @return true if the tables were (re)seeded
 */
bool wwSeedBaselineTables(bool force);

/** @brief Live autotune diagnostics, refreshed every 30Hz update */
const WwAutotuneDiagnostics& wwAutotuneDiag(void);

/** @brief Number of transient events that resulted in a table adjustment (diagnostics) */
uint16_t wwAutotuneLearnedEventCount(void);

/** @brief AFR1 map plus local offset after the learner's 30 Hz quantisation. */
uint16_t wwAutotuneEffectiveDelayMilliseconds(void);

#endif
