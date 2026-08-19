/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

void initialiseCorrections(void);
uint16_t correctionsFuel(void);
uint8_t calculateAfrTarget(table3d16RpmLoad &afrLookUpTable, const statuses &current, const config2 &page2, const config6 &page6);

int8_t correctionsIgn(int8_t advance);
int8_t correctionFixedTiming(int8_t advance);
int8_t correctionCrankingFixedTiming(int8_t advance);

uint16_t correctionsDwell(uint16_t dwell);

// --- Closed-loop idle ignition center autotune states (see corrections.cpp) ---
constexpr uint8_t IDLE_ADV_LEARN_OFF      = 0U; ///< Learning disabled: authority 0, trim disabled, or mode not closed loop
constexpr uint8_t IDLE_ADV_LEARN_INACTIVE = 1U; ///< Closed loop not currently engaged (engine off, throttle open, delay running)
constexpr uint8_t IDLE_ADV_LEARN_COLD     = 2U; ///< Coolant below the learning threshold
constexpr uint8_t IDLE_ADV_LEARN_WAITING  = 3U; ///< Engaged and warm, but no whole degree of trim banked while on target
constexpr uint8_t IDLE_ADV_LEARN_SETTLING = 4U; ///< Fold conditions holding, settle timer running
constexpr uint8_t IDLE_ADV_LEARN_LIMITED  = 5U; ///< A fold is due but the authority or the advance range stops it

/** @brief Live diagnostics of the closed-loop idle advance center autotune, exposed as TS output channels (offsets 157-162) */
struct IdleAdvanceLearnDiagnostics {
  uint8_t state;       ///< IDLE_ADV_LEARN_*
  int16_t trimTenths;  ///< Learned trim currently held against the center, tenths of a degree
  int8_t learnedDelta; ///< Degrees the stored center has moved since power-on
  uint8_t centerRaw;   ///< Current stored center (degrees + 40)
  uint8_t settleSecs;  ///< Seconds the fold conditions have been holding
};

/** @brief Live autotune diagnostics, refreshed whenever the idle advance correction runs */
const IdleAdvanceLearnDiagnostics& idleAdvanceLearnDiag(void);

// --- Closed-loop idle ignition gain (relay) autotune states (see corrections.cpp) ---
constexpr uint8_t IDLE_ADV_GAINTUNE_OFF     = 0U; ///< No autotune requested
constexpr uint8_t IDLE_ADV_GAINTUNE_WAITING = 1U; ///< Requested, waiting for a settled warm idle
constexpr uint8_t IDLE_ADV_GAINTUNE_RELAY   = 2U; ///< Relay test running: advance toggling around the center
constexpr uint8_t IDLE_ADV_GAINTUNE_FAILED  = 3U; ///< Attempt budget exhausted; re-tick the request (or power cycle) to retry

// --- Gain autotune results (lastResult) ---
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_NONE           = 0U;
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_DONE           = 1U; ///< Gains measured and written
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_DISENGAGED     = 2U; ///< Loop disengaged mid test (throttle opened, engine stopped)
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_RUNAWAY        = 3U; ///< RPM left the safe band around the target
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_NO_OSCILLATION = 4U; ///< The relay never switched within the timeout
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_AMPLITUDE      = 5U; ///< Oscillation too small to trust the measurement
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_PERIOD         = 6U; ///< Oscillation period implausible for an idle loop
constexpr uint8_t IDLE_ADV_GAINTUNE_RESULT_AUTHORITY      = 7U; ///< Not enough advance range around the center for the relay

/** @brief Live diagnostics of the closed-loop idle advance gain autotune, exposed as TS output channels (offsets 163-169) */
struct IdleAdvanceGainAutotuneDiagnostics {
  uint8_t state;         ///< IDLE_ADV_GAINTUNE_*
  uint8_t lastResult;    ///< IDLE_ADV_GAINTUNE_RESULT_* of the most recent attempt
  uint8_t kpRaw;         ///< Current proportional gain (0.05 deg/100RPM units)
  uint8_t kdRaw;         ///< Current derivative gain (0.05 deg/1000RPM/s units)
  uint8_t periodTenths;  ///< Measured ultimate period, tenths of a second
  uint16_t amplitudeRpm; ///< Measured oscillation amplitude, RPM
};

/** @brief Live gain autotune diagnostics, refreshed whenever the closed loop runs */
const IdleAdvanceGainAutotuneDiagnostics& idleAdvanceGainAutotuneDiag(void);

#endif // CORRECTIONS_H
