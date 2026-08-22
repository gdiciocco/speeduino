#ifndef IDLE_H
#define IDLE_H

#include <stdint.h>

extern uint16_t idle_pwm_max_count; //Used for variable PWM frequency

//Closed-loop IAC gain (relay) autotune states.
constexpr uint8_t IAC_GAINTUNE_OFF     = 0U;
constexpr uint8_t IAC_GAINTUNE_WAITING = 1U;
constexpr uint8_t IAC_GAINTUNE_RELAY   = 2U;
constexpr uint8_t IAC_GAINTUNE_FAILED  = 3U;

//Last autotune result. Values fit in the four diagnostic status bits.
constexpr uint8_t IAC_GAINTUNE_RESULT_NONE             = 0U;
constexpr uint8_t IAC_GAINTUNE_RESULT_DONE             = 1U;
constexpr uint8_t IAC_GAINTUNE_RESULT_DISENGAGED       = 2U;
constexpr uint8_t IAC_GAINTUNE_RESULT_RUNAWAY          = 3U;
constexpr uint8_t IAC_GAINTUNE_RESULT_NO_OSCILLATION   = 4U;
constexpr uint8_t IAC_GAINTUNE_RESULT_AMPLITUDE        = 5U;
constexpr uint8_t IAC_GAINTUNE_RESULT_PERIOD           = 6U;
constexpr uint8_t IAC_GAINTUNE_RESULT_AUTHORITY        = 7U;
constexpr uint8_t IAC_GAINTUNE_RESULT_INVALID_MODE     = 8U;
constexpr uint8_t IAC_GAINTUNE_RESULT_IGNITION_ACTIVE  = 9U;

/** Live closed-loop IAC gain autotune diagnostics (TS offsets 170-175). */
struct IacGainAutotuneDiagnostics {
  uint8_t state;
  uint8_t lastResult;
  uint8_t kpRaw;
  uint8_t kiRaw;
  uint8_t kdRaw;
  uint8_t periodTenths;
  uint8_t amplitudeRpm;
  bool relayHigh;
  bool stepper;
};

const IacGainAutotuneDiagnostics& iacGainAutotuneDiag(void);
uint8_t buildIacGainAutotuneStatus(void);

void initialiseIdle(bool forcehoming);
void idleControl(void);
void disableIdle(void);
void idleInterrupt(void);

/** @brief True when a closed loop IAC algorithm has driven its output to either
 * configured limit, meaning the air path has no authority left over idle RPM.
 * The idle ignition closed loop only trims its learned center while this holds,
 * so that two integrators never act on the same measurement at the same time.
 */
bool isIdleClosedLoopAtLimit(void);

#endif
