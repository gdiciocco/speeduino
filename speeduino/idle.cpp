/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include <Arduino.h>
#include "idle.h"
#include "elapsed_time.h"
#include "maths.h"
#include "timers.h"
#include "preprocessor.h"
#include "src/PID/integerPID.h"
#include "units.h"
#include "globals.h"
#include "storage.h"
#include "unit_testing.h"
#include "src/pins/fastOutputPin.h"

#define STEPPER_FORWARD 0
#define STEPPER_BACKWARD 1
#define STEPPER_POWER_WHEN_ACTIVE 0

enum StepperStatus {SOFF, STEPPING, COOLING}; //The 2 statuses that a stepper can have. STEPPING means that a high pulse is currently being sent and will need to be turned off at some point.

struct StepperIdle
{
  int curIdleStep; //Tracks the current location of the stepper
  int targetIdleStep; //What the targeted step is
  volatile StepperStatus stepperStatus;
  volatile unsigned long stepStartTime;
};

#define STEPPER_LESS_AIR_DIRECTION() ((configPage9.iacStepperInv == 0) ? STEPPER_BACKWARD : STEPPER_FORWARD)
#define STEPPER_MORE_AIR_DIRECTION() ((configPage9.iacStepperInv == 0) ? STEPPER_FORWARD : STEPPER_BACKWARD)

static uint8_t idleUpOutputHIGH = HIGH; // Used to invert the idle Up Output 
static uint8_t idleUpOutputLOW = LOW;   // Used to invert the idle Up Output 
static uint8_t idleCounter; //Used for tracking the number of calls to the idle control function
static uint8_t idleTaper;

static struct StepperIdle idleStepper;
static bool idleOn; //Simply tracks whether idle was on last time around
static uint8_t idleInitComplete = 99; //Tracks which idle method was initialised. 99 is a method that will never exist
static unsigned int iacStepTime_uS;
static unsigned int iacCoolTime_uS;
static unsigned int completedHomeSteps;

static volatile bool idle_pwm_state;
static bool lastDFCOValue;
uint16_t idle_pwm_max_count; //Used for variable PWM frequency
static volatile unsigned int idle_pwm_cur_value;
static long idle_pid_target_value;
//The PID class does not expose its output limits, so record them here as they are
//configured. isIdleClosedLoopAtLimit() reports when the air path has run out of
//authority, which is the only time a second integrator may safely act on idle RPM.
static long idle_pid_output_min;
static long idle_pid_output_max;
static long FeedForwardTerm;
static uint32_t idle_pwm_target_value;
static long idle_cl_target_rpm;

static fastOutputPin_t idle_pin;
static fastOutputPin_t idle2_pin;

constexpr table2D_u8_u8_10 iacPWMTable(&configPage6.iacBins, &configPage6.iacOLPWMVal);
constexpr table2D_u8_u8_10 iacStepTable(&configPage6.iacBins, &configPage6.iacOLStepVal);
//Open loop tables specifically for cranking
constexpr table2D_u8_u8_4 iacCrankStepsTable(&configPage6.iacCrankBins, &configPage6.iacCrankSteps);
constexpr table2D_u8_u8_4 iacCrankDutyTable(&configPage6.iacCrankBins, &configPage6.iacCrankDuty);

/*
These functions cover the PWM and stepper idle control
*/

/*
Idle Control
Currently limited to on/off control and open loop PWM and stepper drive
*/
integerPID idlePID(&currentStatus.longRPM, &idle_pid_target_value, &idle_cl_target_rpm, configPage6.idleKP, configPage6.idleKI, configPage6.idleKD, DIRECT); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call

//IAC relay-autotune state. The active settings and center are frozen at the
//start of an attempt so a live TunerStudio edit cannot change a running test.
static uint8_t iacGainTuneState;
static uint8_t iacGainTuneResult;
TESTABLE_STATIC uint8_t iacGainTuneAttempts;
TESTABLE_STATIC bool iacGainTuneLastRequest;
static uint16_t iacGainTuneStableTicks;
static uint16_t iacGainTuneTicksSinceSwitch;
static uint8_t iacGainTuneHalfCycles;
static uint16_t iacGainTunePeakError;
static uint16_t iacGainTunePeriodSum;
static uint32_t iacGainTuneAmplitudeSum;
static long iacGainTuneCenter;
static long iacGainTuneRelayAmplitude;
static long iacGainTuneRelayOutput;
static int8_t iacGainTuneRelaySign;
static bool iacGainTuneIdleUp;
static bool iacGainTuneAircon;
static bool iacGainTuneStepper;
static uint8_t iacGainTuneFanDuty;
static uint8_t iacGainTuneWaitingFanDuty;
static bool iacGainTuneWaitingFanValid;
static uint8_t iacGainTuneActiveHysteresis;
static uint8_t iacGainTuneActiveDiscard;
static uint8_t iacGainTuneActiveMeasure;
static uint8_t iacGainTuneActiveMaxTps;
static uint8_t iacGainTuneActiveMaxAttempts;
static uint16_t iacGainTuneActiveTimeoutTicks;
static uint16_t iacGainTuneActiveRunawayRpm;
static uint16_t iacGainTuneActiveMinAmplitude;
static uint16_t iacGainTuneActiveMinPeriod;
static uint16_t iacGainTuneActiveMaxPeriod;
static bool iacGainTuneLastTickValid;
static uint32_t iacGainTuneLastTick;
static IacGainAutotuneDiagnostics iacGainTuneDiagnostics;

static inline bool hasIacGainTuneFanLoadChanged(uint8_t referenceDuty) {
  const int16_t delta = (int16_t)currentStatus.fanDuty - (int16_t)referenceDuty;
  return (delta > 2) || (delta < -2);
}

static inline uint8_t getIacGainTuneMaxAttempts(void) {
  return (uint8_t)clamp((int16_t)configPage15.iacGainTuneMaxAttempts, (int16_t)1, (int16_t)5);
}

static inline uint16_t getIacGainTuneStableTicks(void) {
  return (uint16_t)clamp((int16_t)configPage15.iacGainTuneSettleTime, (int16_t)1, (int16_t)25) * 10U;
}

static inline int32_t getIacGainTuneStableBand(void) {
  return (int32_t)clamp((int16_t)configPage15.iacGainTuneSettleBand, (int16_t)5, (int16_t)100);
}

static inline int32_t getIacGainTuneHysteresis(void) {
  return (configPage15.iacGainTuneHysteresis == 0U)
      ? 10L
      : (int32_t)clamp((int16_t)configPage15.iacGainTuneHysteresis, (int16_t)5, (int16_t)100);
}

static inline uint8_t getIacGainTuneDiscardHalfCycles(void) {
  return (uint8_t)min(configPage15.iacGainTuneDiscard, (uint8_t)8U);
}

static inline uint8_t getIacGainTuneMeasureHalfCycles(void) {
  return (uint8_t)clamp((int16_t)configPage15.iacGainTuneMeasure, (int16_t)4, (int16_t)20);
}

static inline uint16_t getIacGainTuneTimeoutTicks(void) {
  return (uint16_t)clamp((int16_t)configPage15.iacGainTuneTimeout, (int16_t)1, (int16_t)30) * 10U;
}

static inline int32_t getIacGainTuneRunawayRpm(void) {
  return clamp((int32_t)configPage15.iacGainTuneRunawayDiv10 * (int32_t)10,
               (int32_t)100, (int32_t)1000);
}

static inline uint16_t getIacGainTuneMinAmplitude(void) {
  return (uint16_t)clamp((int16_t)configPage15.iacGainTuneMinAmplitude, (int16_t)10, (int16_t)200);
}

static inline uint16_t getIacGainTuneMinPeriodTicks(void) {
  return (uint16_t)clamp((int16_t)configPage15.iacGainTuneMinPeriod, (int16_t)2, (int16_t)30);
}

static inline uint16_t getIacGainTuneMaxPeriodTicks(void) {
  const uint16_t minimum = getIacGainTuneMinPeriodTicks();
  return (uint16_t)clamp((int16_t)configPage15.iacGainTuneMaxPeriod, (int16_t)minimum, (int16_t)200);
}

static inline void updateIacGainTuneDiag(void) {
  iacGainTuneDiagnostics.state = iacGainTuneState;
  iacGainTuneDiagnostics.lastResult = iacGainTuneResult;
  iacGainTuneDiagnostics.kpRaw = configPage6.idleKP;
  iacGainTuneDiagnostics.kiRaw = configPage6.idleKI;
  iacGainTuneDiagnostics.kdRaw = configPage6.idleKD;
  iacGainTuneDiagnostics.relayHigh = (iacGainTuneState == IAC_GAINTUNE_RELAY) && (iacGainTuneRelaySign > 0);
  iacGainTuneDiagnostics.stepper = isStepperIac(configPage6);
}

const IacGainAutotuneDiagnostics& iacGainAutotuneDiag(void) {
  return iacGainTuneDiagnostics;
}

uint8_t buildIacGainAutotuneStatus(void) {
  return (uint8_t)((iacGainTuneDiagnostics.state & 0x03U)
      | ((iacGainTuneDiagnostics.lastResult & 0x0FU) << 2U)
      | (iacGainTuneDiagnostics.relayHigh ? 0x40U : 0U)
      | (iacGainTuneDiagnostics.stepper ? 0x80U : 0U));
}

static inline void seedIacPidFromRelay(void) {
  idle_pid_target_value = clamp(iacGainTuneRelayOutput, idle_pid_output_min, idle_pid_output_max);
  idlePID.Initialize();
}

static inline void abortIacGainAutotune(uint8_t reason) {
  if(iacGainTuneState == IAC_GAINTUNE_RELAY) { seedIacPidFromRelay(); }
  iacGainTuneResult = reason;
  const uint8_t maxAttempts = ((iacGainTuneState == IAC_GAINTUNE_RELAY) && (iacGainTuneActiveMaxAttempts > 0U))
      ? iacGainTuneActiveMaxAttempts : getIacGainTuneMaxAttempts();
  iacGainTuneState = (iacGainTuneAttempts >= maxAttempts)
      ? IAC_GAINTUNE_FAILED : IAC_GAINTUNE_WAITING;
  iacGainTuneStableTicks = 0U;
  updateIacGainTuneDiag();
}

/** Convert relay-test measurements into the integerPID raw Kp/Ki/Kd units. */
TESTABLE_STATIC bool calculateIacGainAutotuneGains(long relayInternal, uint16_t amplitudeRpm,
                                                   uint16_t periodTenths,
                                                   uint8_t &kpRaw, uint8_t &kiRaw, uint8_t &kdRaw) {
  if((relayInternal <= 0L) || (amplitudeRpm == 0U) || (periodTenths == 0U)) { return false; }

  //Ku=4d/(pi*a), classic Ziegler-Nichols PID: Kp=.6Ku, Ti=Tu/2, Td=Tu/8.
  //integerPID stores Kp/Ki in 1/32 and Kd in 1/128 units. The relay amplitude
  //is already expressed in the PID output's x4 internal units.
  const int32_t amplitude = (int32_t)amplitudeRpm;
  const int32_t period = (int32_t)periodTenths;
  const int32_t kp = (7680L * relayInternal) / (314L * amplitude);
  const int32_t ki = (153600L * relayInternal) / (314L * amplitude * period);
  const int32_t kd = (384L * relayInternal * period) / (314L * amplitude);
  kpRaw = (uint8_t)clamp(kp, (int32_t)1, (int32_t)255);
  kiRaw = (uint8_t)clamp(ki, (int32_t)1, (int32_t)255);
  kdRaw = (uint8_t)clamp(kd, (int32_t)0, (int32_t)255);
  return true;
}

static void finalizeIacGainAutotune(void) {
  const uint8_t measured = iacGainTuneActiveMeasure;
  const uint16_t halfPeriod = (uint16_t)(iacGainTunePeriodSum / measured);
  const uint16_t period = (uint16_t)(halfPeriod * 2U);
  const uint16_t amplitude = (uint16_t)(iacGainTuneAmplitudeSum / measured);
  iacGainTuneDiagnostics.periodTenths = (period > UINT8_MAX) ? UINT8_MAX : (uint8_t)period;
  iacGainTuneDiagnostics.amplitudeRpm = (amplitude > UINT8_MAX) ? UINT8_MAX : (uint8_t)amplitude;

  if(amplitude < iacGainTuneActiveMinAmplitude) {
    abortIacGainAutotune(IAC_GAINTUNE_RESULT_AMPLITUDE);
    return;
  }
  if((period < iacGainTuneActiveMinPeriod) || (period > iacGainTuneActiveMaxPeriod)) {
    abortIacGainAutotune(IAC_GAINTUNE_RESULT_PERIOD);
    return;
  }

  uint8_t kp;
  uint8_t ki;
  uint8_t kd;
  if(!calculateIacGainAutotuneGains(iacGainTuneRelayAmplitude, amplitude, period, kp, ki, kd)) {
    abortIacGainAutotune(IAC_GAINTUNE_RESULT_AMPLITUDE);
    return;
  }
  configPage6.idleKP = kp;
  configPage6.idleKI = ki;
  configPage6.idleKD = kd;
  idlePID.SetTunings(kp, ki, kd);
  seedIacPidFromRelay();
  configPage15.iacGainAutotuneRequest = 0U;
  setEepromWritePending(true);
  iacGainTuneResult = IAC_GAINTUNE_RESULT_DONE;
  iacGainTuneState = IAC_GAINTUNE_OFF;
  iacGainTuneStableTicks = 0U;
  updateIacGainTuneDiag();
}

static inline long getIacGainTuneRelayAmplitude(void) {
  if(isStepperIac(configPage6)) {
    const long steps = clamp((long)configPage15.iacGainTuneStep, 1L, 50L);
    return steps << 2;
  }
  const uint8_t percent = (uint8_t)clamp((int16_t)configPage15.iacGainTuneStep, (int16_t)1, (int16_t)20);
  return percentage(percent, idle_pwm_max_count << 2);
}

static inline bool iacGainTuneGatesPass(int32_t rpmError, bool actuatorSettled) {
  const bool taperComplete = ((configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL)
      || (idleTaper >= configPage2.idleTaperTime));
  return (currentStatus.rotationStatus == EngineRotationStatus::Running)
      && (currentStatus.coolant >= temperatureRemoveOffset(configPage15.iacGainTuneMinTemp))
      && (currentStatus.TPS <= (uint8_t)clamp((int16_t)configPage15.iacGainTuneMaxTps, (int16_t)0, (int16_t)40))
      && !currentStatus.isDFCOActive
      && taperComplete
      && (currentStatus.vss == 0U)
      && !currentStatus.idleUpActive
      && !currentStatus.airconTurningOn
      && !currentStatus.airconCompressorOn
      && actuatorSettled
      && (rpmError <= getIacGainTuneStableBand())
      && (rpmError >= -getIacGainTuneStableBand());
}

/** Run the IAC relay test. Returns true while the relay owns the PID output. */
static inline bool updateIacGainAutotune(long currentCenter, bool actuatorSettled, long &relayOutput) {
  const bool requested = (configPage15.iacGainAutotuneRequest == 1U);
  if(requested && !iacGainTuneLastRequest) {
    iacGainTuneAttempts = 0U;
    iacGainTuneResult = IAC_GAINTUNE_RESULT_NONE;
    iacGainTuneDiagnostics.periodTenths = 0U;
    iacGainTuneDiagnostics.amplitudeRpm = 0U;
    iacGainTuneWaitingFanValid = false;
  }
  iacGainTuneLastRequest = requested;

  if(!requested) {
    if(iacGainTuneState == IAC_GAINTUNE_RELAY) { seedIacPidFromRelay(); }
    iacGainTuneState = IAC_GAINTUNE_OFF;
    iacGainTuneStableTicks = 0U;
    iacGainTuneWaitingFanValid = false;
    updateIacGainTuneDiag();
    return false;
  }
  if(!isClosedLoopIac(configPage6)) {
    iacGainTuneState = IAC_GAINTUNE_FAILED;
    iacGainTuneResult = IAC_GAINTUNE_RESULT_INVALID_MODE;
    updateIacGainTuneDiag();
    return false;
  }
  if(configPage2.idleAdvEnabled != IDLEADVANCE_MODE_OFF) {
    if(iacGainTuneState == IAC_GAINTUNE_RELAY) { abortIacGainAutotune(IAC_GAINTUNE_RESULT_IGNITION_ACTIVE); }
    else {
      iacGainTuneState = IAC_GAINTUNE_WAITING;
      iacGainTuneResult = IAC_GAINTUNE_RESULT_IGNITION_ACTIVE;
      iacGainTuneStableTicks = 0U;
      updateIacGainTuneDiag();
    }
    return false;
  }
  if(iacGainTuneResult == IAC_GAINTUNE_RESULT_IGNITION_ACTIVE) {
    iacGainTuneResult = IAC_GAINTUNE_RESULT_NONE;
  }
  if(iacGainTuneAttempts >= getIacGainTuneMaxAttempts()) {
    iacGainTuneState = IAC_GAINTUNE_FAILED;
    updateIacGainTuneDiag();
    return false;
  }

  const int32_t rpmError = ((int32_t)currentStatus.CLIdleTarget * 10L) - (int32_t)currentStatus.RPM;
  //runSecsX10 is the canonical 10Hz epoch. Comparing it directly means a
  //stepper pulse that happens to occupy the exact timer-flag loop cannot make
  //the relay miss a sample; the next actuator-ready loop consumes that epoch.
  const bool newTick = (!iacGainTuneLastTickValid) || (iacGainTuneLastTick != runSecsX10);
  if(newTick) {
    iacGainTuneLastTick = runSecsX10;
    iacGainTuneLastTickValid = true;
  }

  if(iacGainTuneState != IAC_GAINTUNE_RELAY) {
    iacGainTuneState = IAC_GAINTUNE_WAITING;
    if(!iacGainTuneWaitingFanValid || hasIacGainTuneFanLoadChanged(iacGainTuneWaitingFanDuty)) {
      iacGainTuneWaitingFanDuty = currentStatus.fanDuty;
      iacGainTuneWaitingFanValid = true;
      iacGainTuneStableTicks = 0U;
      updateIacGainTuneDiag();
      return false;
    }
    if(!iacGainTuneGatesPass(rpmError, actuatorSettled)) {
      iacGainTuneStableTicks = 0U;
      iacGainTuneResult = IAC_GAINTUNE_RESULT_NONE;
      updateIacGainTuneDiag();
      return false;
    }
    if(!newTick) { updateIacGainTuneDiag(); return false; }
    if(iacGainTuneStableTicks < getIacGainTuneStableTicks()) {
      iacGainTuneStableTicks++;
      updateIacGainTuneDiag();
      return false;
    }

    const long amplitude = getIacGainTuneRelayAmplitude();
    if((amplitude <= 0L) || ((currentCenter - amplitude) < idle_pid_output_min)
    || ((currentCenter + amplitude) > idle_pid_output_max)) {
      iacGainTuneAttempts = getIacGainTuneMaxAttempts();
      abortIacGainAutotune(IAC_GAINTUNE_RESULT_AUTHORITY);
      return false;
    }
    iacGainTuneAttempts++;
    iacGainTuneState = IAC_GAINTUNE_RELAY;
    iacGainTuneCenter = currentCenter;
    iacGainTuneRelayAmplitude = amplitude;
    iacGainTuneRelaySign = 1;
    iacGainTuneIdleUp = currentStatus.idleUpActive;
    iacGainTuneAircon = currentStatus.airconTurningOn || currentStatus.airconCompressorOn;
    iacGainTuneStepper = isStepperIac(configPage6);
    iacGainTuneFanDuty = currentStatus.fanDuty;
    iacGainTuneActiveHysteresis = (uint8_t)getIacGainTuneHysteresis();
    iacGainTuneActiveDiscard = getIacGainTuneDiscardHalfCycles();
    iacGainTuneActiveMeasure = getIacGainTuneMeasureHalfCycles();
    iacGainTuneActiveMaxTps = (uint8_t)clamp((int16_t)configPage15.iacGainTuneMaxTps, (int16_t)0, (int16_t)40);
    iacGainTuneActiveMaxAttempts = getIacGainTuneMaxAttempts();
    iacGainTuneActiveTimeoutTicks = getIacGainTuneTimeoutTicks();
    iacGainTuneActiveRunawayRpm = (uint16_t)getIacGainTuneRunawayRpm();
    iacGainTuneActiveMinAmplitude = getIacGainTuneMinAmplitude();
    iacGainTuneActiveMinPeriod = getIacGainTuneMinPeriodTicks();
    iacGainTuneActiveMaxPeriod = getIacGainTuneMaxPeriodTicks();
    iacGainTuneTicksSinceSwitch = 0U;
    iacGainTuneHalfCycles = 0U;
    iacGainTunePeakError = 0U;
    iacGainTunePeriodSum = 0U;
    iacGainTuneAmplitudeSum = 0UL;
  }

  if((currentStatus.rotationStatus != EngineRotationStatus::Running)
  || (currentStatus.TPS > iacGainTuneActiveMaxTps)
  || currentStatus.isDFCOActive || (currentStatus.vss > 0U)
  || (isStepperIac(configPage6) != iacGainTuneStepper)
  || hasIacGainTuneFanLoadChanged(iacGainTuneFanDuty)
  || (currentStatus.idleUpActive != iacGainTuneIdleUp)
  || ((currentStatus.airconTurningOn || currentStatus.airconCompressorOn) != iacGainTuneAircon)) {
    abortIacGainAutotune(IAC_GAINTUNE_RESULT_DISENGAGED);
    return false;
  }

  if(newTick) {
    const int32_t runaway = (int32_t)iacGainTuneActiveRunawayRpm;
    if((rpmError > runaway) || (rpmError < -runaway)) {
      abortIacGainAutotune(IAC_GAINTUNE_RESULT_RUNAWAY);
      return false;
    }
    iacGainTuneTicksSinceSwitch++;
    if(iacGainTuneTicksSinceSwitch > iacGainTuneActiveTimeoutTicks) {
      abortIacGainAutotune(IAC_GAINTUNE_RESULT_NO_OSCILLATION);
      return false;
    }

    const uint16_t absError = (rpmError < 0L) ? (uint16_t)(-rpmError) : (uint16_t)rpmError;
    if(absError > iacGainTunePeakError) { iacGainTunePeakError = absError; }
    int8_t desiredSign = iacGainTuneRelaySign;
    const int32_t hysteresis = (int32_t)iacGainTuneActiveHysteresis;
    if(rpmError > hysteresis) { desiredSign = 1; }
    else if(rpmError < -hysteresis) { desiredSign = -1; }

    if(desiredSign != iacGainTuneRelaySign) {
      iacGainTuneHalfCycles++;
      const uint8_t discarded = iacGainTuneActiveDiscard;
      const uint8_t measured = iacGainTuneActiveMeasure;
      if(iacGainTuneHalfCycles > discarded) {
        iacGainTunePeriodSum += iacGainTuneTicksSinceSwitch;
        iacGainTuneAmplitudeSum += iacGainTunePeakError;
      }
      iacGainTuneRelaySign = desiredSign;
      iacGainTuneTicksSinceSwitch = 0U;
      iacGainTunePeakError = 0U;
      if(iacGainTuneHalfCycles >= (uint8_t)(discarded + measured)) {
        finalizeIacGainAutotune();
        return false;
      }
    }
  }

  iacGainTuneRelayOutput = clamp(iacGainTuneCenter + ((long)iacGainTuneRelaySign * iacGainTuneRelayAmplitude),
                                 idle_pid_output_min, idle_pid_output_max);
  relayOutput = iacGainTuneRelayOutput;
  updateIacGainTuneDiag();
  return true;
}

//Any common functions associated with starting the Idle
//Typically this is enabling the PWM interrupt
static inline void enableIdle(void)
{
  if (isPwmIac(configPage6))
  {
    IDLE_TIMER_ENABLE();
  }
}

//Set the closed loop output limits, retaining a copy so that saturation of the air
//path can be reported to the idle ignition controller.
static inline void setIdleClOutputLimits(long minimumOutput, long maximumOutput)
{
  idle_pid_output_min = minimumOutput;
  idle_pid_output_max = maximumOutput;
  idlePID.SetOutputLimits(minimumOutput, maximumOutput);
}

bool isIdleClosedLoopAtLimit(void)
{
  if( (!isClosedLoopIac(configPage6))
   || (currentStatus.rotationStatus != EngineRotationStatus::Running)
   || (idle_pid_output_min >= idle_pid_output_max) ) { return false; }

  return (idle_pid_target_value <= idle_pid_output_min)
      || (idle_pid_target_value >= idle_pid_output_max);
}

static inline void initialiseIdleUpOutput(void)
{
  if (configPage2.idleUpOutputInv) { idleUpOutputHIGH = LOW; idleUpOutputLOW = HIGH; }
  else { idleUpOutputHIGH = HIGH; idleUpOutputLOW = LOW; }

  if(configPage2.idleUpEnabled) { digitalWrite(pinIdleUpOutput, idleUpOutputLOW); } //Initialise program with the idle up output in the off state if it is enabled. 
  currentStatus.idleUpOutputActive = false;
}

void initialiseIdle(bool forcehoming)
{
  //By default, turn off the PWM interrupt (It gets turned on below if needed)
  IDLE_TIMER_DISABLE();

  //Pin masks must always be initialised, regardless of whether PWM idle is used. This is required for STM32 to prevent issues if the IRQ function fires on restart/overflow
  idle_pin.setPin(pinIdle1, OUTPUT);
  idle2_pin.setPin(pinIdle2, OUTPUT);

  //Initialising comprises of setting the 2D tables with the relevant values from the config pages
  switch(configPage6.iacAlgorithm)
  {
    case IAC_ALGORITHM_NONE:       
      //Case 0 is no idle control ('None')
      break;

    case IAC_ALGORITHM_ONOFF:
      //Case 1 is on/off idle control
      if ((temperatureAddOffset(currentStatus.coolant)) < configPage6.iacFastTemp)
      {
        idle_pin.setPinHigh();
        idleOn = true;
      }
      break;

    case IAC_ALGORITHM_PWM_OL:
      //Case 2 is PWM open loop
      enableIdle();
      break;

    case IAC_ALGORITHM_PWM_OLCL:
      //Case 6 is PWM closed loop with open loop table used as feed forward
      setIdleClOutputLimits(percentage(configPage2.iacCLminValue, idle_pwm_max_count<<2), percentage(configPage2.iacCLmaxValue, idle_pwm_max_count<<2));
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      idle_pid_target_value = 0;
      idlePID.Initialize();
      idleCounter = 0;

      break;

    case IAC_ALGORITHM_PWM_CL:
      //Case 3 is PWM closed loop
      setIdleClOutputLimits(percentage(configPage2.iacCLminValue, idle_pwm_max_count<<2), percentage(configPage2.iacCLmaxValue, idle_pwm_max_count<<2));
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      idle_pid_target_value = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant));
      idlePID.Initialize();
      idleCounter = 0;

      break;

    case IAC_ALGORITHM_STEP_OL:
      //Case 2 is Stepper open loop
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if (forcehoming)
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      break;

    case IAC_ALGORITHM_STEP_CL:
      //Case 5 is Stepper closed loop
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if (forcehoming)
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      idlePID.SetSampleTime(250); //4Hz means 250ms
      setIdleClOutputLimits(configPage2.iacCLminValue<<2, configPage2.iacCLmaxValue<<2); //Maximum number of steps; always less than home steps count.
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      idle_pid_target_value = currentStatus.CLIdleTarget * 3;
      idlePID.Initialize();
      break;

    case IAC_ALGORITHM_STEP_OLCL:
      //Case 7 is Stepper closed loop with open loop table used as feed forward
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if (forcehoming)
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      idlePID.SetSampleTime(250); //4Hz means 250ms
      setIdleClOutputLimits(configPage2.iacCLminValue<<2, configPage2.iacCLmaxValue<<2); //Maximum number of steps; always less than home steps count.
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      idle_pid_target_value = 0;
      idlePID.Initialize();
      break;

    default:
      //Well this just shouldn't happen
      break;
  }

  initialiseIdleUpOutput();

  idleInitComplete = configPage6.iacAlgorithm; //Sets which idle method was initialised
  currentStatus.idleLoad = 0;
}

/*
Checks whether a step is currently underway or whether the motor is in 'cooling' state (ie whether it's ready to begin another step or not)
Returns:
True: If a step is underway or motor is 'cooling'
False: If the motor is ready for another step
*/
static inline uint8_t checkForStepping(void)
{
  bool isStepping = false;
  unsigned int timeCheck;
  
  if( (idleStepper.stepperStatus == STEPPING) || (idleStepper.stepperStatus == COOLING) )
  {
    if (idleStepper.stepperStatus == STEPPING)
    {
      timeCheck = iacStepTime_uS;
    }
    else 
    {
      timeCheck = iacCoolTime_uS;
    }

    if( hasIntervalElapsed(micros(), idleStepper.stepStartTime, timeCheck) )
    {         
      if(idleStepper.stepperStatus == STEPPING)
      {
        //Means we're currently in a step, but it needs to be turned off
        digitalWrite(pinStepperStep, LOW); //Turn off the step
        idleStepper.stepStartTime = micros();

	//Set status to COOLING. In next cycle, status will be set to SOFF and set stepper power OFF based on given settings
        idleStepper.stepperStatus = COOLING; //'Cooling' is the time the stepper needs to sit in LOW state before the next step can be made
                  
        isStepping = true;
      }
      else
      {
        //Means we're in COOLING status but have been in this state long enough. Go into off state
        idleStepper.stepperStatus = SOFF;
        if(configPage9.iacStepperPower == STEPPER_POWER_WHEN_ACTIVE) 
        { 
          //Disable the DRV8825, but only if we're at the final step in this cycle or within the hysteresis range. 
          if ( (idleStepper.curIdleStep >= (idleStepper.targetIdleStep - configPage6.iacStepHyster)) && (idleStepper.curIdleStep <= (idleStepper.targetIdleStep + configPage6.iacStepHyster))) //Hysteresis check
          { 
            digitalWrite(pinStepperEnable, HIGH); 
          } 
        }
      }
    }
    else
    {
      //Means we're in a step, but it doesn't need to turn off yet. No further action at this time
      isStepping = true;
    }
  }
  return isStepping;
}

/*
Performs a step
*/
static inline void doStep(void)
{
  int16_t error = idleStepper.targetIdleStep - idleStepper.curIdleStep;
  if ( (error < -((int8_t)configPage6.iacStepHyster)) || (error > configPage6.iacStepHyster) ) //Hysteresis check
  {
    // the home position for a stepper is pintle fully seated, i.e. no airflow.
    if (error < 0)
    {
      // we are moving toward the home position (reducing air)
      digitalWrite(pinStepperDir, STEPPER_LESS_AIR_DIRECTION() );
      idleStepper.curIdleStep--;
    }
    else
    {
      // we are moving away from the home position (adding air).
      digitalWrite(pinStepperDir, STEPPER_MORE_AIR_DIRECTION() );
      idleStepper.curIdleStep++;
    }

    digitalWrite(pinStepperEnable, LOW); //Enable the DRV8825
    digitalWrite(pinStepperStep, HIGH);
    idleStepper.stepStartTime = micros();
    idleStepper.stepperStatus = STEPPING;
    idleOn = true;

    currentStatus.idleOn = true;
  }
  else
  {
    currentStatus.idleOn = false;
  }
}

/*
Checks whether the stepper has been homed yet. If it hasn't, will handle the next step
Returns:
True: If the system has been homed. No other action is taken
False: If the motor has not yet been homed. Will also perform another homing step.
*/
static inline uint8_t isStepperHomed(void)
{
  bool isHomed = true; //As it's the most common scenario, default value is true
  if( completedHomeSteps < configPage6.iacStepHome )
  {
    digitalWrite(pinStepperDir, STEPPER_LESS_AIR_DIRECTION() ); //homing the stepper closes off the air bleed
    digitalWrite(pinStepperEnable, LOW); //Enable the DRV8825
    digitalWrite(pinStepperStep, HIGH);
    idleStepper.stepStartTime = micros();
    idleStepper.stepperStatus = STEPPING;
    completedHomeSteps++;
    idleOn = true;
    isHomed = false;
  }
  return isHomed;
}

void idleControl(void)
{
  if( idleInitComplete != configPage6.iacAlgorithm) { initialiseIdle(false); }
  if( (currentStatus.RPM > 0) || (configPage6.iacPWMrun == true) ) { enableIdle(); }

  //Check whether the idleUp is active
  if (configPage2.idleUpEnabled == true)
  {
    if (configPage2.idleUpPolarity == 0) { currentStatus.idleUpActive = !digitalRead(pinIdleUp); } //Normal mode (ground switched)
    else { currentStatus.idleUpActive = digitalRead(pinIdleUp); } //Inverted mode (5v activates idleUp)

    if (configPage2.idleUpOutputEnabled  == true)
    {
      if (currentStatus.idleUpActive == true)
      {
        digitalWrite(pinIdleUpOutput, idleUpOutputHIGH);
        currentStatus.idleUpOutputActive = true;
      }
      else
      {
        digitalWrite(pinIdleUpOutput, idleUpOutputLOW);
        currentStatus.idleUpOutputActive = false;
      }      
    }
  }
  else { currentStatus.idleUpActive = false; }

  //Keep the request state honest even when the selected algorithm has no
  //closed-loop running branch (engine stopped, cranking or mode changed).
  if(configPage15.iacGainAutotuneRequest == 0U) {
    iacGainTuneLastRequest = false;
    if(iacGainTuneState == IAC_GAINTUNE_RELAY) { seedIacPidFromRelay(); }
    iacGainTuneState = IAC_GAINTUNE_OFF;
    iacGainTuneStableTicks = 0U;
    iacGainTuneWaitingFanValid = false;
    updateIacGainTuneDiag();
  } else if(!isClosedLoopIac(configPage6)) {
    if(iacGainTuneState == IAC_GAINTUNE_RELAY) { seedIacPidFromRelay(); }
    iacGainTuneState = IAC_GAINTUNE_FAILED;
    iacGainTuneResult = IAC_GAINTUNE_RESULT_INVALID_MODE;
    updateIacGainTuneDiag();
  } else if((currentStatus.rotationStatus != EngineRotationStatus::Running)
         && (iacGainTuneState == IAC_GAINTUNE_RELAY)) {
    abortIacGainAutotune(IAC_GAINTUNE_RESULT_DISENGAGED);
  }

  bool PID_computed = false;
  switch(configPage6.iacAlgorithm)
  {
    case IAC_ALGORITHM_NONE:       //Case 0 is no idle control ('None')
      break;

    case IAC_ALGORITHM_ONOFF:      //Case 1 is on/off idle control
      if ( (temperatureAddOffset(currentStatus.coolant)) < configPage6.iacFastTemp) //All temps are offset by 40 degrees
      {
        idle_pin.setPinHigh();
        idleOn = true;
        currentStatus.idleOn = true;
		    currentStatus.idleLoad = 100;
      }
      else if (idleOn)
      {
        idle_pin.setPinLow();
        idleOn = false; 
        currentStatus.idleOn = false;
		    currentStatus.idleLoad = 0;
      }
      break;

    case IAC_ALGORITHM_PWM_OL:      //Case 2 is PWM open loop
      //Check for cranking pulsewidth
      if( currentStatus.rotationStatus==EngineRotationStatus::Cranking )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
        idleTaper = 0;
      }
      else if ( currentStatus.rotationStatus!=EngineRotationStatus::Running)
      {
        if( configPage6.iacPWMrun == true)
        {
          //Engine is not running or cranking, but the run before crank flag is set. Use the cranking table
          currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
          idleTaper = 0;
        }
      }
      else
      {
        if ( idleTaper < configPage2.idleTaperTime )
        {
          //Tapering between cranking IAC value and running
          currentStatus.idleLoad = map(idleTaper, 0, configPage2.idleTaperTime,\
          table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)),\
          table2D_getValue(&iacPWMTable, temperatureAddOffset(currentStatus.coolant)));
          if( BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) ) { idleTaper++; }
        }
        else
        {
          //Standard running
          currentStatus.idleLoad = table2D_getValue(&iacPWMTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
        }
        // Add air conditioning idle-up - we only do this if the engine is running (A/C should never engage with engine off).
        if(configPage15.airConIdleSteps>0 && currentStatus.airconTurningOn == true) { currentStatus.idleLoad += configPage15.airConIdleSteps; }
      }

      if(currentStatus.idleUpActive == true) { currentStatus.idleLoad += configPage2.idleUpAdder; } //Add Idle Up amount if active
      
      if( currentStatus.idleLoad > 100 ) { currentStatus.idleLoad = 100; } //Safety Check
      idle_pwm_target_value = percentage(currentStatus.idleLoad, idle_pwm_max_count);
      
      break;

    case IAC_ALGORITHM_PWM_CL:    //Case 3 is PWM closed loop
        //No cranking specific value for closed loop (yet?)
      if( currentStatus.rotationStatus==EngineRotationStatus::Cranking )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
        idle_pwm_target_value = percentage(currentStatus.idleLoad, idle_pwm_max_count);
        idle_pid_target_value = idle_pwm_target_value << 2; //Resolution increased
        idlePID.Initialize(); //Update output to smooth transition
      }
      else if ( currentStatus.rotationStatus!=EngineRotationStatus::Running)
      {
        if( configPage6.iacPWMrun == true)
        {
          //Engine is not running or cranking, but the run before crank flag is set. Use the cranking table
          currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
          idle_pwm_target_value = percentage(currentStatus.idleLoad, idle_pwm_max_count);
        }
      }
      else
      {
        idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
        if( BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ) ) { idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD); } //Re-read the PID settings once per second
        
        long relayOutput = 0L;
        if(updateIacGainAutotune(idle_pid_target_value, true, relayOutput)) {
          idle_pid_target_value = relayOutput;
          PID_computed = true;
        } else {
          PID_computed = idlePID.Compute();
        }
        long TEMP_idle_pwm_target_value;
        if(PID_computed == true)
        {
          TEMP_idle_pwm_target_value = idle_pid_target_value;
          
          // Add an offset to the duty cycle, outside of the closed loop. When tuned correctly, the extra load from
          // the air conditioning should exactly cancel this out and the PID loop will be relatively unaffected.
          if(configPage15.airConIdleSteps>0 && currentStatus.airconTurningOn == true)
          {
            // Add air conditioning idle-up
            // We are adding percentage steps, but the loop doesn't operate in percentage steps - it works in PWM count
            TEMP_idle_pwm_target_value += percentage(configPage15.airConIdleSteps, idle_pwm_max_count<<2);
            if(TEMP_idle_pwm_target_value > (idle_pwm_max_count<<2)) { TEMP_idle_pwm_target_value = (idle_pwm_max_count<<2); }
          }

          // Fixed this by putting it here, however I have not tested it. It used to be after the calculation of idle_pwm_target_value, meaning the percentage would update in currentStatus, but the idle would not actually increase.
          if(currentStatus.idleUpActive == true)
          { 
            // Add Idle Up amount if active
            // Again, we use configPage15.airConIdleSteps * idle_pwm_max_count / 100 because we are adding percentage steps, but the loop doesn't operate in percentage steps - it works in PWM count
            TEMP_idle_pwm_target_value += percentage(configPage2.idleUpAdder, idle_pwm_max_count<<2);
            if(TEMP_idle_pwm_target_value > (idle_pwm_max_count<<2)) { TEMP_idle_pwm_target_value = (idle_pwm_max_count<<2); }
          }

          // Now assign the real PWM value
          idle_pwm_target_value = TEMP_idle_pwm_target_value>>2; //increased resolution
          currentStatus.idleLoad = (uint16_t)(((uint32_t)(idle_pwm_target_value * 100UL)) / idle_pwm_max_count);
        }
        idleCounter++;
      }
      break;


    case IAC_ALGORITHM_PWM_OLCL: //case 6 is PWM Open Loop table as feedforward term plus closed loop. 
      //No cranking specific value for closed loop (yet?)
      if( currentStatus.rotationStatus==EngineRotationStatus::Cranking )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
        idle_pwm_target_value = percentage(currentStatus.idleLoad, idle_pwm_max_count);
        idle_pid_target_value = idle_pwm_target_value << 2; //Resolution increased
        idlePID.Initialize(); //Update output to smooth transition
      }
      else if ( currentStatus.rotationStatus!=EngineRotationStatus::Running)
      {
        if( configPage6.iacPWMrun == true)
        {
          //Engine is not running or cranking, but the run before crank flag is set. Use the cranking table
          currentStatus.idleLoad = table2D_getValue(&iacCrankDutyTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
          idle_pwm_target_value = percentage(currentStatus.idleLoad, idle_pwm_max_count);
        }
      }
      else
      {
        //Read the OL table as feedforward term
        FeedForwardTerm = percentage(table2D_getValue(&iacPWMTable, temperatureAddOffset(currentStatus.coolant)), idle_pwm_max_count<<2); //All temps are offset by 40 degrees
        
        // Add an offset to the feed forward term. When tuned correctly, the extra load from the air conditioning
        // should exactly cancel this out and the PID loop will be relatively unaffected.
        if(configPage15.airConIdleSteps>0 && currentStatus.airconTurningOn == true)
        {
          // Add air conditioning idle-up
          // We are adding percentage steps, but the loop doesn't operate in percentage steps - it works in PWM count <<2 (PWM count * 4)
          FeedForwardTerm += percentage(configPage15.airConIdleSteps, (idle_pwm_max_count<<2));
          if(FeedForwardTerm > (idle_pwm_max_count<<2)) { FeedForwardTerm = (idle_pwm_max_count<<2); }
        }
        
        // Fixed this by putting it here, however I have not tested it. It used to be after the calculation of idle_pwm_target_value, meaning the percentage would update in currentStatus, but the idle would not actually increase.
        if(currentStatus.idleUpActive == true)
        { 
          // Add Idle Up amount if active
          // Again, we are adding percentage steps, but the loop doesn't operate in percentage steps - it works in PWM count <<2 (PWM count * 4)
          FeedForwardTerm += percentage(configPage2.idleUpAdder, (idle_pwm_max_count<<2));
          if(FeedForwardTerm > (idle_pwm_max_count<<2)) { FeedForwardTerm = (idle_pwm_max_count<<2); }
        }
        
    
        idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
        if( BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ) ) { idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD); } //Re-read the PID settings once per second
        if((currentStatus.RPM - idle_cl_target_rpm > configPage2.iacRPMlimitHysteresis*10) || (currentStatus.TPS > configPage2.iacTPSlimit)){ //reset integral to zero when TPS is bigger than set value in TS (opening throttle so not idle anymore). OR when RPM higher than Idle Target + RPM Histeresis (coming back from high rpm with throttle closed)
          idlePID.ResetIntegeral();
        }
        
        long relayOutput = 0L;
        if(updateIacGainAutotune(idle_pid_target_value, true, relayOutput)) {
          idle_pid_target_value = relayOutput;
          PID_computed = true;
        } else {
          PID_computed = idlePID.Compute(true, FeedForwardTerm);
        }

        if(PID_computed == true)
        {
          idle_pwm_target_value = idle_pid_target_value>>2; //increased resolution
          currentStatus.idleLoad = ((unsigned long)(idle_pwm_target_value * 100UL) / idle_pwm_max_count);
        }
        idleCounter++;
      }
        
    break;


    case IAC_ALGORITHM_STEP_OL:    //Case 4 is open loop stepper control
      //First thing to check is whether there is currently a step going on and if so, whether it needs to be turned off
      if( (checkForStepping() == false) && (isStepperHomed() == true) ) //Check that homing is complete and that there's not currently a step already taking place. MUST BE IN THIS ORDER!
      {
        //Check for cranking pulsewidth
        if( currentStatus.rotationStatus!=EngineRotationStatus::Running ) //If ain't running it means off or cranking
        {
          //Currently cranking. Use the cranking table
          idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
          if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active
          idleTaper = 0;
        }
        else
        {
          //Standard running
          if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) && (currentStatus.RPM > 0))
          {
            if ( idleTaper < configPage2.idleTaperTime )
            {
              //Tapering between cranking IAC value and running
              idleStepper.targetIdleStep = map(idleTaper, 0, configPage2.idleTaperTime,\
              table2D_getValue(&iacCrankStepsTable, temperatureAddOffset(currentStatus.coolant)),\
              table2D_getValue(&iacStepTable, temperatureAddOffset(currentStatus.coolant)));
              if( BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) ) { idleTaper++; }
            }
            else
            {
              //Standard running
              idleStepper.targetIdleStep = table2D_getValue(&iacStepTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
            }
            if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active
            
            // Add air conditioning idle-up - we only do this if the engine is running (A/C should never engage with engine off).
            if(configPage15.airConIdleSteps>0 && currentStatus.airconTurningOn == true) { idleStepper.targetIdleStep += configPage15.airConIdleSteps; }
            
            iacStepTime_uS = configPage6.iacStepTime * 1000;
            iacCoolTime_uS = configPage9.iacCoolTime * 1000;
          }
        }
        //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
        if (idleStepper.targetIdleStep > configPage9.iacMaxSteps )
        {
          idleStepper.targetIdleStep = configPage9.iacMaxSteps;
        }
        currentStatus.idleLoad = idleStepper.curIdleStep;
        doStep();
      }
      break;

    case IAC_ALGORITHM_STEP_OLCL:  //Case 7 is closed+open loop stepper control
    case IAC_ALGORITHM_STEP_CL:    //Case 5 is closed loop stepper control
      //First thing to check is whether there is currently a step going on and if so, whether it needs to be turned off
      if( (checkForStepping() == false) && (isStepperHomed() == true) ) //Check that homing is complete and that there's not currently a step already taking place. MUST BE IN THIS ORDER!
      {
        if( currentStatus.rotationStatus!=EngineRotationStatus::Running ) //If ain't running it means off or cranking
        {
          //Currently cranking. Use the cranking table
          idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
          if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active

          //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
          if (idleStepper.targetIdleStep > configPage9.iacMaxSteps )
          {
            idleStepper.targetIdleStep = configPage9.iacMaxSteps;
          }
          
          idleTaper = 0;
          idle_pid_target_value = idleStepper.targetIdleStep << 2; //Resolution increased
          idlePID.ResetIntegeral();
          FeedForwardTerm = idle_pid_target_value;
        }
        else 
        {
          if( BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) )
          {
            idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
            if( idleTaper < configPage2.idleTaperTime )
            {
              uint16_t minValue = table2D_getValue(&iacCrankStepsTable, temperatureAddOffset(currentStatus.coolant));
              if( idle_pid_target_value < minValue<<2 ) { idle_pid_target_value = minValue<<2; }
              uint16_t maxValue = idle_pid_target_value>>2;
              if( configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL ) { maxValue = table2D_getValue(&iacStepTable, temperatureAddOffset(currentStatus.coolant)); }

              //Tapering between cranking IAC value and running
              FeedForwardTerm = map(idleTaper, 0, configPage2.idleTaperTime, minValue, maxValue)<<2;
              idleTaper++;
              idle_pid_target_value = FeedForwardTerm;
            }
            else if (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL)
            {
              //Standard running
              FeedForwardTerm = table2D_getValue(&iacStepTable, temperatureAddOffset(currentStatus.coolant))<<2; //All temps are offset by 40 degrees
              //reset integral to zero when TPS is bigger than set value in TS (opening throttle so not idle anymore). OR when RPM higher than Idle Target + RPM Hysteresis (coming back from high rpm with throttle closed) 
              if (((currentStatus.RPM - idle_cl_target_rpm) > configPage2.iacRPMlimitHysteresis*10) || (currentStatus.TPS > configPage2.iacTPSlimit) || lastDFCOValue )
              {
                idlePID.ResetIntegeral();
              }
            }
            else { FeedForwardTerm = idle_pid_target_value; }
          }

          long relayOutput = 0L;
          const int16_t stepError = (int16_t)(idleStepper.targetIdleStep - idleStepper.curIdleStep);
          const bool actuatorSettled = (stepError >= -((int16_t)configPage6.iacStepHyster))
                                    && (stepError <= (int16_t)configPage6.iacStepHyster);
          if(updateIacGainAutotune(idle_pid_target_value, actuatorSettled, relayOutput)) {
            idle_pid_target_value = relayOutput;
            PID_computed = true;
          } else {
            PID_computed = idlePID.Compute(true, FeedForwardTerm);
          }

          //If DFCO conditions are met keep output from changing
          if( (currentStatus.TPS > configPage2.iacTPSlimit) || lastDFCOValue
          || ((configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) && (idleTaper < configPage2.idleTaperTime)) )
          {
            idle_pid_target_value = FeedForwardTerm;
          }
          idleStepper.targetIdleStep = idle_pid_target_value>>2; //Increase resolution

          // Add air conditioning idle-up - we only do this if the engine is running (A/C should never engage with engine off).
          if(configPage15.airConIdleSteps>0 && currentStatus.airconTurningOn == true) { idleStepper.targetIdleStep += configPage15.airConIdleSteps; }
        }
        
        if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active
        
        //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
        if (idleStepper.targetIdleStep > configPage9.iacMaxSteps )
        {
          idleStepper.targetIdleStep = configPage9.iacMaxSteps;
        }
        currentStatus.idleLoad = idleStepper.curIdleStep;
        doStep();
      }
      if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ)) //Use timer flag instead idle count
      {
        //This only needs to be run very infrequently, once per second
        idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
        iacStepTime_uS = configPage6.iacStepTime * 1000;
        iacCoolTime_uS = configPage9.iacCoolTime * 1000;
      }
      break;

    default:
      //There really should be a valid idle type
      break;
  }
  lastDFCOValue = currentStatus.isDFCOActive;

  //Check for 100% and 0% DC on PWM idle
  if (isPwmIac(configPage6))
  {
    if(currentStatus.idleLoad >= 100)
    {
      currentStatus.idleOn = true;
      IDLE_TIMER_DISABLE();
      if (configPage6.iacPWMdir == 0)
      {
        //Normal direction
        idle_pin.setPinHigh();  // Switch pin high
        if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      }
      else
      {
        //Reversed direction
        idle_pin.setPinLow();  // Switch pin to low
        if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      }
    }
    else if (currentStatus.idleLoad == 0)
    {
      disableIdle();
    }
    else
    {
      currentStatus.idleOn = true;
      IDLE_TIMER_ENABLE();
    }
  }
}


//This function simply turns off the idle PWM and sets the pin low
void disableIdle(void)
{
  if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) )
  {
    IDLE_TIMER_DISABLE();
    if (configPage6.iacPWMdir == 0)
    {
      //Normal direction
      idle_pin.setPinLow();  // Switch pin to low
      if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
    else
    {
      //Reversed direction
      idle_pin.setPinHigh();  // Switch pin high
      if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
  }
  else if( isStepperIac(configPage6) )
  {
    //Only disable the stepper motor if homing is completed
    if( (checkForStepping() == false) && (isStepperHomed() == true) )
    {
        /* for open loop stepper we should just move to the cranking position when
           disabling idle, since the only time this function is called in this scenario
           is if the engine stops.
        */
        idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, temperatureAddOffset(currentStatus.coolant)); //All temps are offset by 40 degrees
        if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active?

        //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
        if (idleStepper.targetIdleStep > configPage9.iacMaxSteps )
        {
          idleStepper.targetIdleStep = configPage9.iacMaxSteps;
        }
        idle_pid_target_value = idleStepper.targetIdleStep<<2;
    }
  }
  currentStatus.idleOn = false;
  currentStatus.idleLoad = 0;
}

void idleInterrupt(void)
{
  if (idle_pwm_state)
  {
    if (configPage6.iacPWMdir == 0)
    {
      //Normal direction
      #if defined (CORE_TEENSY41) //PIT TIMERS count down and have opposite effect on PWM
      idle_pin.setPinHigh();
      if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); }
      #else
      idle_pin.setPinLow();  // Switch pin to low (1 pin mode)
      if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      #endif
    }
    else
    {
      //Reversed direction
      #if defined (CORE_TEENSY41) //PIT TIMERS count down and have opposite effect on PWM
      idle_pin.setPinLow();
      if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); }
      #else
      idle_pin.setPinHigh();  // Switch pin high
      if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      #endif
    }
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + (idle_pwm_max_count - idle_pwm_cur_value) );
    idle_pwm_state = false;
  }
  else
  {
    if (configPage6.iacPWMdir == 0)
    {
      //Normal direction
      #if defined (CORE_TEENSY41) //PIT TIMERS count down and have opposite effect on PWM
      idle_pin.setPinLow();
      if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); }
      #else
      idle_pin.setPinHigh();  // Switch pin high
      if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      #endif
    }
    else
    {
      //Reversed direction
      #if defined (CORE_TEENSY41) //PIT TIMERS count down and have opposite effect on PWM
      idle_pin.setPinHigh();
      if(configPage6.iacChannels == 1) { idle2_pin.setPinLow(); }
      #else
      idle_pin.setPinLow();  // Switch pin to low (1 pin mode)
      if(configPage6.iacChannels == 1) { idle2_pin.setPinHigh(); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
      #endif
    }
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + idle_pwm_target_value);
    idle_pwm_cur_value = idle_pwm_target_value;
    idle_pwm_state = true;
  }
}
