#include "emp_pump.h"

#include <limits.h>

namespace emp_pump
{
namespace
{

constexpr uint32_t COMMAND_BASE_ID = 0x18EF0000UL;
constexpr uint32_t STATUS_1_BASE_ID = 0x18FF0300UL;
constexpr uint32_t STATUS_2_BASE_ID = 0x18FF2300UL;
constexpr uint32_t STATUS_3_BASE_ID = 0x18FF2400UL;
constexpr uint32_t EXTERNAL_TEMPERATURE_BASE_ID = 0x18FF4300UL;
constexpr uint32_t ADDRESS_CLAIM_BASE_ID = 0x18EEFF00UL;

constexpr uint8_t CONTROL_OFF = 0xFCU;
constexpr uint8_t CONTROL_OFF_POWER_HOLD_OFF = 0xF0U;
constexpr uint8_t CONTROL_FORWARD = 0xFDU;
constexpr uint8_t CONTROL_FORWARD_POWER_HOLD_OFF = 0xF1U;
constexpr uint8_t CONTROL_FORWARD_POWER_HOLD_ON = 0xF5U;

State state = State::Disabled;
State previousState = State::Disabled;
ThermalState thermalState = ThermalState::Inactive;
uint16_t targetRpm = 0U;
uint16_t requestedRpm = 0U;
uint16_t actualRpm = 0U;
uint8_t actualPercentRaw = 0U;
uint8_t controllerStatus = 0U;
uint8_t statusSummary = 0U;
uint16_t voltageRaw = 0U;
uint16_t currentRaw = 32000U; //Raw zero for the protocol's -1600 A offset
uint16_t powerWatts = 0U;
uint16_t externalTemperatureRaw = 8736U; //Raw zero Celsius for the protocol's -273 C offset
uint8_t capabilities = 0U;
uint16_t latchedFaults = 0U;
uint16_t diagnosticFaults = 0U;
uint32_t lastMainStatusMs = 0U;
uint32_t lastUpdateMs = 0U;
uint32_t lastTransmitMs = 0U;
uint32_t stateSinceMs = 0U;
uint32_t stopCandidateMs = 0U;
uint32_t afterRunEndMs = 0U;
uint32_t serviceEndMs = 0U;
uint16_t serviceRpm = 0U;
uint8_t transmitFailureCount = 0U;
uint8_t lastCommandControl = CONTROL_OFF;
bool mainStatusSeen = false;
bool transmitSeen = false;
bool engineSeen = false;
bool hotBootEvaluated = false;
bool batteryInhibit = false;
bool commandDirty = true;
bool powerHoldWanted = false;
bool powerHoldCommanded = false;
bool filteredIatSeen = false;
bool slopeSampleSeen = false;
uint8_t controlFlags = 0U;
uint16_t controlFaults = 0U;
int32_t filteredIatX8 = 0;
int32_t integralMilliRpm = 0;
int16_t coolantSlopePerMinute = 0;
int16_t diagnosticFeedForwardRpm = 0;
int16_t diagnosticPiCorrectionRpm = 0;
int16_t diagnosticFilteredIat = 0;
int16_t diagnosticTargetTemperature = 0;
int8_t diagnosticTemperatureError = 0;
uint16_t diagnosticMinimumFlowRpm = 0U;
uint8_t coolingDemandRaw = 0U;
uint32_t lastSlopeSampleMs = 0U;
uint32_t saturationSinceMs = 0U;
int16_t lastSlopeCoolant = 0;

constexpr uint8_t CONTROL_FLAG_CLOSED_LOOP = 1U << 0;
constexpr uint8_t CONTROL_FLAG_AT_MINIMUM = 1U << 1;
constexpr uint8_t CONTROL_FLAG_AT_MAXIMUM = 1U << 2;
constexpr uint8_t CONTROL_FLAG_IAT_VALID = 1U << 3;
constexpr uint8_t CONTROL_FLAG_VSS_VALID = 1U << 4;
constexpr uint8_t CONTROL_FLAG_FAN_ON = 1U << 5;
constexpr uint8_t CONTROL_FLAG_POSITIVE_SLOPE = 1U << 6;
constexpr uint8_t CONTROL_FLAG_AIRFLOW_AT_CAPACITY = 1U << 7;

uint16_t clampRpm(uint16_t rpm, const Config &config)
{
  if (rpm == 0U) { return 0U; }
  if (rpm < config.minimumRunRpm) { rpm = config.minimumRunRpm; }
  if (rpm > config.maximumRpm) { rpm = config.maximumRpm; }
  return rpm;
}

uint16_t curveRpm(int16_t coolant, const Config &config)
{
  if (coolant <= config.temperatureBins[0]) { return config.rpmBins[0]; }
  for (uint8_t index = 1U; index < 6U; index++)
  {
    if (coolant <= config.temperatureBins[index])
    {
      const int16_t x0 = config.temperatureBins[index - 1U];
      const int16_t x1 = config.temperatureBins[index];
      const uint16_t y0 = config.rpmBins[index - 1U];
      const uint16_t y1 = config.rpmBins[index];
      if (x1 <= x0) { return y1; }
      const int32_t numerator = static_cast<int32_t>(coolant - x0) *
                                (static_cast<int32_t>(y1) - static_cast<int32_t>(y0));
      const int32_t result = static_cast<int32_t>(y0) + (numerator / (x1 - x0));
      return (result <= 0) ? 0U : static_cast<uint16_t>(result);
    }
  }
  return config.rpmBins[5];
}

int16_t clampInt16(int32_t value)
{
  if (value > INT16_MAX) { return INT16_MAX; }
  if (value < INT16_MIN) { return INT16_MIN; }
  return static_cast<int16_t>(value);
}

int8_t clampInt8(int16_t value)
{
  if (value > INT8_MAX) { return INT8_MAX; }
  if (value < INT8_MIN) { return INT8_MIN; }
  return static_cast<int8_t>(value);
}

uint16_t minimumFlowRpm(uint16_t engineRpm, const Config &config)
{
  uint16_t result = config.minimumFlowRpmBins[3];
  if (engineRpm <= config.engineRpmBins[0])
  {
    result = config.minimumFlowRpmBins[0];
  }
  else
  {
    for (uint8_t index = 1U; index < 4U; index++)
    {
      if (engineRpm <= config.engineRpmBins[index])
      {
        const uint16_t x0 = config.engineRpmBins[index - 1U];
        const uint16_t x1 = config.engineRpmBins[index];
        const uint16_t y0 = config.minimumFlowRpmBins[index - 1U];
        const uint16_t y1 = config.minimumFlowRpmBins[index];
        const int32_t numerator = static_cast<int32_t>(engineRpm - x0) *
                                  (static_cast<int32_t>(y1) - static_cast<int32_t>(y0));
        result = static_cast<uint16_t>(static_cast<int32_t>(y0) +
                                      (numerator / static_cast<int32_t>(x1 - x0)));
        break;
      }
    }
  }

  if (result < config.minimumRunRpm) { result = config.minimumRunRpm; }
  if (result > config.maximumRpm) { result = config.maximumRpm; }
  return result;
}

void resetClosedLoopRuntime()
{
  integralMilliRpm = 0;
  coolantSlopePerMinute = 0;
  diagnosticFeedForwardRpm = 0;
  diagnosticPiCorrectionRpm = 0;
  diagnosticTemperatureError = 0;
  diagnosticMinimumFlowRpm = 0U;
  coolingDemandRaw = 0U;
  slopeSampleSeen = false;
  saturationSinceMs = 0U;
  controlFlags = 0U;
  controlFaults = 0U;
}

void updateFilteredIat(const Inputs &inputs)
{
  if (!inputs.intakeAirTemperatureValid) { return; }

  const int32_t sampleX8 = static_cast<int32_t>(inputs.intakeAirTemperature) * 8;
  if (!filteredIatSeen)
  {
    filteredIatX8 = sampleX8;
    filteredIatSeen = true;
  }
  else
  {
    filteredIatX8 += (sampleX8 - filteredIatX8) / 8;
  }
  diagnosticFilteredIat = static_cast<int16_t>(filteredIatX8 / 8);
}

void updateCoolantSlope(uint32_t nowMs, int16_t coolant)
{
  if (!slopeSampleSeen)
  {
    slopeSampleSeen = true;
    lastSlopeSampleMs = nowMs;
    lastSlopeCoolant = coolant;
    coolantSlopePerMinute = 0;
    return;
  }

  const uint32_t elapsedMs = nowMs - lastSlopeSampleMs;
  if (elapsedMs < 1000U) { return; }

  int32_t rawSlope = (static_cast<int32_t>(coolant) - static_cast<int32_t>(lastSlopeCoolant)) *
                     60000L / static_cast<int32_t>(elapsedMs);
  if (rawSlope > 600L) { rawSlope = 600L; }
  if (rawSlope < -600L) { rawSlope = -600L; }
  coolantSlopePerMinute = static_cast<int16_t>(
      ((static_cast<int32_t>(coolantSlopePerMinute) * 3L) + rawSlope) / 4L);
  lastSlopeSampleMs = nowMs;
  lastSlopeCoolant = coolant;
}

int16_t effectiveTemperatureError(int16_t coolant, const Config &config)
{
  const int16_t error = static_cast<int16_t>(coolant - config.targetTemperature);
  const int16_t deadband = static_cast<int16_t>(config.temperatureDeadband);
  if (error > deadband) { return static_cast<int16_t>(error - deadband); }
  if (error < -deadband) { return static_cast<int16_t>(error + deadband); }
  return 0;
}

uint16_t closedLoopRpm(uint32_t nowMs, const Config &config, const Inputs &inputs)
{
  updateFilteredIat(inputs);
  updateCoolantSlope(nowMs, inputs.coolant);

  controlFaults &= static_cast<uint16_t>(~(FAULT_IAT_INVALID |
                                           FAULT_COOLING_LIMITED |
                                           FAULT_THERMAL_OVERLOAD));
  controlFlags = CONTROL_FLAG_CLOSED_LOOP;
  if (inputs.intakeAirTemperatureValid) { controlFlags |= CONTROL_FLAG_IAT_VALID; }
  else { controlFaults |= FAULT_IAT_INVALID; }
  if (inputs.vehicleSpeedValid) { controlFlags |= CONTROL_FLAG_VSS_VALID; }
  if (inputs.fanOn) { controlFlags |= CONTROL_FLAG_FAN_ON; }
  if (coolantSlopePerMinute > 0) { controlFlags |= CONTROL_FLAG_POSITIVE_SLOPE; }
  if (inputs.airflowAtMaximumCapacity)
  {
    controlFlags |= CONTROL_FLAG_AIRFLOW_AT_CAPACITY;
  }

  const uint16_t minimumRpm = minimumFlowRpm(inputs.engineRpm, config);
  diagnosticMinimumFlowRpm = minimumRpm;
  diagnosticTemperatureError = clampInt8(
      static_cast<int16_t>(inputs.coolant - config.targetTemperature));

  const uint16_t boundedMap = (inputs.manifoldPressure > 250U) ? 250U : inputs.manifoldPressure;
  const uint16_t boundedEngineRpm = (inputs.engineRpm > 20000U) ? 20000U : inputs.engineRpm;
  int32_t feedForward = (static_cast<uint32_t>(boundedEngineRpm) * boundedMap / 100U) *
                        config.loadFeedForwardGain / 1000U;

  if (inputs.intakeAirTemperatureValid)
  {
    feedForward += static_cast<int32_t>(diagnosticFilteredIat -
                                       config.iatReferenceTemperature) *
                   config.iatCompensationGain;
  }

  uint16_t effectiveAirflowKph = 0U;
  if (inputs.vehicleSpeedValid) { effectiveAirflowKph = inputs.vehicleSpeedKph; }
  if (inputs.fanOn)
  {
    const uint32_t withFan = static_cast<uint32_t>(effectiveAirflowKph) +
                             config.fanEquivalentSpeedKph;
    effectiveAirflowKph = (withFan > UINT16_MAX) ? UINT16_MAX :
                          static_cast<uint16_t>(withFan);
  }
  if ((config.airflowFullSpeedKph > 0U) && (effectiveAirflowKph > 0U))
  {
    if (effectiveAirflowKph > config.airflowFullSpeedKph)
    {
      effectiveAirflowKph = config.airflowFullSpeedKph;
    }
    feedForward -= static_cast<uint32_t>(effectiveAirflowKph) *
                   config.airflowReliefRpm / config.airflowFullSpeedKph;
  }

  if (coolantSlopePerMinute > 0)
  {
    feedForward += static_cast<int32_t>(coolantSlopePerMinute) *
                   config.derivativeGain;
  }
  diagnosticFeedForwardRpm = clampInt16(feedForward);

  const int16_t error = effectiveTemperatureError(inputs.coolant, config);
  const uint32_t elapsedMsRaw = nowMs - lastUpdateMs;
  const uint32_t elapsedMs = (elapsedMsRaw > 1000U) ? 1000U : elapsedMsRaw;
  int32_t candidateIntegral = integralMilliRpm +
      (static_cast<int32_t>(error) * config.integralGain * static_cast<int32_t>(elapsedMs));
  const int32_t integralLimit = static_cast<int32_t>(config.integralLimitRpm) * 1000L;
  if (candidateIntegral > integralLimit) { candidateIntegral = integralLimit; }
  if (candidateIntegral < -integralLimit) { candidateIntegral = -integralLimit; }

  const int32_t proportional = static_cast<int32_t>(error) * config.proportionalGain;
  int32_t piCorrection = proportional + (candidateIntegral / 1000L);
  int32_t rawRequest = static_cast<int32_t>(minimumRpm) + feedForward + piCorrection;

  const bool blocksPositiveIntegration = (rawRequest > config.maximumRpm) && (error > 0);
  const bool blocksNegativeIntegration = (rawRequest < minimumRpm) && (error < 0);
  if (blocksPositiveIntegration || blocksNegativeIntegration)
  {
    piCorrection = proportional + (integralMilliRpm / 1000L);
    rawRequest = static_cast<int32_t>(minimumRpm) + feedForward + piCorrection;
  }
  else
  {
    integralMilliRpm = candidateIntegral;
  }
  diagnosticPiCorrectionRpm = clampInt16(piCorrection);

  uint16_t result;
  if (rawRequest <= minimumRpm)
  {
    result = minimumRpm;
    controlFlags |= CONTROL_FLAG_AT_MINIMUM;
  }
  else if (rawRequest >= config.maximumRpm)
  {
    result = config.maximumRpm;
    controlFlags |= CONTROL_FLAG_AT_MAXIMUM;
  }
  else
  {
    result = static_cast<uint16_t>(rawRequest);
  }

  if (config.maximumRpm > minimumRpm)
  {
    const uint32_t demand = static_cast<uint32_t>(result - minimumRpm) * 200U /
                            static_cast<uint32_t>(config.maximumRpm - minimumRpm);
    coolingDemandRaw = (demand > 200U) ? 200U : static_cast<uint8_t>(demand);
  }
  else
  {
    coolingDemandRaw = 200U;
  }

  const int16_t actualError = static_cast<int16_t>(inputs.coolant -
                                                   config.targetTemperature);
  const bool ramAirAtCapacity = inputs.vehicleSpeedValid &&
                                (inputs.vehicleSpeedKph >= config.airflowFullSpeedKph);
  const bool limited = (result == config.maximumRpm) &&
                       (inputs.airflowAtMaximumCapacity || ramAirAtCapacity) &&
                       (actualError >= config.coolingLimitedDelta);
  if (limited)
  {
    controlFaults |= FAULT_COOLING_LIMITED;
    if (saturationSinceMs == 0U) { saturationSinceMs = nowMs; }
    const bool overloadDelayReached =
        (nowMs - saturationSinceMs) >=
        (static_cast<uint32_t>(config.overloadDelaySeconds) * 1000U);
    if ((actualError >= config.overloadDelta) && overloadDelayReached)
    {
      thermalState = ThermalState::Overload;
      controlFaults |= FAULT_THERMAL_OVERLOAD;
    }
    else
    {
      thermalState = ThermalState::CapacityLimited;
    }
  }
  else
  {
    saturationSinceMs = 0U;
    controlFaults &= static_cast<uint16_t>(~(FAULT_COOLING_LIMITED |
                                             FAULT_THERMAL_OVERLOAD));
    thermalState = ((inputs.coolant <
                    (config.targetTemperature - config.temperatureDeadband)) &&
                    (result == minimumRpm)) ?
                   ThermalState::Warmup : ThermalState::ClosedLoop;
  }

  return result;
}

bool isTimeReached(uint32_t nowMs, uint32_t deadlineMs)
{
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

void setState(State newState, uint32_t nowMs)
{
  if (state != newState)
  {
    previousState = state;
    state = newState;
    stateSinceMs = nowMs;
    commandDirty = true;
  }
}

uint16_t saturatedAge(uint32_t nowMs, uint32_t timestamp)
{
  const uint32_t age = nowMs - timestamp;
  return (age > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(age);
}

void applyRamp(uint32_t nowMs, const Config &config)
{
  if (requestedRpm == 0U)
  {
    if (targetRpm != 0U)
    {
      targetRpm = 0U;
      commandDirty = true;
    }
    return;
  }

  const uint16_t desired = clampRpm(requestedRpm, config);
  if ((targetRpm == 0U) || (config.rampRpmPerSecond == 0U))
  {
    if (targetRpm != desired)
    {
      targetRpm = desired;
      commandDirty = true;
    }
    return;
  }

  const uint32_t elapsedMs = nowMs - lastUpdateMs;
  uint32_t step = (static_cast<uint32_t>(config.rampRpmPerSecond) * elapsedMs) / 1000U;
  if ((elapsedMs > 0U) && (step == 0U)) { step = 1U; }

  uint16_t next = targetRpm;
  if (next < desired)
  {
    const uint32_t increased = static_cast<uint32_t>(next) + step;
    next = (increased >= desired) ? desired : static_cast<uint16_t>(increased);
  }
  else if (next > desired)
  {
    next = (static_cast<uint32_t>(next - desired) <= step) ?
           desired : static_cast<uint16_t>(next - step);
  }

  next = clampRpm(next, config);
  if (next != targetRpm)
  {
    targetRpm = next;
    commandDirty = true;
  }
}

uint8_t desiredControl(const Config &config)
{
  if (targetRpm > 0U)
  {
    if (powerHoldWanted && ((config.flags & POWER_HOLD_ENABLED) != 0U)) { return CONTROL_FORWARD_POWER_HOLD_ON; }
    if (powerHoldCommanded) { return CONTROL_FORWARD_POWER_HOLD_OFF; }
    return CONTROL_FORWARD;
  }

  if (powerHoldCommanded) { return CONTROL_OFF_POWER_HOLD_OFF; }
  return CONTROL_OFF;
}

void updatePowerHoldRequest(const Config &config, const Inputs &inputs)
{
  const bool configured = ((config.flags & AFTER_RUN_ENABLED) != 0U) &&
                          ((config.flags & POWER_HOLD_ENABLED) != 0U);
  const bool armWhileRunning = (state == State::EngineActive) && inputs.coolantValid &&
                               (inputs.coolant >= config.afterRunStartTemperature);
  const bool keepDuringAfterRun = (state == State::AfterRun) && !batteryInhibit &&
                                  (requestedRpm > 0U);
  const bool wanted = configured && (armWhileRunning || keepDuringAfterRun);
  if (wanted != powerHoldWanted)
  {
    powerHoldWanted = wanted;
    commandDirty = true;
  }
}

} // namespace

void reset(uint32_t nowMs)
{
  state = State::Disabled;
  previousState = State::Disabled;
  thermalState = ThermalState::Inactive;
  targetRpm = 0U;
  requestedRpm = 0U;
  actualRpm = 0U;
  actualPercentRaw = 0U;
  controllerStatus = 0U;
  statusSummary = 0U;
  voltageRaw = 0U;
  currentRaw = 32000U;
  powerWatts = 0U;
  externalTemperatureRaw = 8736U;
  capabilities = 0U;
  latchedFaults = 0U;
  diagnosticFaults = 0U;
  lastMainStatusMs = 0U;
  lastUpdateMs = nowMs;
  lastTransmitMs = 0U;
  stateSinceMs = nowMs;
  stopCandidateMs = 0U;
  afterRunEndMs = 0U;
  serviceEndMs = 0U;
  serviceRpm = 0U;
  transmitFailureCount = 0U;
  lastCommandControl = CONTROL_OFF;
  mainStatusSeen = false;
  transmitSeen = false;
  engineSeen = false;
  hotBootEvaluated = false;
  batteryInhibit = false;
  commandDirty = true;
  powerHoldWanted = false;
  powerHoldCommanded = false;
  filteredIatSeen = false;
  filteredIatX8 = 0;
  diagnosticFilteredIat = 0;
  diagnosticTargetTemperature = 0;
  resetClosedLoopRuntime();
}

void update(uint32_t nowMs, const Config &config, const Inputs &inputs)
{
  diagnosticTargetTemperature = config.targetTemperature;
  const bool enabled = config.valid && ((config.flags & ENABLED) != 0U);
  const bool engineActive = inputs.engineRunning ||
                            (inputs.engineCranking && ((config.flags & RUN_DURING_CRANKING) != 0U));

  if (!config.valid) { latchedFaults |= FAULT_CONFIG; }
  if (enabled && !inputs.coolantValid &&
      (engineActive || (state == State::AfterRun) || (state == State::ServiceTest)))
  {
    latchedFaults |= FAULT_CLT_INVALID;
  }

  if (!enabled)
  {
    requestedRpm = 0U;
    serviceEndMs = 0U;
    afterRunEndMs = 0U;
    setState(State::Disabled, nowMs);
    thermalState = ThermalState::Inactive;
    resetClosedLoopRuntime();
    updatePowerHoldRequest(config, inputs);
    applyRamp(nowMs, config);
    lastUpdateMs = nowMs;
    return;
  }

  if ((serviceEndMs != 0U) && !isTimeReached(nowMs, serviceEndMs))
  {
    setState(State::ServiceTest, nowMs);
    requestedRpm = serviceRpm;
    thermalState = ThermalState::Service;
    resetClosedLoopRuntime();
  }
  else if (engineActive)
  {
    engineSeen = true;
    stopCandidateMs = 0U;
    afterRunEndMs = 0U;
    batteryInhibit = false;
    setState(State::EngineActive, nowMs);
    if (!inputs.coolantValid)
    {
      requestedRpm = config.failsafeRpm;
      thermalState = ThermalState::Failsafe;
      resetClosedLoopRuntime();
    }
    else if ((config.flags & CLOSED_LOOP_ENABLED) != 0U)
    {
      requestedRpm = closedLoopRpm(nowMs, config, inputs);
    }
    else
    {
      requestedRpm = curveRpm(inputs.coolant, config);
      thermalState = ThermalState::ClosedLoop;
      resetClosedLoopRuntime();
    }
  }
  else
  {
    serviceEndMs = 0U;

    if (!hotBootEvaluated)
    {
      hotBootEvaluated = true;
      if (((config.flags & HOT_BOOT_RECOVERY) != 0U) && inputs.coolantValid &&
          (inputs.coolant >= config.afterRunStartTemperature))
      {
        engineSeen = true;
        afterRunEndMs = nowMs + (static_cast<uint32_t>(config.afterRunMaximumSeconds) * 1000U);
      }
    }

    if (state == State::EngineActive)
    {
      if (stopCandidateMs == 0U) { stopCandidateMs = nowMs; }
      const uint32_t debounceMs = static_cast<uint32_t>(config.stopDebounce100ms) * 100U;
      if ((nowMs - stopCandidateMs) < debounceMs)
      {
        if (!inputs.coolantValid)
        {
          requestedRpm = config.failsafeRpm;
          thermalState = ThermalState::Failsafe;
          resetClosedLoopRuntime();
        }
        else if ((config.flags & CLOSED_LOOP_ENABLED) != 0U)
        {
          requestedRpm = closedLoopRpm(nowMs, config, inputs);
        }
        else
        {
          requestedRpm = curveRpm(inputs.coolant, config);
          thermalState = ThermalState::ClosedLoop;
          resetClosedLoopRuntime();
        }
        updatePowerHoldRequest(config, inputs);
        applyRamp(nowMs, config);
        lastUpdateMs = nowMs;
        return;
      }
      if ((config.flags & AFTER_RUN_ENABLED) != 0U)
      {
        afterRunEndMs = nowMs + (static_cast<uint32_t>(config.afterRunMaximumSeconds) * 1000U);
      }
    }

    const bool mayStartAfterRun = engineSeen && ((config.flags & AFTER_RUN_ENABLED) != 0U) &&
                                  (afterRunEndMs != 0U) && !isTimeReached(nowMs, afterRunEndMs) &&
                                  inputs.coolantValid &&
                                  (inputs.coolant >= config.afterRunStartTemperature);
    if ((state != State::AfterRun) && mayStartAfterRun)
    {
      setState(State::AfterRun, nowMs);
      batteryInhibit = inputs.battery10 < config.batteryCutoff10;
    }

    if (state == State::AfterRun)
    {
      thermalState = ThermalState::AfterRun;
      resetClosedLoopRuntime();
      if (!inputs.coolantValid || (inputs.coolant <= config.afterRunStopTemperature) ||
          isTimeReached(nowMs, afterRunEndMs))
      {
        afterRunEndMs = 0U;
        engineSeen = false;
        setState(State::Stopped, nowMs);
        requestedRpm = 0U;
        thermalState = ThermalState::Inactive;
      }
      else
      {
        if (inputs.battery10 < config.batteryCutoff10)
        {
          batteryInhibit = true;
          latchedFaults |= FAULT_BATTERY_LOW;
        }
        else if (batteryInhibit && (inputs.battery10 >= config.batteryResume10))
        {
          batteryInhibit = false;
        }

        if (batteryInhibit)
        {
          requestedRpm = 0U;
        }
        else
        {
          const uint16_t thermalRpm = curveRpm(inputs.coolant, config);
          requestedRpm = (thermalRpm < config.afterRunMinimumRpm) ?
                         config.afterRunMinimumRpm : thermalRpm;
        }
      }
    }
    else
    {
      if ((afterRunEndMs != 0U) && isTimeReached(nowMs, afterRunEndMs))
      {
        afterRunEndMs = 0U;
        engineSeen = false;
      }
      setState(State::Stopped, nowMs);
      requestedRpm = 0U;
      thermalState = ThermalState::Inactive;
      resetClosedLoopRuntime();
    }
  }

  updatePowerHoldRequest(config, inputs);
  applyRamp(nowMs, config);
  lastUpdateMs = nowMs;
}

bool handleFrame(const CanFrame &frame, uint32_t nowMs, const Config &config)
{
  if (!frame.extended || (frame.len < 1U)) { return false; }
  const uint32_t sourceAddress = static_cast<uint32_t>(config.controllerAddress);

  if ((frame.id == (STATUS_1_BASE_ID | sourceAddress)) && (frame.len >= 8U))
  {
    capabilities |= CAP_STATUS_1;
    statusSummary = static_cast<uint8_t>((statusSummary & 0xFCU) | (frame.data[0] & 0x03U));
    controllerStatus = static_cast<uint8_t>((frame.data[0] >> 2U) & 0x1FU);
    diagnosticFaults &= static_cast<uint16_t>(~(FAULT_SERVICE_REQUIRED | FAULT_DERATED |
                                                FAULT_NOT_OPERABLE | FAULT_COMMAND_NOT_EXTERNAL));
    if (controllerStatus == 20U)
    {
      diagnosticFaults |= FAULT_SERVICE_REQUIRED | FAULT_NOT_OPERABLE;
    }
    else if ((controllerStatus == 22U) || (controllerStatus == 23U) || (controllerStatus == 25U))
    {
      diagnosticFaults |= FAULT_NOT_OPERABLE;
    }
    else if ((controllerStatus == 6U) || (controllerStatus == 27U) ||
             (controllerStatus == 28U) || (controllerStatus == 29U) ||
             (controllerStatus == 30U))
    {
      diagnosticFaults |= FAULT_DERATED;
    }
    const uint16_t rawSpeed = static_cast<uint16_t>(frame.data[1]) |
                              (static_cast<uint16_t>(frame.data[2]) << 8U);
    actualRpm = (rawSpeed == 0xFFFFU) ? 0U : static_cast<uint16_t>(rawSpeed / 2U);
    const uint16_t rawPower = static_cast<uint16_t>(frame.data[5]) |
                              (static_cast<uint16_t>(frame.data[6]) << 8U);
    powerWatts = (rawPower == 0xFFFFU) ? 0U : rawPower;
    actualPercentRaw = (frame.data[7] == 0xFFU) ? 0U : frame.data[7];
    lastMainStatusMs = nowMs;
    mainStatusSeen = true;
    return true;
  }

  if ((frame.id == (STATUS_2_BASE_ID | sourceAddress)) && (frame.len >= 8U))
  {
    capabilities |= CAP_STATUS_2;
    const uint8_t direction = frame.data[0] & 0x03U;
    controllerStatus = static_cast<uint8_t>((frame.data[0] >> 2U) & 0x0FU);
    const uint8_t commandSource = static_cast<uint8_t>((frame.data[0] >> 6U) & 0x03U);
    const uint8_t service = frame.data[7] & 0x03U;
    const uint8_t operation = static_cast<uint8_t>((frame.data[7] >> 2U) & 0x03U);
    statusSummary = static_cast<uint8_t>(direction | (commandSource << 2U) |
                                        (service << 4U) | (operation << 6U));
    diagnosticFaults &= static_cast<uint16_t>(~(FAULT_SERVICE_REQUIRED | FAULT_DERATED |
                                                FAULT_NOT_OPERABLE | FAULT_COMMAND_NOT_EXTERNAL));
    if (service == 1U) { diagnosticFaults |= FAULT_SERVICE_REQUIRED; }
    if (operation == 1U) { diagnosticFaults |= FAULT_DERATED; }
    if (operation == 2U) { diagnosticFaults |= FAULT_NOT_OPERABLE; }
    if (commandSource != 1U) { diagnosticFaults |= FAULT_COMMAND_NOT_EXTERNAL; }

    const uint16_t rawSpeed = static_cast<uint16_t>(frame.data[1]) |
                              (static_cast<uint16_t>(frame.data[2]) << 8U);
    actualRpm = (rawSpeed == 0xFFFFU) ? 0U : static_cast<uint16_t>(rawSpeed / 2U);
    const uint16_t rawTemperature = static_cast<uint16_t>(frame.data[3]) |
                                    (static_cast<uint16_t>(frame.data[4]) << 8U);
    externalTemperatureRaw = ((rawTemperature == 0xFFFFU) || (rawTemperature == 0xFEFFU) ||
                              (rawTemperature == 0xFFFEU)) ? 8736U : rawTemperature;
    const uint16_t rawPower = static_cast<uint16_t>(frame.data[5]) |
                              (static_cast<uint16_t>(frame.data[6]) << 8U);
    powerWatts = (rawPower == 0xFFFFU) ? 0U : static_cast<uint16_t>(rawPower / 2U);
    lastMainStatusMs = nowMs;
    mainStatusSeen = true;
    return true;
  }

  if ((frame.id == (STATUS_3_BASE_ID | sourceAddress)) && (frame.len >= 5U))
  {
    capabilities |= CAP_STATUS_3;
    const uint16_t rawVoltage = static_cast<uint16_t>(frame.data[0]) |
                                (static_cast<uint16_t>(frame.data[1]) << 8U);
    const uint16_t rawCurrent = static_cast<uint16_t>(frame.data[2]) |
                                (static_cast<uint16_t>(frame.data[3]) << 8U);
    voltageRaw = (rawVoltage == 0xFFFFU) ? 0U : rawVoltage;
    currentRaw = (rawCurrent == 0xFFFFU) ? 32000U : rawCurrent;
    diagnosticFaults &= static_cast<uint16_t>(~FAULT_HVIL);
    const uint8_t hvilStatus = frame.data[4] & 0x03U;
    if ((hvilStatus == 1U) || (hvilStatus == 2U)) { diagnosticFaults |= FAULT_HVIL; }
    return true;
  }

  if ((frame.id == (EXTERNAL_TEMPERATURE_BASE_ID | sourceAddress)) && (frame.len >= 3U))
  {
    capabilities |= CAP_EXTERNAL_TEMPERATURE;
    const uint16_t rawTemperature = static_cast<uint16_t>(frame.data[1]) |
                                    (static_cast<uint16_t>(frame.data[2]) << 8U);
    externalTemperatureRaw = ((rawTemperature == 0xFFFFU) || (rawTemperature == 0xFEFFU) ||
                              (rawTemperature == 0xFFFEU)) ? 8736U : rawTemperature;
    return true;
  }

  if (frame.id == (ADDRESS_CLAIM_BASE_ID | sourceAddress))
  {
    capabilities |= CAP_ADDRESS_CLAIM;
    return true;
  }

  return false;
}

bool takeCommand(uint32_t nowMs, const Config &config, Command &command)
{
  if ((!config.valid || ((config.flags & ENABLED) == 0U)) && (state == State::Disabled) &&
      (previousState == State::Disabled))
  {
    return false;
  }

  const uint8_t control = desiredControl(config);
  const bool heartbeatDue = !transmitSeen || ((nowMs - lastTransmitMs) >= HEARTBEAT_MS);
  if (!commandDirty && !heartbeatDue) { return false; }

  command.id = COMMAND_BASE_ID |
               (static_cast<uint32_t>(config.controllerAddress) << 8U) |
               static_cast<uint32_t>(config.sourceAddress);
  command.data[0] = control;
  if (targetRpm > 0U)
  {
    const uint16_t rawRpm = static_cast<uint16_t>(targetRpm * 2U);
    command.data[1] = static_cast<uint8_t>(rawRpm & 0xFFU);
    command.data[2] = static_cast<uint8_t>(rawRpm >> 8U);
  }
  else
  {
    command.data[1] = 0xFFU;
    command.data[2] = 0xFFU;
  }
  command.data[3] = 0xFFU;
  for (uint8_t index = 4U; index < 8U; index++) { command.data[index] = 0xFFU; }

  lastCommandControl = control;
  if (control == CONTROL_FORWARD_POWER_HOLD_ON) { powerHoldCommanded = true; }
  else if ((control == CONTROL_OFF_POWER_HOLD_OFF) ||
           (control == CONTROL_FORWARD_POWER_HOLD_OFF)) { powerHoldCommanded = false; }
  lastTransmitMs = nowMs;
  transmitSeen = true;
  commandDirty = false;
  if (targetRpm == 0U) { previousState = state; }
  return true;
}

void recordTransmitResult(bool success)
{
  if (success)
  {
    latchedFaults &= static_cast<uint16_t>(~FAULT_TX);
    return;
  }

  latchedFaults |= FAULT_TX;
  if (transmitFailureCount < UINT8_MAX) { transmitFailureCount++; }
  commandDirty = true;
}

bool handleServiceCommand(uint8_t command, uint16_t rpm, uint8_t durationSeconds,
                          uint32_t nowMs, const Config &config)
{
  switch (command)
  {
    case 0U:
      serviceEndMs = 0U;
      serviceRpm = 0U;
      commandDirty = true;
      return true;

    case 1U:
      if (((config.flags & ENABLED) == 0U) || !config.valid) { return false; }
      if ((rpm < config.minimumRunRpm) || (rpm > config.maximumRpm) ||
          (durationSeconds == 0U)) { return false; }
      serviceRpm = rpm;
      serviceEndMs = nowMs + (static_cast<uint32_t>(durationSeconds) * 1000U);
      commandDirty = true;
      return true;

    case 2U:
      latchedFaults = 0U;
      transmitFailureCount = 0U;
      return true;

    default:
      return false;
  }
}

State getState() { return state; }
ThermalState getThermalState() { return thermalState; }

uint16_t getFaults(uint32_t nowMs, const Config &config)
{
  uint16_t faults = latchedFaults;
  faults |= diagnosticFaults;
  faults |= controlFaults;
  if (targetRpm == 0U) { faults &= static_cast<uint16_t>(~FAULT_COMMAND_NOT_EXTERNAL); }
  const uint32_t timeoutMs = static_cast<uint32_t>(config.statusTimeoutSeconds) * 1000U;
  if (((state == State::EngineActive) || (state == State::AfterRun) || (state == State::ServiceTest)) &&
      ((!mainStatusSeen && ((nowMs - stateSinceMs) >= timeoutMs)) ||
       (mainStatusSeen && ((nowMs - lastMainStatusMs) >= timeoutMs))))
  {
    faults |= FAULT_STATUS_TIMEOUT;
  }
  return faults;
}

uint8_t getCapabilities() { return capabilities; }
uint16_t getTargetRpm() { return targetRpm; }
uint16_t getActualRpm() { return actualRpm; }
uint8_t getActualPercentRaw() { return actualPercentRaw; }
uint8_t getControllerStatus() { return controllerStatus; }
uint8_t getStatusSummary() { return statusSummary; }
uint16_t getVoltageRaw() { return voltageRaw; }
uint16_t getCurrentRaw() { return currentRaw; }
uint16_t getPowerWatts() { return powerWatts; }
uint16_t getExternalTemperatureRaw() { return externalTemperatureRaw; }
uint16_t getMainStatusAgeMs(uint32_t nowMs) { return mainStatusSeen ? saturatedAge(nowMs, lastMainStatusMs) : UINT16_MAX; }

uint16_t getAfterRunRemainingSeconds(uint32_t nowMs)
{
  if ((state != State::AfterRun) || isTimeReached(nowMs, afterRunEndMs)) { return 0U; }
  const uint32_t remaining = (afterRunEndMs - nowMs + 999U) / 1000U;
  return (remaining > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(remaining);
}

uint8_t getTransmitFailureCount() { return transmitFailureCount; }
uint8_t getLastCommandControl() { return lastCommandControl; }
uint8_t getControlFlags() { return controlFlags; }
int16_t getTargetTemperature() { return diagnosticTargetTemperature; }
int16_t getFilteredIntakeAirTemperature() { return diagnosticFilteredIat; }
int8_t getTemperatureError() { return diagnosticTemperatureError; }
int16_t getCoolantSlopePerMinute() { return coolantSlopePerMinute; }
uint16_t getMinimumFlowRpm() { return diagnosticMinimumFlowRpm; }
int16_t getFeedForwardRpm() { return diagnosticFeedForwardRpm; }
int16_t getPiCorrectionRpm() { return diagnosticPiCorrectionRpm; }
uint8_t getCoolingDemandRaw() { return coolingDemandRaw; }

uint16_t getSaturationSeconds(uint32_t nowMs)
{
  if (saturationSinceMs == 0U) { return 0U; }
  const uint32_t seconds = (nowMs - saturationSinceMs) / 1000U;
  return (seconds > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(seconds);
}

} // namespace emp_pump
