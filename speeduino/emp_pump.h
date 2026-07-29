#pragma once

#include <stdint.h>

namespace emp_pump
{

constexpr uint16_t CONFIG_MAGIC = 0xE6A5U;
constexpr uint8_t CONFIG_VERSION = 2U;
constexpr uint16_t HEARTBEAT_MS = 500U;

enum ConfigFlag : uint8_t
{
  ENABLED = 1U << 0,
  AFTER_RUN_ENABLED = 1U << 1,
  POWER_HOLD_ENABLED = 1U << 2,
  RUN_DURING_CRANKING = 1U << 3,
  HOT_BOOT_RECOVERY = 1U << 4,
  CLOSED_LOOP_ENABLED = 1U << 5,
};

enum class State : uint8_t
{
  Disabled = 0U,
  Stopped = 1U,
  EngineActive = 2U,
  AfterRun = 3U,
  ServiceTest = 4U,
};

enum class ThermalState : uint8_t
{
  Inactive = 0U,
  Warmup = 1U,
  ClosedLoop = 2U,
  CapacityLimited = 3U,
  Overload = 4U,
  AfterRun = 5U,
  Failsafe = 6U,
  Service = 7U,
};

enum Fault : uint16_t
{
  FAULT_STATUS_TIMEOUT = 1U << 0,
  FAULT_TX = 1U << 1,
  FAULT_CLT_INVALID = 1U << 2,
  FAULT_BATTERY_LOW = 1U << 3,
  FAULT_CONFIG = 1U << 4,
  FAULT_SERVICE_REQUIRED = 1U << 5,
  FAULT_DERATED = 1U << 6,
  FAULT_NOT_OPERABLE = 1U << 7,
  FAULT_COMMAND_NOT_EXTERNAL = 1U << 8,
  FAULT_HVIL = 1U << 9,
  FAULT_IAT_INVALID = 1U << 10,
  FAULT_COOLING_LIMITED = 1U << 11,
  FAULT_THERMAL_OVERLOAD = 1U << 12,
};

enum Capability : uint8_t
{
  CAP_STATUS_1 = 1U << 0,
  CAP_STATUS_2 = 1U << 1,
  CAP_STATUS_3 = 1U << 2,
  CAP_EXTERNAL_TEMPERATURE = 1U << 3,
  CAP_ADDRESS_CLAIM = 1U << 4,
};

struct Config
{
  uint8_t flags;
  uint8_t controllerAddress;
  uint8_t sourceAddress;
  uint8_t stopDebounce100ms;
  uint16_t minimumRunRpm;
  uint16_t maximumRpm;
  uint16_t afterRunMinimumRpm;
  uint16_t failsafeRpm;
  uint16_t rampRpmPerSecond;
  uint16_t afterRunMaximumSeconds;
  int16_t afterRunStartTemperature;
  int16_t afterRunStopTemperature;
  uint8_t batteryCutoff10;
  uint8_t batteryResume10;
  int16_t temperatureBins[6];
  uint16_t rpmBins[6];
  uint16_t manualTestRpm;
  uint8_t manualTestSeconds;
  uint8_t statusTimeoutSeconds;
  int16_t targetTemperature;
  uint8_t temperatureDeadband;
  uint16_t proportionalGain;
  uint16_t integralGain;
  uint16_t integralLimitRpm;
  uint16_t derivativeGain;
  uint16_t loadFeedForwardGain;
  int16_t iatReferenceTemperature;
  uint16_t iatCompensationGain;
  uint8_t airflowFullSpeedKph;
  uint16_t airflowReliefRpm;
  uint8_t fanEquivalentSpeedKph;
  uint8_t coolingLimitedDelta;
  uint8_t overloadDelta;
  uint8_t overloadDelaySeconds;
  uint16_t engineRpmBins[4];
  uint16_t minimumFlowRpmBins[4];
  bool valid;
};

struct Inputs
{
  int16_t coolant;
  int16_t intakeAirTemperature;
  uint16_t engineRpm;
  uint16_t manifoldPressure;
  uint16_t vehicleSpeedKph;
  uint8_t battery10;
  bool coolantValid;
  bool intakeAirTemperatureValid;
  bool vehicleSpeedValid;
  bool fanOn;
  bool airflowAtMaximumCapacity;
  bool engineRunning;
  bool engineCranking;
};

struct CanFrame
{
  uint32_t id;
  bool extended;
  uint8_t len;
  uint8_t data[8];
};

struct Command
{
  uint32_t id;
  uint8_t data[8];
};

void reset(uint32_t nowMs);
void update(uint32_t nowMs, const Config &config, const Inputs &inputs);
bool handleFrame(const CanFrame &frame, uint32_t nowMs, const Config &config);
bool takeCommand(uint32_t nowMs, const Config &config, Command &command);
void recordTransmitResult(bool success);
bool handleServiceCommand(uint8_t command, uint16_t rpm, uint8_t durationSeconds, uint32_t nowMs, const Config &config);

State getState();
ThermalState getThermalState();
uint16_t getFaults(uint32_t nowMs, const Config &config);
uint8_t getCapabilities();
uint16_t getTargetRpm();
uint16_t getActualRpm();
uint8_t getActualPercentRaw();
uint8_t getControllerStatus();
uint8_t getStatusSummary();
uint16_t getVoltageRaw();
uint16_t getCurrentRaw();
uint16_t getPowerWatts();
uint16_t getExternalTemperatureRaw();
uint16_t getMainStatusAgeMs(uint32_t nowMs);
uint16_t getAfterRunRemainingSeconds(uint32_t nowMs);
uint8_t getTransmitFailureCount();
uint8_t getLastCommandControl();
uint8_t getControlFlags();
int16_t getTargetTemperature();
int16_t getFilteredIntakeAirTemperature();
int8_t getTemperatureError();
int16_t getCoolantSlopePerMinute();
uint16_t getMinimumFlowRpm();
int16_t getFeedForwardRpm();
int16_t getPiCorrectionRpm();
uint8_t getCoolingDemandRaw();
uint16_t getSaturationSeconds(uint32_t nowMs);

} // namespace emp_pump
