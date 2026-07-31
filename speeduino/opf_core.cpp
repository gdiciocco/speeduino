#include "opf_core.h"

#include <Arduino.h>

#include "bit_manip.h"
#include "globals.h"
#include "board_definition.h"
#include "comms_CAN.h"
#include "emp_pump.h"
#include "init.h"
#include "timers.h"

#ifdef CAPONORD_BOARD

#define USE_I2C_BARO

#ifdef OIL_SENSOR_OPST
#include "opst_sensor.h"
#endif

#ifdef USE_I2C_BARO
#include <src/LPS25HB/LPS25HBSensor.h>
static TwoWire LPS_dev(PB11, PB10);
static LPS25HBSensor LPS_SensorLow(&LPS_dev, LPS25HB_ADDRESS_LOW);
static LPS25HBSensor LPS_SensorHigh(&LPS_dev, LPS25HB_ADDRESS_HIGH);
static LPS25HBSensor *LPS_Sensor = nullptr;

// A faulty or disconnected sensor makes every Wire transfer run to its HAL
// timeout, stalling the main loop for hundreds of ms per read: track sensor
// health and stop polling after repeated failures
static bool baroSensorOk = false;
static uint8_t baroFailCount = 0U;
static constexpr uint8_t BARO_MAX_FAILURES = 5U;
static constexpr float BARO_MIN_HPA = 260.0f;
static constexpr float BARO_MAX_HPA = 1260.0f;
static constexpr uint8_t LPS25HB_RESET_TIMEOUT_MS = 10U;

static bool waitForLPS25HBBitClear(LPS25HBSensor &sensor, uint8_t reg, uint8_t mask)
{
  for (uint8_t elapsedMs = 0U; elapsedMs < LPS25HB_RESET_TIMEOUT_MS; elapsedMs++)
  {
    uint8_t value = mask;
    if ((sensor.ReadReg(reg, &value) == LPS25HB_STATUS_OK) && ((value & mask) == 0U))
    {
      return true;
    }
    delay(1U);
  }
  return false;
}

static bool resetI2CBaroSensor(LPS25HBSensor &sensor)
{
  uint8_t deviceId = 0U;
  if ((sensor.ReadID(&deviceId) != LPS25HB_STATUS_OK) || (deviceId != LPS25HB_WHO_AM_I_VAL))
  {
    return false;
  }

  // The sensor may stay powered through an MCU-only reset. Restore its power-on
  // configuration so AUTO_ZERO and a previously programmed reference cannot
  // turn an absolute pressure measurement into a differential one.
  if ((sensor.WriteReg(LPS25HB_CTRL_REG2, LPS25HB_SW_RESET_MASK) != LPS25HB_STATUS_OK) ||
      !waitForLPS25HBBitClear(sensor, LPS25HB_CTRL_REG2, LPS25HB_SW_RESET_MASK))
  {
    return false;
  }

  // Reload the factory calibration after the software reset. BOOT takes about
  // 2.2 ms according to the LPS25HB datasheet and self-clears when complete.
  if ((sensor.WriteReg(LPS25HB_CTRL_REG2, LPS25HB_BOOT_MASK) != LPS25HB_STATUS_OK) ||
      !waitForLPS25HBBitClear(sensor, LPS25HB_CTRL_REG2, LPS25HB_BOOT_MASK))
  {
    return false;
  }

  deviceId = 0U;
  return (sensor.ReadID(&deviceId) == LPS25HB_STATUS_OK) &&
         (deviceId == LPS25HB_WHO_AM_I_VAL);
}

static bool configureI2CBaroForAbsolutePressure(LPS25HBSensor &sensor)
{
  static constexpr uint8_t offsetRegisters[] = {
    LPS25HB_REF_P_XL_REG,
    LPS25HB_REF_P_L_REG,
    LPS25HB_REF_P_H_REG,
    LPS25HB_RPDS_L_REG,
    LPS25HB_RPDS_H_REG
  };

  for (uint8_t reg : offsetRegisters)
  {
    if (sensor.WriteReg(reg, 0U) != LPS25HB_STATUS_OK) { return false; }
  }

  // begin() configures averaging and BDU, but the bundled ST driver also
  // enables DIFF_EN. Caponord needs absolute pressure and no pressure interrupt.
  if (sensor.begin() != LPS25HB_STATUS_OK) { return false; }

  uint8_t ctrlReg1 = 0U;
  uint8_t ctrlReg2 = 0U;
  if ((sensor.ReadReg(LPS25HB_CTRL_REG1, &ctrlReg1) != LPS25HB_STATUS_OK) ||
      (sensor.ReadReg(LPS25HB_CTRL_REG2, &ctrlReg2) != LPS25HB_STATUS_OK))
  {
    return false;
  }

  ctrlReg1 &= static_cast<uint8_t>(~(LPS25HB_DIFF_EN_MASK | LPS25HB_RESET_AZ_MASK));
  ctrlReg2 &= static_cast<uint8_t>(~LPS25HB_AUTO_ZERO_MASK);
  if ((sensor.WriteReg(LPS25HB_CTRL_REG1, ctrlReg1) != LPS25HB_STATUS_OK) ||
      (sensor.WriteReg(LPS25HB_CTRL_REG2, ctrlReg2) != LPS25HB_STATUS_OK))
  {
    return false;
  }

  // Read back every setting that could offset PRESS_OUT. Treat a sensor which
  // does not retain this configuration as unavailable rather than publishing
  // a plausible but incorrect barometric pressure.
  for (uint8_t reg : offsetRegisters)
  {
    uint8_t value = 0U;
    if ((sensor.ReadReg(reg, &value) != LPS25HB_STATUS_OK) || (value != 0U)) { return false; }
  }

  ctrlReg1 = 0U;
  ctrlReg2 = 0U;
  return (sensor.ReadReg(LPS25HB_CTRL_REG1, &ctrlReg1) == LPS25HB_STATUS_OK) &&
         (sensor.ReadReg(LPS25HB_CTRL_REG2, &ctrlReg2) == LPS25HB_STATUS_OK) &&
         ((ctrlReg1 & (LPS25HB_DIFF_EN_MASK | LPS25HB_RESET_AZ_MASK)) == 0U) &&
         ((ctrlReg2 & LPS25HB_AUTO_ZERO_MASK) == 0U);
}

static bool initialiseI2CBaroSensor(LPS25HBSensor &sensor)
{
  if (resetI2CBaroSensor(sensor) &&
      configureI2CBaroForAbsolutePressure(sensor) &&
      (sensor.SetODR(7.0f) == LPS25HB_STATUS_OK) &&
      (sensor.Enable() == LPS25HB_STATUS_OK))
  {
    LPS_Sensor = &sensor;
    return true;
  }
  return false;
}

static bool updateI2CBaro()
{
  if (!baroSensorOk || (LPS_Sensor == nullptr)) { return false; }

  float pressure = 0.0f;
  float temperature = 0.0f;
  if ((LPS_Sensor->GetPressure(&pressure) == LPS25HB_STATUS_OK) &&
      (LPS_Sensor->GetTemperature(&temperature) == LPS25HB_STATUS_OK) &&
      (pressure >= BARO_MIN_HPA) && (pressure <= BARO_MAX_HPA))
  {
    baroFailCount = 0U;
    const uint8_t pressureKpa = static_cast<uint8_t>(pressure / 10.0f);
    currentStatus.fuelTemp = static_cast<int8_t>(temperature);

    // Caponord has no analog barometer. Publish the already converted I2C
    // pressure directly to the canonical runtime/TS value. Keep baroADC as a
    // diagnostic mirror for compatibility only; it must never pass through
    // the generic ADC calibration path (which historically changed ~100 kPa
    // into values around 76 kPa).
    currentStatus.baro = pressureKpa;
    currentStatus.baroADC = pressureKpa;
    return true;
  }

  baroFailCount++;
  if (baroFailCount >= BARO_MAX_FAILURES) { baroSensorOk = false; }
  return false;
}
#endif

static constexpr byte UNUSED_PIN = BOARD_MAX_IO_PINS - 1U;

static constexpr byte LED_RUNNING = PG9;
static constexpr byte LED_WARNING = PG10;
static constexpr byte LED_ALERT = PG11;
static constexpr byte LED_COMS = PG12;

static emp_pump::Config caponordEmpPumpConfig()
{
  emp_pump::Config config = {};
  config.flags = configPage15.empPumpFlags;
  config.controllerAddress = configPage15.empPumpControllerAddress;
  config.sourceAddress = configPage15.empPumpSourceAddress;
  config.stopDebounce100ms = configPage15.empPumpStopDebounce100ms;
  config.minimumRunRpm = configPage15.empPumpMinimumRunRpm;
  config.maximumRpm = configPage15.empPumpMaximumRpm;
  config.afterRunMinimumRpm = configPage15.empPumpAfterRunMinimumRpm;
  config.failsafeRpm = configPage15.empPumpFailsafeRpm;
  config.rampRpmPerSecond = configPage15.empPumpRampRpmPerSecond;
  config.afterRunMaximumSeconds = configPage15.empPumpAfterRunMaximumSeconds;
  config.afterRunStartTemperature = static_cast<int16_t>(configPage15.empPumpAfterRunStartTemperature) - 40;
  config.afterRunStopTemperature = static_cast<int16_t>(configPage15.empPumpAfterRunStopTemperature) - 40;
  config.batteryCutoff10 = configPage15.empPumpBatteryCutoff10;
  config.batteryResume10 = configPage15.empPumpBatteryResume10;
  config.manualTestRpm = configPage15.empPumpManualTestRpm;
  config.manualTestSeconds = configPage15.empPumpManualTestSeconds;
  config.statusTimeoutSeconds = configPage15.empPumpStatusTimeoutSeconds;
  config.targetTemperature = static_cast<int16_t>(configPage15.empPumpTargetTemperature) - 40;
  config.temperatureDeadband = configPage15.empPumpTemperatureDeadband;
  config.proportionalGain = configPage15.empPumpProportionalGain;
  config.integralGain = configPage15.empPumpIntegralGain;
  config.integralLimitRpm = configPage15.empPumpIntegralLimitRpm;
  config.derivativeGain = configPage15.empPumpDerivativeGain;
  config.loadFeedForwardGain = configPage15.empPumpLoadFeedForwardGain;
  config.iatReferenceTemperature =
      static_cast<int16_t>(configPage15.empPumpIatReferenceTemperature) - 40;
  config.iatCompensationGain = configPage15.empPumpIatCompensationGain;
  config.airflowFullSpeedKph = configPage15.empPumpAirflowFullSpeedKph;
  config.airflowReliefRpm = configPage15.empPumpAirflowReliefRpm;
  config.fanEquivalentSpeedKph = configPage15.empPumpFanEquivalentSpeedKph;
  config.coolingLimitedDelta = configPage15.empPumpCoolingLimitedDelta;
  config.overloadDelta = configPage15.empPumpOverloadDelta;
  config.overloadDelaySeconds = configPage15.empPumpOverloadDelaySeconds;

  bool binsAscending = true;
  for (uint8_t index = 0U; index < 6U; index++)
  {
    config.temperatureBins[index] = static_cast<int16_t>(configPage15.empPumpTemperatureBins[index]) - 40;
    config.rpmBins[index] = configPage15.empPumpRpmBins[index];
    if ((index > 0U) && (config.temperatureBins[index] <= config.temperatureBins[index - 1U]))
    {
      binsAscending = false;
    }
  }
  bool engineBinsAscending = true;
  bool minimumFlowValid = true;
  for (uint8_t index = 0U; index < 4U; index++)
  {
    config.engineRpmBins[index] = configPage15.empPumpEngineRpmBins[index];
    config.minimumFlowRpmBins[index] = configPage15.empPumpMinimumFlowRpmBins[index];
    if ((index > 0U) && (config.engineRpmBins[index] <= config.engineRpmBins[index - 1U]))
    {
      engineBinsAscending = false;
    }
    if ((config.minimumFlowRpmBins[index] < config.minimumRunRpm) ||
        (config.minimumFlowRpmBins[index] > config.maximumRpm))
    {
      minimumFlowValid = false;
    }
  }

  config.valid = (configPage15.empPumpConfigMagic == emp_pump::CONFIG_MAGIC) &&
                 (configPage15.empPumpConfigVersion == emp_pump::CONFIG_VERSION) &&
                 (config.controllerAddress <= 0xF0U) &&
                 (config.sourceAddress <= 0xF0U) &&
                 (config.controllerAddress != config.sourceAddress) &&
                 (config.minimumRunRpm > 0U) &&
                 (config.maximumRpm >= config.minimumRunRpm) &&
                 (config.maximumRpm <= 32767U) &&
                 (config.afterRunMinimumRpm >= config.minimumRunRpm) &&
                 (config.afterRunMinimumRpm <= config.maximumRpm) &&
                 (config.failsafeRpm >= config.minimumRunRpm) &&
                 (config.failsafeRpm <= config.maximumRpm) &&
                 (config.afterRunStopTemperature < config.afterRunStartTemperature) &&
                 (config.afterRunMaximumSeconds > 0U) &&
                 (config.batteryCutoff10 <= config.batteryResume10) &&
                 (config.statusTimeoutSeconds > 0U) &&
                 (config.targetTemperature >= -20) &&
                 (config.targetTemperature <= 130) &&
                 (config.temperatureDeadband <= 10U) &&
                 (config.proportionalGain <= 2000U) &&
                 (config.integralGain <= 1000U) &&
                 (config.integralLimitRpm <= config.maximumRpm) &&
                 (config.derivativeGain <= 200U) &&
                 (config.loadFeedForwardGain <= 2000U) &&
                 (config.iatCompensationGain <= 500U) &&
                 (config.airflowFullSpeedKph > 0U) &&
                 (config.airflowReliefRpm <= config.maximumRpm) &&
                 (config.coolingLimitedDelta > 0U) &&
                 (config.overloadDelta >= config.coolingLimitedDelta) &&
                 (config.overloadDelaySeconds > 0U) &&
                 binsAscending && engineBinsAscending && minimumFlowValid;
  return config;
}

static void caponordEmpPumpSetDefaultsIfNeeded()
{
  if ((configPage15.empPumpConfigMagic == emp_pump::CONFIG_MAGIC) &&
      (configPage15.empPumpConfigVersion == emp_pump::CONFIG_VERSION))
  {
    return;
  }

  //Safe default: parameters are ready for inspection, but pump control remains disabled.
  configPage15.empPumpFlags = emp_pump::AFTER_RUN_ENABLED |
                              emp_pump::POWER_HOLD_ENABLED |
                              emp_pump::RUN_DURING_CRANKING |
                              emp_pump::CLOSED_LOOP_ENABLED;
  configPage15.empPumpControllerAddress = 0x96U;
  configPage15.empPumpSourceAddress = 0xA3U;
  configPage15.empPumpStopDebounce100ms = 10U;
  configPage15.empPumpMinimumRunRpm = 1500U;
  configPage15.empPumpMaximumRpm = 6000U;
  configPage15.empPumpAfterRunMinimumRpm = 1800U;
  configPage15.empPumpFailsafeRpm = 3000U;
  configPage15.empPumpRampRpmPerSecond = 2000U;
  configPage15.empPumpAfterRunMaximumSeconds = 180U;
  configPage15.empPumpAfterRunStartTemperature = 135U; //95 C, temperatures are stored +40
  configPage15.empPumpAfterRunStopTemperature = 125U; //85 C
  configPage15.empPumpBatteryCutoff10 = 115U;
  configPage15.empPumpBatteryResume10 = 120U;

  static constexpr byte temperatureBins[6] = {80U, 110U, 125U, 135U, 145U, 155U};
  static constexpr uint16_t rpmBins[6] = {1500U, 1500U, 2000U, 3000U, 4500U, 6000U};
  for (uint8_t index = 0U; index < 6U; index++)
  {
    configPage15.empPumpTemperatureBins[index] = temperatureBins[index];
    configPage15.empPumpRpmBins[index] = rpmBins[index];
  }

  configPage15.empPumpManualTestRpm = 2000U;
  configPage15.empPumpManualTestSeconds = 10U;
  configPage15.empPumpStatusTimeoutSeconds = 3U;
  configPage15.empPumpTargetTemperature = 130U; //90 C, temperatures are stored +40
  configPage15.empPumpTemperatureDeadband = 1U;
  configPage15.empPumpProportionalGain = 250U; //RPM per degree C outside the deadband
  configPage15.empPumpIntegralGain = 12U; //RPM per degree C per second
  configPage15.empPumpIntegralLimitRpm = 2000U;
  configPage15.empPumpDerivativeGain = 8U; //RPM per degree C/minute of positive slope
  configPage15.empPumpLoadFeedForwardGain = 220U;
  configPage15.empPumpIatReferenceTemperature = 60U; //20 C
  configPage15.empPumpIatCompensationGain = 18U; //RPM per degree C from reference
  configPage15.empPumpAirflowFullSpeedKph = 100U;
  configPage15.empPumpAirflowReliefRpm = 600U;
  configPage15.empPumpFanEquivalentSpeedKph = 35U;
  configPage15.empPumpCoolingLimitedDelta = 3U;
  configPage15.empPumpOverloadDelta = 8U;
  configPage15.empPumpOverloadDelaySeconds = 5U;
  static constexpr uint16_t engineRpmBins[4] = {0U, 2000U, 5000U, 9000U};
  static constexpr uint16_t minimumFlowRpmBins[4] = {1500U, 1800U, 2300U, 3000U};
  for (uint8_t index = 0U; index < 4U; index++)
  {
    configPage15.empPumpEngineRpmBins[index] = engineRpmBins[index];
    configPage15.empPumpMinimumFlowRpmBins[index] = minimumFlowRpmBins[index];
  }
  for (uint8_t index = 0U; index < 66U; index++) { configPage15.Unused15_190_255[index] = 0U; }
  configPage15.empPumpConfigMagic = emp_pump::CONFIG_MAGIC;
  configPage15.empPumpConfigVersion = emp_pump::CONFIG_VERSION;
  configPage15.empPumpReserved151 = 0U;
}

//Shock absorber preload controller CAN interface
static constexpr uint32_t PRELOAD_CAN_COMMAND_ID = 0x720UL;
static constexpr uint32_t PRELOAD_CAN_STATUS_ID = 0x721UL;
static constexpr uint32_t PRELOAD_CAN_ALARM_ID = 0x722UL;

static constexpr byte PRELOAD_CMD_SET_TARGET = 0x01U;
static constexpr byte PRELOAD_CMD_LOAD_PRESET = 0x02U;
static constexpr byte PRELOAD_CMD_SAVE_CURRENT = 0x03U;
static constexpr byte PRELOAD_CMD_STOP = 0x04U;
static constexpr byte PRELOAD_CMD_CLEAR_ALARMS = 0x05U;
static constexpr byte PRELOAD_CMD_MOVE_RELATIVE = 0x06U;
static constexpr byte PRELOAD_CMD_REQUEST_STATUS = 0x10U;
static constexpr byte PRELOAD_CMD_SET_POSITION_REF = 0x11U;
static constexpr byte PRELOAD_CMD_START_CALIBRATION = 0x12U;
static constexpr byte PRELOAD_STATUS_KIND_PRESET = 0xF0U;

static constexpr byte PRELOAD_MAX_POSITION = 100U;
static constexpr byte PRELOAD_PRESET_COUNT = 5U;
static constexpr uint16_t PRELOAD_CAN_TIMEOUT_MS = 1000U;
static constexpr uint16_t PRELOAD_CAN_POLL_MS = 100U;
static constexpr byte PRELOAD_TX_MAX_ATTEMPTS = 8U;

static constexpr byte PRELOAD_CAN_FLAG_STATUS_SEEN = 1U << 0;
static constexpr byte PRELOAD_CAN_FLAG_ALARM_SEEN = 1U << 1;
static constexpr byte PRELOAD_CAN_FLAG_TIMEOUT = 1U << 2;
static constexpr byte PRELOAD_CAN_FLAG_BRIDGE_ACTIVE = 1U << 3;
static constexpr byte PRELOAD_CAN_FLAG_TX_ATTEMPTED = 1U << 4;
static constexpr byte PRELOAD_CAN_FLAG_TX_OK = 1U << 5;
static constexpr byte PRELOAD_CAN_FLAG_TX_FAILED = 1U << 6;
static constexpr byte PRELOAD_CAN_FLAG_FALLBACK_RX = 1U << 7;

static byte preloadCommand = 0U;
static byte preloadTarget = 0U;
static byte preloadSlot = 0U;
static byte preloadPosition = 0U;
static byte preloadControllerTarget = 0U;
static byte preloadState = 0U;
static uint16_t preloadAlarms = 0U;
static uint16_t preloadCurrent = 0U;
static uint32_t preloadLastRxMs = 0U;
static byte preloadLastCommand = 0U;
static byte preloadCanFlags = 0U;
static byte preloadAnyRxCount = 0U;
static uint16_t preloadLastRxId = 0U;
static byte preloadLastRxDlc = 0U;
static byte preloadLastRxByte0 = 0U;
static byte preloadPresetValues[PRELOAD_PRESET_COUNT] = {};
static byte preloadPresetSeenMask = 0U;

static bool preloadCommandPending = false;
static byte preloadPendingCommand = 0U;
static uint16_t preloadPendingValue = 0U;
static byte preloadPendingSlot = 0U;
static byte preloadPendingAttempts = 0U;
static uint32_t preloadLastPollMs = 0U;

static uint16_t caponordPreloadAgeMs()
{
  if (preloadLastRxMs == 0U)
  {
    return UINT16_MAX;
  }

  const uint32_t ageMs = millis() - preloadLastRxMs;
  return (ageMs > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(ageMs);
}

#if defined(NATIVE_CAN_AVAILABLE)
static uint16_t readCanWord(const CAN_message_t &msg, byte offset)
{
  return word(msg.buf[offset + 1U], msg.buf[offset]);
}
#endif

static void caponordPreloadRefreshTimeoutFlag()
{
  if (caponordPreloadAgeMs() >= PRELOAD_CAN_TIMEOUT_MS)
  {
    preloadCanFlags |= PRELOAD_CAN_FLAG_TIMEOUT;
  }
  else
  {
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TIMEOUT);
  }
}

static bool caponordPreloadSendCommand(byte command, uint16_t value, byte slot)
{
#if defined(NATIVE_CAN_AVAILABLE)
  CAN_message_t txMsg = {};
  txMsg.id = PRELOAD_CAN_COMMAND_ID;
  txMsg.flags.extended = false;
  txMsg.len = 1U;
  txMsg.buf[0] = command;

  switch (command)
  {
    case PRELOAD_CMD_SET_TARGET:
    case PRELOAD_CMD_SET_POSITION_REF:
    case PRELOAD_CMD_MOVE_RELATIVE:
      txMsg.len = 3U;
      txMsg.buf[1] = lowByte(value);
      txMsg.buf[2] = highByte(value);
      break;

    case PRELOAD_CMD_LOAD_PRESET:
    case PRELOAD_CMD_SAVE_CURRENT:
      txMsg.len = 2U;
      txMsg.buf[1] = slot;
      break;

    default:
      break;
  }

  preloadCanFlags |= PRELOAD_CAN_FLAG_TX_ATTEMPTED;

  if (Can0.write(txMsg))
  {
    preloadLastCommand = command;
    preloadCanFlags |= PRELOAD_CAN_FLAG_TX_OK;
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TX_FAILED);
    return true;
  }

  preloadCanFlags |= PRELOAD_CAN_FLAG_TX_FAILED;
  preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TX_OK);
#else
  (void)command;
  (void)value;
  (void)slot;
#endif

  return false;
}

static void caponordPreloadQueueCommand(byte command, uint16_t value, byte slot)
{
  preloadCommand = command;
  preloadPendingCommand = command;
  preloadPendingValue = value;
  preloadPendingSlot = slot;
  preloadPendingAttempts = 0U;
  preloadCommandPending = true;
}

static void caponordPreloadRunCanBridge()
{
#if defined(NATIVE_CAN_AVAILABLE)
  preloadCanFlags |= PRELOAD_CAN_FLAG_BRIDGE_ACTIVE;

  if (configPage9.enable_intcan != 1U)
  {
    //Internal CAN disabled: the main loop is not draining the bus, do it here
    while (CAN_read())
    {
      (void)caponordHandleCanFrame(inMsg);
    }
    preloadCanFlags |= PRELOAD_CAN_FLAG_FALLBACK_RX;
  }
  else
  {
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_FALLBACK_RX);
  }

  if (preloadCommandPending)
  {
    if (caponordPreloadSendCommand(preloadPendingCommand, preloadPendingValue, preloadPendingSlot))
    {
      preloadCommandPending = false;
    }
    else
    {
      //With the controller off or the bus dead the TX can never succeed:
      //drop the command instead of retrying forever
      preloadPendingAttempts++;
      if (preloadPendingAttempts >= PRELOAD_TX_MAX_ATTEMPTS) { preloadCommandPending = false; }
    }
  }

  const uint32_t nowMs = millis();
  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) && ((nowMs - preloadLastPollMs) >= PRELOAD_CAN_POLL_MS))
  {
    (void)caponordPreloadSendCommand(PRELOAD_CMD_REQUEST_STATUS, 0U, 0U);
    preloadLastPollMs = nowMs;
  }
#else
  preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_BRIDGE_ACTIVE);
  preloadCommandPending = false; //No CAN peripheral: a queued command can never be sent
#endif

  caponordPreloadRefreshTimeoutFlag();
}

static void caponordEmpPumpRunCanBridge()
{
  const emp_pump::Config config = caponordEmpPumpConfig();
  emp_pump::Inputs inputs = {};
  inputs.coolant = static_cast<int16_t>(currentStatus.coolant);
  inputs.intakeAirTemperature = static_cast<int16_t>(currentStatus.IAT);
  inputs.engineRpm = currentStatus.RPM;
  inputs.manifoldPressure = (currentStatus.MAP <= 0L) ? 0U :
                            ((currentStatus.MAP > 65535L) ? 65535U :
                             static_cast<uint16_t>(currentStatus.MAP));
  inputs.vehicleSpeedKph = currentStatus.vss;
  inputs.battery10 = currentStatus.battery10;
  inputs.coolantValid = (currentStatus.cltADC > 2U) && (currentStatus.cltADC < 1021U) &&
                        (currentStatus.coolant >= -40) && (currentStatus.coolant <= 215);
  inputs.intakeAirTemperatureValid =
      (currentStatus.iatADC > 2U) && (currentStatus.iatADC < 1021U) &&
      (currentStatus.IAT >= -40) && (currentStatus.IAT <= 215);
  inputs.vehicleSpeedValid = configPage2.vssMode != VSS_MODE_OFF;
  inputs.fanOn = currentStatus.fanOn;
  inputs.airflowAtMaximumCapacity =
      (configPage2.fanEnable == 0U) ||
      ((configPage2.fanEnable == 1U) && currentStatus.fanOn) ||
      ((configPage2.fanEnable == 2U) && (currentStatus.fanDuty >= 200U));
  inputs.engineRunning = currentStatus.rotationStatus == EngineRotationStatus::Running;
  inputs.engineCranking = currentStatus.rotationStatus == EngineRotationStatus::Cranking;

  const uint32_t nowMs = millis();
  emp_pump::update(nowMs, config, inputs);

#if defined(NATIVE_CAN_AVAILABLE)
  emp_pump::Command command = {};
  if (emp_pump::takeCommand(nowMs, config, command))
  {
    CAN_message_t txMsg = {};
    txMsg.id = command.id;
    txMsg.flags.extended = true;
    txMsg.len = 8U;
    for (uint8_t index = 0U; index < 8U; index++) { txMsg.buf[index] = command.data[index]; }
    emp_pump::recordTransmitResult(Can0.write(txMsg));
  }
#endif
}

static void setStatusLedOff(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static void resetPin(byte &pin)
{
  pin = UNUSED_PIN;
}

void setupBoard()
{
  initialiseAll();
  caponordEmpPumpSetDefaultsIfNeeded();
  emp_pump::reset(millis());

  setStatusLedOff(LED_RUNNING);
  setStatusLedOff(LED_WARNING);
  setStatusLedOff(LED_ALERT);
  setStatusLedOff(LED_COMS);

#ifdef OIL_SENSOR_OPST
  pinMode(PIN_OPST, INPUT);
#endif

#ifdef USE_I2C_BARO
  LPS_dev.begin();
  baroSensorOk = initialiseI2CBaroSensor(LPS_SensorLow) ||
                 initialiseI2CBaroSensor(LPS_SensorHigh);
  if (baroSensorOk)
  {
    (void)updateI2CBaro();
  }
  //A failed sensor is reported via LED_ALERT in runLoop, which keeps the
  //indication persistent instead of being overwritten at the first 10Hz tick
#endif
}

void caponordSetPins()
{
  pinTrigger = PE5;
  pinTrigger2 = PE4;
  pinVSS = PC13;

  pinBat = PA0;
  pinCLT = PA3;
  pinTPS = PA1;
  pinIAT = PA4;
  pinO2 = PC1;
  pinO2_2 = PC2;
  // Barometric pressure is supplied by the LPS25HB on I2C2 (PB11/PB10).
  // Deliberately leave no analog barometer pin for generic sensor code.
  pinBaro = NOT_A_PIN;
  pinMAP = PA6;
  pinFuelPressure = PC4; // A4 on the black_f407zg variant

  pinInjector2 = PF14;
  pinInjector1 = PF13;

  pinCoil1 = PE14;
  pinCoil2 = PE15;

  pinTachOut = PD14;
  pinStepperDir = PF7;
  pinStepperStep = PF8;
  pinStepperEnable = PF9;
  pinFuelPump = PG6;
  pinFan = PG7;
}

void caponordResetPins()
{
  resetPin(pinInjector1);
  resetPin(pinInjector2);
  resetPin(pinInjector3);
  resetPin(pinInjector4);
  resetPin(pinInjector5);
  resetPin(pinInjector6);
  resetPin(pinInjector7);
  resetPin(pinInjector8);

  resetPin(pinCoil1);
  resetPin(pinCoil2);
  resetPin(pinCoil3);
  resetPin(pinCoil4);
  resetPin(pinCoil5);
  resetPin(pinCoil6);
  resetPin(pinCoil7);
  resetPin(pinCoil8);

  resetPin(pinTrigger);
  resetPin(pinTrigger2);
  resetPin(pinTrigger3);
  resetPin(pinTPS);
  resetPin(pinMAP);
  resetPin(pinEMAP);
  resetPin(pinMAP2);
  resetPin(pinIAT);
  resetPin(pinCLT);
  resetPin(pinO2);
  resetPin(pinO2_2);
  resetPin(pinBat);
  resetPin(pinDisplayReset);
  resetPin(pinTachOut);
  resetPin(pinFuelPump);
  resetPin(pinIdle1);
  resetPin(pinIdle2);
  resetPin(pinIdleUp);
  resetPin(pinIdleUpOutput);
  resetPin(pinCTPS);
  resetPin(pinFuel2Input);
  resetPin(pinSpark2Input);
  resetPin(pinSpareTemp1);
  resetPin(pinSpareTemp2);
  resetPin(pinSpareOut1);
  resetPin(pinSpareOut2);
  resetPin(pinSpareOut3);
  resetPin(pinSpareOut4);
  resetPin(pinSpareOut5);
  resetPin(pinSpareOut6);
  resetPin(pinSpareHOut1);
  resetPin(pinSpareHOut2);
  resetPin(pinSpareLOut1);
  resetPin(pinSpareLOut2);
  resetPin(pinSpareLOut3);
  resetPin(pinSpareLOut4);
  resetPin(pinSpareLOut5);
  resetPin(pinBoost);
  resetPin(pinVVT_1);
  resetPin(pinVVT_2);
  resetPin(pinFan);
  resetPin(pinStepperDir);
  resetPin(pinStepperStep);
  resetPin(pinStepperEnable);
  resetPin(pinLaunch);
  resetPin(pinIgnBypass);
  resetPin(pinFlex);
  resetPin(pinVSS);
  resetPin(pinBaro);
  resetPin(pinResetControl);
  resetPin(pinFuelPressure);
  resetPin(pinOilPressure);
  resetPin(pinWMIEmpty);
  resetPin(pinWMIIndicator);
  resetPin(pinWMIEnabled);
  resetPin(pinMC33810_1_CS);
  resetPin(pinMC33810_2_CS);
}

void runLoop()
{
  //A fresh command gets its first TX attempt on the next loop; retries run
  //on the 10Hz tick only, so a dead bus cannot drag every loop iteration
  if ((preloadCommandPending && (preloadPendingAttempts == 0U)) || BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ))
  {
    caponordPreloadRunCanBridge();
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ))
  {
    caponordEmpPumpRunCanBridge();

    if (Serial.available() > 0)
    {
      digitalToggle(LED_COMS);
    }
    else
    {
      digitalWrite(LED_COMS, LOW);
    }

    bool alertActive = currentStatus.engineProtect.isActive();
#ifdef USE_I2C_BARO
    alertActive = alertActive || (!baroSensorOk);
#endif
    digitalWrite(LED_ALERT, alertActive);
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ))
  {
#ifdef USE_I2C_BARO
    (void)updateI2CBaro();
#endif
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_4HZ))
  {
#ifdef OIL_SENSOR_OPST
    readOPSt();
#endif
    digitalToggle(LED_RUNNING);
  }
}

bool caponordPreloadHandleSerialCommand(uint8_t command, uint16_t value, uint8_t slot)
{
  switch (command)
  {
    case PRELOAD_CMD_SET_TARGET:
      if (value > PRELOAD_MAX_POSITION) { return false; }
      preloadTarget = static_cast<byte>(value);
      caponordPreloadQueueCommand(command, value, slot);
      return true;

    case PRELOAD_CMD_LOAD_PRESET:
      if (slot >= PRELOAD_PRESET_COUNT) { return false; }
      preloadSlot = slot;
      if (BIT_CHECK(preloadPresetSeenMask, slot))
      {
        preloadTarget = preloadPresetValues[slot];
      }
      caponordPreloadQueueCommand(command, value, slot);
      return true;

    case PRELOAD_CMD_SAVE_CURRENT:
      if (slot >= PRELOAD_PRESET_COUNT) { return false; }
      preloadSlot = slot;
      preloadPresetValues[slot] = preloadPosition;
      preloadPresetSeenMask |= static_cast<byte>(1U << slot);
      caponordPreloadQueueCommand(command, value, slot);
      return true;

    case PRELOAD_CMD_MOVE_RELATIVE:
    {
      const int16_t delta = static_cast<int16_t>(value);
      if ((delta < -static_cast<int16_t>(PRELOAD_MAX_POSITION)) ||
          (delta > static_cast<int16_t>(PRELOAD_MAX_POSITION))) { return false; }
      const int16_t baseTarget = static_cast<int16_t>(preloadControllerTarget);
      const int16_t newTarget = constrain(baseTarget + delta, 0, PRELOAD_MAX_POSITION);
      preloadTarget = static_cast<byte>(newTarget);
      caponordPreloadQueueCommand(command, value, slot);
      return true;
    }

    case PRELOAD_CMD_STOP:
    case PRELOAD_CMD_CLEAR_ALARMS:
    case PRELOAD_CMD_REQUEST_STATUS:
    case PRELOAD_CMD_START_CALIBRATION:
      caponordPreloadQueueCommand(command, value, slot);
      return true;

    case PRELOAD_CMD_SET_POSITION_REF:
      if (value > PRELOAD_MAX_POSITION) { return false; }
      caponordPreloadQueueCommand(command, value, slot);
      return true;

    default:
      return false;
  }
}

bool caponordPreloadHandleCanFrame(const CAN_message_t &msg)
{
#if defined(NATIVE_CAN_AVAILABLE)
  if (preloadAnyRxCount < UINT8_MAX)
  {
    preloadAnyRxCount++;
  }
  preloadLastRxId = static_cast<uint16_t>(msg.id & 0xFFFFU);
  preloadLastRxDlc = msg.len;
  preloadLastRxByte0 = (msg.len > 0U) ? msg.buf[0] : 0U;

  if (msg.flags.extended)
  {
    return false;
  }

  if ((msg.id == PRELOAD_CAN_STATUS_ID) && (msg.len >= 8U))
  {
    if (msg.buf[0] == PRELOAD_STATUS_KIND_PRESET)
    {
      const byte slot = msg.buf[1];
      if (slot < PRELOAD_PRESET_COUNT)
      {
        preloadPresetValues[slot] = static_cast<byte>(min(msg.buf[2], PRELOAD_MAX_POSITION));
        preloadPresetSeenMask |= static_cast<byte>(1U << slot);
      }
      if (msg.buf[3] < PRELOAD_PRESET_COUNT)
      {
        preloadSlot = msg.buf[3];
      }
      preloadPosition = static_cast<byte>(min(msg.buf[4], PRELOAD_MAX_POSITION));
      preloadControllerTarget = static_cast<byte>(min(msg.buf[5], PRELOAD_MAX_POSITION));
      preloadAlarms = word(msg.buf[7], msg.buf[6]);
      preloadLastRxMs = millis();
      preloadCanFlags |= PRELOAD_CAN_FLAG_STATUS_SEEN;
      preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TIMEOUT);
      return true;
    }

    preloadPosition = static_cast<byte>(min(readCanWord(msg, 0U), static_cast<uint16_t>(PRELOAD_MAX_POSITION)));
    preloadControllerTarget = static_cast<byte>(min(readCanWord(msg, 2U), static_cast<uint16_t>(PRELOAD_MAX_POSITION)));
    preloadSlot = msg.buf[4];
    preloadState = msg.buf[5];
    preloadAlarms = readCanWord(msg, 6U);
    preloadLastRxMs = millis();
    preloadCanFlags |= PRELOAD_CAN_FLAG_STATUS_SEEN;
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TIMEOUT);
    return true;
  }

  if ((msg.id == PRELOAD_CAN_ALARM_ID) && (msg.len >= 8U))
  {
    preloadAlarms = readCanWord(msg, 0U);
    preloadPosition = static_cast<byte>(min(readCanWord(msg, 2U), static_cast<uint16_t>(PRELOAD_MAX_POSITION)));
    preloadControllerTarget = static_cast<byte>(min(readCanWord(msg, 4U), static_cast<uint16_t>(PRELOAD_MAX_POSITION)));
    preloadCurrent = readCanWord(msg, 6U);
    preloadLastRxMs = millis();
    preloadCanFlags |= PRELOAD_CAN_FLAG_ALARM_SEEN;
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_TIMEOUT);
    return true;
  }

  return false;
#else
  (void)msg;
  return false;
#endif
}

bool caponordHandleCanFrame(const CAN_message_t &msg)
{
  if (caponordPreloadHandleCanFrame(msg)) { return true; }

#if defined(NATIVE_CAN_AVAILABLE)
  emp_pump::CanFrame frame = {};
  frame.id = msg.id;
  frame.extended = msg.flags.extended;
  frame.len = msg.len;
  for (uint8_t index = 0U; (index < msg.len) && (index < 8U); index++)
  {
    frame.data[index] = msg.buf[index];
  }
  return emp_pump::handleFrame(frame, millis(), caponordEmpPumpConfig());
#else
  (void)msg;
  return false;
#endif
}

bool caponordEmpPumpHandleSerialCommand(uint8_t command, uint16_t rpm, uint8_t durationSeconds)
{
  const emp_pump::Config config = caponordEmpPumpConfig();
  if ((command == 1U) && (rpm == 0U)) { rpm = config.manualTestRpm; }
  if ((command == 1U) && (durationSeconds == 0U)) { durationSeconds = config.manualTestSeconds; }
  const bool accepted = emp_pump::handleServiceCommand(command, rpm, durationSeconds, millis(), config);
  if (accepted) { caponordEmpPumpRunCanBridge(); }
  return accepted;
}

uint8_t caponordPreloadGetCommand()
{
  return preloadCommand;
}

uint8_t caponordPreloadGetTarget()
{
  return preloadTarget;
}

uint8_t caponordPreloadGetSlot()
{
  return preloadSlot;
}

uint8_t caponordPreloadGetPosition()
{
  return preloadPosition;
}

uint8_t caponordPreloadGetControllerTarget()
{
  return preloadControllerTarget;
}

uint8_t caponordPreloadGetState()
{
  return preloadState;
}

uint16_t caponordPreloadGetAlarms()
{
  return preloadAlarms;
}

uint16_t caponordPreloadGetCurrent()
{
  return preloadCurrent;
}

uint16_t caponordPreloadGetAgeMs()
{
  return caponordPreloadAgeMs();
}

uint8_t caponordPreloadGetLastCommand()
{
  return preloadLastCommand;
}

uint8_t caponordPreloadGetCanFlags()
{
  caponordPreloadRefreshTimeoutFlag();
  return preloadCanFlags;
}

uint8_t caponordPreloadGetAnyRxCount()
{
  return preloadAnyRxCount;
}

uint16_t caponordPreloadGetLastRxId()
{
  return preloadLastRxId;
}

uint8_t caponordPreloadGetLastRxDlc()
{
  return preloadLastRxDlc;
}

uint8_t caponordPreloadGetLastRxByte0()
{
  return preloadLastRxByte0;
}

uint8_t caponordPreloadGetPresetSeenMask()
{
  return preloadPresetSeenMask;
}

uint8_t caponordPreloadGetPresetValue(uint8_t slot)
{
  if (slot >= PRELOAD_PRESET_COUNT)
  {
    return 0U;
  }

  return preloadPresetValues[slot];
}

uint8_t caponordEmpPumpGetState() { return static_cast<uint8_t>(emp_pump::getState()); }
uint8_t caponordEmpPumpGetCapabilities() { return emp_pump::getCapabilities(); }
uint16_t caponordEmpPumpGetFaults() { return emp_pump::getFaults(millis(), caponordEmpPumpConfig()); }
uint16_t caponordEmpPumpGetTargetRpm() { return emp_pump::getTargetRpm(); }
uint16_t caponordEmpPumpGetActualRpm() { return emp_pump::getActualRpm(); }
uint8_t caponordEmpPumpGetActualPercentRaw() { return emp_pump::getActualPercentRaw(); }
uint8_t caponordEmpPumpGetControllerStatus() { return emp_pump::getControllerStatus(); }
uint8_t caponordEmpPumpGetStatusSummary() { return emp_pump::getStatusSummary(); }
uint16_t caponordEmpPumpGetVoltageRaw() { return emp_pump::getVoltageRaw(); }
uint16_t caponordEmpPumpGetCurrentRaw() { return emp_pump::getCurrentRaw(); }
uint16_t caponordEmpPumpGetPowerWatts() { return emp_pump::getPowerWatts(); }
uint16_t caponordEmpPumpGetExternalTemperatureRaw() { return emp_pump::getExternalTemperatureRaw(); }
uint16_t caponordEmpPumpGetMainStatusAgeMs() { return emp_pump::getMainStatusAgeMs(millis()); }
uint16_t caponordEmpPumpGetAfterRunRemainingSeconds() { return emp_pump::getAfterRunRemainingSeconds(millis()); }
uint8_t caponordEmpPumpGetTransmitFailureCount() { return emp_pump::getTransmitFailureCount(); }
uint8_t caponordEmpPumpGetLastCommandControl() { return emp_pump::getLastCommandControl(); }
uint8_t caponordEmpPumpGetThermalState() { return static_cast<uint8_t>(emp_pump::getThermalState()); }
uint8_t caponordEmpPumpGetControlFlags() { return emp_pump::getControlFlags(); }
int16_t caponordEmpPumpGetTargetTemperature() { return emp_pump::getTargetTemperature(); }
int16_t caponordEmpPumpGetFilteredIat() { return emp_pump::getFilteredIntakeAirTemperature(); }
int8_t caponordEmpPumpGetTemperatureError() { return emp_pump::getTemperatureError(); }
int16_t caponordEmpPumpGetCoolantSlopePerMinute() { return emp_pump::getCoolantSlopePerMinute(); }
uint16_t caponordEmpPumpGetMinimumFlowRpm() { return emp_pump::getMinimumFlowRpm(); }
int16_t caponordEmpPumpGetFeedForwardRpm() { return emp_pump::getFeedForwardRpm(); }
int16_t caponordEmpPumpGetPiCorrectionRpm() { return emp_pump::getPiCorrectionRpm(); }
uint8_t caponordEmpPumpGetCoolingDemandRaw() { return emp_pump::getCoolingDemandRaw(); }
uint16_t caponordEmpPumpGetSaturationSeconds() { return emp_pump::getSaturationSeconds(millis()); }

#else

void setupBoard()
{
  initialiseAll();
}

void runLoop()
{
}

#endif
