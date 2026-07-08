#include "opf_core.h"

#include <Arduino.h>

#include "bit_manip.h"
#include "globals.h"
#include "board_definition.h"
#include "comms_CAN.h"
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
static LPS25HBSensor LPS_Sensor(&LPS_dev, LPS25HB_ADDRESS_LOW);
#endif

static constexpr byte CAPONORD_PIN_MAPPING = 60U;
static constexpr byte UNUSED_PIN = BOARD_MAX_IO_PINS - 1U;

static constexpr byte LED_RUNNING = PG9;
static constexpr byte LED_WARNING = PG10;
static constexpr byte LED_ALERT = PG11;
static constexpr byte LED_COMS = PG12;

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
      (void)caponordPreloadHandleCanFrame(inMsg);
    }
    preloadCanFlags |= PRELOAD_CAN_FLAG_FALLBACK_RX;
  }
  else
  {
    preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_FALLBACK_RX);
  }

  if (preloadCommandPending && caponordPreloadSendCommand(preloadPendingCommand, preloadPendingValue, preloadPendingSlot))
  {
    preloadCommandPending = false;
  }

  const uint32_t nowMs = millis();
  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ) && ((nowMs - preloadLastPollMs) >= PRELOAD_CAN_POLL_MS))
  {
    (void)caponordPreloadSendCommand(PRELOAD_CMD_REQUEST_STATUS, 0U, 0U);
    preloadLastPollMs = nowMs;
  }
#else
  preloadCanFlags &= static_cast<byte>(~PRELOAD_CAN_FLAG_BRIDGE_ACTIVE);
#endif

  caponordPreloadRefreshTimeoutFlag();
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
  caponordResetPins();
  caponordSetPins();
  configPage2.pinMapping = CAPONORD_PIN_MAPPING;

  initialiseAll();

  setStatusLedOff(LED_RUNNING);
  setStatusLedOff(LED_WARNING);
  setStatusLedOff(LED_ALERT);
  setStatusLedOff(LED_COMS);

#ifdef OIL_SENSOR_OPST
  pinMode(PIN_OPST, INPUT);
#endif

#ifdef USE_I2C_BARO
  LPS_dev.begin();
  if ((LPS_Sensor.begin() != LPS25HB_STATUS_OK) ||
      (LPS_Sensor.SetODR(7.0f) != LPS25HB_STATUS_OK) ||
      (LPS_Sensor.Enable() != LPS25HB_STATUS_OK))
  {
    digitalWrite(LED_ALERT, HIGH);
  }
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
  pinBaro = PC5;
  pinMAP = PA6;
  pinFuelPressure = PB0;

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
  if (preloadCommandPending || BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ))
  {
    caponordPreloadRunCanBridge();
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ))
  {
    if (Serial.available() > 0)
    {
      digitalToggle(LED_COMS);
    }
    else
    {
      digitalWrite(LED_COMS, LOW);
    }

    digitalWrite(LED_ALERT, currentStatus.engineProtect.isActive());
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ))
  {
#ifdef USE_I2C_BARO
    float pressure = 0.0f;
    float temperature = 0.0f;
    if ((LPS_Sensor.GetPressure(&pressure) == LPS25HB_STATUS_OK) &&
        (LPS_Sensor.GetTemperature(&temperature) == LPS25HB_STATUS_OK))
    {
      currentStatus.fuelTemp = static_cast<int8_t>(temperature);
      currentStatus.baroADC = static_cast<uint16_t>(pressure / 10.0f);
    }
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

#else

void setupBoard()
{
  initialiseAll();
}

void runLoop()
{
}

#endif
