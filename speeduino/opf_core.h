#ifndef OPF_CORE_H
#define OPF_CORE_H

#include <stdint.h>

#if defined(CAPONORD_BOARD)
// Optional knock-window GPIO output. The pin is build-time selected to avoid
// EEPROM/page layout churn and keep the ignition timing path lean.
#ifndef CAPONORD_KNOCK_WINDOW_OUTPUT_ENABLED
#define CAPONORD_KNOCK_WINDOW_OUTPUT_ENABLED 0
#endif

#ifndef CAPONORD_KNOCK_WINDOW_OUTPUT_PIN
#define CAPONORD_KNOCK_WINDOW_OUTPUT_PIN PG8
#endif

#ifndef CAPONORD_KNOCK_WINDOW_OUTPUT_ACTIVE_LOW
#define CAPONORD_KNOCK_WINDOW_OUTPUT_ACTIVE_LOW 0
#endif

#if CAPONORD_KNOCK_WINDOW_OUTPUT_ENABLED
#define KNOCK_WINDOW_OUTPUT_PIN CAPONORD_KNOCK_WINDOW_OUTPUT_PIN
#if CAPONORD_KNOCK_WINDOW_OUTPUT_ACTIVE_LOW
#define KNOCK_WINDOW_OUTPUT_ACTIVE_LOW
#endif
#endif
#endif

void setupBoard();
void runLoop();

#ifdef CAPONORD_BOARD
static constexpr uint8_t CAPONORD_PIN_MAPPING = 60U;

/**
 * @brief Stage reached by the LPS25HB I2C barometer, published to TS as capoBaroStage.
 *
 * Anything other than BARO_STAGE_RUNNING identifies the exact gate that rejected
 * the sensor, so a failure can be diagnosed from the TS gauge without a serial log.
 */
enum : uint8_t {
  BARO_STAGE_UNINITIALISED           = 0U,  ///< updateI2CBaro() has never completed a cycle
  BARO_STAGE_RUNNING                 = 1U,  ///< Last cycle published a valid pressure
  BARO_STAGE_ID_READ_FAILED          = 2U,  ///< No I2C response: wiring, address or bus fault
  BARO_STAGE_ID_MISMATCH             = 3U,  ///< Device answered but WHO_AM_I is not 0xBD
  BARO_STAGE_RESET_WRITE_FAILED      = 4U,  ///< CTRL_REG2 reset write was not acknowledged
  BARO_STAGE_RESET_TIMEOUT           = 5U,  ///< SWRESET|BOOT never self-cleared
  BARO_STAGE_ID_LOST_AFTER_RESET     = 6U,  ///< Device stopped answering after the reset
  BARO_STAGE_OFFSET_WRITE_FAILED     = 7U,  ///< REF_P/RPDS could not be zeroed
  BARO_STAGE_BEGIN_FAILED            = 8U,  ///< ST driver begin() failed
  BARO_STAGE_CTRL_READ_FAILED        = 9U,  ///< CTRL_REG1/2 read-modify-write read failed
  BARO_STAGE_CTRL_WRITE_FAILED       = 10U, ///< CTRL_REG1/2 write failed
  BARO_STAGE_OFFSET_VERIFY_FAILED    = 11U, ///< REF_P/RPDS did not read back as zero
  BARO_STAGE_CTRL_VERIFY_FAILED      = 12U, ///< DIFF_EN/RESET_AZ/AUTO_ZERO still set
  BARO_STAGE_ODR_FAILED              = 13U, ///< SetODR() failed
  BARO_STAGE_ENABLE_FAILED           = 14U, ///< Enable() failed
  BARO_STAGE_NO_DATA_READY           = 15U, ///< STATUS_REG never reported P_DA|T_DA
  BARO_STAGE_PRESSURE_READ_FAILED    = 16U, ///< PRESS_OUT transfer failed
  BARO_STAGE_TEMPERATURE_READ_FAILED = 17U, ///< TEMP_OUT transfer failed
  BARO_STAGE_PRESSURE_OUT_OF_RANGE   = 18U, ///< Sample outside the physically plausible band
};

void caponordResetPins();
void caponordSetPins();

/** TS "w" sub-command used to drive the shock absorber preload controller */
static constexpr uint8_t CAPONORD_TS_PRELOAD_WRITE_COMMAND = 0x50U;
/** TS "w" sub-command used for the EMP coolant pump service controls */
static constexpr uint8_t CAPONORD_TS_EMP_PUMP_WRITE_COMMAND = 0x51U;

struct CAN_message_t;

bool caponordPreloadHandleSerialCommand(uint8_t command, uint16_t value, uint8_t slot);
bool caponordPreloadHandleCanFrame(const CAN_message_t &msg);
bool caponordHandleCanFrame(const CAN_message_t &msg);
bool caponordEmpPumpHandleSerialCommand(uint8_t command, uint16_t rpm, uint8_t durationSeconds);
uint8_t caponordPreloadGetCommand();
uint8_t caponordPreloadGetTarget();
uint8_t caponordPreloadGetSlot();
uint8_t caponordPreloadGetPosition();
uint8_t caponordPreloadGetControllerTarget();
uint8_t caponordPreloadGetState();
uint16_t caponordPreloadGetAlarms();
uint16_t caponordPreloadGetCurrent();
uint16_t caponordPreloadGetAgeMs();
uint8_t caponordPreloadGetLastCommand();
uint8_t caponordPreloadGetCanFlags();
uint8_t caponordPreloadGetAnyRxCount();
uint16_t caponordPreloadGetLastRxId();
uint8_t caponordPreloadGetLastRxDlc();
uint8_t caponordPreloadGetLastRxByte0();
uint8_t caponordPreloadGetPresetSeenMask();
uint8_t caponordPreloadGetPresetValue(uint8_t slot);
uint8_t caponordEmpPumpGetState();
uint8_t caponordEmpPumpGetCapabilities();
uint16_t caponordEmpPumpGetFaults();
uint16_t caponordEmpPumpGetTargetRpm();
uint16_t caponordEmpPumpGetActualRpm();
uint8_t caponordEmpPumpGetActualPercentRaw();
uint8_t caponordEmpPumpGetControllerStatus();
uint8_t caponordEmpPumpGetStatusSummary();
uint16_t caponordEmpPumpGetVoltageRaw();
uint16_t caponordEmpPumpGetCurrentRaw();
uint16_t caponordEmpPumpGetPowerWatts();
uint16_t caponordEmpPumpGetExternalTemperatureRaw();
uint16_t caponordEmpPumpGetMainStatusAgeMs();
uint16_t caponordEmpPumpGetAfterRunRemainingSeconds();
uint8_t caponordEmpPumpGetTransmitFailureCount();
uint8_t caponordEmpPumpGetLastCommandControl();
uint8_t caponordEmpPumpGetThermalState();
uint8_t caponordEmpPumpGetControlFlags();
int16_t caponordEmpPumpGetTargetTemperature();
int16_t caponordEmpPumpGetFilteredIat();
int8_t caponordEmpPumpGetTemperatureError();
int16_t caponordEmpPumpGetCoolantSlopePerMinute();
uint16_t caponordEmpPumpGetMinimumFlowRpm();
int16_t caponordEmpPumpGetFeedForwardRpm();
int16_t caponordEmpPumpGetPiCorrectionRpm();
uint8_t caponordEmpPumpGetCoolingDemandRaw();
uint16_t caponordEmpPumpGetSaturationSeconds();
#endif

#endif // OPF_CORE_H
