#ifndef OPF_CORE_H
#define OPF_CORE_H

#include <stdint.h>

#define CAPONORD_BOARD 1

#if defined(CAPONORD_BOARD)
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
void caponordResetPins();
void caponordSetPins();

static constexpr uint8_t CAPONORD_TS_PRELOAD_WRITE_COMMAND = 0x50U;

struct CAN_message_t;

bool caponordPreloadHandleSerialCommand(uint8_t command, uint16_t value, uint8_t slot);
bool caponordPreloadHandleCanFrame(const CAN_message_t &msg);
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
#endif

#endif // OPF_CORE_H
