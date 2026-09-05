#pragma once

#include <stdint.h>

/** @file
 * @brief User-assignable pin functions.
 *
 * Every one of these used to be a 6-bit (some 4- or 5-bit) field wedged into a
 * spare corner of whichever config page its feature happened to live on. That
 * was an ATmega2560 economy: the board had 70 I/O pins and 4 kB of EEPROM, so
 * six bits covered the useful range and every bit was worth saving.
 *
 * It does not cover an F407ZG, which has 75 digital pins, or an FCR Micro F4,
 * which has 100. Anything above 63 simply could not be selected. Worse, each
 * field shared its byte with unrelated flags, so widening one meant moving the
 * layout of the page it sat in.
 *
 * They now live together in one page, one byte each, addressed by function.
 * Adding an assignable pin is an entry in this enum plus a byte of the
 * reserved space - no page moves, no bit arithmetic, and the whole table is
 * contiguous, which is what a runtime pin-assignment editor would want to walk.
 *
 * @note A value of PIN_ASSIGNMENT_BOARD_DEFAULT means "leave whatever
 * setPinMapping() chose for this board".
 */

/** @brief Reserved value meaning "use the board map's choice" */
static constexpr uint8_t PIN_ASSIGNMENT_BOARD_DEFAULT = 0U;

/** @brief Index into config_pins::pin
 *
 * @warning Append only. The index is the storage location and the TunerStudio
 * offset; reordering silently reassigns every pin in an existing tune.
 */
enum PinAssignment : uint8_t
{
    PIN_ASSIGN_TACHO = 0U,
    PIN_ASSIGN_IDLE_UP,
    PIN_ASSIGN_IDLE_UP_OUTPUT,
    PIN_ASSIGN_CTPS,
    PIN_ASSIGN_VSS,
    PIN_ASSIGN_FUEL_PUMP,
    PIN_ASSIGN_RESET_CONTROL,
    PIN_ASSIGN_IGN_BYPASS,
    PIN_ASSIGN_VVT1,
    PIN_ASSIGN_BOOST,
    PIN_ASSIGN_LAUNCH,
    PIN_ASSIGN_FAN,
    PIN_ASSIGN_BARO,            ///< Analog channel index, not a digital pin
    PIN_ASSIGN_EMAP,            ///< Analog channel index
    PIN_ASSIGN_N2O_ARM,
    PIN_ASSIGN_N2O_STAGE1,
    PIN_ASSIGN_N2O_STAGE2,
    PIN_ASSIGN_KNOCK,           ///< Digital pin, or analog channel + KNOCK_PIN_ANALOG_BASE
    PIN_ASSIGN_FUEL2_INPUT,
    PIN_ASSIGN_SPARK2_INPUT,
    PIN_ASSIGN_OIL_PRESSURE,    ///< Analog channel index
    PIN_ASSIGN_FUEL_PRESSURE,   ///< Analog channel index
    PIN_ASSIGN_WMI_INDICATOR,
    PIN_ASSIGN_WMI_EMPTY,
    PIN_ASSIGN_WMI_ENABLED,
    PIN_ASSIGN_VVT2,
    PIN_ASSIGN_SD_ENABLE,
    PIN_ASSIGN_AIRCON_COMP,
    PIN_ASSIGN_AIRCON_REQUEST,
    PIN_ASSIGN_AIRCON_FAN,

    PIN_ASSIGN_COUNT
};

/** @brief Total size of the pin assignment page, including room to grow.
 *
 * Reserved bytes are zero, which reads as "board default", so new functions
 * can be added without disturbing anything already stored.
 */
static constexpr uint8_t PIN_ASSIGNMENT_PAGE_SIZE = 48U;

static_assert(PIN_ASSIGN_COUNT <= PIN_ASSIGNMENT_PAGE_SIZE,
              "More pin functions than the page has room for");

/** @brief Knock can name either a digital pin or an analog channel.
 *
 * Values at or above this are (channel + this); below it they are digital pin
 * numbers. Inherited from the old 6-bit field, where A0 landed at 47.
 */
static constexpr uint8_t KNOCK_PIN_ANALOG_BASE = 47U;
