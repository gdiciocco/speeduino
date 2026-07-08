#include "fastOutputPin.h"
#include "../../atomic.h"
#include "../../board_definition.h"

// LCOV_EXCL_START
// Exclude low level pin manipulation from coverage as it's not testable in a meaningful way

#if defined(CORE_STM32)

// On STM32 the port pointer is aimed at the GPIO BSRR register rather than ODR. Writing the pin mask
// to the lower 16 bits of BSRR sets the pin; writing it to the upper 16 bits resets it. Both are
// single-store atomic operations, so no ATOMIC() interrupt masking is needed (|=/&= on ODR is a
// non-atomic read-modify-write on ARM).

void fastOutputPin_t::setPin(uint8_t pin, uint8_t mode)
{
    if (pin!=NOT_A_PIN)
    {
        pinMode(pin, mode);
        _port_pin.port = &(digitalPinToPort(pin)->BSRR);
        _port_pin.mask = digitalPinToBitMask(pin);
    }
}

/** @brief Set the pin high */
void fastOutputPin_t::setPinHigh(void)
{
    if (isValid())
    {
        *_port_pin.port = _port_pin.mask;
    }
}

/** @brief Set the pin low */
void fastOutputPin_t::setPinLow(void)
{
    if (isValid())
    {
        *_port_pin.port = (_port_pin.mask << 16U);
    }
}

#else

void fastOutputPin_t::setPin(uint8_t pin, uint8_t mode)
{
    if (pin!=NOT_A_PIN)
    {
        pinMode(pin, mode);
        _port_pin.port = portOutputRegister(digitalPinToPort(pin));
        _port_pin.mask = digitalPinToBitMask(pin);
    }
}

/** @brief Set the pin high */
void fastOutputPin_t::setPinHigh(void)
{
    if (isValid())
    {
        ATOMIC() { *_port_pin.port |= _port_pin.mask; }
    }
}

/** @brief Set the pin low */
void fastOutputPin_t::setPinLow(void)
{
    if (isValid())
    {
        ATOMIC() { *_port_pin.port &= ~_port_pin.mask; }
    }
}

#endif

// LCOV_EXCL_STOP
