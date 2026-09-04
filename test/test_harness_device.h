#pragma once

#if !defined(NATIVE_BOARD)

#include <unity.h>
#include <Arduino.h>
#include "board_definition.h"

/** @brief Escape sequence that puts the board back into its bootloader.
 *
 * A test binary does not run the TunerStudio comms loop, so the usual way of
 * reaching the STM32 system bootloader - command button 12801 - is not
 * available once a test image is flashed. On a board whose only connection is
 * USB, that makes the first test upload a one-way door: the next upload has
 * nothing left to ask.
 *
 * So the harness answers for it. Sending "@BOOTLOADER" on the test port at any
 * point after reset jumps to the bootloader, which is what tools/dfu_upload.py
 * falls back to when the firmware does not respond.
 */
static const char HW_TEST_BOOT_MAGIC[] = "@BOOTLOADER";

/** @brief How long after the tests finish before the board parks itself in DFU */
static const uint32_t HW_TEST_PARK_IN_DFU_MS = 4000U;

static inline void hwTestPollForBootloader(void)
{
    static uint8_t matched = 0U;

    while (Serial.available() > 0)
    {
        const char c = (char)Serial.read();
        if (c == HW_TEST_BOOT_MAGIC[matched])
        {
            ++matched;
        }
        else
        {
            matched = (c == HW_TEST_BOOT_MAGIC[0]) ? 1U : 0U;
        }

        if (HW_TEST_BOOT_MAGIC[matched] == '\0')
        {
            matched = 0U;
            Serial.println(F("ENTERING BOOTLOADER"));
            Serial.flush();
            delay(50);
            jumpToBootloader();
        }
    }
}

/** @brief Blink, and stay reachable, without blocking the escape hatch. */
static inline void hwTestIdle(uint16_t millisToWait)
{
    const uint32_t until = millis() + millisToWait;
    while ((int32_t)(millis() - until) < 0)
    {
        hwTestPollForBootloader();
    }
}

void setup(void (*runAllTests)(void))
{
    pinMode(LED_BUILTIN, OUTPUT);

    // Wait for Serial Monitor connection
    // Note: waiting on !Serial doesn't work on STM32
    hwTestIdle(5000U);
    while (!Serial) {
        ; // Wait for serial connection
    }

    UNITY_BEGIN();    // IMPORTANT LINE!

    runAllTests();

    // A small delay here helps STM32
    delay(500);

    UNITY_END(); // stop unit testing

}

#define TEST_HARNESS(testRunner) \
void setup() \
{ \
    setup(testRunner); \
}

void loop()
{
    // UNITY_END() ends up in PlatformIO's unittest_uart_end(), which calls
    // Serial.end(). On a USB CDC board that does not merely stop printing: it
    // takes the device off the bus, and on stm32duino it does not come back.
    // So once the tests are over there is no serial left to ask anything of,
    // and the escape sequence above can never be delivered.
    //
    // Park in the bootloader instead. The board ends every test run sitting in
    // DFU, which is exactly where the next upload wants it - no handshake, no
    // race, nothing to get wrong. dfu_upload.py checks for that first and goes
    // straight to flashing.
    //
    // The delay is to let the runner finish reading the results off the port
    // before the device disappears for good.
    static uint32_t testsFinishedAt = millis();
    if ((millis() - testsFinishedAt) > HW_TEST_PARK_IN_DFU_MS)
    {
        jumpToBootloader();
    }

    // Blink until then, and stay responsive to the escape sequence in case the
    // host catches us before the park.
    digitalWrite(LED_BUILTIN, HIGH);
    hwTestIdle(125U);
    digitalWrite(LED_BUILTIN, LOW);
    hwTestIdle(125U);
}

#endif
