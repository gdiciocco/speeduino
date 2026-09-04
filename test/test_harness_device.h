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
    // Blink to indicate end of test, while staying responsive to the
    // bootloader escape sequence.
    digitalWrite(LED_BUILTIN, HIGH);
    hwTestIdle(250U);
    digitalWrite(LED_BUILTIN, LOW);
    hwTestIdle(250U);
}

#endif
