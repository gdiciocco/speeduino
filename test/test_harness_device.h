#pragma once

#if !defined(NATIVE_BOARD)

#include <unity.h>
#include <Arduino.h>

void setup(void (*runAllTests)(void))
{
    pinMode(LED_BUILTIN, OUTPUT);

    // Wait for Serial Monitor connection
    // Note: waiting on !Serial doesn't work on STM32
    delay(5000);
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
    // Blink to indicate end of test
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}

#endif
