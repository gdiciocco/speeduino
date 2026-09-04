#pragma once

/** @file
 * @brief Watchdog for tests running on real hardware.
 *
 * A test that hangs on a board does more than fail: the MCU stops servicing
 * USB, so there is no CDC port and no DFU device, and the only way back is
 * BOOT0 and a reset by hand. That happened twice with test_pages before this
 * existed.
 *
 * So the independent watchdog is armed for the duration of a suite and fed
 * once per test. A hang inside any single test stops the feeding and the board
 * resets itself back into the window tools/dfu_upload.py can catch.
 *
 * Per test rather than per suite: the longest suites run well past the ~33s
 * the F4's IWDG can be stretched to, while no individual test comes close.
 * Nothing feeds it after the last test either, so the board returns to that
 * window on its own once a run is over.
 */

#if !defined(NATIVE_BOARD) && defined(STM32_CORE_VERSION_MAJOR)

#include <IWatchdog.h>
#define HW_TEST_WATCHDOG_AVAILABLE

/** @brief Arm the watchdog on first use, then feed it.
 *
 * Arming lazily keeps it running only while a suite is executing: after a
 * reset nothing has armed it yet, so the board can sit in the pre-test wait
 * indefinitely without being reset out of it.
 */
static inline void hwTestFeedWatchdog(void)
{
    static bool armed = false;
    if (armed)
    {
        IWatchdog.reload();
    }
    else
    {
        IWatchdog.begin(IWDG_TIMEOUT_MAX);   // ~33s on the F4's 32kHz LSI
        armed = true;
    }
}

#define HW_TEST_FEED_WATCHDOG() hwTestFeedWatchdog()

#else

#define HW_TEST_FEED_WATCHDOG() ((void)0)

#endif
