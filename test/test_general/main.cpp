#include "../test_harness_device.h"
#include "../test_harness_native.h"


void runAllTests(void)
{
    extern void testPinMapping(void);
    extern void testResetControl(void);
    extern void testTSCommandHandler(void);
    extern void testLogEntryWidth(void);

    testPinMapping();
    testResetControl();
    testTSCommandHandler();
    testLogEntryWidth();
}

TEST_HARNESS(runAllTests)
