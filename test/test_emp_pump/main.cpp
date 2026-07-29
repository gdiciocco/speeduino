#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testEmpPump(void);
    testEmpPump();
}

TEST_HARNESS(runAllTests)
