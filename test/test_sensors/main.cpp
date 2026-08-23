#include "../test_harness_device.h"
#include "../test_harness_native.h"


void runAllSensorTests(void)
{
    extern void test_fastMap10Bit(void);
    extern void test_fuel_level(void);
    extern void test_map_sampling(void);
    extern void test_opst_sensor_math(void);
    extern void test_vehicle_distance(void);

    test_fastMap10Bit();
    test_fuel_level();
    test_map_sampling();
    test_opst_sensor_math();
    test_vehicle_distance();
}

TEST_HARNESS(runAllSensorTests)
