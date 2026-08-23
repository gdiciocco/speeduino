#include <unity.h>

#include "globals.h"
#include "storage.h"
#include "vehicle_distance.h"

static void setupVehicleDistance(uint32_t totalDeciKm = 0U, uint32_t tripDeciKm = 0U)
{
    configPage15.vehicleDistanceSource = VEHICLE_DISTANCE_SOURCE_VSS;
    configPage15.gpsSpeedAuxChannel = 0U;
    configPage15.vehicleOdometerDeciKm = totalDeciKm;
    configPage15.vehicleTripDeciKm = tripDeciKm;
    setEepromWritePending(false);
    initialiseVehicleDistance();
    setEepromWritePending(false);
}

static void test_vehicle_distance_integrates_speed_without_rounding_loss(void)
{
    setupVehicleDistance(1234U, 50U);

    //100 km/h for 3.6 seconds is exactly 0.1 km.
    for(uint8_t sample = 0U; sample < 4U; sample++) {
        accumulateVehicleDistance(100U, 900U);
    }

    TEST_ASSERT_EQUAL_UINT32(1235U, getVehicleOdometerDeciKm());
    TEST_ASSERT_EQUAL_UINT32(51U, getVehicleTripDeciKm());
}

static void test_vehicle_distance_selects_vss_or_gps_aux(void)
{
    setupVehicleDistance();
    currentStatus.vss = 47U;
    currentStatus.canin[6] = 83U;

    configPage15.vehicleDistanceSource = VEHICLE_DISTANCE_SOURCE_VSS;
    TEST_ASSERT_EQUAL_UINT16(47U, getVehicleDistanceSpeedKph());

    configPage15.vehicleDistanceSource = VEHICLE_DISTANCE_SOURCE_GPS;
    configPage15.gpsSpeedAuxChannel = 6U;
    TEST_ASSERT_EQUAL_UINT16(83U, getVehicleDistanceSpeedKph());
}

static void test_vehicle_trip_reset_preserves_total(void)
{
    setupVehicleDistance(900U, 120U);

    resetVehicleTrip();

    TEST_ASSERT_EQUAL_UINT32(900U, getVehicleOdometerDeciKm());
    TEST_ASSERT_EQUAL_UINT32(0U, getVehicleTripDeciKm());
    TEST_ASSERT_TRUE(isEepromWritePending());
}

static void test_vehicle_distance_checkpoints_each_kilometre(void)
{
    setupVehicleDistance();

    //100 km/h for 36 seconds is exactly 1 km.
    for(uint8_t second = 0U; second < 36U; second++) {
        accumulateVehicleDistance(100U, 1000U);
    }

    TEST_ASSERT_EQUAL_UINT32(10U, getVehicleOdometerDeciKm());
    TEST_ASSERT_TRUE(isEepromWritePending());
}

void test_vehicle_distance(void)
{
    RUN_TEST(test_vehicle_distance_integrates_speed_without_rounding_loss);
    RUN_TEST(test_vehicle_distance_selects_vss_or_gps_aux);
    RUN_TEST(test_vehicle_trip_reset_preserves_total);
    RUN_TEST(test_vehicle_distance_checkpoints_each_kilometre);
}
