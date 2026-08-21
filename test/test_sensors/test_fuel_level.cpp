#include <unity.h>

#include "../../speeduino/fuel_level.h"

static void test_fuel_level_calibration_boundaries(void)
{
  TEST_ASSERT_EQUAL_UINT8(25U, fuelLevelLitresFromAdc(0U));
  TEST_ASSERT_EQUAL_UINT8(25U, fuelLevelLitresFromAdc(17U));
  TEST_ASSERT_EQUAL_UINT8(24U, fuelLevelLitresFromAdc(18U));
  TEST_ASSERT_EQUAL_UINT8(14U, fuelLevelLitresFromAdc(90U));
  TEST_ASSERT_EQUAL_UINT8(14U, fuelLevelLitresFromAdc(103U));
  TEST_ASSERT_EQUAL_UINT8(13U, fuelLevelLitresFromAdc(104U));
  TEST_ASSERT_EQUAL_UINT8(1U, fuelLevelLitresFromAdc(208U));
  TEST_ASSERT_EQUAL_UINT8(0U, fuelLevelLitresFromAdc(209U));
  TEST_ASSERT_EQUAL_UINT8(0U, fuelLevelLitresFromAdc(254U));
  TEST_ASSERT_EQUAL_UINT8(0U, fuelLevelLitresFromAdc(1023U));
}

static void test_fuel_level_filter_rejects_short_spikes(void)
{
  FuelLevelFilter filter;
  TEST_ASSERT_EQUAL_UINT8(25U, filter.update(9U));

  for(uint8_t count = 0U; count < 4U; count++)
  {
    TEST_ASSERT_EQUAL_UINT8(25U, filter.update(966U));
  }

  TEST_ASSERT_EQUAL_UINT16(9U, filter.filteredAdc());
}

static void test_fuel_level_filter_smooths_sustained_changes(void)
{
  FuelLevelFilter filter;
  filter.update(9U);

  for(uint8_t count = 0U; count < 4U; count++) { filter.update(100U); }
  TEST_ASSERT_EQUAL_UINT16(9U, filter.filteredAdc());

  filter.update(100U);
  TEST_ASSERT_EQUAL_UINT16(10U, filter.filteredAdc());

  for(uint16_t count = 0U; count < 400U; count++) { filter.update(100U); }
  TEST_ASSERT_UINT16_WITHIN(1U, 100U, filter.filteredAdc());
  TEST_ASSERT_EQUAL_UINT8(14U, filter.update(100U));
}

void test_fuel_level(void)
{
  RUN_TEST(test_fuel_level_calibration_boundaries);
  RUN_TEST(test_fuel_level_filter_rejects_short_spikes);
  RUN_TEST(test_fuel_level_filter_smooths_sustained_changes);
}
