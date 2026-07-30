#include <unity.h>

#include "opst_sensor_math.h"

static void test_temperature_decode_uses_hella_offset(void)
{
  TEST_ASSERT_EQUAL_INT16(-40, decodeOPStTemperatureC(4096U, 128U));
  TEST_ASSERT_EQUAL_INT16(20, decodeOPStTemperatureC(4096U, 1280U));
  TEST_ASSERT_EQUAL_INT16(160, decodeOPStTemperatureC(4096U, 3968U));
}

static void test_absolute_pressure_decode_uses_hella_scale(void)
{
  TEST_ASSERT_EQUAL_UINT16(50U, decodeOPStAbsolutePressureKpa(4096U, 128U));
  TEST_ASSERT_EQUAL_UINT16(100U, decodeOPStAbsolutePressureKpa(4096U, 320U));
  TEST_ASSERT_EQUAL_UINT16(1050U, decodeOPStAbsolutePressureKpa(4096U, 3968U));
}

static void test_decode_compensates_sensor_oscillator_period(void)
{
  TEST_ASSERT_EQUAL_INT16(20, decodeOPStTemperatureC(4506U, 1408U));
  TEST_ASSERT_EQUAL_UINT16(100U, decodeOPStAbsolutePressureKpa(4506U, 352U));
  TEST_ASSERT_EQUAL_UINT8(64U, decodeOPStDiagnostic(1126U, 282U));
}

static void test_pressure_is_zeroed_against_barometric_pressure(void)
{
  TEST_ASSERT_EQUAL_UINT8(0U, convertOPStGaugePressureToPsi(100U, 100U));
  TEST_ASSERT_EQUAL_UINT8(3U, convertOPStGaugePressureToPsi(120U, 100U));
  TEST_ASSERT_EQUAL_UINT8(15U, convertOPStGaugePressureToPsi(200U, 100U));
}

void test_opst_sensor_math(void)
{
  RUN_TEST(test_temperature_decode_uses_hella_offset);
  RUN_TEST(test_absolute_pressure_decode_uses_hella_scale);
  RUN_TEST(test_decode_compensates_sensor_oscillator_period);
  RUN_TEST(test_pressure_is_zeroed_against_barometric_pressure);
}
