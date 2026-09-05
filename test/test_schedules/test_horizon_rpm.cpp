#include <unity.h>
#include "../test_utils.h"
#include "scheduler.h"
#include "crankMaths.h"
#include "maths.h"

/** @file
 * @brief Where the 16-bit timer horizon actually bites, in RPM.
 *
 * setSchedule() cannot express a delay of MAX_TIMER_PERIOD or more and does
 * not clamp: the injection or the spark does not happen. The horizon is a
 * time, but what a tuner has in front of them is an RPM, so this pins the
 * conversion and fails if it ever moves.
 *
 * The delay until an event is at most one CRANK_ANGLE_MAX period, so the cliff
 * is where that period crosses the horizon. With the usual 4us tick that is
 * 262140us, which is:
 *
 *   360 deg (wasted spark, 2 squirts) ... about 229 RPM
 *   720 deg (sequential, 1 squirt) ..... about 458 RPM
 *
 * 458 RPM is cranking speed on a large twin. This is not a theoretical limit.
 */

static uint16_t findHorizonRpm(uint16_t crankAngleMax)
{
  //Walk down from a comfortable idle until a full CRANK_ANGLE_MAX period no
  //longer fits inside the timer horizon.
  for (uint16_t rpm = 1200U; rpm > 50U; --rpm)
  {
    setAngleConverterRevolutionTime(UDIV_ROUND_CLOSEST(MICROS_PER_MIN, (uint32_t)rpm, uint32_t));
    if (angleToTimeMicroSecPerDegree(crankAngleMax) >= MAX_TIMER_PERIOD)
    {
      return rpm;
    }
  }
  return 0U;
}

static void test_horizon_rpm_wasted_spark(void)
{
  const uint16_t rpm = findHorizonRpm(360U);
  char message[80];
  sprintf(message, "360 deg exceeds the timer horizon below %" PRIu16 " RPM", rpm);
  TEST_MESSAGE(message);
  TEST_ASSERT_UINT16_WITHIN(5U, 229U, rpm);
}

static void test_horizon_rpm_sequential(void)
{
  const uint16_t rpm = findHorizonRpm(720U);
  char message[80];
  sprintf(message, "720 deg exceeds the timer horizon below %" PRIu16 " RPM", rpm);
  TEST_MESSAGE(message);
  TEST_ASSERT_UINT16_WITHIN(5U, 458U, rpm);
}

// The counter only earns its keep if it actually moves when this happens, so
// drive setSchedule() with a delay taken from the sequential cranking case
// rather than with a synthetic number.
static void test_cranking_delay_is_counted_as_dropped(void)
{
  using raw_counter_t = std::remove_reference<Schedule::counter_t>::type;
  using raw_compare_t = std::remove_reference<Schedule::compare_t>::type;
  raw_counter_t counter = { 0 };
  raw_compare_t compare = { 0 };
  Schedule schedule(counter, compare);

  setAngleConverterRevolutionTime(UDIV_ROUND_CLOSEST(MICROS_PER_MIN, 400UL, uint32_t));
  const uint32_t delay = angleToTimeMicroSecPerDegree(720U);
  TEST_ASSERT_GREATER_OR_EQUAL(MAX_TIMER_PERIOD, delay);

  resetScheduleHorizonDropCount();
  setSchedule(schedule, delay, 2000U, true);
  TEST_ASSERT_EQUAL(OFF, schedule._status);
  TEST_ASSERT_EQUAL(1U, getScheduleHorizonDropCount());
}

void testHorizonRpm(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_horizon_rpm_wasted_spark);
    RUN_TEST_P(test_horizon_rpm_sequential);
    RUN_TEST_P(test_cranking_delay_is_counted_as_dropped);
  }
}
