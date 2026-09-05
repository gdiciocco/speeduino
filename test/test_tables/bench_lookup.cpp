#include <unity.h>
#include "table3d.h"
#include "../test_utils.h"
#include "table3d_test_support.h"

/** @file
 * @brief Timing for the 3D table lookup, on the target rather than in theory.
 *
 * table3d_interpolate.cpp carries an AVR-era note: "on AVR it's much quicker to
 * increment and compare 8-bit indices (and dereference pointers) than to
 * increment and compare 16-bit pointers! Up to 80 loop/sec!". On Cortex-M the
 * argument inverts - every 8-bit operation needs a UXTB - so the question is
 * whether the inversion is worth anything, and that is a measurement, not an
 * opinion.
 *
 * Lookups are the hot path: every fuel and ignition event does several.
 *
 * @warning These figures are the *uncached* path. get3DTableValue() memoises
 * its last input and output, and that memoisation is compiled out under
 * UNIT_TEST - so a test build cannot exercise what the firmware actually does
 * on a repeated lookup, which is to return immediately. Read these numbers as
 * the cost of a cache miss, not as the cost of a lookup in the running engine.
 *
 * What the numbers are good for is comparing two builds of the same code under
 * identical conditions, which is what settled the 8-bit index question:
 *
 *   loop only ................. 147 ns   (the harness, subtract it)
 *   moving lookups ........... 3321 ns   (bin search runs)
 *   same point every time .... 2900 ns   (find_bin_max hits its cached bin)
 *
 * So the bin search is about 13% of a lookup even with deliberately
 * cache-hostile inputs, and the interpolation is the other 87% (~460 cycles at
 * 168MHz). Widening the search index to native width made the whole thing
 * ~1% *slower*, not faster. There is nothing to win there, and the reason is
 * that the search was never where the time goes.
 */

#if !defined(NATIVE_BOARD)

static constexpr uint32_t LOOKUP_ITERATIONS = 20000U;

//The same index arithmetic with no lookup, so the figure below is the lookup
//and not the harness. Without this the number is an upper bound of unknown
//tightness, which is not a measurement.
static void bench_loop_overhead(void)
{
  uint32_t checksum = 0U;
  const uint32_t start = micros();
  for (uint32_t i = 0U; i < LOOKUP_ITERATIONS; ++i)
  {
    const uint16_t rpm = (uint16_t)(((i * 37U) % 7000U) + 200U);
    const uint8_t load = (uint8_t)((i * 13U) % 100U);
    checksum += rpm + load;
  }
  const uint32_t elapsed = micros() - start;

  char message[96];
  sprintf(message, "loop only: %" PRIu32 " iterations in %" PRIu32 " us = %" PRIu32 " ns each",
          LOOKUP_ITERATIONS, elapsed, (elapsed * 1000U) / LOOKUP_ITERATIONS);
  TEST_MESSAGE(message);
  TEST_ASSERT_NOT_EQUAL(0U, checksum);
}

static void bench_table_lookup(void)
{
  table3d8RpmLoad testTable = getDummyTable();

  //A spread of points that lands in different bins, so the linear bin search
  //runs a different number of iterations each time rather than always hitting
  //the cached bin. The multipliers are coprime with the table size on purpose.
  uint32_t checksum = 0U;
  const uint32_t start = micros();
  for (uint32_t i = 0U; i < LOOKUP_ITERATIONS; ++i)
  {
    const uint16_t rpm = (uint16_t)(((i * 37U) % 7000U) + 200U);
    const uint8_t load = (uint8_t)((i * 13U) % 100U);
    checksum += get3DTableValue(&testTable, load, rpm);
  }
  const uint32_t elapsed = micros() - start;

  char message[96];
  sprintf(message, "%" PRIu32 " lookups in %" PRIu32 " us = %" PRIu32 " ns each",
          LOOKUP_ITERATIONS, elapsed, (elapsed * 1000U) / LOOKUP_ITERATIONS);
  TEST_MESSAGE(message);

  //The checksum only exists so the compiler cannot delete the loop.
  TEST_ASSERT_NOT_EQUAL(0U, checksum);
}

//Always the same point, so find_bin_max() hits its cached bin and the linear
//search never runs. The difference from the moving benchmark is what the bin
//search costs - and the bin search is the only part an index type can touch.
static void bench_table_lookup_cached_bin(void)
{
  table3d8RpmLoad testTable = getDummyTable();

  uint32_t checksum = 0U;
  const uint32_t start = micros();
  for (uint32_t i = 0U; i < LOOKUP_ITERATIONS; ++i)
  {
    checksum += get3DTableValue(&testTable, 53, 2250);
  }
  const uint32_t elapsed = micros() - start;

  char message[96];
  sprintf(message, "cached bin: %" PRIu32 " lookups in %" PRIu32 " us = %" PRIu32 " ns each",
          LOOKUP_ITERATIONS, elapsed, (elapsed * 1000U) / LOOKUP_ITERATIONS);
  TEST_MESSAGE(message);
  TEST_ASSERT_NOT_EQUAL(0U, checksum);
}

void benchTableLookup(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(bench_loop_overhead);
    RUN_TEST_P(bench_table_lookup);
    RUN_TEST_P(bench_table_lookup_cached_bin);
  }
}

#else

void benchTableLookup(void)
{
  //Host timing says nothing about the target.
}

#endif
