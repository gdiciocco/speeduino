#pragma once

/**
 * @file
 * @brief Rollover-safe elapsed time calculations for free-running counters
 *
 * Timestamps taken from free-running counters such as micros(), millis() or
 * runSecsX10 wrap back to zero when the counter overflows (micros() every
 * ~71.6 minutes). Comparing absolute timestamps, E.g.
 * `micros() > startTime + interval`, breaks when either the addition
 * overflows or the counter wraps between the two samples. Unsigned modular
 * subtraction is exact across a single wrap, so elapsed-time based
 * comparisons remain correct provided the real elapsed time is less than one
 * full counter period (I.e. the check runs at least once per counter period).
 *
 * All functions take @c now as a parameter rather than calling micros()
 * internally so that callers can be unit tested with arbitrary counter
 * values, including values either side of the wrap point.
 */

#include <stdint.h>

/**
 * @brief Compute the number of counter ticks elapsed from start to now.
 *
 * Correct across counter rollover thanks to unsigned modular arithmetic,
 * as long as the real elapsed time is less than one full counter period.
 *
 * @param now The current counter reading (E.g. micros())
 * @param start The counter reading taken when the interval started
 * @return The number of ticks elapsed between the two readings
 */
static inline uint32_t timeElapsed(uint32_t now, uint32_t start) {
  return now - start;
}

/**
 * @brief Check whether at least @p interval counter ticks have elapsed since @p start.
 *
 * Rollover-safe replacement for `now >= start + interval` style comparisons.
 * The boundary is inclusive: an elapsed time exactly equal to @p interval
 * counts as elapsed.
 *
 * @param now The current counter reading (E.g. micros())
 * @param start The counter reading taken when the interval started
 * @param interval The minimum number of ticks that must have elapsed
 * @return true if at least @p interval ticks have elapsed since @p start
 */
static inline bool hasIntervalElapsed(uint32_t now, uint32_t start, uint32_t interval) {
  return timeElapsed(now, start) >= interval;
}

/**
 * @brief As hasIntervalElapsed(), but also returns false when @p start is *after* @p now.
 *
 * hasIntervalElapsed() assumes @p start was taken before @p now. When that does not
 * hold - E.g. @p start is written by an ISR that preempts the code which sampled
 * @p now - the modular subtraction underflows to a value near UINT32_MAX and every
 * interval looks elapsed. Comparing as signed keeps the rollover safety (the
 * subtraction is still modular) while treating a start in the future as "not yet
 * elapsed": the usable range becomes +/-INT32_MAX ticks either side of @p start,
 * I.e. ~35 minutes for a microsecond counter.
 *
 * @param now The current counter reading (E.g. micros())
 * @param start The counter reading taken when the interval started
 * @param interval The minimum number of ticks that must have elapsed. Must be <= INT32_MAX
 * @return true if at least @p interval ticks have elapsed since @p start; false if
 *         the interval has not elapsed or @p start is in the future
 */
static inline bool hasIntervalElapsedSigned(uint32_t now, uint32_t start, uint32_t interval) {
  return (int32_t)timeElapsed(now, start) >= (int32_t)interval;
}
