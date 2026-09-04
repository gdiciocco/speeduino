#ifndef MATH_H
#define MATH_H

#include <stdint.h>
#include "unit_testing.h"

uint8_t random1to100(void) noexcept;

/** @brief Self-explanatory */
static constexpr uint32_t MICROS_PER_SEC = UINT32_C(1000000);

/** @brief Self-explanatory */
static constexpr uint32_t MICROS_PER_MIN = MICROS_PER_SEC*60U;

/** @brief Self-explanatory */
static constexpr uint32_t MICROS_PER_HOUR = MICROS_PER_MIN*60U;

/** @brief Self-explanatory */
static constexpr uint32_t MILLI_PER_SEC = MICROS_PER_SEC/1000;

/**
 * @defgroup group-rounded-div Rounding integer division
 * 
 * @brief Integer division returns the quotient. I.e. rounds to zero. This
 * code will round the result to nearest integer. Rounding behavior is
 * controlled by #DIV_ROUND_BEHAVIOR
 * 
 * @{
 */

/**
 * @defgroup group-rounded-div-behavior Rounding behavior
 * @{
 */


/** @brief Rounding behavior: always round down */
#define DIV_ROUND_DOWN -1

/** @brief Rounding behavior: always round up */
#define DIV_ROUND_UP 1

/** @brief Rounding behavior: round to nearest 
 * 
 * This rounds 0.5 away from zero. This is the same behavior
 * as the standard library round() function.
 */
#define DIV_ROUND_NEAREST 0

/** @brief Integer division rounding behavior. */
#define DIV_ROUND_BEHAVIOR DIV_ROUND_NEAREST
// (Unit tests expect DIV_ROUND_NEAREST behavior)

/**
 * @brief Computes the denominator correction for rounding division
 * based on our rounding behavior.
 * 
 * @param d The divisor (an integer)
 * @param t The type of the result. E.g. uint16_t
 */
#define DIV_ROUND_CORRECT(d, t) ((t)(((d)>>1U)+(t)DIV_ROUND_BEHAVIOR))
///@}

/**
 * @brief Rounded integer division
 * 
 * Integer division returns the quotient. I.e. rounds to zero. This
 * macro will round the result to nearest integer. Rounding behavior
 * is controlled by #DIV_ROUND_BEHAVIOR
 * 
 * @warning For performance reasons, this macro does not promote integers.
 * So it will overflow if n>MAX(t)-(d/2).
 * 
 * @param n The numerator (dividee) (an integer)
 * @param d The denominator (divider) (an integer)
 * @param t The type of the result. E.g. uint16_t
 */
#define DIV_ROUND_CLOSEST(n, d, t) ( \
    (((n) < (t)(0)) ^ ((d) < (t)(0))) ? \
        ((t)((n) - DIV_ROUND_CORRECT(d, t))/(t)(d)) : \
        ((t)((n) + DIV_ROUND_CORRECT(d, t))/(t)(d)))

/**
 * @brief Rounded \em unsigned integer division
 * 
 * This is slightly faster than the signed version (DIV_ROUND_CLOSEST(n, d, t))
 * 
 * @warning For performance reasons, this macro does not promote integers.
 * So it will overflow if n>MAX(t)-(d/2).
 * 
 * @param n The numerator (dividee) (an \em unsigned integer)
 * @param d The denominator (divider) (an \em unsigned integer)
 * @param t The type of the result. E.g. uint16_t
 */
#define UDIV_ROUND_CLOSEST(n, d, t) ((t)((n) + DIV_ROUND_CORRECT(d, t))/(t)(d))

/**
 * @brief Rounded \em unsigned integer division optimized for compile time constants
 * 
 * @tparam divisor Divisor
 * @param n Dividend
 * @return uint16_t 
 */
template <uint16_t divisor>
TESTABLE_STATIC_CONSTEXPR uint16_t div_round_closest_u16(uint16_t n) {
    // Compile time version of UDIV_ROUND_CLOSEST. The rounding correction is
    // added at 32 bits, so unlike the macro this cannot wrap for large n.
    return (uint16_t)(((uint32_t)n + DIV_ROUND_CORRECT(divisor, uint32_t)) / (uint32_t)divisor);
}

/** @brief Rounding up \em unsigned integer division */
#define UDIV_ROUND_UP(n, d, t) ((t)((n) + (t)((d)+1U))/(t)(d))

///@}

/** @brief Test whether the parameter is an integer or not. */
#define IS_INTEGER(d) ((d) == (int32_t)(d))

/** 
 * @{
 * @brief Integer division by 100, i.e. same as n/100
 * 
 * Uses the rounding behaviour controlled by @ref DIV_ROUND_BEHAVIOR
 * 
 * @param n Dividend to divide by 100
 * @return n/100, with rounding behavior applied
 */
static inline uint16_t div100(uint16_t n) {
    // The rounding correction is applied at 32 bits. On an 8-bit core that
    // would have cost a wider division, which is why UDIV_ROUND_CLOSEST()
    // does it in the result type and documents that it wraps once
    // n > MAX(t)-d/2. Here the wider type is free, so the limit goes away.
    return (uint16_t)(((uint32_t)n + UINT32_C(50)) / UINT32_C(100));
}

static inline int16_t div100(int16_t n) {
    const int32_t wide = (int32_t)n;
    return (int16_t)((wide < 0 ? wide - INT32_C(50) : wide + INT32_C(50)) / INT32_C(100));
}

static inline uint32_t div100(uint32_t n) {
    // No wider type needed: quotient and remainder cannot overflow, and the
    // compiler folds both into one multiply-high sequence.
    const uint32_t quotient = n / UINT32_C(100);
    return (n % UINT32_C(100)) >= UINT32_C(50) ? quotient + 1U : quotient;
}

static inline int32_t div100(int32_t n) {
    const int32_t quotient = n / INT32_C(100);
    const int32_t remainder = n % INT32_C(100); // Truncates toward zero, so this carries n's sign
    if (remainder >= INT32_C(50))  { return quotient + 1; }
    if (remainder <= INT32_C(-50)) { return quotient - 1; }
    return quotient;
}
///@}

/**
 * @brief Integer division by 360
 * 
 * @param n The numerator (dividee) (an integer)
 * @return uint32_t 
 */
static inline uint32_t div360(uint32_t n) {
    const uint32_t quotient = n / UINT32_C(360);
    return (n % UINT32_C(360)) >= UINT32_C(180) ? quotient + 1U : quotient;
}

/**
 * @brief Rounded arithmetic right shift
 * 
 * Right shifting throws away bits. When use for fixed point division, this
 * effectively rounds down (towards zero). To round-to-the-nearest-integer
 * when right-shifting by S, just add in 2 power b−1 (which is the 
 * fixed-point equivalent of 0.5) first
 *  
 * @tparam b number of bits to shift by
 * @param a value to shift
 * @return uint32_t 
 */
template <uint8_t b> 
static inline uint32_t rshift_round(uint32_t a) { 
    constexpr uint8_t CORRECTION_SHIFT = b-1U; // cppcheck-suppress misra-c2012-10.4
    constexpr uint32_t CORRECTION = 1UL<<CORRECTION_SHIFT;
    return ((uint32_t)(a+CORRECTION) >> b);
}

/** @brief This is only here to eliminate magic numbers
 * 
 * DO NOT USE UNLESS YOU REALLY ARE WORKING IN PERCENTAGES - it will be very
 * confusing for maintainers (which is what we are trying to avoid!)
 */
static constexpr uint8_t ONE_HUNDRED_PCT = 100U;

/**
 * @brief Integer based percentage calculation. I.e. value * (percent/100)
 * 
 * Rounds to nearest, per @ref DIV_ROUND_BEHAVIOR.
 * 
 * The intermediate product is 64-bit, so this is exact across the whole
 * uint32 x uint16 domain. A Cortex-M multiplies 32x32->64 in one instruction
 * and the compiler turns the constant /100 into a multiply-high, so the
 * 8-bit-era games (fixed-point approximation, 32-bit intermediates that wrap
 * once value*percent reaches 2^32) buy nothing here.
 * 
 * @param percent The percent to apply to value
 * @param value The value to operate on
 */
static inline uint32_t percentage(uint16_t percent, uint32_t value) 
{
    return (uint32_t)(((uint64_t)value * (uint64_t)percent + UINT64_C(50)) / UINT64_C(100));
}


/**
 * @brief Integer based half-percentage calculation.
 * 
 * @param percent The percent to calculate ([0, 100])
 * @param value The value to operate on
 * @return uint16_t 
 */
static inline uint16_t halfPercentage(uint8_t percent, uint16_t value) {
    const uint32_t x200 = (uint32_t)percent * (uint32_t)value;
    const uint32_t quotient = x200 / UINT32_C(200);
    return (uint16_t)((x200 % UINT32_C(200)) >= UINT32_C(100) ? quotient + 1U : quotient);
}

/**
 * @brief Make one pass at correcting the value into the range [min, max)
 * 
 * @param min Minimum value (inclusive)
 * @param max Maximum value (exclusive)
 * @param value Value to nudge
 * @param nudgeAmount Amount to change value by 
 * @return int16_t 
 */
static inline int16_t nudge(int16_t min, int16_t max, int16_t value, int16_t nudgeAmount)
{
    if (value<min) { return value + nudgeAmount; }
    if (value>max) { return value - nudgeAmount; }
    return value;
}

/**
 * @brief Integer division that rounds to nearest instead of truncating.
 * 
 * Minor performance drop compared to the plain division operator.
 **/
template <typename TDividend, typename TDivisor>
TESTABLE_STATIC_CONSTEXPR TDividend fast_div_closest(TDividend dividend, TDivisor divisor) {
    return (TDividend)((dividend + DIV_ROUND_CORRECT(divisor, TDivisor)) / divisor);
}

/**
 * @brief clamps a given value between the minimum and maximum thresholds.
 * 
 * Uses operator< to compare the values.
 * 
 * @tparam T Any type that supports operator<
 * @param v The value to clamp 
 * @param lo The minimum threshold
 * @param hi The maximum threshold
 * @return if v compares less than lo, returns lo; otherwise if hi compares less than v, returns hi; otherwise returns v.
 */
// LCOV_EXCL_START
template<class T>
TESTABLE_STATIC_CONSTEXPR const T& clamp(const T& v, const T& lo, const T& hi){
    return v<lo ? lo : hi<v ? hi : v;
}
// LCOV_EXCL_STOP

/// @cond

template <typename T, typename TPrime>
static inline T LOW_PASS_FILTER_8BIT(T input, uint8_t alpha, T prior) {
  // Intermediate steps are for MISRA compliance
  // Equivalent to: (input * (256 - alpha) + (prior * alpha)) >> 8
  static constexpr uint16_t ALPHA_MAX_SHIFT = 8U;
  static constexpr uint16_t ALPHA_MAX = 2U << (ALPHA_MAX_SHIFT-1U);
  uint16_t inv_alpha = ALPHA_MAX - alpha;
  TPrime prior_alpha = (prior * (TPrime)alpha);
  TPrime preshift = (input * (TPrime)inv_alpha) + prior_alpha;
  return (T)(preshift >> (TPrime)ALPHA_MAX_SHIFT);
}

/// @endcond

/**
 * @brief Simple low pass IIR filter 16-bit values
 * 
 * This is effectively implementing the smooth filter from playground.arduino.cc/Main/Smooth
 * But removes the use of floats and uses 8 bits of fixed precision.
 * 
 * @param input incoming unfiltered value
 * @param alpha filter factor. 0=off, 255=full smoothing (0.00 to 0.99 in float, 0-99%)
 * @param prior previous *filtered* value.
 * @return uint16_t The filtered input
 */
static inline uint16_t LOW_PASS_FILTER(uint16_t input, uint8_t alpha, uint16_t prior) {
    return LOW_PASS_FILTER_8BIT<uint16_t, uint32_t>(input, alpha, prior);
}

/** @brief Simple low pass IIR filter for S16 values */
static inline int16_t LOW_PASS_FILTER(int16_t input, uint8_t alpha, int16_t prior) {
    return LOW_PASS_FILTER_8BIT<int16_t, int32_t>(input, alpha, prior);
}

/**
 * @brief Scale a value from one range to another.
 * 
 * Takes a value from a range of [0, fromRange] and scales it to a range of [0, toRange].
 * @warning from must be within the range [0, fromRange].
 * 
 * @param from Value to scale [0, fromRange]
 * @param fromRange Zero based range of the from value
 * @param toRange Zero based range of the to value
 * @return uint8_t from scaled to toRange
 */
static inline uint8_t scale(const uint8_t from, const uint8_t fromRange, const uint8_t toRange) {
  // Using uint16_t to avoid overflow when calculating the result
  return fromRange==0U ? 0U : (((uint16_t)from * (uint16_t)toRange) / (uint16_t)fromRange);
}

/**
 * @brief Specialist version of map(long, long, long, long, long) for performance.
 * 
 * Maps a value from one range to another. 
 * @warning from must be within the range [fromLow, fromHigh].
 * 
 * @param from Value to map [fromLow, fromHigh]
 * @param fromLow Lower bound of the from range
 * @param fromHigh Upper bound of the from range
 * @param toLow Lower bound of the to range
 * @param toHigh Upper bound of the to range
 * @return uint8_t Mapped value in the new range [toLow, toHigh]
 */
static inline uint8_t fast_map(const uint8_t from, const uint8_t fromLow, const uint8_t fromHigh, const uint8_t toLow, const uint8_t toHigh) {
  // Stick to unsigned math for performance, so need to check for output range inversion
  if (toLow>toHigh) {
    return toLow - scale(from - fromLow, fromHigh - fromLow, toLow-toHigh);
  } else {
    return scale(from - fromLow, fromHigh - fromLow, toHigh-toLow) + toLow;
  }
}

#endif