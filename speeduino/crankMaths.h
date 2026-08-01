#ifndef CRANKMATHS_H
#define CRANKMATHS_H

#include "maths.h"
#include "globals.h"

/**
 * @file
 * 
 * @brief Crank revolution based mathematical functions. 
 * 
 */

/** @brief At 1 RPM, each degree of angular rotation takes this many microseconds */
static constexpr uint32_t MICROS_PER_DEG_1_RPM = UDIV_ROUND_CLOSEST(MICROS_PER_MIN, 360UL, uint32_t);

/** @brief The maximum rpm that the ECU will attempt to run at. 
 * 
 * It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be
 * allowed to run. Lower number gives better performance 
 **/
static constexpr uint16_t MAX_RPM = 18000U;

/** @brief Absolute minimum RPM that the crank math (& therefore all of Speeduino) can be used with.
 * 
 * This is dictated by the use of uint16_t as the base type for storing
 * time --> angle conversion factor (degreesPerMicro)
*/
static constexpr uint16_t MIN_RPM = (uint16_t)UDIV_ROUND_UP(MICROS_PER_DEG_1_RPM, (uint32_t)UINT16_MAX/16UL, uint32_t);

/**
 * @brief Minimum time in µS that one crank revolution can take.
 * 
 * @note: many calculations are done over 2 revolutions (cycles), in which case this would be doubled 
 */
static constexpr uint16_t MIN_REVOLUTION_TIME = MICROS_PER_MIN/MAX_RPM;

/**
 * @brief Maximum time in µS that one crank revolution can take.
 * 
 * @note: many calculations are done over 2 revolutions (cycles), in which case this would be doubled 
 */
static constexpr uint32_t MAX_REVOLUTION_TIME = MICROS_PER_MIN/MIN_RPM;

/** @brief The number of angle units per crank degree used by the ignition path.
 *
 * The ignition timing chain (spark table -> corrections -> discharge/charge angle ->
 * schedule) works in tenths of a crank degree. This gives the closed loop idle advance
 * controller a usable actuator resolution: at 1 degree granularity its authority of a
 * few degrees only spans a handful of discrete steps, which shows up as limit cycling.
 *
 * The injection path is unaffected and still works in whole degrees.
 */
static constexpr int16_t ANGLE_TENTHS_PER_DEGREE = 10;

/** @brief Minimum ignition advance, in tenths of a degree.
 *
 * Kept at the same physical range the int8_t representation used, so the clamping
 * behaviour of the correction chain is unchanged by the move to tenths.
 */
static constexpr int16_t ADVANCE_TENTHS_MIN = (int16_t)INT8_MIN * ANGLE_TENTHS_PER_DEGREE;

/** @brief Maximum ignition advance, in tenths of a degree. @see ADVANCE_TENTHS_MIN */
static constexpr int16_t ADVANCE_TENTHS_MAX = (int16_t)INT8_MAX * ANGLE_TENTHS_PER_DEGREE;

/** @brief Convert whole degrees (E.g. a config page value) to tenths */
static constexpr int16_t degreesToTenths(int16_t degrees) {
    return degrees * ANGLE_TENTHS_PER_DEGREE;
}

/** @brief Convert tenths to whole degrees, rounding to nearest */
static constexpr int16_t tenthsToDegrees(int16_t tenths) {
    return (int16_t)DIV_ROUND_CLOSEST(tenths, ANGLE_TENTHS_PER_DEGREE, int16_t);
}

/** @brief CRANK_ANGLE_MAX_IGN expressed in tenths of a degree */
static inline int16_t crankAngleMaxIgnTenths(void) {
    return CRANK_ANGLE_MAX_IGN * ANGLE_TENTHS_PER_DEGREE;
}

/**
 * @brief Makes one pass at nudging the angle to within [0,CRANK_ANGLE_MAX_IGN]
 *
 * @param angle A crank angle in degrees
 * @return int16_t
 */
static inline int16_t ignitionLimits(int16_t angle) {
    return nudge(0, CRANK_ANGLE_MAX_IGN-1, angle, CRANK_ANGLE_MAX_IGN);
}

/**
 * @brief Makes one pass at nudging the angle to within [0,CRANK_ANGLE_MAX_IGN*10]
 *
 * @param angleTenths A crank angle in tenths of a degree
 * @return int16_t
 */
static inline int16_t ignitionLimitsTenths(int16_t angleTenths) {
    const int16_t maxTenths = crankAngleMaxIgnTenths();
    return nudge(0, maxTenths-1, angleTenths, maxTenths);
}

/** @brief Clamp the angle to within [0,CRANK_ANGLE_MAX_INJ] */
static inline uint16_t injectorLimits(uint16_t angle)
{
    while(angle >= (uint16_t)CRANK_ANGLE_MAX_INJ ) { angle -= (uint16_t)CRANK_ANGLE_MAX_INJ; }
    return angle;
}

/** @brief Clamp the angle to within [0,CRANK_ANGLE_MAX_INJ] */
static inline uint16_t injectorLimits(int16_t angle)
{
    while(angle < 0) { angle += CRANK_ANGLE_MAX_INJ; }
    return injectorLimits((uint16_t)angle);
}

/**
 * @brief Set the revolution time, from which some of the degree<-->angle conversions are derived
 * 
 * @param revolutionTime The crank revolution time.
 */
void setAngleConverterRevolutionTime(uint32_t revolutionTime) noexcept;

/**
 * @brief Converts angular degrees to the time interval that amount of rotation
 * will take at current RPM.
 * 
 * Based on angle of [0,720] and min/max RPM, result ranges from
 * 9 (MAX_RPM, 1 deg) to 2926828 (MIN_RPM, 720 deg)
 *
 * @param angle Angle in degrees
 * @return Time interval in uS
 */
uint32_t angleToTimeMicroSecPerDegree(uint16_t angle) noexcept;

/**
 * @brief Converts angular degrees to the equivalent timer ticks at current RPM.
 * 
 * @param angle Angle in degrees
 * @return Number of timer ticks 
 */
COMPARE_TYPE angleToTimerTicks(uint16_t angle) noexcept;

/**
 * @brief Converts tenths of a crank degree to the time that amount of rotation
 * will take at current RPM.
 *
 * @param angleTenths Angle in tenths of a degree
 * @return Time interval in uS
 */
uint32_t angleTenthsToTimeMicroSec(uint16_t angleTenths) noexcept;

/**
 * @brief Converts tenths of a crank degree to the equivalent timer ticks at current RPM.
 *
 * @param angleTenths Angle in tenths of a degree
 * @return Number of timer ticks
 */
COMPARE_TYPE angleTenthsToTimerTicks(uint16_t angleTenths) noexcept;

/**
 * @brief Converts a time interval in microsecods to the equivalent degrees of angular (crank)
 * rotation at current RPM.
 *
 * Inverse of angleToTimeMicroSecPerDegree
 *
 * @param time Time interval in uS
 * @return Angle in degrees
 */
uint16_t timeToAngleDegPerMicroSec(uint32_t time) noexcept;

/**
 * @brief Converts a time interval in microseconds to the equivalent tenths of a crank
 * degree at current RPM.
 *
 * Inverse of angleTenthsToTimeMicroSec
 *
 * @param time Time interval in uS
 * @return Angle in tenths of a degree
 */
uint16_t timeToAngleTenthsPerMicroSec(uint32_t time) noexcept;

#endif