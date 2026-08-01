#pragma once
// Note that all functions with an underscore prefix are NOT part 
// of the public API. They are only here so we can inline them.

#include "scheduler.h"
#include "crankMaths.h"
#include "maths.h"
#include "timers.h"

/**
 * @brief Compute the injector open angle for an injection channel
 * 
 * @param pwDegrees How many crank degrees the calculated PW will take at the current speed
 * @param tdcOffset The number of crank degrees until cylinder is at TDC (at rest)
 * @param injAngle The requested injection angle
 * @return uint16_t 
 */
static inline uint16_t _calculateOpenAngle(FuelSchedule &schedule, uint16_t pwDegrees, uint16_t injAngle)
{
  // 0<=injAngle<=720°
  // 0<=injChannelDegrees<=720°
  // 0<pwDegrees<=??? (could be many crank rotations in the worst case!)
  // 45<=CRANK_ANGLE_MAX_INJ<=720
  // (CRANK_ANGLE_MAX_INJ can be as small as 360/nCylinders. E.g. 45° for 8 cylinder)

  uint16_t startAngle = injAngle + schedule.channelDegrees;
  
  while (startAngle<pwDegrees) { startAngle = startAngle + (uint16_t)CRANK_ANGLE_MAX_INJ; } // Avoid underflow
  startAngle = startAngle - pwDegrees; // startAngle guaranteed to be >=0.
  while (startAngle>=(uint16_t)CRANK_ANGLE_MAX_INJ) { startAngle = startAngle - (uint16_t)CRANK_ANGLE_MAX_INJ; } // Clamp to 0<=startAngle<=CRANK_ANGLE_MAX_INJ

  return startAngle;
}

struct injectorAngleCalcCache {
  uint16_t pw = 0U;
  uint16_t pwDegrees = 0U;
};

static inline uint16_t updatePwAngleCache(uint16_t pw, injectorAngleCalcCache *pCache) {
  // We can afford to be a bit loose updating the cache since injection timing doesn't 
  // need to be precise (the PW calcs liberally use approximations)
  //
  // 1% of a revolution at max RPM should be plenty accurate.
  constexpr int16_t PW_DELTA_THRESHOLD = MIN_REVOLUTION_TIME/100U; // in µS
  if (abs((int16_t)pCache->pw-(int16_t)pw)>PW_DELTA_THRESHOLD) {
    pCache->pwDegrees = timeToAngleDegPerMicroSec(pw);
    pCache->pw = pw;
  }
  return pCache->pwDegrees;
}

static FORCE_INLINE uint32_t _calculateAngularTime(const Schedule &schedule, uint16_t eventAngle, uint16_t crankAngle, uint16_t maxAngle) {
  int16_t delta = eventAngle - crankAngle;
  if ( (isRunning(schedule)) || (schedule._status == OFF)) {
    while(delta < 0) { delta += (int16_t)maxAngle; }
  } 

  return delta > 0 ? angleToTimeMicroSecPerDegree((uint16_t)delta) : 0U;
}

static FORCE_INLINE uint16_t _adjustToTDC(int16_t angle, uint16_t angleOffset, uint16_t maxAngle) {
  angle = angle - (int16_t)angleOffset;
  if( angle < 0) { return angle + (int16_t)maxAngle; }
  return angle;
}

static FORCE_INLINE uint32_t _calculateAngularTime(const Schedule &schedule, uint16_t angleOffset, uint16_t eventAngle, uint16_t crankAngle, uint16_t maxAngle) {
  if (angleOffset==0U) { // Optimize for zero channel angle - no need to adjust start & crank angles
    return _calculateAngularTime(schedule, eventAngle, crankAngle, maxAngle);
  }
  // Realign the current crank angle and the desired start angle around 0 degrees for the given cylinder/output
  // Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (E.g. a standard 4 cylinder engine), then
  // adjusted crank angle is 180* less than the current crank angle and adjusted start angle is the desired open angle less 180*. 
  // Thus the cylinder is being treated relative to its own TDC, regardless of its offset
  //
  // This is done to avoid very small or very large deltas between crank angle and start angle.
  return _calculateAngularTime(schedule, 
            _adjustToTDC(eventAngle, angleOffset, maxAngle),
            _adjustToTDC(crankAngle, angleOffset, maxAngle),
            maxAngle);
}

/**
 * @brief Calculate the time in uS from now to when the injector should be opened.
 * 
 * @param schedule The ignition channel
 * @param openAngle The angle at which to open the injector
 * @param crankAngle The current crank angle
 * @return uint32_t 
 */
static inline uint32_t calculateInjectorTimeout(const FuelSchedule &schedule, int16_t crankAngle, uint16_t openAngle)
{
  int16_t delta = openAngle - crankAngle;

  if (delta<0)
  {
    if (schedule._status != PENDING)
    {
      while(delta < 0) { delta += CRANK_ANGLE_MAX_INJ; }
    }
    else
    {
      delta = 0;
      return 0U;
    }
  }
  return angleToTimeMicroSecPerDegree((uint16_t)delta);
}

/* The ignition angle domain below works in TENTHS of a crank degree (@see ANGLE_TENTHS_PER_DEGREE).
 * schedule.dischargeAngle and schedule.chargeAngle are therefore tenths, as is dwellAngle.
 * schedule.channelDegrees stays in whole degrees (it is shared with the fuel schedules) and is
 * converted at each use. The injection path is untouched and still works in whole degrees. */

static FORCE_INLINE uint32_t _calculateAngularTimeTenths(const Schedule &schedule, uint16_t eventAngleTenths, uint16_t crankAngleTenths, uint16_t maxAngleTenths) {
  int16_t delta = eventAngleTenths - crankAngleTenths;
  if ( (isRunning(schedule)) || (schedule._status == OFF)) {
    while(delta < 0) { delta += (int16_t)maxAngleTenths; }
  }

  return delta > 0 ? angleTenthsToTimeMicroSec((uint16_t)delta) : 0U;
}

static FORCE_INLINE uint32_t _calculateAngularTimeTenths(const Schedule &schedule, uint16_t angleOffsetTenths, uint16_t eventAngleTenths, uint16_t crankAngleTenths, uint16_t maxAngleTenths) {
  if (angleOffsetTenths==0U) { // Optimize for zero channel angle - no need to adjust start & crank angles
    return _calculateAngularTimeTenths(schedule, eventAngleTenths, crankAngleTenths, maxAngleTenths);
  }
  // See the whole-degree overload for why the angles are realigned around the channel's own TDC.
  return _calculateAngularTimeTenths(schedule,
            _adjustToTDC(eventAngleTenths, angleOffsetTenths, maxAngleTenths),
            _adjustToTDC(crankAngleTenths, angleOffsetTenths, maxAngleTenths),
            maxAngleTenths);
}

static inline int16_t _calculateSparkAngle(const IgnitionSchedule &schedule, int16_t advanceTenths) {
  const int16_t maxTenths = crankAngleMaxIgnTenths();
  int16_t angle = (schedule.channelDegrees==0U ? maxTenths : degreesToTenths((int16_t)schedule.channelDegrees)) - advanceTenths;
  if(angle > maxTenths) {angle -= maxTenths;}
  return angle;
}

static inline int16_t _calculateCoilChargeAngle(uint16_t dwellAngleTenths, int16_t dischargeAngleTenths) {
  if (dischargeAngleTenths>(int16_t)dwellAngleTenths) {
    return dischargeAngleTenths - (int16_t)dwellAngleTenths;
  }
  return dischargeAngleTenths + crankAngleMaxIgnTenths() - (int16_t)dwellAngleTenths;
}

static inline void calculateIgnitionAngles(IgnitionSchedule &schedule, uint16_t dwellAngleTenths, int16_t advanceTenths)
{
  schedule.dischargeAngle = _calculateSparkAngle(schedule,  advanceTenths);
  schedule.chargeAngle = _calculateCoilChargeAngle(dwellAngleTenths, schedule.dischargeAngle);
}


static inline void calculateIgnitionTrailingRotary(IgnitionSchedule &leading, uint16_t dwellAngleTenths, int16_t rotarySplitDegrees, IgnitionSchedule &trailing)
{
  trailing.dischargeAngle = ignitionLimitsTenths(leading.dischargeAngle + degreesToTenths(rotarySplitDegrees));
  trailing.chargeAngle = ignitionLimitsTenths(trailing.dischargeAngle - (int16_t)dwellAngleTenths);
}

static inline uint32_t _calculateIgnitionTimeout(const IgnitionSchedule &schedule, int16_t crankAngle)
{
  return _calculateAngularTimeTenths(schedule, degreesToTenths((int16_t)schedule.channelDegrees), schedule.chargeAngle, degreesToTenths(crankAngle), crankAngleMaxIgnTenths());
}

/**
 * @brief Adjust the crank angle used to originally set the schedule.
 * 
 * The assumption here is that we have a more accurate crank angle than
 * was originally passed to calculateIgnitionTimeout. So we can increase the
 * spark accuracy
 * 
 * @param schedule The schedule to modify 
 * @param crankAngle The new crank angle in degrees
 */
static inline void adjustCrankAngle(IgnitionSchedule &schedule, int16_t crankAngle) {
  constexpr uint8_t MIN_CYCLES_FOR_CORRECTION = 6U;

  // crankAngle arrives in whole degrees; the schedule angles are in tenths.
  const int16_t crankAngleTenths = degreesToTenths(ignitionLimits(crankAngle));
  ATOMIC() { // Prevent race conditions with the timer interrupt.
    // We only want to adjust the crank angle if we are running and the coil is charging or we are waiting for the timer to fire.
    if( isRunning(schedule) ) {
      if  (schedule.dischargeAngle>crankAngleTenths) {
        // Coil is charging so change the charge time so the spark fires at
        // the requested crank angle (this could reduce dwell time & potentially
        // result in a weaker spark).
        SET_COMPARE(schedule._compare, schedule._counter + angleTenthsToTimerTicks( schedule.dischargeAngle-crankAngleTenths ));
      }
    }
    else if( (schedule._status==PENDING) ) {
      if ((currentStatus.startRevolutions > MIN_CYCLES_FOR_CORRECTION) && (schedule.chargeAngle>crankAngleTenths)) {
        // We are waiting for the timer to fire & start charging the coil.
        // Keep dwell (I.e. duration) constant (for better spark) - instead adjust the waiting period so
        // the spark fires at the requested crank angle.
        SET_COMPARE(schedule._compare, schedule._counter + angleTenthsToTimerTicks( schedule.chargeAngle-crankAngleTenths ));
      }
    } else {
      // Unknown state, so no adjustment possible
    }
  }
}
