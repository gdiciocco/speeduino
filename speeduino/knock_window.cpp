#include "knock_window.h"

#if defined(KNOCK_WINDOW_OUTPUT_PIN)

#include "globals.h"
#include "statuses.h"
#include "crankMaths.h"
#include "table2d.h"
#include "src/pins/fastOutputPin.h"
#include "atomic.h"

volatile bool knockWindowOutputEnabled = false;
volatile COMPARE_TYPE knockWindowDelayCompare = 0U;
volatile COMPARE_TYPE knockWindowDurationCompare = 0U;

static fastOutputPin_t knockWindowPin;
// Shared GPIO: the pin stays active while at least one cylinder window is open
static volatile uint8_t knockWindowActiveCount = 0U;

static constexpr table2D_u8_u8_6 knockWindowStartTable(&configPage10.knock_window_rpms, &configPage10.knock_window_angle);
static constexpr table2D_u8_u8_6 knockWindowDurationTable(&configPage10.knock_window_rpms, &configPage10.knock_window_dur);

static inline void setPinInactive(void)
{
#if defined(KNOCK_WINDOW_OUTPUT_ACTIVE_LOW)
  knockWindowPin.setPinHigh();
#else
  knockWindowPin.setPinLow();
#endif
}

static inline void setPinActive(void)
{
#if defined(KNOCK_WINDOW_OUTPUT_ACTIVE_LOW)
  knockWindowPin.setPinLow();
#else
  knockWindowPin.setPinHigh();
#endif
}

void initialiseKnockWindowOutput(void)
{
  if(!pinIsUsed(KNOCK_WINDOW_OUTPUT_PIN))
  {
    knockWindowPin.setPin(KNOCK_WINDOW_OUTPUT_PIN);
    setPinInactive();
    knockWindowActiveCount = 0U;
    knockWindowOutputEnabled = true;
  }
  else
  {
    knockWindowOutputEnabled = false;
  }
}

// The ignition timers run at different NVIC priorities, so window on/off can
// nest: the refcount update must be atomic
void knockWindowOutputOn(void)
{
  ATOMIC()
  {
    if(knockWindowActiveCount == 0U) { setPinActive(); }
    knockWindowActiveCount++;
  }
}

void knockWindowOutputOff(void)
{
  ATOMIC()
  {
    if(knockWindowActiveCount > 0U) { knockWindowActiveCount--; }
    if(knockWindowActiveCount == 0U) { setPinInactive(); }
  }
}

static inline COMPARE_TYPE knockWindowCompareFromUs(uint32_t duration)
{
  uint32_t clampedDuration = duration;
  if(clampedDuration >= MAX_TIMER_PERIOD) { clampedDuration = MAX_TIMER_PERIOD - 1U; }

  COMPARE_TYPE compareValue = uS_TO_TIMER_COMPARE(clampedDuration);
  if((clampedDuration > 0UL) && (compareValue == 0U)) { compareValue = 1U; }
  return compareValue;
}

void updateKnockWindowSchedule(const statuses &current)
{
  COMPARE_TYPE delayCompare = 0U;
  COMPARE_TYPE durationCompare = 0U;

  if(knockWindowOutputEnabled)
  {
    uint8_t durationAngle = table2D_getValue(&knockWindowDurationTable, current.RPMdiv100);
    if(durationAngle > 0U)
    {
      durationCompare = knockWindowCompareFromUs(angleToTimeMicroSecPerDegree(durationAngle));

      // TunerStudio stores the window start angle as an 8-bit value. Cast
      // through int8_t so negative BTDC values keep their sign.
      int16_t windowStartAngle = (int8_t)table2D_getValue(&knockWindowStartTable, current.RPMdiv100);
      int16_t delayAngle = (int16_t)current.advance + windowStartAngle;

      // The window is intentionally post-spark. If the tune asks for a
      // pre-spark window (delayAngle <= 0), open right at spark end instead.
      if(delayAngle > 0)
      {
        uint32_t delayTime = angleToTimeMicroSecPerDegree((uint16_t)delayAngle);
        if(delayTime < MAX_TIMER_PERIOD) { delayCompare = knockWindowCompareFromUs(delayTime); }
        else { durationCompare = 0U; } // Window start is beyond timer reach; skip this window
      }
    }
  }

  // COMPARE_TYPE stores are single-word on ARM, so the ISRs always read a
  // consistent individual value even without locking
  knockWindowDelayCompare = delayCompare;
  knockWindowDurationCompare = durationCompare;
}

#endif // KNOCK_WINDOW_OUTPUT_PIN
