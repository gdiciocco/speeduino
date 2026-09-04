/** @file afr_delay.cpp
 * @brief AFR measurement-delay map lookup and learner-time conversion.
 */
#include "afr_delay.h"

#include "globals.h"

uint16_t afrDelayMilliseconds(uint8_t channel, uint16_t rpm, uint16_t load)
{
  const uint8_t tableIndex = (channel == AFR_DELAY_CHANNEL_2) ? AFR_DELAY_CHANNEL_2 : AFR_DELAY_CHANNEL_1;
  return (uint16_t)get3DTableValue(&afrDelayTables[tableIndex], load, rpm) * AFR_DELAY_MS_PER_COUNT;
}

uint8_t afrDelayTicks30Hz(uint8_t channel, uint16_t rpm, uint16_t load, uint8_t maxTicks)
{
  return afrDelayTicks30HzWithOffset(channel, rpm, load, 0, maxTicks);
}

uint8_t afrDelayTicks30HzWithOffset(uint8_t channel, uint16_t rpm, uint16_t load,
                                    int16_t offsetMs, uint8_t maxTicks)
{
  int32_t delayMs = (int32_t)afrDelayMilliseconds(channel, rpm, load) + offsetMs;
  if (delayMs < 0) { delayMs = 0; }
  uint16_t ticks = ((uint32_t)delayMs * 3U + 50U) / 100U;
  if (ticks < 1U) { ticks = 1U; }
  if (ticks > maxTicks) { ticks = maxTicks; }
  return (uint8_t)ticks;
}

uint16_t afrDelayAppliedMilliseconds(uint8_t channel, uint16_t rpm, uint16_t load, uint8_t maxTicks)
{
  return afrDelayAppliedMillisecondsWithOffset(channel, rpm, load, 0, maxTicks);
}

uint16_t afrDelayAppliedMillisecondsWithOffset(uint8_t channel, uint16_t rpm, uint16_t load,
                                               int16_t offsetMs, uint8_t maxTicks)
{
  const uint16_t ticks = afrDelayTicks30HzWithOffset(channel, rpm, load, offsetMs, maxTicks);
  return (uint16_t)((ticks * 100U + 1U) / 3U);
}
