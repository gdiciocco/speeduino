/** @file afr_delay.h
 * @brief Lookup helpers for the AFR1/AFR2 measurement-delay maps.
 */
#pragma once

#include <stdint.h>

constexpr uint8_t AFR_DELAY_CHANNEL_1 = 0U;
constexpr uint8_t AFR_DELAY_CHANNEL_2 = 1U;
constexpr uint16_t AFR_DELAY_MS_PER_COUNT = 10U;

/** Bilinearly interpolate the selected RPM/load delay map. */
uint16_t afrDelayMilliseconds(uint8_t channel, uint16_t rpm, uint16_t load);

/** Convert the selected map value to 30 Hz learner ticks and clamp to history. */
uint8_t afrDelayTicks30Hz(uint8_t channel, uint16_t rpm, uint16_t load, uint8_t maxTicks);

/** Convert the selected map plus a consumer-local signed offset to learner ticks. */
uint8_t afrDelayTicks30HzWithOffset(uint8_t channel, uint16_t rpm, uint16_t load,
                                    int16_t offsetMs, uint8_t maxTicks);

/** Return the actually applied, 30 Hz-quantised delay in milliseconds. */
uint16_t afrDelayAppliedMilliseconds(uint8_t channel, uint16_t rpm, uint16_t load, uint8_t maxTicks);

/** Return the applied, quantised delay after a consumer-local signed offset. */
uint16_t afrDelayAppliedMillisecondsWithOffset(uint8_t channel, uint16_t rpm, uint16_t load,
                                               int16_t offsetMs, uint8_t maxTicks);
