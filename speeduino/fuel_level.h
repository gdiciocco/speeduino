#ifndef FUEL_LEVEL_H
#define FUEL_LEVEL_H

#include <stdint.h>

/** Convert the AUX0 ADC value with the calibration formerly stored in benzina.inc. */
uint8_t fuelLevelLitresFromAdc(uint16_t adcValue);

/**
 * Heavy fuel-level filter for a 4 Hz input.
 *
 * A 9-sample median rejects short spikes, then an EMA with alpha 1/64
 * suppresses sender noise and fuel slosh (time constant about 16 seconds).
 */
class FuelLevelFilter
{
public:
  uint8_t update(uint16_t adcValue);
  void reset();
  uint16_t filteredAdc() const;

private:
  static constexpr uint8_t MEDIAN_WINDOW_SIZE = 9U;
  static constexpr uint8_t EMA_SHIFT = 6U;
  static constexpr uint8_t FRACTIONAL_BITS = 10U;

  uint16_t samples[MEDIAN_WINDOW_SIZE] = {0U};
  uint32_t filteredAdcFixed = 0U;
  uint8_t sampleIndex = 0U;
  bool initialised = false;
};

#endif // FUEL_LEVEL_H
