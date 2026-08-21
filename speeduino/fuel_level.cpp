#include "fuel_level.h"

#include <stddef.h>

namespace
{
constexpr uint16_t MAX_CALIBRATED_ADC = 254U;

// Last ADC index for each value from 25 L down to 0 L. This is an exact,
// compact transcription of the 255 entries in benzina.inc.
constexpr uint8_t fuelLevelUpperAdc[] = {
  17U, 33U, 48U, 66U, 73U, 74U, 76U, 78U, 82U, 85U, 89U, 103U, 113U,
  120U, 136U, 162U, 170U, 177U, 182U, 186U, 190U, 194U, 199U, 203U,
  208U, 254U
};

uint16_t clampCalibrationAdc(uint16_t adcValue)
{
  return (adcValue > MAX_CALIBRATED_ADC) ? MAX_CALIBRATED_ADC : adcValue;
}

uint16_t medianOfNine(const uint16_t *values)
{
  uint16_t sorted[9];
  for(uint8_t index = 0U; index < 9U; index++) { sorted[index] = values[index]; }

  for(uint8_t index = 1U; index < 9U; index++)
  {
    const uint16_t value = sorted[index];
    uint8_t insertAt = index;
    while((insertAt > 0U) && (sorted[insertAt - 1U] > value))
    {
      sorted[insertAt] = sorted[insertAt - 1U];
      insertAt--;
    }
    sorted[insertAt] = value;
  }

  return sorted[4];
}
}

uint8_t fuelLevelLitresFromAdc(uint16_t adcValue)
{
  const uint16_t calibratedAdc = clampCalibrationAdc(adcValue);
  for(uint8_t index = 0U; index < (sizeof(fuelLevelUpperAdc) / sizeof(fuelLevelUpperAdc[0])); index++)
  {
    if(calibratedAdc <= fuelLevelUpperAdc[index]) { return (uint8_t)(25U - index); }
  }

  return 0U;
}

uint8_t FuelLevelFilter::update(uint16_t adcValue)
{
  const uint16_t calibratedAdc = clampCalibrationAdc(adcValue);
  if(!initialised)
  {
    for(uint8_t index = 0U; index < MEDIAN_WINDOW_SIZE; index++) { samples[index] = calibratedAdc; }
    filteredAdcFixed = ((uint32_t)calibratedAdc << FRACTIONAL_BITS);
    initialised = true;
  }
  else
  {
    samples[sampleIndex] = calibratedAdc;
    sampleIndex = (uint8_t)((sampleIndex + 1U) % MEDIAN_WINDOW_SIZE);

    const uint32_t target = ((uint32_t)medianOfNine(samples) << FRACTIONAL_BITS);
    if(target >= filteredAdcFixed)
    {
      filteredAdcFixed += (target - filteredAdcFixed) >> EMA_SHIFT;
    }
    else
    {
      filteredAdcFixed -= (filteredAdcFixed - target) >> EMA_SHIFT;
    }
  }

  return fuelLevelLitresFromAdc(filteredAdc());
}

void FuelLevelFilter::reset()
{
  filteredAdcFixed = 0U;
  sampleIndex = 0U;
  initialised = false;
}

uint16_t FuelLevelFilter::filteredAdc() const
{
  if(!initialised) { return 0U; }
  return (uint16_t)((filteredAdcFixed + (1UL << (FRACTIONAL_BITS - 1U))) >> FRACTIONAL_BITS);
}
