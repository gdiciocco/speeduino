#pragma once

#include <stdint.h>

static constexpr uint16_t OPST_PRESSURE_KPA_PER_PSI_X1000 = 6895U;
static constexpr uint8_t OPST_DIAGNOSTIC_OK = 64U;
static constexpr uint8_t OPST_DIAGNOSTIC_TOLERANCE = 8U;

static inline int16_t roundOPStSigned(float value)
{
  return static_cast<int16_t>(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static inline uint16_t roundOPStUnsigned(float value)
{
  if (value <= 0.0f) { return 0U; }
  if (value >= 65535.0f) { return UINT16_MAX; }
  return static_cast<uint16_t>(value + 0.5f);
}

static inline int16_t decodeOPStTemperatureC(uint32_t periodUs, uint32_t onTimeUs)
{
  if (periodUs == 0U) { return 0; }

  const float normalizedPulseUs = (4096.0f / static_cast<float>(periodUs)) * static_cast<float>(onTimeUs);
  const int16_t temperatureC = roundOPStSigned(((normalizedPulseUs - 128.0f) / 19.2f) - 40.0f);

  if (temperatureC < -40) { return -40; }
  if (temperatureC > 160) { return 160; }
  return temperatureC;
}

static inline uint16_t decodeOPStAbsolutePressureKpa(uint32_t periodUs, uint32_t onTimeUs)
{
  if (periodUs == 0U) { return 0U; }

  const float normalizedPulseUs = (4096.0f / static_cast<float>(periodUs)) * static_cast<float>(onTimeUs);
  const float pressureKpa = (((normalizedPulseUs - 128.0f) / 384.0f) + 0.5f) * 100.0f;
  return roundOPStUnsigned(pressureKpa);
}

static inline uint8_t decodeOPStDiagnostic(uint32_t periodUs, uint32_t onTimeUs)
{
  if (periodUs == 0U) { return 0U; }

  const float diagnostic = ((1024.0f / static_cast<float>(periodUs)) * static_cast<float>(onTimeUs)) / 4.0f;
  if (diagnostic <= 0.0f) { return 0U; }
  if (diagnostic >= 255.0f) { return UINT8_MAX; }
  return static_cast<uint8_t>(diagnostic + 0.5f);
}

static inline bool isOPStDiagnosticNear(uint8_t diagnostic, uint8_t expected)
{
  const uint8_t difference = (diagnostic > expected) ? (diagnostic - expected) : (expected - diagnostic);
  return difference <= OPST_DIAGNOSTIC_TOLERANCE;
}

static inline bool isOPStReadingValid(uint8_t hasFrame, uint32_t ageUs, uint32_t maxAgeUs, uint8_t diagnostic)
{
  return (hasFrame != 0U) &&
         (ageUs <= maxAgeUs) &&
         isOPStDiagnosticNear(diagnostic, OPST_DIAGNOSTIC_OK);
}

static inline uint8_t convertOPStGaugePressureToPsi(uint16_t absolutePressureKpa, uint16_t barometricPressureKpa)
{
  if (absolutePressureKpa <= barometricPressureKpa) { return 0U; }

  const uint32_t gaugePressureKpa = absolutePressureKpa - barometricPressureKpa;
  const uint32_t roundedPsi =
    ((gaugePressureKpa * 1000UL) + (OPST_PRESSURE_KPA_PER_PSI_X1000 / 2U)) /
    OPST_PRESSURE_KPA_PER_PSI_X1000;

  return (roundedPsi > UINT8_MAX) ? UINT8_MAX : static_cast<uint8_t>(roundedPsi);
}
