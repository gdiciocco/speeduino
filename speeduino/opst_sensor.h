#ifndef OPST_SENSOR_H
#define OPST_SENSOR_H

#ifdef OIL_SENSOR_OPST

#include <Arduino.h>
#include <stdint.h>

static constexpr uint8_t OPST_DIAGNOSTIC_OK = 64U;
static constexpr uint8_t OPST_DIAGNOSTIC_PRESSURE_FAULT = 96U;
static constexpr uint8_t OPST_DIAGNOSTIC_TEMPERATURE_FAULT = 128U;
static constexpr uint8_t OPST_DIAGNOSTIC_HARDWARE_FAULT = 160U;

static constexpr uint8_t OPST_FLAG_FRESH = 0U;
static constexpr uint8_t OPST_FLAG_DIAGNOSTIC_OK = 1U;
static constexpr uint8_t OPST_FLAG_PRESSURE_FAULT = 2U;
static constexpr uint8_t OPST_FLAG_TEMPERATURE_FAULT = 3U;
static constexpr uint8_t OPST_FLAG_HARDWARE_FAULT = 4U;
static constexpr uint8_t OPST_FLAG_HAS_FRAME = 5U;

#define PIN_OPST PF3

#if defined(CORE_AVR)
#define READ_OPST_TRIGGER() ((*oilSensorOPSt_pin_port & oilSensorOPSt_pin_mask) ? true : false)
#else
#define READ_OPST_TRIGGER() digitalRead(PIN_OPST)
#endif

void readOPSt();

struct oilSensorOPStSnapshot {
  int16_t temperature;
  uint16_t absolutePressureKpa;
  uint32_t ageUs;
  uint8_t status;
  uint8_t flags;
};

oilSensorOPStSnapshot getOPStSnapshot();

extern volatile struct oilSensorOPStPulse {
  uint8_t index;
  unsigned long onTime;
  unsigned long offTime;
  unsigned long totalTime;
  unsigned long curEvent;
  unsigned long lastEvent;
  uint8_t lastLevel;
  uint8_t gotSync;
  uint8_t periodReady;
  int16_t pendingTemperature;
  uint8_t pendingStatus;
} oilSensorOPStPulse;

extern volatile struct oilSensorOPStData {
  int16_t temperature;
  uint16_t absolutePressureKpa;
  uint32_t lastFrameTime;
  uint8_t status;
  uint8_t hasFrame;
} oilSensorOPStData;

#endif

#endif // OPST_SENSOR_H
