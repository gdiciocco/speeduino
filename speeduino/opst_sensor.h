#ifndef OPST_SENSOR_H
#define OPST_SENSOR_H

#ifdef OIL_SENSOR_OPST

#include <Arduino.h>
#include <stdint.h>

#define PIN_OPST PF3

#if defined(CORE_AVR)
#define READ_OPST_TRIGGER() ((*oilSensorOPSt_pin_port & oilSensorOPSt_pin_mask) ? true : false)
#else
#define READ_OPST_TRIGGER() digitalRead(PIN_OPST)
#endif

void readOPSt();

extern volatile struct oilSensorOPStPulse {
  uint8_t index;
  unsigned long onTime;
  unsigned long offTime;
  unsigned long totalTime;
  unsigned long curEvent;
  unsigned long lastEvent;
  uint8_t lastLevel;
  uint8_t gotSync;
} oilSensorOPStPulse;

extern volatile struct oilSensorOPStData {
  int16_t temperature;
  int16_t pressure;
  uint8_t status;
} oilSensorOPStData;

#endif

#endif // OPST_SENSOR_H
