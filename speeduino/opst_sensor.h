#ifndef OPST_SENSOR_H
#define OPST_SENSOR_H

#ifdef OIL_SENSOR_OPST

#include <Arduino.h>
#include <stdint.h>

#define PIN_OPST PF3

#define READ_OPST_TRIGGER() digitalRead(PIN_OPST)

void readOPSt();

struct oilSensorOPStSnapshot {
  int16_t temperature;
  uint16_t absolutePressureKpa;
  bool valid;
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
