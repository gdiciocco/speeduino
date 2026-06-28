#ifdef OIL_SENSOR_OPST

#include "sensors.h"
#include "globals.h"


#define PIN_OPST  PF3

/*
 * Called by the 4 hz hardware timer, enables OPS+T interrupt
 * when timer clicks, ISR will detach itself when it has got readings
*/



#if defined(CORE_AVR)
  #define READ_OPST_TRIGGER() ((*oilSensorOPSt_pin_port & oilSensorOPSt_pin_mask) ? true : false)
#else
  #define READ_OPST_TRIGGER() digitalRead(PIN_OPST)
#endif

extern void readOPSt();

extern volatile struct oilSensorOPStPulse {
  uint8_t index; // Index of the pulse we are on, frame is composed by three pulses
  unsigned long onTime; // Time duration of the HIGH level 
  unsigned long offTime; // Time duration of the LOW level
  unsigned long totalTime; // Time duration of the whole symbol
  unsigned long curEvent; // micros() time of current ISR call
  unsigned long lastEvent; // micros() time of the last ISR call
  uint8_t lastLevel; // last level
  uint8_t gotSync; // Have we synced to the pulse sequence ?
} oilSensorOPStPulse;

extern volatile struct oilSensorOPStData {
  int16_t temperature; // Celsius temperature + 40 C to avoid problems with negative values 
  int16_t pressure; // Pressure in PSI
  uint8_t status; // Diagnostic pulse, containing its (error corrected) value
} oilSensorOPStData;

#endif
