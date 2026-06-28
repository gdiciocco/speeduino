#ifdef OIL_SENSOR_OPST

#include "sensors.h"
#include "globals.h"    
#include "opst_sensor.h"
/*
 * Called by the 4 hz hardware timer, enables OPS+T interrupt
 * when timer clicks, ISR will detach itself when it has got readings
*/
// ISR to decode PPM data from HELLA oil temperature and pressure sensor
static void oilSensorOPStISR() //Most ARM chips can simply call a function
{
  oilSensorOPStPulse.curEvent = micros();

  if(oilSensorOPStPulse.lastLevel == 0 && READ_OPST_TRIGGER()) // Last event was LOW and we have got a rising edge
  {
      oilSensorOPStPulse.offTime = oilSensorOPStPulse.curEvent - oilSensorOPStPulse.lastEvent;
      oilSensorOPStPulse.totalTime = oilSensorOPStPulse.offTime + oilSensorOPStPulse.onTime;
      oilSensorOPStPulse.lastLevel = 1;
      oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;
      if (oilSensorOPStPulse.totalTime <= 1024 && oilSensorOPStPulse.index == 0) 
      {
        oilSensorOPStPulse.index++;
        oilSensorOPStPulse.gotSync = 1; 
        oilSensorOPStData.status = ((1024.0/oilSensorOPStPulse.totalTime)*oilSensorOPStPulse.onTime)/4.0;
      } 
      else if (oilSensorOPStPulse.index == 1 && oilSensorOPStPulse.gotSync == 1) 
      {
        oilSensorOPStData.temperature = (((4096.0/oilSensorOPStPulse.totalTime)*oilSensorOPStPulse.onTime)-128)/19.2-40;
        oilSensorOPStPulse.index++;
      } 
      else if (oilSensorOPStPulse.index == 2 && oilSensorOPStPulse.gotSync == 1) 
      {
        oilSensorOPStData.pressure = (((4096.0/oilSensorOPStPulse.totalTime)*oilSensorOPStPulse.onTime)-128.0)/26.475+7.252-10;
        oilSensorOPStPulse.index = 0;
        detachInterrupt(digitalPinToInterrupt(PIN_OPST));       
      } 
      else 
      {
        oilSensorOPStPulse.index = 0;
        oilSensorOPStPulse.gotSync = 0;
      }
  } 
  else if(oilSensorOPStPulse.lastLevel == 1 && !READ_OPST_TRIGGER()) // Last event was HIGH and we have got a falling edge 
  { 
      oilSensorOPStPulse.onTime = oilSensorOPStPulse.curEvent - oilSensorOPStPulse.lastEvent;
      oilSensorOPStPulse.lastLevel = 0;
      oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;
  }
}

void readOPSt () {
  attachInterrupt(digitalPinToInterrupt(PIN_OPST), oilSensorOPStISR, CHANGE);
}

#endif
