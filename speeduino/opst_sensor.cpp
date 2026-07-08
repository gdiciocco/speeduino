#ifdef OIL_SENSOR_OPST

#include "opst_sensor.h"

static void oilSensorOPStISR()
{
  oilSensorOPStPulse.curEvent = micros();

  if ((oilSensorOPStPulse.lastLevel == 0U) && READ_OPST_TRIGGER())
  {
    oilSensorOPStPulse.offTime = oilSensorOPStPulse.curEvent - oilSensorOPStPulse.lastEvent;
    oilSensorOPStPulse.totalTime = oilSensorOPStPulse.offTime + oilSensorOPStPulse.onTime;
    oilSensorOPStPulse.lastLevel = 1U;
    oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;

    if ((oilSensorOPStPulse.totalTime <= 1024UL) && (oilSensorOPStPulse.index == 0U))
    {
      oilSensorOPStPulse.index++;
      oilSensorOPStPulse.gotSync = 1U;
      oilSensorOPStData.status = ((1024.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) / 4.0f;
    }
    else if ((oilSensorOPStPulse.index == 1U) && (oilSensorOPStPulse.gotSync == 1U))
    {
      oilSensorOPStData.temperature = (((4096.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) - 128.0f) / 19.2f - 40.0f;
      oilSensorOPStPulse.index++;
    }
    else if ((oilSensorOPStPulse.index == 2U) && (oilSensorOPStPulse.gotSync == 1U))
    {
      oilSensorOPStData.pressure = (((4096.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) - 128.0f) / 26.475f + 7.252f - 10.0f;
      oilSensorOPStPulse.index = 0U;
      detachInterrupt(digitalPinToInterrupt(PIN_OPST));
    }
    else
    {
      oilSensorOPStPulse.index = 0U;
      oilSensorOPStPulse.gotSync = 0U;
    }
  }
  else if ((oilSensorOPStPulse.lastLevel == 1U) && !READ_OPST_TRIGGER())
  {
    oilSensorOPStPulse.onTime = oilSensorOPStPulse.curEvent - oilSensorOPStPulse.lastEvent;
    oilSensorOPStPulse.lastLevel = 0U;
    oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;
  }
}

void readOPSt()
{
  attachInterrupt(digitalPinToInterrupt(PIN_OPST), oilSensorOPStISR, CHANGE);
}

#endif
