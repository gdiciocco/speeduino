#ifdef OIL_SENSOR_OPST

#include "opst_sensor.h"

static constexpr unsigned long OPST_STATUS_PERIOD_MIN_US = 500UL;
static constexpr unsigned long OPST_STATUS_PERIOD_MAX_US = 1500UL;
static constexpr unsigned long OPST_DATA_PERIOD_MIN_US = 2500UL;
static constexpr unsigned long OPST_DATA_PERIOD_MAX_US = 5500UL;

static bool isOPStStatusPeriod(unsigned long period)
{
  return (period >= OPST_STATUS_PERIOD_MIN_US) && (period <= OPST_STATUS_PERIOD_MAX_US);
}

static bool isOPStDataPeriod(unsigned long period)
{
  return (period >= OPST_DATA_PERIOD_MIN_US) && (period <= OPST_DATA_PERIOD_MAX_US);
}

static void syncOPStFromStatus()
{
  oilSensorOPStPulse.index = 1U;
  oilSensorOPStPulse.gotSync = 1U;
  oilSensorOPStData.status = ((1024.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) / 4.0f;
}

static void oilSensorOPStISR()
{
  oilSensorOPStPulse.curEvent = micros();

  if ((oilSensorOPStPulse.lastLevel == 0U) && READ_OPST_TRIGGER())
  {
    oilSensorOPStPulse.offTime = oilSensorOPStPulse.curEvent - oilSensorOPStPulse.lastEvent;
    oilSensorOPStPulse.totalTime = oilSensorOPStPulse.offTime + oilSensorOPStPulse.onTime;
    oilSensorOPStPulse.lastLevel = 1U;
    oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;

    // The first rising edge after attaching may contain a partial HIGH or LOW
    // interval. Only subsequent rising edges describe a complete symbol.
    if (oilSensorOPStPulse.periodReady == 0U)
    {
      oilSensorOPStPulse.periodReady = 1U;
    }
    else if (oilSensorOPStPulse.index == 0U)
    {
      if (isOPStStatusPeriod(oilSensorOPStPulse.totalTime))
      {
        syncOPStFromStatus();
      }
    }
    else if ((oilSensorOPStPulse.index == 1U) && (oilSensorOPStPulse.gotSync == 1U))
    {
      if (isOPStDataPeriod(oilSensorOPStPulse.totalTime))
      {
        oilSensorOPStData.temperature = (((4096.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) - 128.0f) / 19.2f - 40.0f;
        oilSensorOPStPulse.index = 2U;
      }
      else if (isOPStStatusPeriod(oilSensorOPStPulse.totalTime))
      {
        syncOPStFromStatus();
      }
      else
      {
        oilSensorOPStPulse.index = 0U;
        oilSensorOPStPulse.gotSync = 0U;
      }
    }
    else if ((oilSensorOPStPulse.index == 2U) && (oilSensorOPStPulse.gotSync == 1U))
    {
      if (isOPStDataPeriod(oilSensorOPStPulse.totalTime))
      {
        oilSensorOPStData.pressure = (((4096.0f / oilSensorOPStPulse.totalTime) * oilSensorOPStPulse.onTime) - 128.0f) / 26.475f + 7.252f - 10.0f;
        oilSensorOPStPulse.index = 0U;
        oilSensorOPStPulse.gotSync = 0U;
        detachInterrupt(digitalPinToInterrupt(PIN_OPST));
      }
      else if (isOPStStatusPeriod(oilSensorOPStPulse.totalTime))
      {
        syncOPStFromStatus();
      }
      else
      {
        oilSensorOPStPulse.index = 0U;
        oilSensorOPStPulse.gotSync = 0U;
      }
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
  detachInterrupt(digitalPinToInterrupt(PIN_OPST));

  oilSensorOPStPulse.index = 0U;
  oilSensorOPStPulse.onTime = 0UL;
  oilSensorOPStPulse.offTime = 0UL;
  oilSensorOPStPulse.totalTime = 0UL;
  oilSensorOPStPulse.curEvent = micros();
  oilSensorOPStPulse.lastEvent = oilSensorOPStPulse.curEvent;
  oilSensorOPStPulse.lastLevel = READ_OPST_TRIGGER() ? 1U : 0U;
  oilSensorOPStPulse.gotSync = 0U;
  oilSensorOPStPulse.periodReady = 0U;

  attachInterrupt(digitalPinToInterrupt(PIN_OPST), oilSensorOPStISR, CHANGE);
}

#endif
