#ifdef OIL_SENSOR_OPST

#include "opst_sensor.h"
#include "opst_sensor_math.h"
#include "atomic.h"

static constexpr unsigned long OPST_STATUS_PERIOD_MIN_US = 500UL;
static constexpr unsigned long OPST_STATUS_PERIOD_MAX_US = 1500UL;
static constexpr unsigned long OPST_DATA_PERIOD_MIN_US = 2500UL;
static constexpr unsigned long OPST_DATA_PERIOD_MAX_US = 5500UL;
static constexpr unsigned long OPST_DATA_FRESH_MAX_US = 1000000UL;

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
  oilSensorOPStPulse.pendingStatus = decodeOPStDiagnostic(oilSensorOPStPulse.totalTime, oilSensorOPStPulse.onTime);
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
        oilSensorOPStPulse.pendingTemperature =
          decodeOPStTemperatureC(oilSensorOPStPulse.totalTime, oilSensorOPStPulse.onTime);
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
        oilSensorOPStData.temperature = oilSensorOPStPulse.pendingTemperature;
        oilSensorOPStData.absolutePressureKpa =
          decodeOPStAbsolutePressureKpa(oilSensorOPStPulse.totalTime, oilSensorOPStPulse.onTime);
        oilSensorOPStData.status = oilSensorOPStPulse.pendingStatus;
        oilSensorOPStData.lastFrameTime = oilSensorOPStPulse.curEvent;
        oilSensorOPStData.hasFrame = 1U;
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
  oilSensorOPStPulse.pendingTemperature = 0;
  oilSensorOPStPulse.pendingStatus = 0U;

  attachInterrupt(digitalPinToInterrupt(PIN_OPST), oilSensorOPStISR, CHANGE);
}

oilSensorOPStSnapshot getOPStSnapshot()
{
  oilSensorOPStSnapshot snapshot = {};
  uint32_t lastFrameTime = 0U;
  uint8_t hasFrame = 0U;
  uint8_t diagnostic = 0U;

  ATOMIC()
  {
    snapshot.temperature = oilSensorOPStData.temperature;
    snapshot.absolutePressureKpa = oilSensorOPStData.absolutePressureKpa;
    diagnostic = oilSensorOPStData.status;
    lastFrameTime = oilSensorOPStData.lastFrameTime;
    hasFrame = oilSensorOPStData.hasFrame;
  }

  const uint32_t ageUs = (hasFrame != 0U) ? (micros() - lastFrameTime) : UINT32_MAX;
  snapshot.valid = isOPStReadingValid(hasFrame, ageUs, OPST_DATA_FRESH_MAX_US, diagnostic);

  return snapshot;
}

#endif
