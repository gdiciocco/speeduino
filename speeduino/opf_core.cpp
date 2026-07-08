#include "opf_core.h"

#include <Arduino.h>

#include "bit_manip.h"
#include "globals.h"
#include "init.h"
#include "timers.h"

#ifdef CAPONORD_BOARD

#define USE_I2C_BARO

#ifdef OIL_SENSOR_OPST
#include "opst_sensor.h"
#endif

#ifdef USE_I2C_BARO
#include <src/LPS25HB/LPS25HBSensor.h>
static TwoWire LPS_dev(PB11, PB10);
static LPS25HBSensor LPS_Sensor(&LPS_dev, LPS25HB_ADDRESS_LOW);
#endif

static constexpr byte CAPONORD_PIN_MAPPING = 60U;
static constexpr byte UNUSED_PIN = BOARD_MAX_IO_PINS - 1U;

static constexpr byte LED_RUNNING = PG9;
static constexpr byte LED_WARNING = PG10;
static constexpr byte LED_ALERT = PG11;
static constexpr byte LED_COMS = PG12;

static void setStatusLedOff(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static void resetPin(byte &pin)
{
  pin = UNUSED_PIN;
}

void setupBoard()
{
  caponordResetPins();
  caponordSetPins();
  configPage2.pinMapping = CAPONORD_PIN_MAPPING;

  initialiseAll();

  setStatusLedOff(LED_RUNNING);
  setStatusLedOff(LED_WARNING);
  setStatusLedOff(LED_ALERT);
  setStatusLedOff(LED_COMS);

#ifdef OIL_SENSOR_OPST
  pinMode(PIN_OPST, INPUT);
#endif

#ifdef USE_I2C_BARO
  LPS_dev.begin();
  if ((LPS_Sensor.begin() != LPS25HB_STATUS_OK) ||
      (LPS_Sensor.SetODR(7.0f) != LPS25HB_STATUS_OK) ||
      (LPS_Sensor.Enable() != LPS25HB_STATUS_OK))
  {
    digitalWrite(LED_ALERT, HIGH);
  }
#endif
}

void caponordSetPins()
{
  pinTrigger = PE5;
  pinTrigger2 = PE4;
  pinVSS = PC13;

  pinBat = PA0;
  pinCLT = PA3;
  pinTPS = PA1;
  pinIAT = PA4;
  pinO2 = PC1;
  pinO2_2 = PC2;
  pinBaro = PC5;
  pinMAP = PA6;
  pinFuelPressure = PB0;

  pinInjector2 = PF14;
  pinInjector1 = PF13;

  pinCoil1 = PE14;
  pinCoil2 = PE15;

  pinTachOut = PD14;
  pinStepperDir = PF7;
  pinStepperStep = PF8;
  pinStepperEnable = PF9;
  pinFuelPump = PG6;
  pinFan = PG7;
}

void caponordResetPins()
{
  resetPin(pinInjector1);
  resetPin(pinInjector2);
  resetPin(pinInjector3);
  resetPin(pinInjector4);
  resetPin(pinInjector5);
  resetPin(pinInjector6);
  resetPin(pinInjector7);
  resetPin(pinInjector8);

  resetPin(pinCoil1);
  resetPin(pinCoil2);
  resetPin(pinCoil3);
  resetPin(pinCoil4);
  resetPin(pinCoil5);
  resetPin(pinCoil6);
  resetPin(pinCoil7);
  resetPin(pinCoil8);

  resetPin(pinTrigger);
  resetPin(pinTrigger2);
  resetPin(pinTrigger3);
  resetPin(pinTPS);
  resetPin(pinMAP);
  resetPin(pinEMAP);
  resetPin(pinMAP2);
  resetPin(pinIAT);
  resetPin(pinCLT);
  resetPin(pinO2);
  resetPin(pinO2_2);
  resetPin(pinBat);
  resetPin(pinDisplayReset);
  resetPin(pinTachOut);
  resetPin(pinFuelPump);
  resetPin(pinIdle1);
  resetPin(pinIdle2);
  resetPin(pinIdleUp);
  resetPin(pinIdleUpOutput);
  resetPin(pinCTPS);
  resetPin(pinFuel2Input);
  resetPin(pinSpark2Input);
  resetPin(pinSpareTemp1);
  resetPin(pinSpareTemp2);
  resetPin(pinSpareOut1);
  resetPin(pinSpareOut2);
  resetPin(pinSpareOut3);
  resetPin(pinSpareOut4);
  resetPin(pinSpareOut5);
  resetPin(pinSpareOut6);
  resetPin(pinSpareHOut1);
  resetPin(pinSpareHOut2);
  resetPin(pinSpareLOut1);
  resetPin(pinSpareLOut2);
  resetPin(pinSpareLOut3);
  resetPin(pinSpareLOut4);
  resetPin(pinSpareLOut5);
  resetPin(pinBoost);
  resetPin(pinVVT_1);
  resetPin(pinVVT_2);
  resetPin(pinFan);
  resetPin(pinStepperDir);
  resetPin(pinStepperStep);
  resetPin(pinStepperEnable);
  resetPin(pinLaunch);
  resetPin(pinIgnBypass);
  resetPin(pinFlex);
  resetPin(pinVSS);
  resetPin(pinBaro);
  resetPin(pinResetControl);
  resetPin(pinFuelPressure);
  resetPin(pinOilPressure);
  resetPin(pinWMIEmpty);
  resetPin(pinWMIIndicator);
  resetPin(pinWMIEnabled);
  resetPin(pinMC33810_1_CS);
  resetPin(pinMC33810_2_CS);
}

void runLoop()
{
  if (Serial.available() > 0)
  {
    digitalToggle(LED_COMS);
  }
  else
  {
    digitalWrite(LED_COMS, LOW);
  }

  digitalWrite(LED_ALERT, currentStatus.engineProtect.isActive());

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_1HZ))
  {
#ifdef USE_I2C_BARO
    float pressure = 0.0f;
    float temperature = 0.0f;
    if ((LPS_Sensor.GetPressure(&pressure) == LPS25HB_STATUS_OK) &&
        (LPS_Sensor.GetTemperature(&temperature) == LPS25HB_STATUS_OK))
    {
      currentStatus.fuelTemp = static_cast<int8_t>(temperature);
      currentStatus.baroADC = static_cast<uint16_t>(pressure / 10.0f);
    }
#endif
  }

  if (BIT_CHECK(currentStatus.LOOP_TIMER, BIT_TIMER_4HZ))
  {
#ifdef OIL_SENSOR_OPST
    readOPSt();
#endif
    digitalToggle(LED_RUNNING);
  }
}

#else

void setupBoard()
{
  initialiseAll();
}

void runLoop()
{
}

#endif
