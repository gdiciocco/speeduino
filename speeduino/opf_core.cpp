#ifndef OPF_CORE
#define OPF_CORE

#include "opf_core.h"

#ifdef OIL_SENSOR_OPST
 #include "opst_sensor.h"
#endif

#ifdef USE_I2C_BARO
TwoWire LPS_dev(PB11, PB10);  // I2C2, SDA=PB11, SCL=PB10
LPS25HBSensor LPS_Sensor(&LPS_dev, LPS25HB_ADDRESS_LOW);
#endif //USE_I2C_BARO

#ifdef USE_DBW_IFX9201

HardwareTimer Timer10(TIM10);
IFX9201 IFX9201_HBridge = IFX9201();

#endif //USE_DBW_IFX9201

//STM32_CAN Can0(_CAN1, DEF);
//STM32_CAN Can1(_CAN2, DEF);

void setupBoard()
{
  resetPins();
  setPins();
  configPage2.pinMapping = 60;

  initialiseAll();


  //STATUS LED
  pinMode(LED_RUNNING, OUTPUT);
  digitalWrite(LED_RUNNING, LOW);
  pinMode(LED_WARNING, OUTPUT);
  digitalWrite(LED_WARNING, LOW);
  pinMode(LED_ALERT, OUTPUT);
  digitalWrite(LED_ALERT, LOW);
  pinMode(LED_COMS, OUTPUT);
  digitalWrite(LED_COMS, LOW);
#ifdef USE_SPI_EEPROM
  SPIClass SPI_for_flash(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK); //SPI1_MOSI, SPI1_MISO, SPI1_SCK

  //windbond W25Q16 SPI flash EEPROM emulation
  EEPROM_Emulation_Config EmulatedEEPROMMconfig{255UL, 16384UL, 31, 0x00100000UL};
  Flash_SPI_Config SPIconfig{USE_SPI_EEPROM, SPI_for_flash};
  SPI_EEPROM_Class EEPROM(EmulatedEEPROMMconfig, SPIconfig);
  EEPROM.begin(SPI_for_flash, PIN_SPI_SS);

#ifdef CLEAN_SPI_EEPROM
  EEPROM.clear();
#endif
#endif
#ifdef USE_I2C_BARO
    LPS_dev.begin();
    if (LPS_Sensor.begin()   != LPS25HB_STATUS_OK ||
        LPS_Sensor.SetODR(7.0f) != LPS25HB_STATUS_OK ||
        LPS_Sensor.Enable()  != LPS25HB_STATUS_OK) {
        digitalWrite(LED_ALERT, HIGH);  // segnala errore init sensore
    }
#endif

#ifdef USE_DBW_IFX9201
  Timer10.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, DIS_PIN); //DBW PWM output fixed to PB8/
  Timer10.setOverflow(20000, HERTZ_FORMAT);
  Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT);
  Timer10.resume();
  //IFX9201_HBridge.begin( DIR_PIN, STP_PIN, DIS_PIN );

  //IFX9201_HBridge.forwards( 50 );       // Same as forwards( )
  //IFX9201_HBridge.stop( );
  //IFX9201_HBridge.backwards( 50 );
  //IFX9201_HBridge.stop( );

  // TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(DIS_PIN), PinMap_PWM);
  // uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(DIS_PIN), PinMap_PWM));
  //Timer10->setPWM(channel, DIS_PIN, 10, 50, dbwScheduleInterrupt);

  //DBWMotor.begin();
  //DBWMotor.setSpeed(100);
#endif //USE_DBW_IFX9201

/*
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableFIFO();

  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
*/
}

#ifdef USE_DBW_IFX9201
void dbwScheduleInterrupt()
{
  digitalToggle(LED_WARNING);
}
#endif //USE_DBW_IFX9201

void setPins()
{

   //******************************************
  //******** Trigger CONNECTIONS ***************
  //******************************************

  pinTrigger = PE5; //106
  pinTrigger2 = PE4;  //107
  //pinTrigger3 = PE4;  //104
  //pinVSS = PE5;       //105
  pinVSS = PC13;       //105

  //******************************************
  //******** ANALOG CONNECTIONS ***************
  //******************************************
  //ADC1 = STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 6, 0)

  pinBat = PA0;  //A12
  pinCLT = PA3;  //A7
  pinTPS = PA1;  //A9
  pinIAT = PA4;  //A8
  pinO2 = PC1;   //A13
  pinO2_2 = PC2; //A14
  pinBaro = PC5; //A1
  pinMAP = PA6;   //A5
  // pinOilPressure = PB1;  //A0
  pinFuelPressure = PB0; //A2

  //******************************************
  //******** INJECTOR CONNECTIONS ***************
  //******************************************

  /*
    pinInjector8 = PD13; //9
    pinInjector7 = PD12; //8
    pinInjector6 = PD11; //7
    pinInjector5 = PD10; //6
    pinInjector4 = PD9;  //5
    pinInjector3 = PD8;  //4
  */
  pinInjector2 = PF14; //71
  pinInjector1 = PF13; //70

  //******************************************
  //******** COIL CONNECTIONS ***************
  //******************************************

  pinCoil1 = PE14; //59
  pinCoil2 = PE15; //58
  //pinCoil3 = A0;
/*
    pinCoil3 = PE13; //61
    pinCoil4 = PE12; //60
    pinCoil5 = PE11; //63
    pinCoil6 = PF15; //68
    pinCoil7 = PG0;  //69
    pinCoil8 = PG1;  //66
*/

  //******************************************
  //******** OTHER CONNECTIONS ***************
  //******************************************

  pinTachOut = PD14;    //10
  /*
    pinIdle1 = PD15;      //11
    pinIdle2 = PG2;       //12
    pinBoost = PG3;       //13
  */
  pinStepperDir = PF7;  //93
  pinStepperStep = PF8; //91
  pinStepperEnable = PF9; //94
  pinFuelPump = PG6;    //16
  pinFan = PG7;         //17

}
void resetPins()
{
  pinInjector1 = BOARD_MAX_IO_PINS - 1;
  pinInjector2 = BOARD_MAX_IO_PINS - 1;
  pinInjector3 = BOARD_MAX_IO_PINS - 1;
  pinInjector4 = BOARD_MAX_IO_PINS - 1;
  pinInjector5 = BOARD_MAX_IO_PINS - 1;
  pinInjector6 = BOARD_MAX_IO_PINS - 1;
  pinInjector7 = BOARD_MAX_IO_PINS - 1;
  pinInjector8 = BOARD_MAX_IO_PINS - 1;
  injectorOutputControl = 0;
  pinCoil1 = BOARD_MAX_IO_PINS - 1;
  pinCoil2 = BOARD_MAX_IO_PINS - 1;
  pinCoil3 = BOARD_MAX_IO_PINS - 1;
  pinCoil4 = BOARD_MAX_IO_PINS - 1;
  pinCoil5 = BOARD_MAX_IO_PINS - 1;
  pinCoil6 = BOARD_MAX_IO_PINS - 1;
  pinCoil7 = BOARD_MAX_IO_PINS - 1;
  pinCoil8 = BOARD_MAX_IO_PINS - 1;
  ignitionOutputControl = 0;
  pinTrigger = BOARD_MAX_IO_PINS - 1;
  pinTrigger2 = BOARD_MAX_IO_PINS - 1;
  pinTrigger3 = BOARD_MAX_IO_PINS - 1;
  pinTPS = BOARD_MAX_IO_PINS - 1;
  pinMAP = BOARD_MAX_IO_PINS - 1;
  pinEMAP = BOARD_MAX_IO_PINS - 1;
  pinMAP2 = BOARD_MAX_IO_PINS - 1;
  pinIAT = BOARD_MAX_IO_PINS - 1;
  pinCLT = BOARD_MAX_IO_PINS - 1;
  pinO2 = BOARD_MAX_IO_PINS - 1;
  pinO2_2 = BOARD_MAX_IO_PINS - 1;
  pinBat = BOARD_MAX_IO_PINS - 1;
  pinDisplayReset = BOARD_MAX_IO_PINS - 1;
  pinTachOut = BOARD_MAX_IO_PINS - 1;
  pinFuelPump = BOARD_MAX_IO_PINS - 1;
  pinIdle1 = BOARD_MAX_IO_PINS - 1;
  pinIdle2 = BOARD_MAX_IO_PINS - 1;
  pinIdleUp = BOARD_MAX_IO_PINS - 1;
  pinIdleUpOutput = BOARD_MAX_IO_PINS - 1;
  pinCTPS = BOARD_MAX_IO_PINS - 1;
  pinFuel2Input = BOARD_MAX_IO_PINS - 1;
  pinSpark2Input = BOARD_MAX_IO_PINS - 1;
  pinSpareTemp1 = BOARD_MAX_IO_PINS - 1;
  pinSpareTemp2 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut1 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut2 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut3 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut4 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut5 = BOARD_MAX_IO_PINS - 1;
  pinSpareOut6 = BOARD_MAX_IO_PINS - 1;
  pinSpareHOut1 = BOARD_MAX_IO_PINS - 1;
  pinSpareHOut2 = BOARD_MAX_IO_PINS - 1;
  pinSpareLOut1 = BOARD_MAX_IO_PINS - 1;
  pinSpareLOut2 = BOARD_MAX_IO_PINS - 1;
  pinSpareLOut3 = BOARD_MAX_IO_PINS - 1;
  pinSpareLOut4 = BOARD_MAX_IO_PINS - 1;
  pinSpareLOut5 = BOARD_MAX_IO_PINS - 1;
  pinBoost = BOARD_MAX_IO_PINS - 1;
  pinVVT_1 = BOARD_MAX_IO_PINS - 1;
  pinVVT_2 = BOARD_MAX_IO_PINS - 1;
  pinFan = BOARD_MAX_IO_PINS - 1;
  pinStepperDir = BOARD_MAX_IO_PINS - 1;
  pinStepperStep = BOARD_MAX_IO_PINS - 1;
  pinStepperEnable = BOARD_MAX_IO_PINS - 1;
  pinLaunch = BOARD_MAX_IO_PINS - 1;
  pinIgnBypass = BOARD_MAX_IO_PINS - 1;
  pinFlex = BOARD_MAX_IO_PINS - 1;
  pinVSS = BOARD_MAX_IO_PINS - 1;
  pinBaro = BOARD_MAX_IO_PINS - 1;
  pinResetControl = BOARD_MAX_IO_PINS - 1;
  pinFuelPressure = BOARD_MAX_IO_PINS - 1;
  pinOilPressure = BOARD_MAX_IO_PINS - 1;
  pinWMIEmpty = BOARD_MAX_IO_PINS - 1;
  pinWMIIndicator = BOARD_MAX_IO_PINS - 1;
  pinWMIEnabled = BOARD_MAX_IO_PINS - 1;
  pinMC33810_1_CS = BOARD_MAX_IO_PINS - 1;
  pinMC33810_2_CS = BOARD_MAX_IO_PINS - 1;
}

void runLoop()
{
  if ((Serial.available()) > 0)
  {
    digitalToggle(LED_COMS);
  }
  else
  {
    digitalWrite(LED_COMS, LOW);
  }

  digitalWrite(LED_ALERT, currentStatus.engineProtectStatus);
#ifdef USE_CAN_DASH
  dash_generic(&Can1);
#endif
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //1 hertz
  {
#ifdef USE_I2C_BARO
    float pressure = 0.0f;   // ← inizializza sempre
    float temperature = 0.0f;
    if (LPS_Sensor.GetPressure(&pressure)    == LPS25HB_STATUS_OK &&
        LPS_Sensor.GetTemperature(&temperature) == LPS25HB_STATUS_OK) {
        currentStatus.fuelTemp = temperature;
        currentStatus.baroADC = pressure / 10.0f;
    }
#endif

    //DBWMotor.move_revolution(4);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ)) //4 hertz
  {
    readOPSt(); // Activate the sensor PPM reading interrupt
    digitalToggle(LED_RUNNING);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //10 hertz
  {
    //digitalToggle(LED_ALERT);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) //15 hertz
  {
    //Timer10.setCaptureCompare(1, abs(2048), RESOLUTION_12B_COMPARE_FORMAT);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ)) //30 hertz
  {
  }
}

void dash_generic(STM32_CAN *can)
{
  //BMW iDrive controller
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ))
  {
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
  {
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ))
  {
  }
}

#endif
