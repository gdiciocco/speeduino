#ifndef OPF_CORE
#define OPF_CORE

#include "opf_core.h"

#ifdef USE_I2C_BARO
TwoWire LPS_dev(PIN_WIRE_SDA, PIN_WIRE_SCL);
LPS25HBSensor LPS_Sensor(&LPS_dev, LPS25HB_ADDRESS_LOW);
#endif //USE_I2C_BARO

#ifdef USE_DBW_IFX9201

HardwareTimer Timer10(TIM10);
IFX9201 IFX9201_HBridge = IFX9201();

#endif //USE_DBW_IFX9201

void setupBoard()
{
  resetPins();
  setPins();
  configPage2.pinMapping = 60;

  //STATUS LED
  pinMode(LED_RUNNING, OUTPUT);
  digitalWrite(LED_RUNNING, LOW);
  pinMode(LED_WARNING, OUTPUT);
  digitalWrite(LED_WARNING, LOW);
  pinMode(LED_ALERT, OUTPUT);
  digitalWrite(LED_ALERT, LOW);
  pinMode(LED_COMS, OUTPUT);
  digitalWrite(LED_COMS, LOW);

#ifdef USE_I2C_BARO
  LPS_dev.begin();
  LPS_Sensor.begin();
  LPS_Sensor.SetODR(7.0f);
  LPS_Sensor.Enable();
#endif //USE_I2C_BARO

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

  initialiseAll();

}

#ifdef USE_DBW_IFX9201
void dbwScheduleInterrupt()
{
  digitalToggle(LED_WARNING);
}
#endif //USE_DBW_IFX9201

void setPins()
{
        //Pin definitions for experimental board Tjeerd 
        //Black F407VE wiki.stm32duino.com/index.php?title=STM32F407
        //https://github.com/Tjeerdie/SPECTRE/tree/master/SPECTRE_V0.5
        
        //******************************************
        //******** PORTA CONNECTIONS *************** 
        //******************************************
        // = PA0; //Wakeup ADC123
        // = PA1; //ADC123
        // = PA2; //ADC123
        // = PA3; //ADC123
        // = PA4; //ADC12
        // = PA5; //ADC12
        // = PA6; //ADC12 LED_BUILTIN_1
        // = PA7; //ADC12 LED_BUILTIN_2
        pinCoil3 = PA8;
        // = PA9;  //TXD1=Bluetooth module
        // = PA10; //RXD1=Bluetooth module
        // = PA11; //(DO NOT USE FOR SPEEDUINO) USB
        // = PA12; //(DO NOT USE FOR SPEEDUINO) USB 
        // = PA13;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        // = PA14;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        // = PA15;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK

        //******************************************
        //******** PORTB CONNECTIONS *************** 
        //******************************************
        // = PB0;  //(DO NOT USE FOR SPEEDUINO) ADC123 - SPI FLASH CHIP CS pin
        pinBaro = PB1; //ADC12
        // = PB2;  //(DO NOT USE FOR SPEEDUINO) BOOT1 
        // = PB3;  //(DO NOT USE FOR SPEEDUINO) SPI1_SCK FLASH CHIP
        // = PB4;  //(DO NOT USE FOR SPEEDUINO) SPI1_MISO FLASH CHIP
        // = PB5;  //(DO NOT USE FOR SPEEDUINO) SPI1_MOSI FLASH CHIP
        // = PB6;  //NRF_CE
        // = PB7;  //NRF_CS
        // = PB8;  //NRF_IRQ
        pinCoil2 = PB9; //
        // = PB9;  //
        // = PB10; //TXD3
        // = PB11; //RXD3
        // = PB12; //
        // = PB13;  //SPI2_SCK
        // = PB14;  //SPI2_MISO
        // = PB15;  //SPI2_MOSI

        //******************************************
        //******** PORTC CONNECTIONS *************** 
        //******************************************
        pinIAT = PC0; //ADC123 
        pinTPS = PC1; //ADC123
        pinMAP = PC2; //ADC123 
        pinCLT = PC3; //ADC123
        pinO2 = PC4; //ADC12
        pinBat = PC5;  //ADC12
        pinBoost = PC6; //
        pinIdle1 = PC7; //
        // = PC8;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D0
        // = PC9;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D1
        // = PC10;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D2
        // = PC11;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D3
        // = PC12;  //(DO NOT USE FOR SPEEDUINO) - SDIO_SCK
        pinTachOut = PC13; //
        // = PC14;  //(DO NOT USE FOR SPEEDUINO) - OSC32_IN
        // = PC15;  //(DO NOT USE FOR SPEEDUINO) - OSC32_OUT

        //******************************************
        //******** PORTD CONNECTIONS *************** 
        //******************************************
        // = PD0;  //CANRX
        // = PD1;  //CANTX
        // = PD2;  //(DO NOT USE FOR SPEEDUINO) - SDIO_CMD
        pinIdle2 = PD3; //
        // = PD4;  //
        pinFlex = PD4;
        // = PD5; //TXD2
        // = PD6;  //RXD2
        pinCoil1 = PD7; //
        // = PD7;  //
        // = PD8;  //
        pinCoil5 = PD9;//
        pinCoil4 = PD10;//
        // = PD11;  //
        pinInjector1 = PD12; //
        pinInjector2 = PD13; //

        //******************************************
        //******** PORTE CONNECTIONS *************** 
        //******************************************
        pinTrigger = PE0; //
        pinTrigger2 = PE1; //
        pinStepperEnable = PE2; //
        pinFuelPump = PE3; //ONBOARD KEY1
        // = PE4;  //ONBOARD KEY2
        pinStepperStep = PE5; //
        pinFan = PE6; //
        pinStepperDir = PE7; //
        // = PE8;  //
        pinInjector5 = PE9; //
        // = PE10;  //
        pinInjector6 = PE11; //
        // = PE12; //
        pinInjector8 = PE13; //
        pinInjector7 = PE14; //
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
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //1 hertz
  {
#ifdef USE_I2C_BARO
    float pressure;
    float temperature;
    LPS_Sensor.GetPressure(&pressure);
    LPS_Sensor.GetTemperature(&temperature);
    currentStatus.fuelTemp = temperature;
    currentStatus.baro = pressure / 10.0f;
    digitalToggle(LED_RUNNING);
#endif

    //DBWMotor.move_revolution(4);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ)) //4 hertz
  {
  //  digitalToggle(LED_RUNNING);
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


#endif
