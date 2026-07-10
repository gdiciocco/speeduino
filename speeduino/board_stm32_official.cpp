#include "board_definition.h"

#if defined(STM32_CORE_VERSION_MAJOR)
#include "auxiliaries.h"
#include "idle.h"
#include "HardwareTimer.h"
#include "timers.h"
#include "comms_secondary.h"
#include "scheduler_ignition_controller.h"
#include "scheduler_fuel_controller.h"
#include "PeripheralPins.h"
#include "pinmap.h"
#include "crc32.h"
#include <FastCRC.h>

/*
***********************************************************************************************************
* Hardware CRC32 (see board_stm32_official.h)
*/
static bool hwCrcAvailable = false;
static bool hwCrcBusy = false; //A byte stream currently owns the unit: one-shots must not clobber its state

//One step of the standard reflected CRC32, used for the <4 byte tail the word-only hardware cannot take
static inline uint32_t crc32_soft_update(uint32_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0U; i < 8U; i++) { crc = (crc >> 1U) ^ (0xEDB88320UL & (0UL - (crc & 1UL))); }
  return crc;
}

uint32_t crc32_oneshot(const uint8_t *pData, uint16_t length)
{
  if (hwCrcAvailable && (!hwCrcBusy))
  {
    CRC->CR = CRC_CR_RESET;
    const uint8_t *pWord = pData;
    uint16_t words = length >> 2U;
    while (words != 0U)
    {
      uint32_t word;
      (void)memcpy(&word, pWord, sizeof(word));
      CRC->DR = __RBIT(word);
      pWord += sizeof(word);
      words--;
    }
    uint32_t crc = __RBIT(CRC->DR); //After reset with no words fed this reads as the 0xFFFFFFFF init value
    for (uint16_t i = length & ~(uint16_t)3U; i < length; i++) { crc = crc32_soft_update(crc, pData[i]); }
    return ~crc;
  }

  FastCRC32 crcCalc;
  return crcCalc.crc32(pData, length);
}

void crc32ByteStream_t::begin(void)
{
  window = 0U;
  count = 0U;
  crc = 0xFFFFFFFFUL;
  useHw = hwCrcAvailable && (!hwCrcBusy);
  if (useHw)
  {
    hwCrcBusy = true;
    CRC->CR = CRC_CR_RESET;
  }
}

void crc32ByteStream_t::push(uint8_t value)
{
  if (useHw)
  {
    window |= (uint32_t)value << (8U * (count & 3U));
    count++;
    if ((count & 3U) == 0U)
    {
      CRC->DR = __RBIT(window);
      window = 0U;
    }
  }
  else { crc = crc32_soft_update(crc, value); }
}

uint32_t crc32ByteStream_t::finish(void)
{
  if (useHw)
  {
    crc = __RBIT(CRC->DR);
    for (uint16_t i = 0U; i < (count & 3U); i++) { crc = crc32_soft_update(crc, (uint8_t)(window >> (8U * i))); }
    hwCrcBusy = false;
  }
  return ~crc;
}

/*
***********************************************************************************************************
* Primary trigger input capture (see board_stm32_official.h)
*
* The trigger keeps using the EXTI interrupt to run the decoder; the spare timer channel only latches
* the exact edge time in hardware. primaryTriggerEdgeTimeMicros() reconstructs the edge time from the
* capture register: age_of_edge = CNT - CCR (the timer ticks at 1MHz, so ticks are µS), edge time =
* micros() - age. The CCxIF flag is consumed on each read: if the capture ever stops following the pin
* (E.g. a pinMode()/attachInterrupt() call reverted the pin out of AF mode), the flag stays clear and
* the function silently degrades to plain micros().
*/
static HardwareTimer *captureTimer = nullptr;
static TIM_TypeDef * volatile captureTIM = nullptr; //nullptr disables the ISR side. Must be written last during setup
static volatile uint32_t *captureCCR = nullptr;
static uint32_t captureFlag = 0;

static bool timerIsFreeForCapture(const void *peripheral)
{
  //Timers not used for schedules (TIM1-5) or the 1ms tick (TIM11, or TIM4 on F103)
  bool isFree = false;
  #if defined(TIM8_BASE)
    isFree = isFree || (peripheral == TIM8);
  #endif
  #if defined(TIM9_BASE)
    isFree = isFree || (peripheral == TIM9);
  #endif
  #if defined(TIM10_BASE)
    isFree = isFree || (peripheral == TIM10);
  #endif
  #if defined(TIM12_BASE)
    isFree = isFree || (peripheral == TIM12);
  #endif
  #if defined(TIM13_BASE)
    isFree = isFree || (peripheral == TIM13);
  #endif
  #if defined(TIM14_BASE)
    isFree = isFree || (peripheral == TIM14);
  #endif
  return isFree;
}

void initPrimaryTriggerCapture(uint8_t pin, uint8_t edge)
{
  captureTIM = nullptr; //Disable the ISR side while reconfiguring

  if (captureTimer != nullptr)
  {
    captureTimer->pause();
    delete captureTimer;
    captureTimer = nullptr;
  }

  if ((edge != RISING) && (edge != FALLING) && (edge != CHANGE)) { return; }
  const PinName pinName = digitalPinToPinName(pin);
  if (pinName == NC) { return; }

  for (const PinMap *entry = PinMap_TIM; entry->pin != NC; entry++)
  {
    if ( ((uint32_t)entry->pin & ~(uint32_t)ALTX_MASK) == (uint32_t)pinName
      && timerIsFreeForCapture(entry->peripheral)
      && (STM_PIN_INVERTED(entry->function) == 0U) ) //TIMx_CHxN complementary inputs cannot capture
    {
      TIM_TypeDef *tim = (TIM_TypeDef *)entry->peripheral;
      const uint32_t channel = STM_PIN_CHANNEL(entry->function);

      captureTimer = new HardwareTimer(tim);
      captureTimer->setPrescaleFactor(captureTimer->getTimerClkFreq() / 1000000U); //1 tick = 1µS, so capture ages are directly in µS
      captureTimer->setOverflow(0x10000U, TICK_FORMAT); //Free running over the full 16 bit range
      const TimerModes_t mode = (edge == RISING) ? TIMER_INPUT_CAPTURE_RISING :
                               ((edge == FALLING) ? TIMER_INPUT_CAPTURE_FALLING : TIMER_INPUT_CAPTURE_BOTHEDGE);
      //Passing the map entry's own PinName (which may be an ALTx alias) guarantees the pin is AF-routed to *this* timer.
      //This must run after attachInterrupt(): AF mode keeps the input path alive, so the EXTI keeps firing
      captureTimer->setMode(channel, mode, entry->pin);
      captureTimer->resume(); //No channel interrupt is attached; the capture register is read from the trigger ISR

      captureCCR = &tim->CCR1 + (channel - 1U);
      captureFlag = (uint32_t)TIM_SR_CC1IF << (channel - 1U);
      tim->SR = 0U; //Discard captures that occurred during setup
      captureTIM = tim;
      return;
    }
  }
}

uint32_t primaryTriggerEdgeTimeMicros(void)
{
  TIM_TypeDef *tim = captureTIM;
  if (tim != nullptr)
  {
    if ((tim->SR & captureFlag) != 0U)
    {
      tim->SR = ~captureFlag; //Registers are rc_w0: this clears only our channel's capture flag
      const uint16_t age = (uint16_t)tim->CNT - (uint16_t)*captureCCR;
      return micros() - age;
    }
  }
  return micros();
}

bool primaryTriggerCaptureActive(void)
{
  return captureTIM != nullptr;
}

#if defined(BOARD_FCR_MICRO_F4)
extern "C" void __real_pinMode(uint8_t pin, uint8_t mode);
extern "C" void __wrap_pinMode(uint8_t pin, uint8_t mode)
{
  if (pinIsReserved(pin)) { return; }
  __real_pinMode(pin, mode);
}

__attribute__((constructor)) static void fcrInitFlashChipSelect(void)
{
  __real_pinMode((uint8_t)USE_SPI_EEPROM, OUTPUT);
  digitalWrite((uint8_t)USE_SPI_EEPROM, HIGH);
}

// The stm32duino generic F429VITx variant ships an EMPTY (weak) SystemClock_Config(), so the chip
// would run on the 16MHz High Speed Internal reset clock with no 48MHz USB clock and USB never enumerates.
// Override it for the FCR Micro F4: 8MHz HSE -> 168MHz SYSCLK, PLLQ=7 -> 48MHz for USB FS.
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;    // 8MHz HSE / 8 = 1MHz PLL input
  RCC_OscInitStruct.PLL.PLLN       = 336;  // 1MHz * 336 = 336MHz VCO
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2; // 336/2 = 168MHz SYSCLK
  RCC_OscInitStruct.PLL.PLLQ       = 7;    // 336/7 = 48MHz USB/SDIO clock
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1; // 168MHz AHB
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   // 42MHz APB1 (max 45)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   // 84MHz APB2 (max 90)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}
#endif

#if defined(SRAM_AS_EEPROM) // Use 4K battery backed SRAM, requires a 3V continuous source (like battery) connected to Vbat pin
  #include "src/BackupSram/BackupSramAsEEPROM.h"
  BackupSramAsEEPROM EEPROM;
#elif defined(USE_SPI_EEPROM) // Use M25Qxx SPI flash on BlackF407VE
  #include "src/SPIAsEEPROM/SPIAsEEPROM.h"
    #if defined(STM32F407xx)
      SPIClass SPI_for_flash(PB5, PB4, PB3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
    #elif defined(BOARD_FCR_MICRO_F4)
      SPIClass SPI_for_flash(PC12, PC11, PC10); //SPI3_MOSI, SPI3_MISO, SPI3_SCK (FCR Micro F4)
    #else //Blue/Black Pills
      SPIClass SPI_for_flash(PB15, PB14, PB13);
    #endif
 
    //winbond W25Q16 SPI flash EEPROM emulation
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{255UL, 4096UL, 31, 0x00100000UL};
    Flash_SPI_Config SPIconfig{USE_SPI_EEPROM, SPI_for_flash};
    SPI_EEPROM_Class EEPROM(EmulatedEEPROMMconfig, SPIconfig);
#elif defined(FRAM_AS_EEPROM) // Use FRAM like FM25xxx, MB85RSxxx or any SPI compatible
  #include "src/FRAM/Fram.h"
  #if defined(STM32F407xx)
    SPIClass SPI_for_FRAM(PB5, PB4, PB3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
    FramClass EEPROM(PB0, SPI_for_FRAM);
  #else //Blue/Black Pills
    SPIClass SPI_for_FRAM(PB15, PB14, PB13);
    FramClass EEPROM(PB12, SPI_for_FRAM);
  #endif
#else //default case, internal flash as EEPROM
  #include "src/SPIAsEEPROM/SPIAsEEPROM.h"
  #if defined(STM32F7xx)
    #if defined(DUAL_BANK)
      EEPROM_Emulation_Config EmulatedEEPROMMconfig{4UL, 131072UL, 2047UL, 0x08120000UL};
    #else
      EEPROM_Emulation_Config EmulatedEEPROMMconfig{2UL, 262144UL, 4095UL, 0x08180000UL};
    #endif
    InternalSTM32F7_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
  #elif defined(STM32F401xC)
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{1UL, 131072UL, 4095UL, 0x08020000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
  #elif defined(STM32F411xE)
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{2UL, 131072UL, 4095UL, 0x08040000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
  #else //default case, internal flash as EEPROM for STM32F4
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{4UL, 131072UL, 2047UL, 0x08080000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
  #endif
#endif
#include "board_eeprom_adapter.hpp"

#if HAL_CAN_MODULE_ENABLED
//This activates CAN1 interface on STM32, but it's named as Can0, because that's how Teensy implementation is done
STM32_CAN Can0 (CAN1, ALT_2, RX_SIZE_256, TX_SIZE_16);
/*
These CAN interfaces and pins are available for use, depending on the chip/package:
Default CAN1 pins are PA11 and PA12. Alternative (ALT) pins are PB8 & PB9 and ALT_2 pins are PD0 & PD1.
Default CAN2 pins are PB12 & PB13. Alternative (ALT) pins are PB5 & PB6.
Default CAN3 pins are PA8 & PA15. Alternative (ALT) pins are PB3 & PB4.
*/
#endif

#if defined SD_LOGGING
    SPIClass SD_SPI(PC12, PC11, PC10); //SPI3_MOSI, SPI3_MISO, SPI3_SCK
#endif

HardwareTimer Timer1(TIM1);
HardwareTimer Timer2(TIM2);
HardwareTimer Timer3(TIM3);
HardwareTimer Timer4(TIM4);
#if !defined(ARDUINO_BLUEPILL_F103C8) && !defined(ARDUINO_BLUEPILL_F103CB) //F103 just have 4 timers
HardwareTimer Timer5(TIM5);
#if defined(TIM11)
HardwareTimer Timer11(TIM11);
#elif defined(TIM7)
HardwareTimer Timer11(TIM7);
#endif
#endif

#ifdef RTC_ENABLED
STM32RTC& rtc = STM32RTC::getInstance();
#endif

  /*
  ***********************************************************************************************************
  * Interrupt callback functions
  */
  #define IGNITION_INTERRUPT_NAME(index) CONCAT(CONCAT(ignitionSchedule, index), Interrupt)
  #define FUEL_INTERRUPT_NAME(index) CONCAT(CONCAT(fuelSchedule, index), Interrupt)


  #if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1)) 
  void oneMSInterval(HardwareTimer*){oneMSInterval();}
  void boostInterrupt(HardwareTimer*){boostInterrupt();}
  void idleInterrupt(HardwareTimer*){idleInterrupt();}
  void vvtInterrupt(HardwareTimer*){vvtInterrupt();}
  void fanInterrupt(HardwareTimer*){fanInterrupt();}
  #define STM_FUEL_INTERRUPT(index) void FUEL_INTERRUPT_NAME(index)(HardwareTimer*) {moveToNextState(fuelSchedule ## index);}
  #define STM_IGNITION_INTERRUPT(index) void IGNITION_INTERRUPT_NAME(index)(HardwareTimer*) {moveToNextState(ignitionSchedule ## index);}
  #else //End core<=1.8
  #define STM_FUEL_INTERRUPT(index) void FUEL_INTERRUPT_NAME(index)(void) {moveToNextState(fuelSchedule ## index);}
  #define STM_IGNITION_INTERRUPT(index) void IGNITION_INTERRUPT_NAME(index)(void) {moveToNextState(ignitionSchedule ## index);}
  #endif

  STM_FUEL_INTERRUPT(1)
  #if (INJ_CHANNELS >= 2)
  STM_FUEL_INTERRUPT(2)
  #endif
  #if (INJ_CHANNELS >= 3)
  STM_FUEL_INTERRUPT(3)
  #endif
  #if (INJ_CHANNELS >= 4)
  STM_FUEL_INTERRUPT(4)
  #endif
  #if (INJ_CHANNELS >= 5)
  STM_FUEL_INTERRUPT(5)
  #endif
  #if (INJ_CHANNELS >= 6)
  STM_FUEL_INTERRUPT(6)
  #endif
  #if (INJ_CHANNELS >= 7)
  STM_FUEL_INTERRUPT(7)
  #endif
  #if (INJ_CHANNELS >= 8)
  STM_FUEL_INTERRUPT(8)
  #endif

  STM_IGNITION_INTERRUPT(1)
  #if (IGN_CHANNELS >= 2)
  STM_IGNITION_INTERRUPT(2)
  #endif
  #if (IGN_CHANNELS >= 3)
  STM_IGNITION_INTERRUPT(3)
  #endif
  #if (IGN_CHANNELS >= 4)
  STM_IGNITION_INTERRUPT(4)
  #endif
  #if (IGN_CHANNELS >= 5)
  STM_IGNITION_INTERRUPT(5)
  #endif
  #if (IGN_CHANNELS >= 6)
  STM_IGNITION_INTERRUPT(6)
  #endif
  #if (IGN_CHANNELS >= 7)
  STM_IGNITION_INTERRUPT(7)
  #endif
  #if (IGN_CHANNELS >= 8)
  STM_IGNITION_INTERRUPT(8)
  #endif


  void initBoard(uint32_t baudRate)
  {
    /*
    ***********************************************************************************************************
    * General
    */
    delay(10);

    #ifndef HAVE_HWSERIAL2 //Hack to get the code to compile on BlackPills
    #define Serial2 Serial1
    #endif
    pSecondarySerial = &Serial2;

    /*
    ***********************************************************************************************************
    * Real Time clock for datalogging/time stamping
    */
    #ifdef RTC_ENABLED
      //Check if RTC time has been set earlier. If yes, RTC will use LSE_CLOCK. If not, default LSI_CLOCK is used, to prevent hanging on boot.
      if (rtc.isTimeSet()) {
        rtc.setClockSource(STM32RTC::LSE_CLOCK); //Initialise external clock for RTC if clock is set. That is the only clock running of VBAT
      }
      rtc.begin(); // initialise RTC 24H format
    #endif
    /*
    ***********************************************************************************************************
    * Idle
    */
    if (isPwmIac(configPage6))
    {
        idle_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.idleFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 5KHz
    } 

    //This must happen at the end of the idle init
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer1.setMode(4, TIMER_OUTPUT_COMPARE);
    #else
    Timer1.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer1.attachInterrupt(4, idleInterrupt);  //on first flash the configPage4.iacAlgorithm is invalid


    /*
    ***********************************************************************************************************
    * Timers
    */
    #if defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_BLUEPILL_F103CB)
      Timer4.setOverflow(1000, MICROSEC_FORMAT);  // Set up period
      #if ( STM32_CORE_VERSION_MAJOR < 2 )
      Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
      Timer4.attachInterrupt(1, oneMSInterval);
      #else //2.0 forward
      Timer4.attachInterrupt(oneMSInterval);
      #endif
      Timer4.resume(); //Start Timer
    #else
      Timer11.setOverflow(1000, MICROSEC_FORMAT);  // Set up period
      #if ( STM32_CORE_VERSION_MAJOR < 2 )
      Timer11.setMode(1, TIMER_OUTPUT_COMPARE);
      Timer11.attachInterrupt(1, oneMSInterval);
      #else
      Timer11.attachInterrupt(oneMSInterval);
      #endif
      Timer11.resume(); //Start Timer
    #endif
    pinMode(LED_BUILTIN, OUTPUT); //Visual WDT

    /*
    ***********************************************************************************************************
    * Auxiliaries
    */
    //2uS resolution Min 8Hz, Max 5KHz
    boost_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.boostFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow frequencies up to 511Hz
    vvt_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.vvtFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle
    fan_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.fanFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle

    //Need to be initialised last due to instant interrupt
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer1.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
	Timer1.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer1.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer1.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer1.attachInterrupt(1, fanInterrupt);
    Timer1.attachInterrupt(2, boostInterrupt);
    Timer1.attachInterrupt(3, vvtInterrupt);

    /*
    ***********************************************************************************************************
    * Schedules
    */
    Timer1.setOverflow((numeric_limits<COMPARE_TYPE>::max)(), TICK_FORMAT);
    Timer2.setOverflow((numeric_limits<COMPARE_TYPE>::max)(), TICK_FORMAT);
    Timer3.setOverflow((numeric_limits<COMPARE_TYPE>::max)(), TICK_FORMAT);

    Timer1.setPrescaleFactor(((Timer1.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    Timer2.setPrescaleFactor(((Timer2.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    Timer3.setPrescaleFactor(((Timer3.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution

    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(4, TIMER_OUTPUT_COMPARE);

    Timer3.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);

    Timer3.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    //Attach interrupt functions
    //Injection
    Timer3.attachInterrupt(1, FUEL_INTERRUPT_NAME(1));
    #if (INJ_CHANNELS >= 2)
    Timer3.attachInterrupt(2, FUEL_INTERRUPT_NAME(2));
    #endif
    #if (INJ_CHANNELS >= 3)
    Timer3.attachInterrupt(3, FUEL_INTERRUPT_NAME(3));
    #endif
    #if (INJ_CHANNELS >= 4)
    Timer3.attachInterrupt(4, FUEL_INTERRUPT_NAME(4));
    #endif
    #if (INJ_CHANNELS >= 5)
    Timer5.setOverflow((numeric_limits<COMPARE_TYPE>::max)(), TICK_FORMAT);
    Timer5.setPrescaleFactor(((Timer5.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(1, FUEL_INTERRUPT_NAME(5));
    #endif
    #if (INJ_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(2, FUEL_INTERRUPT_NAME(6));
    #endif
    #if (INJ_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(3, FUEL_INTERRUPT_NAME(7));
    #endif
    #if (INJ_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(4, FUEL_INTERRUPT_NAME(8));
    #endif

    //Ignition
    Timer2.attachInterrupt(1, IGNITION_INTERRUPT_NAME(1)); 
    #if (IGN_CHANNELS >= 2)
    Timer2.attachInterrupt(2, IGNITION_INTERRUPT_NAME(2));
    #endif
    #if (IGN_CHANNELS >= 3)
    Timer2.attachInterrupt(3, IGNITION_INTERRUPT_NAME(3));
    #endif
    #if (IGN_CHANNELS >= 4)
    Timer2.attachInterrupt(4, IGNITION_INTERRUPT_NAME(4));
    #endif
    #if (IGN_CHANNELS >= 5)
    Timer4.setOverflow((numeric_limits<COMPARE_TYPE>::max)(), TICK_FORMAT);
    Timer4.setPrescaleFactor(((Timer4.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(1, IGNITION_INTERRUPT_NAME(5));
    #endif
    #if (IGN_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(2, IGNITION_INTERRUPT_NAME(6));
    #endif
    #if (IGN_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(3, IGNITION_INTERRUPT_NAME(7));
    #endif
    #if (IGN_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(4, IGNITION_INTERRUPT_NAME(8));
    #endif

    /*
    ***********************************************************************************************************
    * Hardware CRC32
    */
    #if defined(RCC_AHB1ENR_CRCEN)
      RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
      (void)RCC->AHB1ENR; //Read back to ensure the clock is running before first use
    #elif defined(RCC_AHBENR_CRCEN)
      RCC->AHBENR |= RCC_AHBENR_CRCEN;
      (void)RCC->AHBENR;
    #endif
    hwCrcAvailable = true;
    {
      //Verify the RBIT technique against the standard CRC32 check vector (exercises both the word path and the tail path). On failure fall back to FastCRC
      static const uint8_t crcCheckVector[9] = {'1','2','3','4','5','6','7','8','9'};
      hwCrcAvailable = (crc32_oneshot(crcCheckVector, sizeof(crcCheckVector)) == 0xCBF43926UL);
    }

    #if defined(SRAM_AS_EEPROM)
    //rtc.begin()/setClockSource above can force a backup domain reset, which clears the RTC backup
    //registers holding the backup SRAM CRC seal (the SRAM itself is unaffected). Reseal now, which
    //also covers any storage writes made by doUpdates()/reset handling before initBoard() ran.
    EEPROM.sealCrc();
    #endif

    /*
    ***********************************************************************************************************
    * Interrupt priorities (0 = highest. Core defaults: SysTick=0, EXTI/trigger=6, USB=1, UART=1, all timers=14)
    * The fuel/ignition compare ISRs must sit below the trigger wheel EXTI (so a compare ISR can never delay a
    * trigger edge timestamp) but above everything non engine-critical. USB/UART default to priority 1, which
    * lets comms preempt the trigger: they are lowered via the USBD_IRQ_PRIO/UART_IRQ_PRIO build flags in
    * platformio.ini, as those apply to core files compiled outside this project.
    */
    #if ( STM32_CORE_VERSION_MAJOR >= 2 ) //setInterruptPriority is not available on old cores; their default (14) still leaves the trigger EXTI (6) on top
    Timer1.setInterruptPriority(8, 0);  //Idle/boost/VVT/fan software PWM
    Timer2.setInterruptPriority(7, 0);  //Schedules
    Timer3.setInterruptPriority(7, 0);  //Schedules
    #if defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_BLUEPILL_F103CB)
      Timer4.setInterruptPriority(9, 0);  //1ms tick
    #else
      Timer4.setInterruptPriority(7, 0);  //Schedules 5-8
      Timer5.setInterruptPriority(7, 0);  //Schedules 5-8
      Timer11.setInterruptPriority(9, 0); //1ms tick
    #endif
    #endif

    Serial.begin(baudRate);
  }

  uint16_t freeRam()
  {
    uint32_t freeRam = 0;
    uint32_t stackTop = 0;
    uint32_t heapTop = 0;

    // current position of the stack.
    stackTop = (uint32_t)&stackTop;

    // current position of heap.
    void *hTop = malloc(1);
    heapTop = (uint32_t)hTop;
    free(hTop);
    freeRam = stackTop - heapTop;

    return min((uint32_t)(numeric_limits<uint16_t>::max)(), freeRam);
  }

  void doSystemReset( void )
  {
    __disable_irq();
    NVIC_SystemReset();
  }

  void jumpToBootloader( void ) // https://github.com/3devo/Arduino_Core_STM32/blob/jumpSysBL/libraries/SrcWrapper/src/stm32/bootloader.c
  { // https://github.com/markusgritsch/SilF4ware/blob/master/SilF4ware/drv_reset.c
    #if !defined(STM32F103xB)
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->VAL = SysTick->LOAD = SysTick->CTRL = 0;
    SYSCFG->MEMRMP = 0x01;

    #if defined(STM32F7xx) || defined(STM32H7xx)
    const uint32_t DFU_addr = 0x1FF00000; // From AN2606
    #else
    const uint32_t DFU_addr = 0x1FFF0000; // Default for STM32F10xxx and STM32F40xxx/STM32F41xxx from AN2606
    #endif
    // This is assembly to prevent modifying the stack pointer after
    // loading it, and to ensure a jump (not call) to the bootloader.
    // Not sure if the barriers are really needed, they were taken from
    // https://github.com/GrumpyOldPizza/arduino-STM32L4/blob/ac659033eadd50cfe001ba1590a1362b2d87bb76/system/STM32L4xx/Source/boot_stm32l4xx.c#L159-L165
    asm volatile (
      "ldr r0, [%[DFU_addr], #0]   \n\t"  // get address of stack pointer
      "msr msp, r0            \n\t"  // set stack pointer
      "ldr r0, [%[DFU_addr], #4]   \n\t"  // get address of reset handler
      "dsb                    \n\t"  // data sync barrier
      "isb                    \n\t"  // instruction sync barrier
      "bx r0                  \n\t"  // branch to bootloader
      : : [DFU_addr] "l" (DFU_addr) : "r0"
    );
    __builtin_unreachable();
    #endif
  }


uint8_t getSystemTemp(void)
{
  //stm32F4xx does have an internal temperature sensor, but needs to be implemented
  return 0;
}

void boardInitRTC(void)
{
  // Do nothing
}


void boardInitPins(uint8_t)
{
  // Do nothing
}

static uint16_t getEepromWriteBlockSize(const statuses &current)
{
#if defined(USE_SPI_EEPROM)
  //For use with common Winbond SPI EEPROMs Eg W25Q16JV
  uint16_t maxWrite = 20; //This needs tuning
#else
  uint16_t maxWrite = 64;
#endif

  // Write to EEPROM more aggressively if the engine is not running
  if(current.RPM==0U)
  { 
    return maxWrite * 8U;
  } 

  return maxWrite;
}

/** @brief Get the EEPROM storage API for the board */
storage_api_t getBoardStorageApi(void)
{
#if defined(SRAM_AS_EEPROM)
  //Boot-time integrity check of the battery backed SRAM against its flash snapshot.
  //This is the first storage call made by initialiseAll(), before any page is loaded and before
  //initBoard()/schedules/triggers run: ALL flash operations (erase included) happen inside here,
  //never once the ECU is operational. Runs once; later calls are a no-op.
  (void)backupSramBootSync();
#endif
  return getEEPROMStorageApi(getEepromWriteBlockSize);
}

#endif
