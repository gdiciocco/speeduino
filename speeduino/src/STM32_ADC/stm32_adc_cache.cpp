#include "stm32_adc_cache.h"

#if defined(STM32_ADC_CACHE_AVAILABLE)

#include <Arduino.h>
#include <analog.h>
#include <pinmap.h>

#ifndef ADC_REGULAR_RANK_1
#define ADC_REGULAR_RANK_1 1U
#endif

static constexpr uint8_t STM32_ADC_MAX_CHANNELS = 16U;

struct Stm32AdcCacheChannel {
  uint8_t pin;
  PinName pinName;
  uint32_t channel;
  uint16_t value;
};

static Stm32AdcCacheChannel adcChannels[STM32_ADC_MAX_CHANNELS];
static volatile uint16_t adcDmaValues[STM32_ADC_MAX_CHANNELS];
static uint8_t adcChannelCount = 0U;
static bool adcDmaActive = false;

static ADC_HandleTypeDef adcHandle;
static DMA_HandleTypeDef adcDmaHandle;

static void stopDma(void)
{
  if(adcDmaActive)
  {
    (void)HAL_ADC_Stop_DMA(&adcHandle);
    adcDmaActive = false;
  }

  if(adcHandle.Instance != nullptr)
  {
    (void)HAL_ADC_DeInit(&adcHandle);
  }
  if(adcDmaHandle.Instance != nullptr)
  {
    (void)HAL_DMA_DeInit(&adcDmaHandle);
  }
}

static int8_t findPinIndex(uint8_t pin)
{
  for(uint8_t index = 0U; index < adcChannelCount; index++)
  {
    if(adcChannels[index].pin == pin) { return (int8_t)index; }
  }

  return -1;
}

static bool getPinAdcChannel(uint8_t pin, PinName &pinName, uint32_t &channel)
{
  pinName = analogInputToPinName(pin);
  if(pinName == NC) { return false; }
  if(pinmap_peripheral(pinName, PinMap_ADC) != ADC1) { return false; }

  uint32_t bank = 0U;
  channel = get_adc_channel(pinName, &bank);
  return IS_ADC_CHANNEL(channel);
}

static uint32_t adcResolution10Bit(void)
{
#if defined(ADC_RESOLUTION_10B)
  return ADC_RESOLUTION_10B;
#else
  return ADC_RESOLUTION_12B;
#endif
}

static uint16_t readInitialPinValue(uint8_t pin)
{
  return (uint16_t)analogRead(pin);
}

static bool configureDma(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  adcDmaHandle = {};
  adcDmaHandle.Instance                 = DMA2_Stream0;
  adcDmaHandle.Init.Channel             = DMA_CHANNEL_0;
  adcDmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  adcDmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  adcDmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  adcDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  adcDmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  adcDmaHandle.Init.Mode                = DMA_CIRCULAR;
  adcDmaHandle.Init.Priority            = DMA_PRIORITY_LOW;
  adcDmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

  return HAL_DMA_Init(&adcDmaHandle) == HAL_OK;
}

static bool configureAdc(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  adcHandle = {};
  adcHandle.Instance                      = ADC1;
  adcHandle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  adcHandle.Init.Resolution               = adcResolution10Bit();
  adcHandle.Init.ScanConvMode             = ENABLE;
  adcHandle.Init.ContinuousConvMode       = ENABLE;
  adcHandle.Init.DiscontinuousConvMode    = DISABLE;
  adcHandle.Init.NbrOfDiscConversion      = 0U;
  adcHandle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adcHandle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  adcHandle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  adcHandle.Init.NbrOfConversion          = adcChannelCount;
  adcHandle.Init.DMAContinuousRequests    = ENABLE;
  adcHandle.Init.EOCSelection             = ADC_EOC_SEQ_CONV;

  return HAL_ADC_Init(&adcHandle) == HAL_OK;
}

static bool configureChannels(void)
{
  for(uint8_t index = 0U; index < adcChannelCount; index++)
  {
    ADC_ChannelConfTypeDef channelConfig = {};
    channelConfig.Channel = adcChannels[index].channel;
    channelConfig.Rank = ADC_REGULAR_RANK_1 + index;
    channelConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if(HAL_ADC_ConfigChannel(&adcHandle, &channelConfig) != HAL_OK)
    {
      return false;
    }
  }

  return true;
}

static bool startDma(void)
{
  if(adcChannelCount == 0U) { return false; }

  stopDma();
  for(uint8_t index = 0U; index < adcChannelCount; index++)
  {
    adcDmaValues[index] = adcChannels[index].value;
  }

  if(!configureDma())
  {
    stopDma();
    return false;
  }
  if(!configureAdc())
  {
    stopDma();
    return false;
  }

  __HAL_LINKDMA(&adcHandle, DMA_Handle, adcDmaHandle);

  if(!configureChannels())
  {
    stopDma();
    return false;
  }

  if(HAL_ADC_Start_DMA(&adcHandle, (uint32_t *)adcDmaValues, adcChannelCount) != HAL_OK)
  {
    stopDma();
    return false;
  }

  adcDmaActive = true;
  return true;
}

void stm32AdcCacheBegin(void)
{
  stopDma();
  adcHandle = {};
  adcDmaHandle = {};
  adcChannelCount = 0U;
}

bool stm32AdcCacheRegisterPin(uint8_t pin)
{
  if(findPinIndex(pin) >= 0) { return true; }
  if(adcChannelCount >= STM32_ADC_MAX_CHANNELS) { return false; }

  PinName pinName = NC;
  uint32_t channel = 0U;
  if(!getPinAdcChannel(pin, pinName, channel)) { return false; }

  const bool restartDma = adcDmaActive;
  if(restartDma) { stopDma(); }

#ifdef INPUT_ANALOG
  pinMode(pin, INPUT_ANALOG);
#else
  pinMode(pin, INPUT);
#endif

  adcChannels[adcChannelCount] = {
    pin,
    pinName,
    channel,
    readInitialPinValue(pin)
  };
  adcDmaValues[adcChannelCount] = adcChannels[adcChannelCount].value;
  adcChannelCount++;

  if(restartDma) { return startDma(); }
  return true;
}

void stm32AdcCacheStart(void)
{
  (void)startDma();
}

bool stm32AdcCacheRead(uint8_t pin, uint16_t &value)
{
  int8_t index = findPinIndex(pin);
  if(index < 0)
  {
    if(!stm32AdcCacheRegisterPin(pin)) { return false; }
    index = findPinIndex(pin);
  }

  if(index < 0) { return false; }

  value = adcDmaActive ? adcDmaValues[index] : adcChannels[index].value;
  return true;
}

#endif
