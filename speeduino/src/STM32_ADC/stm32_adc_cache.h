#pragma once

#include <stdint.h>
#include "../../board_definition.h"

#if defined(CORE_STM32) && defined(HAL_ADC_MODULE_ENABLED) && (defined(STM32F4) || defined(STM32F4xx))
  #define STM32_ADC_CACHE_AVAILABLE
#endif

#if defined(STM32_ADC_CACHE_AVAILABLE)
void stm32AdcCacheBegin(void);
bool stm32AdcCacheRegisterPin(uint8_t pin);
void stm32AdcCacheStart(void);
bool stm32AdcCacheRead(uint8_t pin, uint16_t &value);
#else
static inline void stm32AdcCacheBegin(void) {}
static inline bool stm32AdcCacheRegisterPin(uint8_t) { return false; }
static inline void stm32AdcCacheStart(void) {}
static inline bool stm32AdcCacheRead(uint8_t, uint16_t &) { return false; }
#endif
