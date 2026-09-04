#pragma once

/** @file
 * @brief Critical sections.
 *
 * ATOMIC() { ... } runs its block with interrupts masked and restores the
 * previous mask state on every exit path, including an early `break`.
 *
 * A board header may define ATOMIC() itself (board_native.h does, for the
 * unit-test host). Otherwise the Cortex-M implementation below is used.
 */

#include "board_definition.h"

#if !defined(ATOMIC)

#if !defined(__arm__) && !defined(__thumb__)
#error No ATOMIC() implementation for this architecture
#endif

#include <stdint.h>

/// @cond

/** @brief Read PRIMASK (0 = interrupts enabled) */
static inline uint32_t _atomicGetPrimask(void)
{
  uint32_t primask;
  __asm__ __volatile__ ("MRS %0, primask" : "=r" (primask));
  return primask;
}

/** @brief Mask all maskable interrupts. Returns a value so it can sit in a for-init */
static inline uint32_t _atomicDisableIrq(void)
{
  __asm__ __volatile__ ("cpsid i" ::: "memory");
  return 1U;
}

/** @brief Restore PRIMASK. Invoked by __cleanup__ on every exit from the block */
static inline void _atomicRestorePrimask(const uint32_t *pSaved)
{
  __asm__ __volatile__ ("MSR primask, %0" : : "r" (*pSaved) : "memory");
}

/// @endcond

/**
 * @brief Run the following block with interrupts masked.
 *
 * @note This masks via PRIMASK, i.e. *everything*. A BASEPRI-based section
 * could leave the low-priority comms IRQs (USBD_IRQ_PRIO=10, UART_IRQ_PRIO=11)
 * running, but it would have to keep masking the trigger EXTI (6) and the
 * schedule timers (7-9) - the trigger ISR calls setSchedule() itself, so
 * letting it preempt a section that is editing a schedule would be a
 * re-entrancy bug. The blocks are a few tens of cycles, so the gain would be
 * small and needs bench validation; PRIMASK is the safe default.
 */
#define ATOMIC()                                                             \
  for (uint32_t _atomicSaved __attribute__((__cleanup__(_atomicRestorePrimask))) \
         = _atomicGetPrimask(), _atomicDone = _atomicDisableIrq();           \
       _atomicDone; _atomicDone = 0U)

#endif
