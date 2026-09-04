
#pragma once
#include "table3d.h"

#include <stdint.h>
#include <unity.h>
#include <Arduino.h>
#include <unity.h>
#include "table2d.h"
#include "table3d.h"
#include "maths.h"
#
template<size_t MAX_LEN, size_t N>
constexpr void STR_LEN_CHECK(char const (&)[N]) 
{
    static_assert(N < MAX_LEN, "String overflow!");
}

#if !defined(_countof)
#define _countof(x) (sizeof(x) / sizeof (x[0]))
#endif

#if !defined(NATIVE_BOARD) && defined(STM32_CORE_VERSION_MAJOR)
#include <IWatchdog.h>
#define HW_TEST_WATCHDOG_AVAILABLE

/** @brief Arm the independent watchdog on first use, then feed it.
 *
 * Arming lazily means it is only running while a suite is executing: after a
 * reset nothing has armed it yet, so the board can sit in the pre-test wait
 * indefinitely, and after the last test nothing feeds it, so it resets back
 * into that wait ~33s later. Both are what we want.
 */
static inline void hwTestFeedWatchdog(void)
{
    static bool armed = false;
    if (armed)
    {
        IWatchdog.reload();
    }
    else
    {
        IWatchdog.begin(IWDG_TIMEOUT_MAX);   // ~33s on the F4's 32kHz LSI
        armed = true;
    }
}
#endif

// A test that hangs on real hardware takes the board off the USB bus with
// it: no CDC, no DFU, and the only way back is BOOT0 and a reset by hand.
// So arm the independent watchdog and feed it once per test. A hang inside
// any one test stops the feeding and the board resets itself back to the
// window where tools/dfu_upload.py can pick it up.
//
// Per test, not per suite: the longest suites run well past the ~32s the
// F4's IWDG can be stretched to, but no single test comes close.
#if defined(HW_TEST_WATCHDOG_AVAILABLE)
#define HW_TEST_FEED_WATCHDOG() hwTestFeedWatchdog()
#else
#define HW_TEST_FEED_WATCHDOG() ((void)0)
#endif

// RUN_TEST_P used to copy the test name out of flash into a 128 byte stack
// buffer, because on AVR a string literal handed straight to
// UnityDefaultTestRun() lands in the data segment and eats SRAM. On ARM
// literals live in .rodata, so the copy - and the buffer, and its length
// check - bought nothing. The _P in the name is now vestigial.
#define RUN_TEST_P(func) \
  do { \
    HW_TEST_FEED_WATCHDOG(); \
    UnityDefaultTestRun(func, #func, __LINE__); \
  } while (0)

// ============================ SET_UNITY_FILENAME ============================ 

static inline uint8_t ufname_set(const char *newFName)
{
    Unity.TestFile = newFName;
    return 1;
}

static inline void ufname_szrestore(char** __s)
{
    Unity.TestFile = *__s;
    __asm__ volatile ("" ::: "memory");
}


#define UNITY_FILENAME_RESTORE char* _ufname_saved                           \
    __attribute__((__cleanup__(ufname_szrestore))) = (char*)Unity.TestFile

#define SET_UNITY_FILENAME()                                                        \
for ( UNITY_FILENAME_RESTORE, _ufname_done = ufname_set(__FILE__);                  \
    _ufname_done; _ufname_done = 0 )

// ============================ end SET_UNITY_FILENAME ============================ 

// Test data. Was pushed into flash with PROGMEM on AVR; with a single
// address space a constexpr array is already read-only data.
#define TEST_DATA static constexpr

template <typename table3d_t>
static inline void fill_table_values(table3d_t &table, table3d_value_t value) {
  // for (uint8_t i=0; i<table.values.row_size*table.values.num_rows; ++i) {
  //   table.values.values[i] = value;
  // }
  table_value_iterator itZ = table.values.begin();
  while (!itZ.at_end())
  {
    table_row_iterator itRow = *itZ;
    while (!itRow.at_end())
    {
      *itRow = value;
      ++itRow;
    }
    ++itZ;
  }  
  invalidate_cache(&table.get_value_cache);
}

static inline void populate_table_axis(table_axis_iterator it, 
                                       table3d_axis_t value) {
  while (!it.at_end())
  {
    *it = value;
    ++it;
  }
}

static inline void populate_table_axis(table_axis_iterator it, 
                                       const table3d_axis_t *pXValues) {
  while (!it.at_end())
  {
    *it = *pXValues;
    ++pXValues;
    ++it;
  }
}

// Populate a 3d table. The 3 source arrays are typically declared TEST_DATA.
template <typename table3d_t>
static inline void populate_table(table3d_t &table, 
                                  const table3d_axis_t *pXValues,
                                  const table3d_axis_t *pYValues,
                                  const table3d_value_t *pZValues)
{
  populate_table_axis(table.axisX.begin(), pXValues);
  populate_table_axis(table.axisY.begin(), pYValues);
  {
    table_value_iterator itZ = table.values.begin();
    while (!itZ.at_end())
    {
      table_row_iterator itRow = *itZ;
      while (!itRow.at_end())
      {
        *itRow = *pZValues;
        ++pZValues;
        ++itRow;
      }
      ++itZ;
    }
  }
}

// Populate a 2d table with constant values
template <typename axis_t, typename value_t, uint8_t sizeT>
static inline void populate_2dtable(table2D<axis_t, value_t, sizeT> *pTable, value_t value, axis_t bin) {
  for (uint8_t index=0; index<sizeT; ++index) {
    (value_t&)(pTable->values[index]) = value;
    (axis_t&)(pTable->axis[index]) = bin;
  }
  pTable->cache.cacheTime = UINT8_MAX;
}

template <typename axis_t, typename value_t, uint8_t sizeT>
static inline void populate_2dtable(table2D<axis_t, value_t, sizeT> *pTable, const value_t (&values)[sizeT], const axis_t (&bins)[sizeT]) {
  memcpy((void*)pTable->axis, bins, sizeT * sizeof(axis_t));
  memcpy((void*)pTable->values, values, sizeT * sizeof(value_t));
  pTable->cache.cacheTime = UINT8_MAX;
}


template <typename T>
T intermediate(T const& min, T const& max, uint8_t const& frac)
{
  if (max<min) {
    return min - percentage(frac, (min - max));
  }
  return min + percentage(frac, (max - min));
}
