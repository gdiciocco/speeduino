
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

// This used to copy the test name out of flash into a 128 byte stack buffer,
// because on AVR a string literal handed straight to UnityDefaultTestRun()
// lands in the data segment and eats SRAM. On ARM literals live in .rodata,
// so the copy - and the buffer, and its length check - bought nothing.
//
// The _P in the name is now vestigial. Renaming the ~500 call sites to
// Unity's own RUN_TEST() is a separate mechanical change.
#define RUN_TEST_P(func) UnityDefaultTestRun(func, #func, __LINE__)

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
