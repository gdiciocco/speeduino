#include <unity.h>
#include "../test_utils.h"
#include "logger.h"

/** @file
 * @brief is2ByteEntry() against a linear scan of the same table.
 *
 * The function is a hand-rolled binary search with a `bot += mid++ / 2U` in the
 * middle of it, over a hand-maintained array. That shape works for the length
 * it was written against and there is no reason to assume it works for another
 * one - and the array just grew from 46 entries to 70, because everything this
 * fork appended to the live data block was missing from it.
 *
 * So this checks every index a byte-wide source selector can name, against the
 * only definition that cannot itself be wrong: the list, read straight through.
 */

// The same values as logger.cpp. Deliberately a second copy: a test that
// imported the array would agree with the search by construction and prove
// nothing about whether the search finds what the list says.
static const uint16_t twoByteEntries[] = {
  4, 14, 17, 22, 26, 28, 33, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 76,
  78, 80, 82, 86, 88, 90, 93, 95, 99, 104, 111, 121, 125, 130, 132, 134, 136, 140, 142, 146, 148,
  150, 155, 158, 168, 176, 180, 182, 195, 197, 199, 204, 216, 218, 220, 225, 227, 229, 231, 233,
  235, 244, 246, 248, 250, 253
};

static bool listedAsTwoByte(uint16_t index)
{
  for (uint16_t i = 0U; i < (uint16_t)_countof(twoByteEntries); ++i)
  {
    if (twoByteEntries[i] == index) { return true; }
  }
  return false;
}

static void test_is2ByteEntry_matches_the_list_exhaustively(void)
{
  for (uint16_t index = 0U; index <= 255U; ++index)
  {
    char message[48];
    sprintf(message, "index %" PRIu16, index);
    TEST_ASSERT_EQUAL_MESSAGE(listedAsTwoByte(index), is2ByteEntry(index), message);
  }
}

// The array must stay sorted or the search is meaningless, and it is edited by
// hand every time a 16-bit channel is added.
static void test_two_byte_entries_are_ascending(void)
{
  for (uint16_t i = 1U; i < (uint16_t)_countof(twoByteEntries); ++i)
  {
    TEST_ASSERT_GREATER_THAN_UINT16(twoByteEntries[i - 1U], twoByteEntries[i]);
  }
}

void testLogEntryWidth(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_is2ByteEntry_matches_the_list_exhaustively);
    RUN_TEST_P(test_two_byte_entries_are_ascending);
  }
}
