#pragma once

/** @file
 * @brief CRC32 helpers (Ethernet/zlib: polynomial 0x04C11DB7 reflected, init 0xFFFFFFFF, final XOR).
 *
 * Boards with a hardware CRC unit define BOARD_HAS_HW_CRC32 and implement these functions;
 * other platforms fall back to FastCRC. Results are identical to FastCRC32::crc32().
 *
 * @warning The hardware unit holds a single computation state: a crc32ByteStream_t must be
 * begun and finished within one call scope, with no crc32_oneshot() call in between. Neither
 * may be used from an interrupt.
 */

#include <stdint.h>
#include "board_definition.h"

#if defined(BOARD_HAS_HW_CRC32)

/** @brief CRC32 of a complete buffer */
uint32_t crc32_oneshot(const uint8_t *pData, uint16_t length);

/** @brief Accumulates a CRC32 one byte at a time, for data without a contiguous buffer (E.g. TS pages) */
struct crc32ByteStream_t
{
  void begin(void);
  void push(uint8_t value);
  uint32_t finish(void);
private:
  uint32_t window;
  uint32_t crc;
  uint16_t count;
  bool useHw;
};

#else

#include <FastCRC.h>

static inline uint32_t crc32_oneshot(const uint8_t *pData, uint16_t length)
{
  FastCRC32 crcCalc;
  return crcCalc.crc32(pData, length);
}

struct crc32ByteStream_t
{
  void begin(void) { started = false; crc = 0U; }
  void push(uint8_t value)
  {
    if (started) { crc = crcCalc.crc32_upd(&value, 1U); }
    else { crc = crcCalc.crc32(&value, 1U); started = true; }
  }
  uint32_t finish(void) { return crc; }
private:
  FastCRC32 crcCalc;
  uint32_t crc = 0U;
  bool started = false;
};

#endif
