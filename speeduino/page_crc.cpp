#include "crc32.h"
#include "page_crc.h"
#include "pages.h"

uint32_t __attribute__((optimize("Os"))) calculatePageCRC32(uint8_t pageNum)
{
    crc32ByteStream_t crcStream;
    crcStream.begin();

    crcStream.push(getPageValue(pageNum, 0));
    for (uint16_t offset=1; offset<getPageSize(pageNum); ++offset)
    {
        crcStream.push(getPageValue(pageNum, offset));
    }

    return crcStream.finish();
}
