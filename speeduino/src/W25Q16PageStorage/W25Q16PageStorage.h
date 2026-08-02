#pragma once

#if defined(W25Q16_PAGE_STORAGE) && defined(STM32F407xx)

#include <Arduino.h>
#include <SPI.h>
#include "../SPIAsEEPROM/winbondflash.h"

enum class W25Q16BootStatus : uint8_t {
  NotRun,
  PrimaryValid,
  BackupUpdated,
  PrimaryRestored,
  Formatted,
  PrimaryUnavailable,
  BackupWriteFailed,
  RestoreFailed,
  RuntimeFull,
  RuntimeWriteFailed
};

/**
 * A 4096-byte EEPROM-compatible store backed by atomic W25Q16 snapshots.
 *
 * Sectors 0-1 contain an append-only commit log. Sectors 2-511 contain complete
 * 4KiB images. At runtime only erased sectors are programmed and the commit record
 * is written last; sector erase and garbage collection happen exclusively at boot.
 */
class W25Q16PageStorage {
public:
  W25Q16PageStorage(uint8_t chipSelect, SPIClass &spi);

  uint8_t read(uint16_t address) const;
  int8_t write(uint16_t address, uint8_t value);
  int8_t update(uint16_t address, uint8_t value) { return write(address, value); }
  uint16_t length(void) const { return 4096U; }

  template<typename T> T &get(int index, T &value) {
    uint8_t *p = reinterpret_cast<uint8_t *>(&value);
    for (size_t i = 0U; i < sizeof(T); i++) { p[i] = read((uint16_t)(index + i)); }
    return value;
  }

  template<typename T> const T &put(int index, const T &value) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&value);
    for (size_t i = 0U; i < sizeof(T); i++) { (void)write((uint16_t)(index + i), p[i]); }
    return value;
  }

  /** Initialize, recover if necessary and perform all boot-only erase/backup work. */
  W25Q16BootStatus bootSync(void);

  /** Program at most one 256-byte page (or one commit-record phase). Never erases. */
  void service(void);

  W25Q16BootStatus status(void) const { return _status; }

private:
  enum class CommitState : uint8_t { Idle, Data, RecordBody, RecordMagic };

  uint8_t _cache[4096];
  bool _slotAvailable[510];
  SPIClass *_spi;
  winbondFlashSPI _flash;
  uint8_t _chipSelect;
  bool _initialized;
  bool _flashAvailable;
  bool _dirty;
  CommitState _commitState;
  uint32_t _revision;
  uint32_t _commitRevision;
  uint32_t _generation;
  uint32_t _commitGeneration;
  uint32_t _commitCrc;
  uint32_t _lastWriteMillis;
  uint16_t _activeSector;
  uint16_t _targetSector;
  uint16_t _nextSector;
  uint16_t _recordIndex;
  uint8_t _pageIndex;
  W25Q16BootStatus _status;

  bool beginFlash(void);
  bool waitReady(uint32_t timeoutMs);
  bool readRaw(uint32_t address, uint8_t *data, uint16_t length);
  bool programRaw(uint32_t address, const uint8_t *data, uint16_t length);
  bool eraseSector(uint16_t sector);
  bool sectorErased(uint16_t sector);
  bool loadSector(uint16_t sector);
  uint32_t sectorCrc(uint16_t sector);
  bool programCacheToSector(uint16_t sector);
  bool writeCommitRecord(uint16_t recordIndex, uint16_t dataSector, uint32_t generation, uint32_t crc);
  bool compactAtBoot(uint16_t activeSector, uint32_t generation, uint32_t crc);
  bool findFreeSector(uint16_t &sector);
  void failRuntime(W25Q16BootStatus status);
};

extern W25Q16PageStorage EEPROM;

#endif
