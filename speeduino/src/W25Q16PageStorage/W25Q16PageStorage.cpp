#if defined(W25Q16_PAGE_STORAGE) && defined(STM32F407xx)

#include "W25Q16PageStorage.h"
#include "../../crc32.h"
#include <string.h>

namespace {

constexpr uint32_t STORAGE_BYTES = 4096UL;
constexpr uint16_t FLASH_SECTORS = 512U;
constexpr uint16_t METADATA_SECTORS = 2U;
constexpr uint16_t FIRST_DATA_SECTOR = METADATA_SECTORS;
constexpr uint16_t DATA_SECTORS = FLASH_SECTORS - METADATA_SECTORS;
constexpr uint32_t SECTOR_BYTES = 4096UL;
constexpr uint16_t PROGRAM_PAGE_BYTES = 256U;
constexpr uint16_t COMMIT_RECORDS = (METADATA_SECTORS * SECTOR_BYTES) / 16U;
constexpr uint32_t RECORD_MAGIC = 0x57323531UL; //'W251', format version 1
constexpr uint32_t COMMIT_QUIET_MS = 250UL;

constexpr uint32_t SNAPSHOT_BASE = 0x080E0000UL;
constexpr uint32_t SNAPSHOT_MAGIC_ADDR = SNAPSHOT_BASE;
constexpr uint32_t SNAPSHOT_CRC_ADDR = SNAPSHOT_BASE + 4UL;
constexpr uint32_t SNAPSHOT_DATA_ADDR = SNAPSHOT_BASE + 8UL;
constexpr uint32_t SNAPSHOT_MAGIC = 0x57324231UL; //'W2B1'
constexpr uint32_t SNAPSHOT_SECTOR = FLASH_SECTOR_11;
constexpr uint32_t LEGACY_SRAM_MAGIC = 0x53524D31UL; //'SRM1'

struct CommitRecord {
  uint32_t magic;
  uint32_t generation;
  uint32_t dataSector;
  uint32_t crc;
};

static_assert(sizeof(CommitRecord) == 16U, "W25Q16 commit record layout changed");

uint32_t recordAddress(uint16_t index)
{
  return (uint32_t)index * sizeof(CommitRecord);
}

bool generationNewer(uint32_t candidate, uint32_t current)
{
  return (int32_t)(candidate - current) > 0;
}

bool internalSnapshotValid(void)
{
  if (*(const uint32_t *)SNAPSHOT_MAGIC_ADDR != SNAPSHOT_MAGIC) { return false; }
  return crc32_oneshot((const uint8_t *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES)
      == *(const uint32_t *)SNAPSHOT_CRC_ADDR;
}

bool legacyInternalSnapshotValid(void)
{
  if (*(const uint32_t *)SNAPSHOT_MAGIC_ADDR != LEGACY_SRAM_MAGIC) { return false; }
  return crc32_oneshot((const uint8_t *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES)
      == *(const uint32_t *)SNAPSHOT_CRC_ADDR;
}

bool loadLegacyBackupSram(uint8_t *destination)
{
  //One-time migration path from the former SRAM_AS_EEPROM implementation. The
  //backup SRAM is never used again after its valid image has been copied to W25Q16.
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  PWR->CR |= PWR_CR_DBP;
  const uint32_t crc = crc32_oneshot((const uint8_t *)BKPSRAM_BASE, STORAGE_BYTES);
  if ((RTC->BKP18R != LEGACY_SRAM_MAGIC) || (RTC->BKP19R != crc)) { return false; }
  memcpy(destination, (const void *)BKPSRAM_BASE, STORAGE_BYTES);
  return true;
}

void flushInternalFlashCaches(void)
{
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

bool programInternalWord(uint32_t address, uint32_t value)
{
  return (value == 0xFFFFFFFFUL)
      || (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, value) == HAL_OK);
}

bool programInternalSnapshot(const uint8_t *data, uint32_t crc)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit = {};
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseInit.Sector = SNAPSHOT_SECTOR;
  eraseInit.NbSectors = 1U;
  uint32_t sectorError = 0U;
  bool success = (HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK);

  for (uint32_t offset = 0U; success && offset < STORAGE_BYTES; offset += sizeof(uint32_t))
  {
    uint32_t word;
    memcpy(&word, data + offset, sizeof(word));
    success = programInternalWord(SNAPSHOT_DATA_ADDR + offset, word);
  }
  if (success) { success = programInternalWord(SNAPSHOT_CRC_ADDR, crc); }
  if (success) { success = programInternalWord(SNAPSHOT_MAGIC_ADDR, SNAPSHOT_MAGIC); }

  HAL_FLASH_Lock();
  flushInternalFlashCaches();
  return success && internalSnapshotValid() && (*(const uint32_t *)SNAPSHOT_CRC_ADDR == crc);
}

} //namespace

W25Q16PageStorage::W25Q16PageStorage(uint8_t chipSelect, SPIClass &spi) :
  _spi(&spi),
  _chipSelect(chipSelect),
  _initialized(false),
  _flashAvailable(false),
  _dirty(false),
  _commitState(CommitState::Idle),
  _revision(0U),
  _commitRevision(0U),
  _generation(0U),
  _commitGeneration(0U),
  _commitCrc(0U),
  _lastWriteMillis(0U),
  _activeSector(UINT16_MAX),
  _targetSector(UINT16_MAX),
  _nextSector(FIRST_DATA_SECTOR),
  _recordIndex(0U),
  _pageIndex(0U),
  _status(W25Q16BootStatus::NotRun)
{
  memset(_cache, 0xFF, sizeof(_cache));
  memset(_slotAvailable, 0, sizeof(_slotAvailable));
}

uint8_t W25Q16PageStorage::read(uint16_t address) const
{
  return (address < sizeof(_cache)) ? _cache[address] : 0U;
}

int8_t W25Q16PageStorage::write(uint16_t address, uint8_t value)
{
  if (address >= sizeof(_cache)) { return -1; }
  if (_cache[address] == value) { return 0; }
  _cache[address] = value;
  _revision++;
  _dirty = true;
  _lastWriteMillis = millis();
  return 1;
}

bool W25Q16PageStorage::beginFlash(void)
{
  pinMode(_chipSelect, OUTPUT);
  digitalWrite(_chipSelect, HIGH);
  _spi->begin();
  _spi->beginTransaction(SPISettings(22500000UL, MSBFIRST, SPI_MODE0));
  _flashAvailable = _flash.begin(winbondFlashClass::partNumber::autoDetect, *_spi, _chipSelect)
                 && (_flash.bytes() == (long)(FLASH_SECTORS * SECTOR_BYTES));
  return _flashAvailable;
}

bool W25Q16PageStorage::waitReady(uint32_t timeoutMs)
{
  const uint32_t started = millis();
  while (_flash.busy())
  {
    if ((uint32_t)(millis() - started) >= timeoutMs) { return false; }
  }
  return true;
}

bool W25Q16PageStorage::readRaw(uint32_t address, uint8_t *data, uint16_t length)
{
  return waitReady(20U) && (_flash.read(address, data, length) == length);
}

bool W25Q16PageStorage::programRaw(uint32_t address, const uint8_t *data, uint16_t length)
{
  if (((address & 0xFFUL) + length) > PROGRAM_PAGE_BYTES) { return false; }
  _flash.setWriteEnable(true);
  _flash.writePage(address, const_cast<uint8_t *>(data), length);
  return waitReady(20U);
}

bool W25Q16PageStorage::eraseSector(uint16_t sector)
{
  if (sector >= FLASH_SECTORS) { return false; }
  _flash.setWriteEnable(true);
  _flash.eraseSector((uint32_t)sector * SECTOR_BYTES);
  return waitReady(1000U);
}

bool W25Q16PageStorage::sectorErased(uint16_t sector)
{
  uint8_t data[64];
  const uint32_t base = (uint32_t)sector * SECTOR_BYTES;
  for (uint32_t offset = 0U; offset < SECTOR_BYTES; offset += sizeof(data))
  {
    if (!readRaw(base + offset, data, sizeof(data))) { return false; }
    for (uint8_t value : data) { if (value != 0xFFU) { return false; } }
  }
  return true;
}

bool W25Q16PageStorage::loadSector(uint16_t sector)
{
  const uint32_t base = (uint32_t)sector * SECTOR_BYTES;
  for (uint32_t offset = 0U; offset < STORAGE_BYTES; offset += PROGRAM_PAGE_BYTES)
  {
    if (!readRaw(base + offset, _cache + offset, PROGRAM_PAGE_BYTES)) { return false; }
  }
  return true;
}

uint32_t W25Q16PageStorage::sectorCrc(uint16_t sector)
{
  uint8_t data[PROGRAM_PAGE_BYTES];
  crc32ByteStream_t crc;
  crc.begin();
  const uint32_t base = (uint32_t)sector * SECTOR_BYTES;
  for (uint32_t offset = 0U; offset < STORAGE_BYTES; offset += sizeof(data))
  {
    if (!readRaw(base + offset, data, sizeof(data))) { return 0U; }
    for (uint8_t value : data) { crc.push(value); }
  }
  return crc.finish();
}

bool W25Q16PageStorage::programCacheToSector(uint16_t sector)
{
  const uint32_t base = (uint32_t)sector * SECTOR_BYTES;
  uint8_t verify[PROGRAM_PAGE_BYTES];
  for (uint32_t offset = 0U; offset < STORAGE_BYTES; offset += PROGRAM_PAGE_BYTES)
  {
    if (!programRaw(base + offset, _cache + offset, PROGRAM_PAGE_BYTES)
        || !readRaw(base + offset, verify, sizeof(verify))
        || (memcmp(verify, _cache + offset, sizeof(verify)) != 0)) { return false; }
  }
  return true;
}

bool W25Q16PageStorage::writeCommitRecord(uint16_t index, uint16_t dataSector, uint32_t generation, uint32_t crc)
{
  if (index >= COMMIT_RECORDS) { return false; }
  const CommitRecord record = { RECORD_MAGIC, generation, dataSector, crc };
  const uint32_t address = recordAddress(index);
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&record);
  if (!programRaw(address + sizeof(record.magic), raw + sizeof(record.magic), sizeof(record) - sizeof(record.magic))) { return false; }
  if (!programRaw(address, raw, sizeof(record.magic))) { return false; }
  CommitRecord verify;
  return readRaw(address, reinterpret_cast<uint8_t *>(&verify), sizeof(verify))
      && (memcmp(&record, &verify, sizeof(record)) == 0);
}

bool W25Q16PageStorage::compactAtBoot(uint16_t activeSector, uint32_t generation, uint32_t crc)
{
  for (uint16_t sector = FIRST_DATA_SECTOR; sector < FLASH_SECTORS; sector++)
  {
    const uint16_t index = sector - FIRST_DATA_SECTOR;
    if (sector == activeSector) { _slotAvailable[index] = false; continue; }
    if (!sectorErased(sector) && !eraseSector(sector)) { return false; }
    _slotAvailable[index] = sectorErased(sector);
    if (!_slotAvailable[index]) { return false; }
  }

  //The internal backup is valid before this point. Therefore an interruption while
  //rebuilding the metadata log is recoverable at the next boot.
  if (!eraseSector(0U) || !eraseSector(1U)) { return false; }
  if (!writeCommitRecord(0U, activeSector, generation, crc)) { return false; }

  _activeSector = activeSector;
  _generation = generation;
  _recordIndex = 1U;
  _nextSector = (activeSector + 1U < FLASH_SECTORS) ? activeSector + 1U : FIRST_DATA_SECTOR;
  return true;
}

bool W25Q16PageStorage::findFreeSector(uint16_t &sector)
{
  for (uint16_t count = 0U; count < DATA_SECTORS; count++)
  {
    const uint16_t candidate = FIRST_DATA_SECTOR
        + (uint16_t)((_nextSector - FIRST_DATA_SECTOR + count) % DATA_SECTORS);
    if (_slotAvailable[candidate - FIRST_DATA_SECTOR])
    {
      _slotAvailable[candidate - FIRST_DATA_SECTOR] = false;
      _nextSector = (candidate + 1U < FLASH_SECTORS) ? candidate + 1U : FIRST_DATA_SECTOR;
      sector = candidate;
      return true;
    }
  }
  return false;
}

W25Q16BootStatus W25Q16PageStorage::bootSync(void)
{
  if (_status != W25Q16BootStatus::NotRun) { return _status; }

  if (!beginFlash())
  {
    if (internalSnapshotValid())
    {
      memcpy(_cache, (const void *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES);
      _initialized = true; //Read-only fallback: service() remains disabled without the W25Q16.
    }
    else if (loadLegacyBackupSram(_cache))
    {
      _initialized = true;
    }
    else if (legacyInternalSnapshotValid())
    {
      memcpy(_cache, (const void *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES);
      _initialized = true;
    }
    _status = W25Q16BootStatus::PrimaryUnavailable;
    return _status;
  }

  bool havePrimary = false;
  CommitRecord newest = {};
  for (uint16_t index = 0U; index < COMMIT_RECORDS; index++)
  {
    CommitRecord record;
    if (!readRaw(recordAddress(index), reinterpret_cast<uint8_t *>(&record), sizeof(record))) { continue; }
    if ((record.magic != RECORD_MAGIC)
        || (record.dataSector < FIRST_DATA_SECTOR)
        || (record.dataSector >= FLASH_SECTORS)) { continue; }
    if (havePrimary && !generationNewer(record.generation, newest.generation)) { continue; }
    if (sectorCrc((uint16_t)record.dataSector) != record.crc) { continue; }
    newest = record;
    havePrimary = true;
  }

  uint16_t activeSector = FIRST_DATA_SECTOR;
  uint32_t generation = 0U;
  uint32_t crc;
  W25Q16BootStatus result;

  if (havePrimary)
  {
    activeSector = (uint16_t)newest.dataSector;
    generation = newest.generation;
    if (!loadSector(activeSector)) { _status = W25Q16BootStatus::RestoreFailed; return _status; }
    crc = newest.crc;
    if (internalSnapshotValid() && (*(const uint32_t *)SNAPSHOT_CRC_ADDR == crc))
    {
      result = W25Q16BootStatus::PrimaryValid;
    }
    else
    {
      result = programInternalSnapshot(_cache, crc) ? W25Q16BootStatus::BackupUpdated
                                                    : W25Q16BootStatus::BackupWriteFailed;
    }
  }
  else
  {
    bool rewriteInternalBackup = false;
    if (internalSnapshotValid())
    {
      memcpy(_cache, (const void *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES);
      result = W25Q16BootStatus::PrimaryRestored;
    }
    else if (loadLegacyBackupSram(_cache))
    {
      result = W25Q16BootStatus::PrimaryRestored;
      rewriteInternalBackup = true;
    }
    else if (legacyInternalSnapshotValid())
    {
      memcpy(_cache, (const void *)SNAPSHOT_DATA_ADDR, STORAGE_BYTES);
      result = W25Q16BootStatus::PrimaryRestored;
      rewriteInternalBackup = true;
    }
    else
    {
      memset(_cache, 0xFF, sizeof(_cache));
      result = W25Q16BootStatus::Formatted;
    }
    crc = crc32_oneshot(_cache, sizeof(_cache));
    if (!eraseSector(activeSector) || !programCacheToSector(activeSector))
    {
      _status = W25Q16BootStatus::RestoreFailed;
      return _status;
    }
    if (((result == W25Q16BootStatus::Formatted) || rewriteInternalBackup)
        && !programInternalSnapshot(_cache, crc))
    {
      _status = W25Q16BootStatus::BackupWriteFailed;
      return _status;
    }
  }

  //Do not destroy stale external commit records unless the internal recovery copy is known-good.
  if (result == W25Q16BootStatus::BackupWriteFailed)
  {
    _initialized = true;
    _flashAvailable = false; //Keep serving the valid RAM image, but disable runtime commits.
    _status = result;
    return _status;
  }

  if (!compactAtBoot(activeSector, generation, crc))
  {
    _status = W25Q16BootStatus::RestoreFailed;
    return _status;
  }

  _dirty = false;
  _initialized = true;
  _status = result;
  return _status;
}

void W25Q16PageStorage::failRuntime(W25Q16BootStatus status)
{
  if ((_commitState == CommitState::RecordBody) || (_commitState == CommitState::RecordMagic))
  {
    _recordIndex++; //The record body may be partially programmed and cannot safely be reused.
  }
  _commitState = CommitState::Idle;
  _status = status;
  _lastWriteMillis = millis();
}

void W25Q16PageStorage::service(void)
{
  if (!_initialized || !_flashAvailable) { return; }

  if ((_commitState != CommitState::Idle) && (_revision != _commitRevision))
  {
    //Leave the uncommitted target sector untouched. It has no commit record and is
    //reclaimed at the next boot; restart from another pre-erased sector after quiet time.
    if ((_commitState == CommitState::RecordBody) || (_commitState == CommitState::RecordMagic)) { _recordIndex++; }
    _commitState = CommitState::Idle;
    _lastWriteMillis = millis();
  }

  if (_commitState == CommitState::Idle)
  {
    if (!_dirty || ((uint32_t)(millis() - _lastWriteMillis) < COMMIT_QUIET_MS)) { return; }
    if ((_recordIndex >= COMMIT_RECORDS) || !findFreeSector(_targetSector))
    {
      _status = W25Q16BootStatus::RuntimeFull;
      return;
    }
    _commitRevision = _revision;
    _commitGeneration = _generation + 1U;
    _commitCrc = crc32_oneshot(_cache, sizeof(_cache));
    _pageIndex = 0U;
    _commitState = CommitState::Data;
  }

  if (_commitState == CommitState::Data)
  {
    const uint32_t offset = (uint32_t)_pageIndex * PROGRAM_PAGE_BYTES;
    const uint32_t address = (uint32_t)_targetSector * SECTOR_BYTES + offset;
    uint8_t verify[PROGRAM_PAGE_BYTES];
    if (!programRaw(address, _cache + offset, PROGRAM_PAGE_BYTES)
        || !readRaw(address, verify, sizeof(verify))
        || (memcmp(verify, _cache + offset, sizeof(verify)) != 0))
    {
      failRuntime(W25Q16BootStatus::RuntimeWriteFailed);
      return;
    }
    _pageIndex++;
    if (_pageIndex == (STORAGE_BYTES / PROGRAM_PAGE_BYTES)) { _commitState = CommitState::RecordBody; }
    return;
  }

  const CommitRecord record = { RECORD_MAGIC, _commitGeneration, _targetSector, _commitCrc };
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&record);
  const uint32_t address = recordAddress(_recordIndex);

  if (_commitState == CommitState::RecordBody)
  {
    if (!programRaw(address + sizeof(record.magic), raw + sizeof(record.magic), sizeof(record) - sizeof(record.magic)))
    {
      failRuntime(W25Q16BootStatus::RuntimeWriteFailed);
      return;
    }
    _commitState = CommitState::RecordMagic;
    return;
  }

  if (!programRaw(address, raw, sizeof(record.magic)))
  {
    failRuntime(W25Q16BootStatus::RuntimeWriteFailed);
    return;
  }

  CommitRecord verify;
  if (!readRaw(address, reinterpret_cast<uint8_t *>(&verify), sizeof(verify))
      || (memcmp(&record, &verify, sizeof(record)) != 0))
  {
    failRuntime(W25Q16BootStatus::RuntimeWriteFailed);
    return;
  }

  _activeSector = _targetSector;
  _generation = _commitGeneration;
  _recordIndex++;
  _dirty = false;
  _commitState = CommitState::Idle;
}

#endif
