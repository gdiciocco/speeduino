#if defined(STM32F407xx)
#include <Arduino.h>
#include <string.h>
#include "BackupSramAsEEPROM.h"
#include "../../crc32.h"

//CRC seal of the backup SRAM image, kept in the RTC backup registers: same VBAT domain as the
//SRAM itself, but outside the 4KB so the EEPROM address space/layout is untouched.
//BKP18R/BKP19R are the highest registers, away from the ones bootloaders commonly use (DR0/DR4/DR10).
#define SEAL_MAGIC_REG  (RTC->BKP18R)
#define SEAL_CRC_REG    (RTC->BKP19R)
#define SEAL_MAGIC      0x53524D31UL //'SRM1': bump if the seal/snapshot layout ever changes

//Flash snapshot of the backup SRAM image: sector 11, the last 128KB of the 1MB F407 flash
//(the same hidden-1MB region the internal flash EEPROM emulation mode already relies on).
//Layout: [magic32][crc32 of data][4096 bytes of data]. The magic is programmed last so a
//power cut mid-program leaves an invalid (0xFFFFFFFF) magic and the snapshot is retried.
#define SNAPSHOT_BASE       0x080E0000UL
#define SNAPSHOT_SECTOR     FLASH_SECTOR_11
#define SNAPSHOT_MAGIC_ADDR (SNAPSHOT_BASE)
#define SNAPSHOT_CRC_ADDR   (SNAPSHOT_BASE + 4UL)
#define SNAPSHOT_DATA_ADDR  (SNAPSHOT_BASE + 8UL)
#define BKPSRAM_SIZE        4096U

    BackupSramAsEEPROM::BackupSramAsEEPROM(){
          //Enable the power interface clock
          RCC->APB1ENR |= RCC_APB1ENR_PWREN;

          //Enable the backup SRAM clock by setting BKPSRAMEN bit i
          RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;

          /** If the HSE divided by 2, 3, ..31 is used as the RTC clock, the
            * Backup Domain Access should be kept enabled. */

          // Enable access to Backup domain
          PWR->CR |= PWR_CR_DBP;

          /** enable the backup regulator (used to maintain the backup SRAM content in
            * standby and Vbat modes).  NOTE : this bit is not reset when the device
            * wakes up from standby, system reset or power reset. You can check that
            * the backup regulator is ready on PWR->CSR.brr, see rm p144 */

          //Enable the backup power regulator. This makes the sram backup possible. bit is not reset by software!
          PWR->CSR |= PWR_CSR_BRE;

          //Wait until the backup power regulator is ready
          while ((PWR->CSR & PWR_CSR_BRR) == 0);
    }
    uint16_t BackupSramAsEEPROM::length(){ return 4096; }
    int8_t BackupSramAsEEPROM::write_byte( uint8_t *data, uint16_t bytes, uint16_t offset ) {
        uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
        uint16_t i;
          if( bytes + offset >= backup_size ) {
            /* ERROR : the last byte is outside the backup SRAM region */
            return -1;
          }

          /* disable backup domain write protection */
          //Set the Disable Backup Domain write protection (DBP) bit in PWR power control register
          //PWR->CR |= PWR_CR_DBP;

          for( i = 0; i < bytes; i++ ) {
            *(base_addr + offset + i) = *(data + i);
          }
          //Enable write protection backup sram when finished
          //PWR->CR &= ~PWR_CR_DBP;
          crcDirty = true; //The stored CRC seal no longer matches; sealCrcIfDirty() will refresh it
          return 0;
        }

    int8_t BackupSramAsEEPROM::read_byte( uint8_t *data, uint16_t bytes, uint16_t offset ) {
          uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
          uint16_t i;
          if( bytes + offset >= backup_size ) {
            /* ERROR : the last byte is outside the backup SRAM region */
            return -1;
          }

          for( i = 0; i < bytes; i++ ) {
            *(data + i) = *(base_addr + offset + i);
          }
          return 0;
        }

    uint8_t BackupSramAsEEPROM::read(uint16_t address) {
        uint8_t val = 0;
        read_byte(&val, 1, address);

        return val;
    }

    int8_t BackupSramAsEEPROM::write(uint16_t address, uint8_t val) {
        write_byte(&val, 1, address);
        return 0;
    }

    int8_t BackupSramAsEEPROM::update(uint16_t address, uint8_t val) {
        write_byte(&val, 1, address);
        return 0;
    }

    void BackupSramAsEEPROM::sealCrc(void) {
        crcDirty = false;
        SEAL_CRC_REG = crc32_oneshot((const uint8_t *)BKPSRAM_BASE, BKPSRAM_SIZE);
        SEAL_MAGIC_REG = SEAL_MAGIC;
    }

    void BackupSramAsEEPROM::sealCrcIfDirty(void) {
        if (crcDirty) { sealCrc(); }
    }

/*
***********************************************************************************************************
* Boot-time integrity check & flash snapshot sync
*/

static BackupSramBootStatus bootStatus = BackupSramBootStatus::NotRun;

static bool isSramSealValid(uint32_t sramCrc)
{
  return (SEAL_MAGIC_REG == SEAL_MAGIC) && (SEAL_CRC_REG == sramCrc);
}

static bool isSnapshotValid(void)
{
  if (*(const uint32_t *)SNAPSHOT_MAGIC_ADDR != SEAL_MAGIC) { return false; }
  return crc32_oneshot((const uint8_t *)SNAPSHOT_DATA_ADDR, BKPSRAM_SIZE) == *(const uint32_t *)SNAPSHOT_CRC_ADDR;
}

//The ART accelerator caches can serve stale flash contents after an erase/program
static void flushFlashCaches(void)
{
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

static bool programWords(uint32_t address, const uint32_t *pData, uint32_t words)
{
  for (uint32_t i = 0U; i < words; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * 4U), pData[i]) != HAL_OK) { return false; }
  }
  return true;
}

//Erase the snapshot sector and reprogram it from the backup SRAM. The CPU stalls on flash
//fetches during the erase (~1-2s), which is why this only ever runs from backupSramBootSync().
static bool programSnapshot(uint32_t sramCrc)
{
  //Clear any stale flash error flags, otherwise erase/program can fail spuriously
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseInit.Sector = SNAPSHOT_SECTOR;
  eraseInit.NbSectors = 1;
  uint32_t sectorError = 0;
  bool success = (HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK);

  if (success) { success = programWords(SNAPSHOT_DATA_ADDR, (const uint32_t *)BKPSRAM_BASE, BKPSRAM_SIZE / 4U); }
  if (success) { success = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SNAPSHOT_CRC_ADDR, sramCrc) == HAL_OK); }
  //Magic goes in last: an interrupted program leaves it 0xFFFFFFFF and the snapshot reads as invalid
  if (success) { success = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SNAPSHOT_MAGIC_ADDR, SEAL_MAGIC) == HAL_OK); }

  HAL_FLASH_Lock();
  flushFlashCaches();

  return success
      && (*(const uint32_t *)SNAPSHOT_MAGIC_ADDR == SEAL_MAGIC)
      && (*(const uint32_t *)SNAPSHOT_CRC_ADDR == sramCrc)
      && (memcmp((const void *)SNAPSHOT_DATA_ADDR, (const void *)BKPSRAM_BASE, BKPSRAM_SIZE) == 0);
}

BackupSramBootStatus backupSramBootSync(void)
{
  //Run-once: the flash-operations-only-at-boot guarantee relies on this
  if (bootStatus != BackupSramBootStatus::NotRun) { return bootStatus; }

  //The EEPROM global constructor already enabled these; re-assert in case of ordering changes
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  PWR->CR |= PWR_CR_DBP;

  const uint32_t sramCrc = crc32_oneshot((const uint8_t *)BKPSRAM_BASE, BKPSRAM_SIZE);

  if (isSramSealValid(sramCrc))
  {
    //SRAM is the valid master copy: refresh the flash snapshot only if it doesn't match
    if (isSnapshotValid() && (memcmp((const void *)SNAPSHOT_DATA_ADDR, (const void *)BKPSRAM_BASE, BKPSRAM_SIZE) == 0))
    {
      bootStatus = BackupSramBootStatus::SramValid;
    }
    else
    {
      bootStatus = programSnapshot(sramCrc) ? BackupSramBootStatus::FlashUpdated
                                            : BackupSramBootStatus::FlashWriteFailed;
    }
  }
  else if (isSnapshotValid())
  {
    //SRAM corrupt (battery lost/first boot after flashing): restore from the flash snapshot
    memcpy((void *)BKPSRAM_BASE, (const void *)SNAPSHOT_DATA_ADDR, BKPSRAM_SIZE);
    EEPROM.sealCrc();
    bootStatus = BackupSramBootStatus::SramRestored;
  }
  else
  {
    //No valid copy anywhere: present a blank (0xFF) EEPROM, like a factory-fresh AVR, so the
    //firmware's first-run handling (pinMapping==255 -> empty tune) kicks in deterministically
    memset((void *)BKPSRAM_BASE, 0xFF, BKPSRAM_SIZE);
    EEPROM.sealCrc();
    bootStatus = programSnapshot(SEAL_CRC_REG) ? BackupSramBootStatus::Formatted
                                               : BackupSramBootStatus::FlashWriteFailed;
  }

  return bootStatus;
}

BackupSramBootStatus getBackupSramBootStatus(void)
{
  return bootStatus;
}

#endif
