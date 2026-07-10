//Backup sram stores data in the battery backuped sram portion.
//The backup battery is available on the ebay stm32F407VET6 black boards.
//
//Data integrity scheme:
// - The whole 4KB backup SRAM image is covered by a CRC32 "seal" stored in the RTC backup
//   registers (same VBAT domain, but outside the 4KB so the EEPROM layout is unchanged).
// - Every write marks the image dirty; sealCrcIfDirty() must be called periodically from the
//   main loop (NOT from an interrupt: it uses the shared CRC32 unit) to refresh the seal.
// - backupSramBootSync() verifies the seal at boot and keeps a snapshot of the image in one
//   internal flash sector. All flash operations (erase included) happen ONLY inside that call,
//   which must run before the ECU is brought up.
#ifndef BACKUPSRAMASEEPROM_H
#define BACKUPSRAMASEEPROM_H
#if defined(STM32F407xx)
#include <stdint.h>
#include "stm32f407xx.h"

class BackupSramAsEEPROM {
  private:
    const uint16_t backup_size = 4096; //maximum of 4kb backuped sram available.
    volatile bool crcDirty = false; //Set on every write; cleared when the CRC seal is refreshed
    int8_t write_byte( uint8_t *data, uint16_t bytes, uint16_t offset );
    int8_t read_byte( uint8_t *data, uint16_t bytes, uint16_t offset );

  public:
    BackupSramAsEEPROM();
    uint8_t read(uint16_t address);
    int8_t write(uint16_t address, uint8_t val);
    int8_t update(uint16_t address, uint8_t val);
    uint16_t length();

    /** @brief Recompute the CRC32 of the whole backup SRAM and store it in the RTC backup registers */
    void sealCrc(void);
    /** @brief Refresh the CRC seal, but only if a write occurred since the last seal. Call from the main loop only */
    void sealCrcIfDirty(void);

    template< typename T > T &get( int idx, T &t ){
        uint16_t e = idx;
        uint8_t *ptr = (uint8_t*) &t;
        for( int count = sizeof(T) ; count ; --count, ++e )  *ptr++ = read(e);
        return t;
    }
    template< typename T > const T &put( int idx, const T &t ){
        const uint8_t *ptr = (const uint8_t*) &t;
        uint16_t e = idx;
        for( int count = sizeof(T) ; count ; --count, ++e )  write(e, *ptr++);
        return t;
    }
};

extern BackupSramAsEEPROM EEPROM;

/** @brief Result of the boot-time backup SRAM integrity check */
enum class BackupSramBootStatus : uint8_t {
  NotRun,           //!< backupSramBootSync() has not executed yet
  SramValid,        //!< SRAM CRC valid and the flash snapshot already matched: no flash operation performed
  FlashUpdated,     //!< SRAM CRC valid; the flash snapshot was stale/absent and has been rewritten
  SramRestored,     //!< SRAM CRC invalid; the image was restored from the flash snapshot
  Formatted,        //!< Both SRAM and flash snapshot invalid (first boot/battery lost): SRAM blanked to 0xFF
  FlashWriteFailed  //!< The snapshot erase/program did not verify; SRAM is still the (valid) master copy
};

/** @brief Boot-time integrity check and flash snapshot sync.
 *
 * Verifies the backup SRAM against its CRC seal:
 *  - Seal valid: if the flash snapshot differs, it is erased and reprogrammed from SRAM.
 *  - Seal invalid: SRAM is restored from the flash snapshot (or blanked to 0xFF if the
 *    snapshot is invalid too) and resealed.
 *
 * This is the ONLY place where flash erase/program operations occur, so it MUST be called
 * before the ECU becomes operational. Runs once; later calls return the first result.
 */
BackupSramBootStatus backupSramBootSync(void);

/** @brief Outcome of the boot-time integrity check, for diagnostics */
BackupSramBootStatus getBackupSramBootStatus(void);

#endif
#endif
