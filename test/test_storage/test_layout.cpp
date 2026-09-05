#include <unity.h>
#include "../test_utils.h"
#include "board_definition.h"
#include "storage.h"
#include "pages.h"
#include "config_pages.h"
#include "scheduler.h"

// Page 17 offsets, copied from the ini. The enum *is* the offset, so a
// reordered PinAssignment silently reassigns every pin in an existing tune -
// this is the tripwire.
static_assert(PIN_ASSIGN_TACHO == 0U, "Page 17 / INI tachoPin offset mismatch");
static_assert(PIN_ASSIGN_IDLE_UP == 1U, "Page 17 / INI idleUpPin offset mismatch");
static_assert(PIN_ASSIGN_IDLE_UP_OUTPUT == 2U, "Page 17 / INI idleUpOutputPin offset mismatch");
static_assert(PIN_ASSIGN_CTPS == 3U, "Page 17 / INI CTPSPin offset mismatch");
static_assert(PIN_ASSIGN_VSS == 4U, "Page 17 / INI vssPin offset mismatch");
static_assert(PIN_ASSIGN_FUEL_PUMP == 5U, "Page 17 / INI fuelPumpPin offset mismatch");
static_assert(PIN_ASSIGN_RESET_CONTROL == 6U, "Page 17 / INI resetControlPin offset mismatch");
static_assert(PIN_ASSIGN_IGN_BYPASS == 7U, "Page 17 / INI ignBypassPin offset mismatch");
static_assert(PIN_ASSIGN_VVT1 == 8U, "Page 17 / INI vvt1Pin offset mismatch");
static_assert(PIN_ASSIGN_BOOST == 9U, "Page 17 / INI boostPin offset mismatch");
static_assert(PIN_ASSIGN_LAUNCH == 10U, "Page 17 / INI launchPin offset mismatch");
static_assert(PIN_ASSIGN_FAN == 11U, "Page 17 / INI fanPin offset mismatch");
static_assert(PIN_ASSIGN_BARO == 12U, "Page 17 / INI baroPin offset mismatch");
static_assert(PIN_ASSIGN_EMAP == 13U, "Page 17 / INI EMAPPin offset mismatch");
static_assert(PIN_ASSIGN_N2O_ARM == 14U, "Page 17 / INI n2o_arming_pin offset mismatch");
static_assert(PIN_ASSIGN_N2O_STAGE1 == 15U, "Page 17 / INI n2o_stage1_pin offset mismatch");
static_assert(PIN_ASSIGN_N2O_STAGE2 == 16U, "Page 17 / INI n2o_stage2_pin offset mismatch");
static_assert(PIN_ASSIGN_KNOCK == 17U, "Page 17 / INI knock_pin offset mismatch");
static_assert(PIN_ASSIGN_FUEL2_INPUT == 18U, "Page 17 / INI fuel2InputPin offset mismatch");
static_assert(PIN_ASSIGN_SPARK2_INPUT == 19U, "Page 17 / INI spark2InputPin offset mismatch");
static_assert(PIN_ASSIGN_OIL_PRESSURE == 20U, "Page 17 / INI oilPressurePin offset mismatch");
static_assert(PIN_ASSIGN_FUEL_PRESSURE == 21U, "Page 17 / INI fuelPressurePin offset mismatch");
static_assert(PIN_ASSIGN_WMI_INDICATOR == 22U, "Page 17 / INI wmiIndicatorPin offset mismatch");
static_assert(PIN_ASSIGN_WMI_EMPTY == 23U, "Page 17 / INI wmiEmptyPin offset mismatch");
static_assert(PIN_ASSIGN_WMI_ENABLED == 24U, "Page 17 / INI wmiEnabledPin offset mismatch");
static_assert(PIN_ASSIGN_VVT2 == 25U, "Page 17 / INI vvt2Pin offset mismatch");
static_assert(PIN_ASSIGN_SD_ENABLE == 26U, "Page 17 / INI onboard_log_tr5_Epin_pin offset mismatch");
static_assert(PIN_ASSIGN_AIRCON_COMP == 27U, "Page 17 / INI airConCompPin offset mismatch");
static_assert(PIN_ASSIGN_AIRCON_REQUEST == 28U, "Page 17 / INI airConReqPin offset mismatch");
static_assert(PIN_ASSIGN_AIRCON_FAN == 29U, "Page 17 / INI airConFanPin offset mismatch");

extern uint16_t getEntityStartAddress(page_iterator_t entity);
extern const uint16_t MAX_PAGE_ADDRESS;
extern uint16_t getSensorCalibrationCrcAddress(SensorCalibrationTable sensor);
extern const uint16_t STORAGE_SIZE;

static void test_getEntityStartAddress_invalid_entity(void) {
    config10 localPage10;
    page_iterator_t iter(page_entity_t(entity_t(&localPage10, sizeof(localPage10)), 0U), entity_page_location_t(2, 1));
    TEST_ASSERT_EQUAL(0, getEntityStartAddress(iter));
}

struct block {
    uint16_t start;
    uint16_t length;
};

static void assert_nocalibration_overlap(const block &newBlock, uint8_t idxCurrBlock, SensorCalibrationTable table) {
    const block calibrationCrc = { getSensorCalibrationCrcAddress(table), sizeof(uint32_t) };
    char msg[64];
    sprintf(msg, "EEPROM storage: entity %" PRIu16 " overlaps calibration CRC %" PRIu16, idxCurrBlock, (uint16_t)table);
    const bool overlapsCrc = (newBlock.start < calibrationCrc.start + calibrationCrc.length)
                          && (calibrationCrc.start < newBlock.start + newBlock.length);
    TEST_ASSERT_FALSE_MESSAGE(overlapsCrc, msg);
}

static void assert_nocalibration_overlap(const block &newBlock, uint8_t idxCurrBlock) {
    assert_nocalibration_overlap(newBlock, idxCurrBlock, SensorCalibrationTable::CoolantSensor);
    assert_nocalibration_overlap(newBlock, idxCurrBlock, SensorCalibrationTable::IntakeAirTempSensor);
    assert_nocalibration_overlap(newBlock, idxCurrBlock, SensorCalibrationTable::O2Sensor);
}

static bool inline overlaps(const block &a, const block &b) {
    return (a.start < b.start + b.length) && (b.start < a.start + a.length);
}

static uint8_t find_overlap(const block blocks[], uint8_t idxCurrBlock, const block &newBlock) {
    uint8_t i = 0;
    while ((i < idxCurrBlock) && !overlaps(blocks[i], newBlock)) {
        ++i;
    }
    return i;
}

static uint8_t test_no_overlap_page(uint8_t pageNum, block blocks[], size_t length, uint8_t idxCurrBlock) {
  page_iterator_t iter = page_begin(pageNum);
  while (iter.entity.type!=EntityType::End) {
    if (iter.entity.type!=EntityType::NoEntity) {
        TEST_ASSERT_LESS_THAN(length, idxCurrBlock);

        block newBlock = { getEntityStartAddress(iter), iter.entity.size };
        TEST_ASSERT_GREATER_THAN(0, newBlock.start);
        TEST_ASSERT_LESS_THAN(MAX_PAGE_ADDRESS, newBlock.start+newBlock.length);
        assert_nocalibration_overlap(newBlock, idxCurrBlock);
        uint8_t overlapBlock = find_overlap(blocks, idxCurrBlock, newBlock);
        if (overlapBlock!=idxCurrBlock) {
            char msg[64];
            sprintf(msg, "EEPROM storage: entity %" PRIu8 " overlaps entity %" PRIu8, overlapBlock, idxCurrBlock);
            TEST_FAIL_MESSAGE(msg);
        }
        TEST_ASSERT_TRUE(overlapBlock==idxCurrBlock);
        blocks[idxCurrBlock] = newBlock;

        ++idxCurrBlock;
    }
    iter = advance(iter);
  }

  return idxCurrBlock;
}

static void test_no_entity_overlap(void) {
    block blocks[40];
    uint8_t idxCurrBlock = 0U;

    for (uint8_t i = MIN_PAGE_NUM; i < MAX_PAGE_NUM; i++) {
        idxCurrBlock = test_no_overlap_page(i, blocks, _countof(blocks), idxCurrBlock);
    }
}

const char* getEntityTypeName(const page_iterator_t &iter) {
    switch (iter.entity.type) {
        case EntityType::Raw: return "Raw";
        case EntityType::Table: return "Table";
        case EntityType::NoEntity: return "NoEntity";
        case EntityType::End: return "End";
        default: return "Unknown";
    }
}

const char *getEntityName(const page_iterator_t &it) {
  #define GET_VARIABLE_NAME(Variable) (#Variable)

  struct entity_name_map_t {
      void *pEntity;
      const char *name;
  };

  // Store a map of entity to EEPROM address in FLASH memory.
  static const entity_name_map_t entityMap[] = {
    { &fuelTable, GET_VARIABLE_NAME(fuelTable) },
    { &configPage2, GET_VARIABLE_NAME(configPage2) },
    { &ignitionTable, GET_VARIABLE_NAME(ignitionTable) },
    { &configPage4, GET_VARIABLE_NAME(configPage4) },
    { &afrTable, GET_VARIABLE_NAME(afrTable) },
    { &configPage6, GET_VARIABLE_NAME(configPage6) },
    { &boostTable, GET_VARIABLE_NAME(boostTable) },
    { &vvtTable, GET_VARIABLE_NAME(vvtTable) },
    { &stagingTable, GET_VARIABLE_NAME(stagingTable) },
    { &trimTables[0], GET_VARIABLE_NAME(trimTables[0]) },
#if INJ_CHANNELS >= 2
    { &trimTables[1], GET_VARIABLE_NAME(trimTables[1]) },
#endif
#if INJ_CHANNELS >= 3
    { &trimTables[2], GET_VARIABLE_NAME(trimTables[2]) },
#endif
#if INJ_CHANNELS >= 4
    { &trimTables[3], GET_VARIABLE_NAME(trimTables[3]) },
#endif
#if INJ_CHANNELS >= 5
    { &trimTables[4], GET_VARIABLE_NAME(trimTables[4]) },
#endif
#if INJ_CHANNELS >= 6
    { &trimTables[5], GET_VARIABLE_NAME(trimTables[5]) },
#endif
#if INJ_CHANNELS >= 7
    { &trimTables[6], GET_VARIABLE_NAME(trimTables[6]) },
#endif
#if INJ_CHANNELS >= 8
    { &trimTables[7], GET_VARIABLE_NAME(trimTables[7]) },
#endif
    { &configPage9, GET_VARIABLE_NAME(configPage9) },
    { &configPage10, GET_VARIABLE_NAME(configPage10) },
    { &fuelTable2, GET_VARIABLE_NAME(fuelTable2) },
    { &wmiTable, GET_VARIABLE_NAME(wmiTable) },
    { &vvt2Table, GET_VARIABLE_NAME(vvt2Table) },
    { &dwellTable, GET_VARIABLE_NAME(dwellTable) },
    { &configPage13, GET_VARIABLE_NAME(configPage13) },
    { &ignitionTable2, GET_VARIABLE_NAME(ignitionTable2) },
    { &boostTableLookupDuty, GET_VARIABLE_NAME(boostTableLookupDuty) },
    { &configPage15, GET_VARIABLE_NAME(configPage15) },
    { &wallWettingAddTable, GET_VARIABLE_NAME(wallWettingAddTable) },
    { &wallWettingRemoveTable, GET_VARIABLE_NAME(wallWettingRemoveTable) },
    { &afrDelayTables[0], GET_VARIABLE_NAME(afrDelayTables[0]) },
    { &afrDelayTables[1], GET_VARIABLE_NAME(afrDelayTables[1]) },
    { &afrDelayConfig, GET_VARIABLE_NAME(afrDelayConfig) },
  };
  static const constexpr entity_name_map_t* entityMapEnd = entityMap + _countof(entityMap);  

  // Linear search of the name map.
  const entity_name_map_t *pMapEntry = entityMap;
  while ((pMapEntry!=entityMapEnd) && (it.entity.pRaw!=pMapEntry->pEntity)) {
    ++pMapEntry;
  }
  if (pMapEntry!=entityMapEnd) {
    return pMapEntry->name;
  }
  static const char *unknown = "Unknown";
  return unknown;
}

static void print_entity(const page_iterator_t &iter)
{
    if (EntityType::NoEntity!=iter.entity.type)
    {
        char msg[128];
        sprintf(msg, "%" PRIu8 ", %" PRIu8 ", %s, %s, %" PRIu16 ", %" PRIu16, iter.location.page, iter.location.index, getEntityName(iter), getEntityTypeName(iter), getEntityStartAddress(iter), iter.entity.size);
        UnityPrint(msg); UNITY_PRINT_EOL();
    }
}

static void print_page_layout(uint8_t pageNum)
{
    page_iterator_t iter = page_begin(pageNum);
    while (iter.entity.type!=EntityType::End) {
        print_entity(iter);
        iter = advance(iter);
    }
}

// An informational function to print the layout of the EEPROM as CSV
// Requires "-v" flag on pio unit test runner 
static void print_eeprom_layout(void) {
    // Informational CSV, not an assertion - as the comment above says, it is
    // only meant to be read with "-v". Dumping every entity of every page over
    // a USB CDC link stops the board partway through and takes the rest of the
    // suite with it, the same way print_all_page_entity_layout did in
    // test_pages. Reproduced identically on caponord-stm32-optimized, so it
    // predates the 8-bit removal.
#if !defined(NATIVE_BOARD)
    TEST_IGNORE_MESSAGE("EEPROM layout dump: blocks on a real board, native host only");
#else
    UnityPrint("Page, Index, Item, Type, Start Address, Length"); UNITY_PRINT_EOL();
    for (uint8_t page = MIN_PAGE_NUM; page < MAX_PAGE_NUM; page++) {
        print_page_layout(page);
    }

    #define GET_VARIABLE_NAME(Variable) (#Variable)
    char msg[128];
    sprintf(msg, "Calib CRC, %d, %s, CRC, %" PRIu16 ", %" PRIu16, (int)SensorCalibrationTable::CoolantSensor, GET_VARIABLE_NAME(CoolantSensor), getSensorCalibrationCrcAddress(SensorCalibrationTable::CoolantSensor), (uint16_t)sizeof(uint32_t));
    UnityPrint(msg); UNITY_PRINT_EOL();
    sprintf(msg, "Calib CRC, %d, %s, CRC, %" PRIu16 ", %" PRIu16, (int)SensorCalibrationTable::IntakeAirTempSensor, GET_VARIABLE_NAME(IntakeAirTempSensor), getSensorCalibrationCrcAddress(SensorCalibrationTable::IntakeAirTempSensor), (uint16_t)sizeof(uint32_t));
    UnityPrint(msg); UNITY_PRINT_EOL();
    sprintf(msg, "Calib CRC, %d, %s, CRC, %" PRIu16 ", %" PRIu16, (int)SensorCalibrationTable::O2Sensor, GET_VARIABLE_NAME(O2Sensor), getSensorCalibrationCrcAddress(SensorCalibrationTable::O2Sensor), (uint16_t)sizeof(uint32_t));
    UnityPrint(msg); UNITY_PRINT_EOL();
    sprintf(msg, "Calibrations, 0, Calib, Calib, %" PRIu16 ", %" PRIu16, MAX_PAGE_ADDRESS, (uint16_t)(STORAGE_SIZE-MAX_PAGE_ADDRESS));
    UnityPrint(msg); UNITY_PRINT_EOL();
#endif
}

void test_layout(void) {
    SET_UNITY_FILENAME() {     
        RUN_TEST_P(test_getEntityStartAddress_invalid_entity);
        RUN_TEST_P(test_no_entity_overlap);
        RUN_TEST_P(print_eeprom_layout);
    }
}
