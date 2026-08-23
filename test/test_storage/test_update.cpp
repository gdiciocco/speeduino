#include <unity.h>
#include "../test_utils.h"
#include "storage.h"
#include "fake_storage.h"
#include "table2d.h"
#include "sensors.h"
#include "globals.h"
#include "units.h"
#include "emp_pump.h"
#include <string.h>

extern void updateTableU16toU8(table2D_u16_u8_32 &targetTable, uint16_t u16EEpromBinAddress);
extern void upgradeV25toV26(void);
extern void upgradeV27toV28(void);
extern void upgradeV28toV29(void);
extern void upgradeV29toV30(void);
extern void upgradeV30toV31(void);
extern void upgradeV31toV32(void);
extern void upgradeV32toV33(void);
extern void upgradeV33toV34(uint8_t originalVersion);
extern void upgradeV34toV35(void);

static void assert_2dTable(table2D_u16_u8_32 &testSubject, uint16_t newAxis, uint8_t newValue)
{
    TEST_ASSERT_EACH_EQUAL_UINT16(newAxis, testSubject.axis, testSubject.size());
    TEST_ASSERT_EACH_EQUAL_UINT8(newValue, testSubject.values, testSubject.size());
}

static void test_updateTableU16toU8(void)
{
    uint8_t values[32];
    uint16_t axis[32];
    table2D_u16_u8_32 testSubject(&axis, &values);
    constexpr uint16_t OLD_AXIS = UINT16_MAX;
    constexpr uint8_t OLD_VALUE = 0U;
    populate_2dtable(&testSubject, OLD_VALUE, OLD_AXIS);

    constexpr uint8_t EEPROM_BYTE = 127U;
    setStorageAPI(getOneByteStorageApi(0xFFF, 0xFFF, EEPROM_BYTE));
    updateTableU16toU8(testSubject, 1024 /* Address doesn't matter */);

    // Reads but no writes
    TEST_ASSERT_NOT_EQUAL(0, oneByteEeprom.readCount);
    TEST_ASSERT_EQUAL(0, oneByteEeprom.writeCount);

    // Table has been updated
    constexpr uint16_t NEW_AXIS_VALUE = ((uint16_t)EEPROM_BYTE << 8U) | EEPROM_BYTE;
    static_assert(NEW_AXIS_VALUE!=OLD_AXIS, "Old & new need to be different for a valid test");
    static_assert(EEPROM_BYTE!=OLD_VALUE, "Old & new need to be different for a valid test");
    assert_2dTable(testSubject, NEW_AXIS_VALUE, EEPROM_BYTE);
}

static storage_api_t innerApi;
static uint8_t eepromVersion = 0;

static byte eepromVersionReadWrapper(uint16_t address)
{
    byte read = innerApi.read(address);
    if (address==0U)
    {
        read = eepromVersion;
    }
    return read;
}
static storage_api_t setupEepromReadApi(uint8_t version, storage_api_t inner)
{
    eepromVersion = version;
    innerApi = inner;
    storage_api_t wrapper = inner;
    wrapper.read = eepromVersionReadWrapper;
    return wrapper;
}

static void test_upgradeV25toV26_positive(void)
{
    constexpr uint16_t OLD_AXIS = UINT16_MAX;
    constexpr uint8_t OLD_VALUE = 0U;
    populate_2dtable(&cltCalibrationTable, OLD_VALUE, OLD_AXIS);
    populate_2dtable(&iatCalibrationTable, OLD_VALUE, OLD_AXIS);
    populate_2dtable(&o2CalibrationTable, OLD_VALUE, OLD_AXIS);
   
    constexpr uint8_t EEPROM_BYTE = 127U;
    setStorageAPI(setupEepromReadApi(25U, getOneByteStorageApi(0xFFF, 0xFFF, EEPROM_BYTE)));
    upgradeV25toV26();

    TEST_ASSERT_EQUAL(644, oneByteEeprom.readCount);
    TEST_ASSERT_EQUAL(1, oneByteEeprom.writeCount);

    // Tables have been updated
    constexpr uint16_t NEW_AXIS_VALUE = ((uint16_t)EEPROM_BYTE << 8U) | EEPROM_BYTE;
    static_assert(NEW_AXIS_VALUE!=OLD_AXIS, "Old & new need to be different for a valid test");
    static_assert(EEPROM_BYTE!=OLD_VALUE, "Old & new need to be different for a valid test");
    assert_2dTable(o2CalibrationTable, NEW_AXIS_VALUE, EEPROM_BYTE);
    assert_2dTable(iatCalibrationTable, NEW_AXIS_VALUE, EEPROM_BYTE);
    assert_2dTable(cltCalibrationTable, NEW_AXIS_VALUE, EEPROM_BYTE);
}

static void test_upgradeV25toV26_negative(void)
{
    constexpr uint8_t EEPROM_BYTE = 127U;
    setStorageAPI(setupEepromReadApi(24U, getOneByteStorageApi(0xFFF, 0xFFF, EEPROM_BYTE)));
    upgradeV25toV26();

    TEST_ASSERT_EQUAL(1, oneByteEeprom.readCount);
    TEST_ASSERT_EQUAL(0, oneByteEeprom.writeCount);
}

static void test_upgradeV27toV28_initialises_idle_advance_closed_loop(void)
{
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_CLOSED_LOOP; //Previously INVALID.
    configPage9.idleAdvClMinAdvance = 0U;
    configPage9.idleAdvClMaxAdvance = 0U;
    setStorageAPI(setupEepromReadApi(27U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV27toV28();

    TEST_ASSERT_EQUAL(IDLEADVANCE_MODE_OFF, configPage2.idleAdvEnabled);
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(0), configPage9.idleAdvClMinAdvance);
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(20), configPage9.idleAdvClMaxAdvance);
}

static void test_upgradeV28toV29_replaces_bands_with_gains(void)
{
    //The V28 authority bands carried no information that maps onto a gain, so
    //every closed-loop setting is rewritten from the defaults.
    configPage9.idleAdvClKp = 0U;
    configPage9.idleAdvClKd = 0U;
    configPage15.idleAdvClCenter = 0U;
    configPage15.idleAdvClDeadband = 0U;
    configPage15.idleAdvClRpmFilter = 0U;
    configPage15.idleAdvClTrimRate = 99U;
    configPage15.idleAdvClTrimRange = 0U;
    configPage15.idleAdvClTrimRequiresIacLimit = 0U;
    setStorageAPI(setupEepromReadApi(28U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV28toV29();

    TEST_ASSERT_EQUAL(40U, configPage9.idleAdvClKp);
    TEST_ASSERT_EQUAL(60U, configPage9.idleAdvClKd);
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(10), configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL(15U, configPage15.idleAdvClDeadband);
    TEST_ASSERT_EQUAL(50U, configPage15.idleAdvClRpmFilter);
    //The trim defaults to off: with a closed-loop IAC a second integrator on the
    //same measurement is what makes the two loops hunt against each other.
    TEST_ASSERT_EQUAL(0U, configPage15.idleAdvClTrimRate);
    TEST_ASSERT_EQUAL(5U, configPage15.idleAdvClTrimRange);
    TEST_ASSERT_EQUAL(1U, configPage15.idleAdvClTrimRequiresIacLimit);
}

static void test_upgradeV29toV30_initialises_center_autotune(void)
{
    //The autotune settings land in bytes that were previously unused, so their
    //content is undefined and must be rewritten from the defaults.
    configPage15.idleAdvClLearnAuthority = 255U;
    configPage15.idleAdvClLearnMinTemp = 255U;
    setStorageAPI(setupEepromReadApi(29U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV29toV30();

    //Learning defaults to off so an upgrade cannot start moving the tune by itself.
    TEST_ASSERT_EQUAL(0U, configPage15.idleAdvClLearnAuthority);
    TEST_ASSERT_EQUAL(temperatureAddOffset(70), configPage15.idleAdvClLearnMinTemp);
}

static void test_upgradeV30toV31_converts_stepper_positions(void)
{
    configPage6.iacAlgorithm = IAC_ALGORITHM_STEP_OLCL;
    for (uint8_t i = 0U; i < 10U; i++) { configPage6.iacOLStepVal[i] = i; }
    for (uint8_t i = 0U; i < 4U; i++) { configPage6.iacCrankSteps[i] = (uint8_t)(20U + i); }
    configPage6.iacOLStepVal[9] = 86U; //Would exceed the new one-byte range.
    configPage6.iacStepHome = 27U;
    configPage9.iacMaxSteps = 26U;
    configPage2.iacCLminValue = 10U;
    configPage2.iacCLmaxValue = 59U;
    setStorageAPI(setupEepromReadApi(30U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV30toV31();

    TEST_ASSERT_EQUAL_UINT8(0U, configPage6.iacOLStepVal[0]);
    TEST_ASSERT_EQUAL_UINT8(24U, configPage6.iacOLStepVal[8]);
    TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, configPage6.iacOLStepVal[9]);
    TEST_ASSERT_EQUAL_UINT8(60U, configPage6.iacCrankSteps[0]);
    TEST_ASSERT_EQUAL_UINT8(69U, configPage6.iacCrankSteps[3]);
    TEST_ASSERT_EQUAL_UINT8(81U, configPage6.iacStepHome);
    TEST_ASSERT_EQUAL_UINT8(78U, configPage9.iacMaxSteps);
    TEST_ASSERT_EQUAL_UINT8(30U, configPage2.iacCLminValue);
    TEST_ASSERT_EQUAL_UINT8(177U, configPage2.iacCLmaxValue);
}

static void test_upgradeV30toV31_preserves_pwm_limits(void)
{
    configPage6.iacAlgorithm = IAC_ALGORITHM_PWM_CL;
    configPage2.iacCLminValue = 10U;
    configPage2.iacCLmaxValue = 90U;
    setStorageAPI(setupEepromReadApi(30U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV30toV31();

    TEST_ASSERT_EQUAL_UINT8(10U, configPage2.iacCLminValue);
    TEST_ASSERT_EQUAL_UINT8(90U, configPage2.iacCLmaxValue);
}

static void test_upgradeV31toV32_initialises_gain_autotune_setup(void)
{
    configPage15.idleAdvClGainTuneStep = UINT8_MAX;
    configPage15.idleAdvClGainTuneSettleTime = UINT8_MAX;
    configPage15.idleAdvClGainTuneSettleBand = UINT8_MAX;
    configPage15.idleAdvClGainTuneHysteresis = UINT8_MAX;
    configPage15.idleAdvClGainTuneDiscard = UINT8_MAX;
    configPage15.idleAdvClGainTuneMeasure = UINT8_MAX;
    configPage15.idleAdvClGainTuneTimeout = UINT8_MAX;
    configPage15.idleAdvClGainTuneRunawayDiv10 = UINT8_MAX;
    configPage15.idleAdvClGainTuneMinAmplitude = UINT8_MAX;
    configPage15.idleAdvClGainTuneMinPeriod = UINT8_MAX;
    configPage15.idleAdvClGainTuneMaxPeriod = UINT8_MAX;
    configPage15.idleAdvClGainTuneMaxAttempts = UINT8_MAX;
    setStorageAPI(setupEepromReadApi(31U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV31toV32();

    TEST_ASSERT_EQUAL_UINT8(3U, configPage15.idleAdvClGainTuneStep);
    TEST_ASSERT_EQUAL_UINT8(3U, configPage15.idleAdvClGainTuneSettleTime);
    TEST_ASSERT_EQUAL_UINT8(20U, configPage15.idleAdvClGainTuneSettleBand);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.idleAdvClGainTuneHysteresis);
    TEST_ASSERT_EQUAL_UINT8(2U, configPage15.idleAdvClGainTuneDiscard);
    TEST_ASSERT_EQUAL_UINT8(6U, configPage15.idleAdvClGainTuneMeasure);
    TEST_ASSERT_EQUAL_UINT8(5U, configPage15.idleAdvClGainTuneTimeout);
    TEST_ASSERT_EQUAL_UINT8(40U, configPage15.idleAdvClGainTuneRunawayDiv10);
    TEST_ASSERT_EQUAL_UINT8(20U, configPage15.idleAdvClGainTuneMinAmplitude);
    TEST_ASSERT_EQUAL_UINT8(4U, configPage15.idleAdvClGainTuneMinPeriod);
    TEST_ASSERT_EQUAL_UINT8(60U, configPage15.idleAdvClGainTuneMaxPeriod);
    TEST_ASSERT_EQUAL_UINT8(3U, configPage15.idleAdvClGainTuneMaxAttempts);
}

static void test_upgradeV32toV33_initialises_iac_gain_autotune_setup(void)
{
    configPage15.iacGainAutotuneRequest = 1U;
    configPage15.iacGainAutotuneUnused126 = 127U;
    configPage15.iacGainTuneMinTemp = UINT8_MAX;
    configPage15.iacGainTuneStep = UINT8_MAX;
    configPage15.iacGainTuneSettleTime = UINT8_MAX;
    configPage15.iacGainTuneSettleBand = UINT8_MAX;
    configPage15.iacGainTuneHysteresis = UINT8_MAX;
    configPage15.iacGainTuneDiscard = UINT8_MAX;
    configPage15.iacGainTuneMeasure = UINT8_MAX;
    configPage15.iacGainTuneTimeout = UINT8_MAX;
    configPage15.iacGainTuneRunawayDiv10 = UINT8_MAX;
    configPage15.iacGainTuneMinAmplitude = UINT8_MAX;
    configPage15.iacGainTuneMinPeriod = UINT8_MAX;
    configPage15.iacGainTuneMaxPeriod = UINT8_MAX;
    configPage15.iacGainTuneMaxAttempts = UINT8_MAX;
    configPage15.iacGainTuneMaxTps = UINT8_MAX;
    setStorageAPI(setupEepromReadApi(32U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV32toV33();

    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.iacGainAutotuneRequest);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.iacGainAutotuneUnused126);
    TEST_ASSERT_EQUAL_UINT8(temperatureAddOffset(70), configPage15.iacGainTuneMinTemp);
    TEST_ASSERT_EQUAL_UINT8(5U, configPage15.iacGainTuneStep);
    TEST_ASSERT_EQUAL_UINT8(5U, configPage15.iacGainTuneSettleTime);
    TEST_ASSERT_EQUAL_UINT8(25U, configPage15.iacGainTuneSettleBand);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.iacGainTuneHysteresis);
    TEST_ASSERT_EQUAL_UINT8(2U, configPage15.iacGainTuneDiscard);
    TEST_ASSERT_EQUAL_UINT8(6U, configPage15.iacGainTuneMeasure);
    TEST_ASSERT_EQUAL_UINT8(10U, configPage15.iacGainTuneTimeout);
    TEST_ASSERT_EQUAL_UINT8(40U, configPage15.iacGainTuneRunawayDiv10);
    TEST_ASSERT_EQUAL_UINT8(20U, configPage15.iacGainTuneMinAmplitude);
    TEST_ASSERT_EQUAL_UINT8(5U, configPage15.iacGainTuneMinPeriod);
    TEST_ASSERT_EQUAL_UINT8(200U, configPage15.iacGainTuneMaxPeriod);
    TEST_ASSERT_EQUAL_UINT8(3U, configPage15.iacGainTuneMaxAttempts);
    TEST_ASSERT_EQUAL_UINT8(10U, configPage15.iacGainTuneMaxTps);
}

static void test_upgradeV33toV34_compacts_page15_and_preserves_idle_settings(void)
{
    constexpr uint8_t PAGE15_CONFIG_TS_OFFSET = 80U;
    constexpr uint8_t OLD_IDLE_ADV_TS_OFFSET = 190U;
    constexpr uint8_t OLD_IAC_TUNE_TS_OFFSET = 210U;
    byte *rawPage15 = reinterpret_cast<byte *>(&configPage15);
    (void)memset(rawPage15, 0, sizeof(configPage15));
    for(uint8_t index = 0U; index < 20U; index++) {
        rawPage15[OLD_IDLE_ADV_TS_OFFSET - PAGE15_CONFIG_TS_OFFSET + index] = (byte)(index + 1U);
    }
    for(uint8_t index = 0U; index < 15U; index++) {
        rawPage15[OLD_IAC_TUNE_TS_OFFSET - PAGE15_CONFIG_TS_OFFSET + index] = (byte)(index + 101U);
    }
    setStorageAPI(setupEepromReadApi(33U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV33toV34(33U);

    TEST_ASSERT_EQUAL_UINT8(1U, configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL_UINT8(20U, configPage15.idleAdvClGainTuneMaxAttempts);
    TEST_ASSERT_EQUAL_UINT8(1U, configPage15.iacGainAutotuneRequest);
    TEST_ASSERT_EQUAL_UINT8(102U, configPage15.iacGainTuneMinTemp);
    TEST_ASSERT_EQUAL_UINT8(115U, configPage15.iacGainTuneMaxTps);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.empPumpFlags & emp_pump::ENABLED);
    TEST_ASSERT_EQUAL_UINT16(1500U, configPage15.empPumpMinimumRunRpm);
    TEST_ASSERT_EQUAL_UINT16(6000U, configPage15.empPumpMaximumRpm);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.vehicleDistanceSource);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.gpsSpeedAuxChannel);
    TEST_ASSERT_EQUAL_UINT32(0U, configPage15.vehicleOdometerDeciKm);
    TEST_ASSERT_EQUAL_UINT32(0U, configPage15.vehicleTripDeciKm);
    TEST_ASSERT_EACH_EQUAL_UINT8(0U, configPage15.Unused15_231_255, 25U);
}

static void test_upgradeV33toV34_preserves_v32_idle_and_defaults_new_iac(void)
{
    constexpr uint8_t PAGE15_CONFIG_TS_OFFSET = 80U;
    constexpr uint8_t OLD_IDLE_ADV_TS_OFFSET = 190U;
    constexpr uint8_t OLD_IAC_TUNE_TS_OFFSET = 210U;
    byte *rawPage15 = reinterpret_cast<byte *>(&configPage15);
    (void)memset(rawPage15, 0, sizeof(configPage15));
    for(uint8_t index = 0U; index < 20U; index++) {
        rawPage15[OLD_IDLE_ADV_TS_OFFSET - PAGE15_CONFIG_TS_OFFSET + index] = (byte)(index + 1U);
    }
    for(uint8_t index = 0U; index < 15U; index++) {
        rawPage15[OLD_IAC_TUNE_TS_OFFSET - PAGE15_CONFIG_TS_OFFSET + index] = UINT8_MAX;
    }
    setStorageAPI(setupEepromReadApi(33U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV33toV34(32U);

    TEST_ASSERT_EQUAL_UINT8(1U, configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL_UINT8(20U, configPage15.idleAdvClGainTuneMaxAttempts);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.iacGainAutotuneRequest);
    TEST_ASSERT_EQUAL_UINT8(temperatureAddOffset(70), configPage15.iacGainTuneMinTemp);
    TEST_ASSERT_EQUAL_UINT8(10U, configPage15.iacGainTuneMaxTps);
}

static void test_upgradeV34toV35_initialises_vehicle_distance(void)
{
    configPage15.vehicleDistanceSource = UINT8_MAX;
    configPage15.gpsSpeedAuxChannel = UINT8_MAX;
    configPage15.vehicleOdometerDeciKm = UINT32_MAX;
    configPage15.vehicleTripDeciKm = UINT32_MAX;
    (void)memset(configPage15.Unused15_231_255, UINT8_MAX,
                 sizeof(configPage15.Unused15_231_255));
    setStorageAPI(setupEepromReadApi(34U, getOneByteStorageApi(0xFFF, 0xFFF, 127U)));

    upgradeV34toV35();

    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.vehicleDistanceSource);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.gpsSpeedAuxChannel);
    TEST_ASSERT_EQUAL_UINT32(0U, configPage15.vehicleOdometerDeciKm);
    TEST_ASSERT_EQUAL_UINT32(0U, configPage15.vehicleTripDeciKm);
    TEST_ASSERT_EACH_EQUAL_UINT8(0U, configPage15.Unused15_231_255, 25U);
}

void test_update(void) {
    SET_UNITY_FILENAME() {
        RUN_TEST(test_updateTableU16toU8);
        RUN_TEST(test_upgradeV25toV26_positive);
        RUN_TEST(test_upgradeV25toV26_negative);
        RUN_TEST(test_upgradeV27toV28_initialises_idle_advance_closed_loop);
        RUN_TEST(test_upgradeV28toV29_replaces_bands_with_gains);
        RUN_TEST(test_upgradeV29toV30_initialises_center_autotune);
        RUN_TEST(test_upgradeV30toV31_converts_stepper_positions);
        RUN_TEST(test_upgradeV30toV31_preserves_pwm_limits);
        RUN_TEST(test_upgradeV31toV32_initialises_gain_autotune_setup);
        RUN_TEST(test_upgradeV32toV33_initialises_iac_gain_autotune_setup);
        RUN_TEST(test_upgradeV33toV34_compacts_page15_and_preserves_idle_settings);
        RUN_TEST(test_upgradeV33toV34_preserves_v32_idle_and_defaults_new_iac);
        RUN_TEST(test_upgradeV34toV35_initialises_vehicle_distance);
    }
}
