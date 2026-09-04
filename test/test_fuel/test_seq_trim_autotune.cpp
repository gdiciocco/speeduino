#include <unity.h>

#include "../test_utils.h"
#include "globals.h"
#include "seq_trim_autotune.h"
#include "storage.h"
#include "units.h"

namespace {

constexpr uint8_t TRIM_RAW_ZERO = 128U;
constexpr uint8_t CELL_AT_3000_RPM_60_LOAD = 20U;

void initialiseTrimTables(void)
{
  const uint8_t rpmAxis[6] = {60U, 50U, 40U, 30U, 20U, 10U};
  const uint8_t loadAxis[6] = {60U, 50U, 40U, 30U, 20U, 10U};
  for (uint8_t trim = 0U; trim < (uint8_t)INJ_CHANNELS; trim++) {
    for (uint8_t cell = 0U; cell < 36U; cell++) { trimTables[trim].values.values[cell] = TRIM_RAW_ZERO; }
    for (uint8_t bin = 0U; bin < 6U; bin++) {
      trimTables[trim].axisX.axis[bin] = rpmAxis[bin];
      trimTables[trim].axisY.axis[bin] = loadAxis[bin];
    }
  }
}

void setupLearner(uint8_t resistancePreset = 0U, uint8_t authority = 10U)
{
  initialiseTrimTables();
  configPage2.nCylinders = 2U;
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage6.fuelTrimEnabled = true;
  configPage6.egoType = EGO_TYPE_WIDE;
  configPage6.ego_sdelay = 0U;

  for (uint8_t trim = 0U; trim < 8U; trim++) {
    configPage15.seqTrimAutotuneConfig[trim] = 0U;
    configPage15.seqTrimAutotuneAuthority[trim] = authority;
  }
  configPage15.seqTrimAutotuneConfig[0] = (uint8_t)(SEQ_TRIM_AUTOTUNE_LEARN | (resistancePreset << 3U));
  configPage15.seqTrimAutotuneFlags = 0x03U; //Target objective, save at stop
  configPage15.seqTrimAutotuneDeadband = 0U;
  configPage15.seqTrimAutotuneStableTime = 0U;
  configPage15.seqTrimAutotuneMinClt = temperatureAddOffset(60);
  configPage15.seqTrimAutotuneMinRpmDiv100 = 10U;
  configPage15.seqTrimAutotuneMaxRpmDiv100 = 70U;
  configPage15.seqTrimAutotuneMinLoadDiv2 = 10U;
  configPage15.seqTrimAutotuneMaxLoadDiv2 = 70U;
  configPage15.seqTrimAutotuneSavePeriod = 1U;

  currentStatus.rotationStatus = EngineRotationStatus::Running;
  currentStatus.runSecs = 255U;
  currentStatus.coolant = 80;
  currentStatus.setRpm(3000U);
  currentStatus.fuelLoad = 60;
  currentStatus.battery10 = 130U;
  currentStatus.afrTarget = 147U;
  currentStatus.O2 = 147U;
  currentStatus.O2_2 = 147U;
  currentStatus.egoCorrection = 100U;
  currentStatus.tpsDOT = 0;
  currentStatus.mapDOT = 0;
  currentStatus.isDFCOActive = false;
  currentStatus.aseIsActive = false;
  currentStatus.wueIsActive = false;
  currentStatus.isAcceleratingTPS = false;
  currentStatus.isDeceleratingTPS = false;
  currentStatus.launchingHard = false;
  currentStatus.launchingSoft = false;
  currentStatus.flatShiftingHard = false;
  currentStatus.flatShiftSoftCut = false;
  currentStatus.nitrousActive = false;
  currentStatus.stagingActive = false;
  currentStatus.engineProtect.reset();
  setEepromWritePending(false);
  seqTrimAutotuneInit();
}

void runDualSamples(uint16_t rpm, uint16_t load, uint8_t afr1, uint8_t afr2, uint8_t count)
{
  currentStatus.setRpm(rpm);
  currentStatus.fuelLoad = load;
  currentStatus.O2 = afr1;
  currentStatus.O2_2 = afr2;
  for (uint8_t index = 0U; index < count; index++) { seqTrimAutotuneUpdate(); }
}

void runSamples(uint16_t rpm, uint16_t load, uint8_t afr, uint8_t count)
{
  runDualSamples(rpm, load, afr, 147U, count);
}

void primeHistory(void)
{
  runSamples(3000U, 60U, 147U, 30U);
}

void test_fuel_trim_translation_matches_ini_zero(void)
{
  TEST_ASSERT_EQUAL_UINT8(128U, FUEL_TRIM.toRaw(0));
  TEST_ASSERT_EQUAL_INT8(0, FUEL_TRIM.toUser(128U));
}

void test_seq_trim_learning_adjusts_exact_visited_cell(void)
{
  setupLearner(0U, 10U); //4 seconds at a one-percent error
  primeHistory();
  runSamples(3000U, 60U, 180U, 12U); //Error clamps to +20 percent
  runSamples(5000U, 100U, 147U, 20U); //Drain delayed AFR samples

  TEST_ASSERT_GREATER_THAN_UINT8(TRIM_RAW_ZERO, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);
  TEST_ASSERT_GREATER_THAN_UINT16(0U, seqTrimAutotuneDiag().cellUpdates);
  TEST_ASSERT_EQUAL_UINT8(0U, seqTrimAutotuneDiag().lastTrim);
  TEST_ASSERT_EQUAL_UINT8(CELL_AT_3000_RPM_60_LOAD, seqTrimAutotuneDiag().lastCell);
}

void test_seq_trim_evidence_survives_visits_to_other_cells(void)
{
  setupLearner(7U, 10U); //512 seconds prevents a table step in this short test
  primeHistory();
  runSamples(3000U, 60U, 162U, 8U);
  runSamples(5000U, 100U, 147U, 20U);
  const int32_t evidenceAfterFirstVisit = seqTrimAutotuneCellAccumulator(0U, CELL_AT_3000_RPM_60_LOAD);
  TEST_ASSERT_GREATER_THAN_INT32(0, evidenceAfterFirstVisit);

  runSamples(5000U, 100U, 147U, 10U);
  TEST_ASSERT_EQUAL_INT32(evidenceAfterFirstVisit,
                          seqTrimAutotuneCellAccumulator(0U, CELL_AT_3000_RPM_60_LOAD));

  runSamples(3000U, 60U, 162U, 8U);
  runSamples(5000U, 100U, 147U, 20U);
  TEST_ASSERT_GREATER_THAN_INT32(evidenceAfterFirstVisit,
                                 seqTrimAutotuneCellAccumulator(0U, CELL_AT_3000_RPM_60_LOAD));
}

void test_seq_trim_authority_is_relative_to_drive_cycle_baseline(void)
{
  setupLearner(0U, 1U);
  primeHistory();
  runSamples(3000U, 60U, 180U, 100U);
  runSamples(5000U, 100U, 147U, 20U);

  TEST_ASSERT_EQUAL_UINT8(TRIM_RAW_ZERO + 1U, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);
}

void test_seq_trim_can_learn_from_selected_afr2_channel(void)
{
  setupLearner(0U, 10U);
  configPage15.seqTrimAutotuneConfig[0] |= 0x04U; //AFR2
  seqTrimAutotuneInit();
  primeHistory();
  runDualSamples(3000U, 60U, 147U, 180U, 12U);
  runDualSamples(5000U, 100U, 147U, 147U, 20U);

  TEST_ASSERT_GREATER_THAN_UINT8(TRIM_RAW_ZERO, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);
}

void test_seq_trim_balance_moves_two_feedback_channels_oppositely(void)
{
  setupLearner(0U, 10U);
  configPage15.seqTrimAutotuneConfig[1] = SEQ_TRIM_AUTOTUNE_LEARN | 0x04U; //Trim 2 uses AFR2
  configPage15.seqTrimAutotuneFlags = 0x02U; //Balance objective, save at stop
  seqTrimAutotuneInit();
  primeHistory();
  runDualSamples(3000U, 60U, 165U, 130U, 12U);
  runDualSamples(5000U, 100U, 147U, 147U, 20U);

  TEST_ASSERT_GREATER_THAN_UINT8(TRIM_RAW_ZERO, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);
  TEST_ASSERT_LESS_THAN_UINT8(TRIM_RAW_ZERO, trimTables[1].values.values[CELL_AT_3000_RPM_60_LOAD]);
}

void test_seq_trim_ram_only_restores_baseline_at_stop(void)
{
  setupLearner(0U, 10U);
  configPage15.seqTrimAutotuneFlags = 0x01U; //Target objective, RAM-only persistence
  primeHistory();
  runSamples(3000U, 60U, 180U, 12U);
  runSamples(5000U, 100U, 147U, 20U);
  TEST_ASSERT_GREATER_THAN_UINT8(TRIM_RAW_ZERO, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);

  currentStatus.rotationStatus = EngineRotationStatus::Stopped;
  seqTrimAutotuneUpdate();
  TEST_ASSERT_EQUAL_UINT8(TRIM_RAW_ZERO, trimTables[0].values.values[CELL_AT_3000_RPM_60_LOAD]);
  TEST_ASSERT_FALSE(isEepromWritePending());
}

} //namespace

void testSeqTrimAutotune(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST(test_fuel_trim_translation_matches_ini_zero);
    RUN_TEST(test_seq_trim_learning_adjusts_exact_visited_cell);
    RUN_TEST(test_seq_trim_evidence_survives_visits_to_other_cells);
    RUN_TEST(test_seq_trim_authority_is_relative_to_drive_cycle_baseline);
    RUN_TEST(test_seq_trim_can_learn_from_selected_afr2_channel);
    RUN_TEST(test_seq_trim_balance_moves_two_feedback_channels_oppositely);
    RUN_TEST(test_seq_trim_ram_only_restores_baseline_at_stop);
  }
}
