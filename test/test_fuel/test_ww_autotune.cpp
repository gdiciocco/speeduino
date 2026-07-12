#include <unity.h>
#include "../test_utils.h"
#include "globals.h"
#include "ww_autotune.h"
#include "units.h"

static void clear_ww_tables(void) {
  for (uint8_t i = 0; i < 64U; i++) {
    wallWettingAddTable.values.values[i] = 0;
    wallWettingRemoveTable.values.values[i] = 0;
  }
  for (uint8_t i = 0; i < 8U; i++) {
    wallWettingAddTable.axisX.axis[i] = 0;
    wallWettingAddTable.axisY.axis[i] = 0;
    wallWettingRemoveTable.axisX.axis[i] = 0;
    wallWettingRemoveTable.axisY.axis[i] = 0;
  }
}

static void setup_ww_engine_params(void) {
  configPage4.HardRevLim = 110; // 11000 RPM
  configPage2.fuelAlgorithm = LOAD_SOURCE_MAP;
  configPage2.mapMax = 101;
  configPage2.injType = INJ_TYPE_PORT;
  configPage2.strokes = 0; // four-stroke
}

static void test_ww_baseline_seeds_empty_tables(void) {
  clear_ww_tables();
  setup_ww_engine_params();

  TEST_ASSERT_TRUE(wwSeedBaselineTables(false));

  // Axes are stored descending in memory, compressed (RPM/100, load/2)
  TEST_ASSERT_EQUAL_UINT8(110, wallWettingAddTable.axisX.axis[0]); // rev limit
  TEST_ASSERT_EQUAL_UINT8(5, wallWettingAddTable.axisX.axis[7]);   // 500 RPM
  for (uint8_t i = 1; i < 8U; i++) {
    TEST_ASSERT_TRUE(wallWettingAddTable.axisX.axis[i - 1U] > wallWettingAddTable.axisX.axis[i]);
    TEST_ASSERT_TRUE(wallWettingAddTable.axisY.axis[i - 1U] > wallWettingAddTable.axisY.axis[i]);
  }

  // Both tables share the same axes
  TEST_ASSERT_EQUAL_UINT8_ARRAY(wallWettingAddTable.axisX.axis, wallWettingRemoveTable.axisX.axis, 8);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(wallWettingAddTable.axisY.axis, wallWettingRemoveTable.axisY.axis, 8);

  // X (added to wall) grows with load and shrinks with RPM.
  // Memory layout: row 0 = highest load; within a row, column 0 = lowest RPM.
  const uint8_t xHighLoadLowRpm = wallWettingAddTable.values.values[0];
  const uint8_t xHighLoadHighRpm = wallWettingAddTable.values.values[7];
  const uint8_t xLowLoadLowRpm = wallWettingAddTable.values.values[7U * 8U];
  TEST_ASSERT_TRUE(xHighLoadLowRpm > xHighLoadHighRpm);
  TEST_ASSERT_TRUE(xHighLoadLowRpm > xLowLoadLowRpm);

  // Y (removed from wall) grows with RPM
  const uint8_t yLowRpm = wallWettingRemoveTable.values.values[7U * 8U];
  const uint8_t yHighRpm = wallWettingRemoveTable.values.values[7U * 8U + 7U];
  TEST_ASSERT_TRUE(yHighRpm > yLowRpm);
}

static void test_ww_baseline_does_not_overwrite_valid_tables(void) {
  clear_ww_tables();
  setup_ww_engine_params();
  TEST_ASSERT_TRUE(wwSeedBaselineTables(false));

  wallWettingAddTable.values.values[10] = 123;
  TEST_ASSERT_FALSE(wwSeedBaselineTables(false));
  TEST_ASSERT_EQUAL_UINT8(123, wallWettingAddTable.values.values[10]);

  TEST_ASSERT_TRUE(wwSeedBaselineTables(true)); // force always reseeds
  TEST_ASSERT_NOT_EQUAL(123, wallWettingAddTable.values.values[10]);
}

static void test_ww_baseline_tbody_scales_up(void) {
  clear_ww_tables();
  setup_ww_engine_params();
  TEST_ASSERT_TRUE(wwSeedBaselineTables(false));
  const uint8_t portValue = wallWettingAddTable.values.values[0];

  configPage2.injType = INJ_TYPE_TBODY;
  TEST_ASSERT_TRUE(wwSeedBaselineTables(true));
  TEST_ASSERT_TRUE(wallWettingAddTable.values.values[0] > portValue);
  configPage2.injType = INJ_TYPE_PORT;
}

/** Make every learning gate pass */
static void setup_ww_learn_conditions(void) {
  configPage2.aeMode = AE_MODE_WALL_WETTING;
  configPage2.wallWettingFuel = 10; // learning on, authority 10 counts
  configPage2.aeColdTaperMax = 0;
  configPage2.aeTaperMin = 50;
  configPage2.aeTaperMax = 0; // taper disabled (max <= min)
  configPage2.taeThresh = 50;
  configPage2.maeThresh = 50;
  configPage6.egoType = EGO_TYPE_WIDE;
  configPage6.egoTemp = 0;
  configPage6.egoRPM = 10; // 1000 RPM
  configPage6.ego_sdelay = 10;

  currentStatus.rotationStatus = EngineRotationStatus::Running;
  currentStatus.runSecs = 255;
  currentStatus.coolant = 85;
  currentStatus.setRpm(3000U);
  currentStatus.fuelLoad = 60;
  currentStatus.battery10 = 130;
  currentStatus.egoCorrection = 100;
  currentStatus.afrTarget = 147;
  currentStatus.O2 = 147;
  currentStatus.tpsDOT = 0;
  currentStatus.mapDOT = 0;
  currentStatus.isDFCOActive = false;
  currentStatus.aseIsActive = false;
  currentStatus.wueIsActive = false;
  currentStatus.launchingHard = false;
  currentStatus.launchingSoft = false;
  currentStatus.flatShiftingHard = false;
  currentStatus.flatShiftSoftCut = false;
  currentStatus.nitrousActive = false;
  currentStatus.stagingActive = false;
  currentStatus.isDeceleratingTPS = false;
}

/** Simulate a tip-in followed by a persistently lean mixture, through the full
 * observation window. Returns after the event has been finalized. */
static void run_lean_tip_in(void) {
  // Fill the history ring under steady conditions
  for (uint8_t i = 0; i < 30U; i++) { wwAutotuneUpdate(); }

  // Tip-in: DOT above threshold for two ticks, mixture goes lean by 0.8 AFR
  currentStatus.tpsDOT = 200;
  currentStatus.O2 = currentStatus.afrTarget + 8U;
  wwAutotuneUpdate();
  wwAutotuneUpdate();
  currentStatus.tpsDOT = 0;

  // Ride out capture + wideband-delay drain (30 + 23 ticks) plus margin
  for (uint8_t i = 0; i < 60U; i++) { wwAutotuneUpdate(); }
}

static void test_ww_learner_lean_tip_in_adjusts_tables(void) {
  clear_ww_tables();
  setup_ww_engine_params();
  wwAutotuneInit(); // seeds baseline, resets learner state
  setup_ww_learn_conditions();

  // Locate the cell the event will anchor to: RPM 3000 -> compressed 30,
  // load 60 -> compressed 30 (nearest bins looked up on the seeded axes)
  uint8_t xIdx = 0, yIdx = 0;
  for (uint8_t i = 1; i < 8U; i++) {
    if (abs((int16_t)wallWettingAddTable.axisX.axis[i] - 30) < abs((int16_t)wallWettingAddTable.axisX.axis[xIdx] - 30)) { xIdx = i; }
    if (abs((int16_t)wallWettingAddTable.axisY.axis[i] - 30) < abs((int16_t)wallWettingAddTable.axisY.axis[yIdx] - 30)) { yIdx = i; }
  }
  const uint8_t cell = (uint8_t)((yIdx * 8U) + (7U - xIdx));
  const uint8_t xBefore = wallWettingAddTable.values.values[cell];
  const uint8_t yBefore = wallWettingRemoveTable.values.values[cell];

  run_lean_tip_in();

  TEST_ASSERT_EQUAL_UINT16(1, wwAutotuneLearnedEventCount());
  // Lean peak -> more fuel deposits than modelled -> X up
  TEST_ASSERT_TRUE(wallWettingAddTable.values.values[cell] > xBefore);
  // Lean tail -> film evaporates slower than modelled -> Y down
  TEST_ASSERT_TRUE(wallWettingRemoveTable.values.values[cell] < yBefore);
}

static void test_ww_learner_disabled_by_zero_authority(void) {
  clear_ww_tables();
  setup_ww_engine_params();
  wwAutotuneInit();
  setup_ww_learn_conditions();
  configPage2.wallWettingFuel = 0; // autotune off

  run_lean_tip_in();

  TEST_ASSERT_EQUAL_UINT16(0, wwAutotuneLearnedEventCount());
}

static void test_ww_learner_aborts_on_gate_failure(void) {
  clear_ww_tables();
  setup_ww_engine_params();
  wwAutotuneInit();
  setup_ww_learn_conditions();

  for (uint8_t i = 0; i < 30U; i++) { wwAutotuneUpdate(); }
  currentStatus.tpsDOT = 200;
  currentStatus.O2 = currentStatus.afrTarget + 8U;
  wwAutotuneUpdate();
  wwAutotuneUpdate();
  currentStatus.tpsDOT = 0;
  currentStatus.isDFCOActive = true; // fuel cut mid-window poisons the event
  for (uint8_t i = 0; i < 60U; i++) { wwAutotuneUpdate(); }
  currentStatus.isDFCOActive = false;

  TEST_ASSERT_EQUAL_UINT16(0, wwAutotuneLearnedEventCount());
}

void testWwAutotune(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_ww_baseline_seeds_empty_tables);
    RUN_TEST_P(test_ww_baseline_does_not_overwrite_valid_tables);
    RUN_TEST_P(test_ww_baseline_tbody_scales_up);
    RUN_TEST_P(test_ww_learner_lean_tip_in_adjusts_tables);
    RUN_TEST_P(test_ww_learner_disabled_by_zero_authority);
    RUN_TEST_P(test_ww_learner_aborts_on_gate_failure);
  }
}
