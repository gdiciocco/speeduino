#include <unity.h>
#include "globals.h"
#include "corrections.h"
#include "../test_utils.h"
#include "sensors.h"
#include "units.h"

extern int8_t correctionFixedTiming(int8_t advance);
extern bool calculateIacGainAutotuneGains(long relayInternal, uint16_t amplitudeRpm,
                                          uint16_t periodTenths,
                                          uint8_t &kpRaw, uint8_t &kiRaw, uint8_t &kdRaw);

static void test_iac_gain_autotune_converts_relay_measurement(void) {
    uint8_t kp = 0U;
    uint8_t ki = 0U;
    uint8_t kd = 0U;
    TEST_ASSERT_TRUE(calculateIacGainAutotuneGains(20L, 100U, 12U, kp, ki, kd));
    TEST_ASSERT_EQUAL_UINT8(4U, kp);
    TEST_ASSERT_EQUAL_UINT8(8U, ki);
    TEST_ASSERT_EQUAL_UINT8(2U, kd);
    TEST_ASSERT_FALSE(calculateIacGainAutotuneGains(20L, 0U, 12U, kp, ki, kd));
}

static void test_correctionFixedTiming_inactive(void) {
    configPage2.fixAngEnable = 0;
    configPage4.FixAng = 13;

    TEST_ASSERT_EQUAL(8, correctionFixedTiming(8));
    TEST_ASSERT_EQUAL(-3, correctionFixedTiming(-3));
}

static void test_correctionFixedTiming_active(void) {
    configPage2.fixAngEnable = 1;
    configPage4.FixAng = 13;

    TEST_ASSERT_EQUAL(configPage4.FixAng, correctionFixedTiming(8));
    TEST_ASSERT_EQUAL(configPage4.FixAng, correctionFixedTiming(-3));
}

static void test_correctionFixedTiming(void) {
    RUN_TEST_P(test_correctionFixedTiming_inactive);
    RUN_TEST_P(test_correctionFixedTiming_active);
}

extern int8_t correctionCLTadvance(int8_t advance);
extern table2D_u8_u8_6 CLTAdvanceTable; ///< 6 bin ignition adjustment based on coolant temperature  (2D)

static void setup_clt_advance_table(void) {
  initialiseCorrections();
  currentStatus.LOOP_TIMER = 0;
  BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_4HZ);
  TEST_DATA_P uint8_t bins[] = { 60, 70, 80, 90, 100, 110 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&CLTAdvanceTable, values, bins);
}

static void test_correctionCLTadvance_lookup(void) {
    setup_clt_advance_table();

    currentStatus.coolant = temperatureRemoveOffset(105);
    TEST_ASSERT_EQUAL(8 + 8 - 15, correctionCLTadvance(8));

    currentStatus.coolant = temperatureRemoveOffset(65);
    TEST_ASSERT_EQUAL(1 + 28 - 15, correctionCLTadvance(1));

    currentStatus.coolant = temperatureRemoveOffset(105);
    TEST_ASSERT_EQUAL(-3 + 8 - 15, correctionCLTadvance(-3));
}

static void test_correctionCLTadvance(void) {
    RUN_TEST_P(test_correctionCLTadvance_lookup);
}

static void test_correctionCrankingFixedTiming_nocrank_inactive(void) {
    setup_clt_advance_table();
    currentStatus.rotationStatus = EngineRotationStatus::Running;
    configPage2.crkngAddCLTAdv = 0;
    configPage4.CrankAng = 8;

    TEST_ASSERT_EQUAL(-7, correctionCrankingFixedTiming(-7));
}

static void test_correctionCrankingFixedTiming_crank_fixed(void) {
    setup_clt_advance_table();
    currentStatus.rotationStatus = EngineRotationStatus::Cranking;
    configPage2.crkngAddCLTAdv = 0;

    configPage4.CrankAng = 8;
    TEST_ASSERT_EQUAL(configPage4.CrankAng, correctionCrankingFixedTiming(-7));

    configPage4.CrankAng = -8;
    TEST_ASSERT_EQUAL(configPage4.CrankAng, correctionCrankingFixedTiming(-7));
}

static void test_correctionCrankingFixedTiming_crank_coolant(void) {
    setup_clt_advance_table();
    currentStatus.rotationStatus = EngineRotationStatus::Cranking;
    configPage2.crkngAddCLTAdv = 1;
    
    configPage4.CrankAng = 8;

    currentStatus.coolant = temperatureRemoveOffset(65);
    TEST_ASSERT_EQUAL(1 + 28 - 15, correctionCLTadvance(1));
}

static void test_correctionCrankingFixedTiming(void) {
    RUN_TEST_P(test_correctionCrankingFixedTiming_nocrank_inactive);
    RUN_TEST_P(test_correctionCrankingFixedTiming_crank_fixed);
    RUN_TEST_P(test_correctionCrankingFixedTiming_crank_coolant);
}

extern int8_t correctionFlexTiming(int8_t advance);
extern table2D_u8_u8_6 flexAdvTable;   ///< 6 bin flex fuel correction table for timing advance (2D)

static void setup_flexAdv(void) {
  initialiseCorrections();
  TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&flexAdvTable, values, bins);

  configPage2.flexEnabled = 1;
  currentStatus.ethanolPct = 55;
}

static void test_correctionFlexTiming_inactive(void) {
    setup_flexAdv();
    configPage2.flexEnabled = 0;

    TEST_ASSERT_EQUAL(-7, correctionFlexTiming(-7));
    TEST_ASSERT_EQUAL(3, correctionFlexTiming(3));
}

static void test_correctionFlexTiming_table_lookup(void) {
    setup_flexAdv();

    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(8 + 18), correctionFlexTiming(8));
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(18), currentStatus.flexIgnCorrection);    

    currentStatus.ethanolPct = 35;
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(-4 + 28), correctionFlexTiming(-4));
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(28), currentStatus.flexIgnCorrection);    
}

static void test_correctionFlexTiming(void) {
    RUN_TEST_P(test_correctionFlexTiming_inactive);
    RUN_TEST_P(test_correctionFlexTiming_table_lookup);
}

extern int8_t correctionWMITiming(int8_t advance);
extern table2D_u8_u8_6 wmiAdvTable; //6 bin wmi correction table for timing advance (2D)

static void setup_WMIAdv(void) {
    initialiseCorrections();

    configPage10.wmiEnabled= 1;
    configPage10.wmiAdvEnabled = 1;
    currentStatus.wmiTankEmpty = false;
    configPage10.wmiTPS = 50;
    currentStatus.TPS = configPage10.wmiTPS + 1;
    configPage10.wmiRPM = 30;
    currentStatus.setRpm( RPM_COARSE.toUser(configPage10.wmiRPM + 1U));
    configPage10.wmiMAP = 35;
    currentStatus.MAP = MAP.toUser(configPage10.wmiMAP+1L);
    configPage10.wmiIAT = 155;
    currentStatus.IAT = temperatureRemoveOffset(configPage10.wmiIAT) + 1;

    TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
    TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
    populate_2dtable_P(&wmiAdvTable, values, bins);
}

static void test_correctionWMITiming_table_lookup(void) {
    setup_WMIAdv();

    currentStatus.MAP = (55*2U)+1U;
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(8 + 18), correctionWMITiming(8));

    currentStatus.MAP = (35*2U)+1U;
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toUser(-4 + 28), correctionWMITiming(-4));
}

static void test_correctionWMITiming_wmidisabled_inactive(void) {
    setup_WMIAdv();
    configPage10.wmiEnabled= 0;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}


static void test_correctionWMITiming_wmiadvdisabled_inactive(void) {
    setup_WMIAdv();
    configPage10.wmiAdvEnabled = 0;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}

static void test_correctionWMITiming_empty_inactive(void) {
    setup_WMIAdv();
    currentStatus.wmiTankEmpty = true;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}

static void test_correctionWMITiming_tpslow_inactive(void) {
    setup_WMIAdv();
    currentStatus.TPS = configPage10.wmiTPS - 1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}

static void test_correctionWMITiming_rpmlow_inactive(void) {
    setup_WMIAdv();
    currentStatus.setRpm( configPage10.wmiRPM - 1U);

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}
   
static void test_correctionWMITiming_maplow_inactive(void) {
    setup_WMIAdv();
    currentStatus.MAP = (configPage10.wmiMAP*2)-1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}
    
static void test_correctionWMITiming_iatlow_inactive(void) {
    setup_WMIAdv();
    currentStatus.IAT = temperatureRemoveOffset(configPage10.wmiIAT) - 1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
    TEST_ASSERT_EQUAL(-3, correctionWMITiming(-3));
}   

static void test_correctionWMITiming(void) {
    RUN_TEST_P(test_correctionWMITiming_table_lookup);
    RUN_TEST_P(test_correctionWMITiming_tpslow_inactive);
    RUN_TEST_P(test_correctionWMITiming_wmidisabled_inactive);
    RUN_TEST_P(test_correctionWMITiming_wmiadvdisabled_inactive);
    RUN_TEST_P(test_correctionWMITiming_empty_inactive);
    RUN_TEST_P(test_correctionWMITiming_tpslow_inactive);
    RUN_TEST_P(test_correctionWMITiming_rpmlow_inactive);
    RUN_TEST_P(test_correctionWMITiming_maplow_inactive);
    RUN_TEST_P(test_correctionWMITiming_iatlow_inactive);
}

extern int8_t correctionIATretard(int8_t advance);
extern table2D_u8_u8_6 IATRetardTable; ///< 6 bin ignition adjustment based on inlet air temperature  (2D)

static void setup_IATRetard(void) {
  initialiseCorrections();
  currentStatus.LOOP_TIMER = 0;
  BIT_SET(currentStatus.LOOP_TIMER, IAT_READ_TIMER_BIT);
  TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&IATRetardTable, values, bins);

  currentStatus.IAT = 75;
}

static void test_correctionIATretard_table_lookup(void) {
    setup_IATRetard();

    currentStatus.IAT = 75;
    TEST_ASSERT_EQUAL(-11-8, correctionIATretard(-11));

    currentStatus.IAT = 45;
    TEST_ASSERT_EQUAL(-11-23, correctionIATretard(-11));
}

static void test_correctionIATretard(void) {
    RUN_TEST_P(test_correctionIATretard_table_lookup);
}

extern int8_t correctionIdleAdvance(int8_t advance);
extern int16_t computeIdleAdvanceClosedLoopTarget(int16_t centerTenths, int32_t rpmDot);
extern int16_t idleAdvanceClCenter;
extern int16_t idleAdvanceClTrim;
extern int8_t idleAdvanceClLearnedDelta;
extern uint8_t idleAdvanceGainTuneAttempts;
extern bool idleAdvanceGainTuneLastRequest;

static void setup_idleadv_tps(void) {
    configPage2.idleAdvAlgorithm = IDLEADVANCE_ALGO_TPS;
    configPage2.idleAdvTPS = 30;
    currentStatus.TPS = configPage2.idleAdvTPS - 1U;
}

static void setup_idleadv_ctps(void) {
    configPage2.idleAdvAlgorithm = IDLEADVANCE_ALGO_CTPS;
    currentStatus.CTPSActive = 1;
}

extern table2D_u8_u8_6 idleAdvanceTable; ///< 6 bin idle advance adjustment table based on RPM difference  (2D)

static void setup_correctionIdleAdvance(void) {
    initialiseCorrections();

    TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
    TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
    populate_2dtable_P(&idleAdvanceTable, values, bins);
  
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_ADDED;
    configPage2.idleAdvDelay = 5;
    configPage2.idleAdvRPM = 20;
    configPage2.vssMode = VSS_MODE_OFF;
    configPage6.iacAlgorithm = IAC_ALGORITHM_NONE;
    configPage9.idleAdvStartDelay = 0U;

    runSecsX10 = configPage2.idleAdvDelay * 5;
    currentStatus.rotationStatus = EngineRotationStatus::Running;
    // int idleRPMdelta = (currentStatus.CLIdleTarget - (currentStatus.RPM / 10) ) + 50;
    currentStatus.CLIdleTarget = 100;
    currentStatus.setRpm( (configPage2.idleAdvRPM * 100U) - 1U);
    
    setup_idleadv_tps();
    // Run once to initialise internal state
    correctionIdleAdvance(8);
}

static void assert_correctionIdleAdvance(int8_t advance, uint8_t expectedLookupValue) {
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_ADDED;
    TEST_ASSERT_EQUAL(advance + expectedLookupValue - 15, correctionIdleAdvance(advance));

    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_SWITCHED;
    TEST_ASSERT_EQUAL(expectedLookupValue - 15, correctionIdleAdvance(advance));
}

static void test_correctionIdleAdvance_tps_lookup_nodelay(void) {
    setup_correctionIdleAdvance();

    setup_idleadv_tps();

    currentStatus.setRpm( (currentStatus.CLIdleTarget * 10U) + 100U);
    assert_correctionIdleAdvance(8, 25);

    currentStatus.setRpm( (currentStatus.CLIdleTarget * 10U) - 100U);
    assert_correctionIdleAdvance(-3, 15);
}

static void test_correctionIdleAdvance_ctps_lookup_nodelay(void) {
    setup_correctionIdleAdvance();

    setup_idleadv_ctps();

    currentStatus.setRpm( (currentStatus.CLIdleTarget * 10) + 100);
    assert_correctionIdleAdvance(8, 25);

    currentStatus.setRpm( (currentStatus.CLIdleTarget * 10) - 100);
    assert_correctionIdleAdvance(-3, 15);
}

static void test_correctionIdleAdvance_inactive_notrunning(void) {
    setup_correctionIdleAdvance();
    
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.rotationStatus = EngineRotationStatus::Stopped;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_modeoff(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_OFF;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_rpmtoohigh(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.setRpm( (configPage2.idleAdvRPM * 100)+1);
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_vsslimit(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    configPage2.vssMode = VSS_MODE_INTERNAL_PIN;
    configPage2.idleAdvVss = 15;
    currentStatus.vss = configPage2.idleAdvVss + 1;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_tpslimit(void) {
    setup_correctionIdleAdvance();
    setup_idleadv_tps();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.TPS = configPage2.idleAdvTPS + 1U;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_ctpsinactive(void) {
    setup_correctionIdleAdvance();
    setup_idleadv_ctps();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.CTPSActive = 0;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_rundelay(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    runSecsX10 = (configPage2.idleAdvDelay * 5)-1;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_delay(void) {
    setup_correctionIdleAdvance();
    configPage9.idleAdvStartDelay = 3;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    ++runSecsX10;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    ++runSecsX10;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    ++runSecsX10;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_delay_updates_once_per_tick(void) {
    setup_correctionIdleAdvance();
    configPage9.idleAdvStartDelay = 2;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    ++runSecsX10;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    // A second ignition correction in the same 10Hz tick must not shorten the delay.
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    ++runSecsX10;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_delay_resets_after_engine_stop(void) {
    setup_correctionIdleAdvance();
    configPage9.idleAdvStartDelay = 2;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    runSecsX10 += 2U;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));

    currentStatus.rotationStatus = EngineRotationStatus::Stopped;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    currentStatus.rotationStatus = EngineRotationStatus::Running;
    runSecsX10 = TIME_TWENTY_MILLIS.toUser(configPage2.idleAdvDelay);
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    runSecsX10 += 2U;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_iac_start_threshold_is_latched(void) {
    setup_correctionIdleAdvance();
    configPage6.iacAlgorithm = IAC_ALGORITHM_PWM_CL;
    initialiseCorrections(); //Clear the arm set by setup with IAC_ALGORITHM_NONE.

    currentStatus.setRpm(700U); //More than 200 RPM below the 1000 RPM target.
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));

    currentStatus.setRpm(850U); //Cross the one-time arming threshold.
    TEST_ASSERT_NOT_EQUAL(8, correctionIdleAdvance(8));

    currentStatus.setRpm(700U); //A subsequent dip must not turn idle advance off.
    TEST_ASSERT_EQUAL(-2, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_low_target_does_not_underflow(void) {
    setup_correctionIdleAdvance();
    configPage6.iacAlgorithm = IAC_ALGORITHM_PWM_CL;
    currentStatus.CLIdleTarget = 10U; //100 RPM, below the 200 RPM arming threshold.
    currentStatus.setRpm(50U);
    initialiseCorrections();

    TEST_ASSERT_NOT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_delta_does_not_wrap_above_2550rpm(void) {
    setup_correctionIdleAdvance();
    configPage2.idleAdvRPM = 40U;
    currentStatus.setRpm(3000U);

    // RPM is far above target, so the lookup must clamp to the lowest delta bin.
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_added_mode_clamps_overflow(void) {
    setup_correctionIdleAdvance();
    currentStatus.setRpm(1500U); //Select the +15 degree end of the test table.

    TEST_ASSERT_EQUAL(INT8_MAX, correctionIdleAdvance(120));
}

static void setup_correctionIdleAdvance_closed_loop(void) {
    setup_correctionIdleAdvance();
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_CLOSED_LOOP;
    configPage9.idleAdvClMinAdvance = IGNITION_ADVANCE_LARGE.toRaw(0);
    configPage9.idleAdvClMaxAdvance = IGNITION_ADVANCE_LARGE.toRaw(20);
    configPage9.idleAdvClKp = 40U; //2.0 degrees per 100 RPM of error
    configPage9.idleAdvClKd = 60U; //3.0 degrees per 1000 RPM/s
    configPage15.idleAdvClCenter = IGNITION_ADVANCE_LARGE.toRaw(10);
    configPage15.idleAdvClDeadband = 15U;
    configPage15.idleAdvClRpmFilter = 50U;
    configPage15.idleAdvClTrimRate = 0U;
    configPage15.idleAdvClTrimRange = 5U;
    configPage15.idleAdvClTrimRequiresIacLimit = 1U;
    configPage15.idleAdvClLearnAuthority = 0U; //Center autotune off unless a test enables it
    configPage15.idleAdvClGainAutotuneRequest = 0U; //Gain autotune off unless a test requests it
    configPage15.idleAdvClGainTuneStep = 3U;
    configPage15.idleAdvClGainTuneSettleTime = 3U;
    configPage15.idleAdvClGainTuneSettleBand = 20U;
    configPage15.idleAdvClGainTuneHysteresis = 0U;
    configPage15.idleAdvClGainTuneDiscard = 2U;
    configPage15.idleAdvClGainTuneMeasure = 6U;
    configPage15.idleAdvClGainTuneTimeout = 5U;
    configPage15.idleAdvClGainTuneRunawayDiv10 = 40U;
    configPage15.idleAdvClGainTuneMinAmplitude = 20U;
    configPage15.idleAdvClGainTuneMinPeriod = 4U;
    configPage15.idleAdvClGainTuneMaxPeriod = 60U;
    configPage15.idleAdvClGainTuneMaxAttempts = 3U;
    currentStatus.CLIdleTarget = RPM_MEDIUM.toRaw(1000);
    currentStatus.setRpm(1000U);
    initialiseCorrections();
}

//All expectations below are in tenths of a degree, with a center of 10.0 degrees,
//Kp of 2.0 deg/100RPM, Kd of 3.0 deg per 1000 RPM/s and a 15 RPM deadband.
static void test_idleAdvanceClosedLoop_pd_target(void) {
    setup_correctionIdleAdvance_closed_loop();

    //On target with no trend: the loop commands the center exactly.
    TEST_ASSERT_EQUAL(100, computeIdleAdvanceClosedLoopTarget(100, 0L));

    //100 RPM low, less the 15 RPM deadband, at 2.0 deg per 100 RPM.
    currentStatus.setRpm(900U);
    TEST_ASSERT_EQUAL(117, computeIdleAdvanceClosedLoopTarget(100, 0L));
    currentStatus.setRpm(1100U);
    TEST_ASSERT_EQUAL(83, computeIdleAdvanceClosedLoopTarget(100, 0L));

    //The gain is fixed, so it does not change with the center. Scaling the
    //authority relative to the center instead would halve this response.
    TEST_ASSERT_EQUAL(133, computeIdleAdvanceClosedLoopTarget(150, 0L));

    //Rising RPM removes advance, falling RPM adds it.
    currentStatus.setRpm(1000U);
    TEST_ASSERT_EQUAL(85, computeIdleAdvanceClosedLoopTarget(100, 500L));
    TEST_ASSERT_EQUAL(115, computeIdleAdvanceClosedLoopTarget(100, -500L));

    //The output stays inside the configured authority.
    currentStatus.setRpm(400U);
    TEST_ASSERT_EQUAL(200, computeIdleAdvanceClosedLoopTarget(100, 0L));
    currentStatus.setRpm(1600U);
    TEST_ASSERT_EQUAL(0, computeIdleAdvanceClosedLoopTarget(100, 0L));
}

static void test_idleAdvanceClosedLoop_deadband_has_no_step(void) {
    setup_correctionIdleAdvance_closed_loop();

    //Inside the deadband the proportional term is inactive...
    currentStatus.setRpm(990U);
    TEST_ASSERT_EQUAL(100, computeIdleAdvanceClosedLoopTarget(100, 0L));
    //...and it resumes from zero at the edge rather than jumping.
    currentStatus.setRpm(985U);
    TEST_ASSERT_EQUAL(100, computeIdleAdvanceClosedLoopTarget(100, 0L));
    currentStatus.setRpm(980U);
    TEST_ASSERT_EQUAL(101, computeIdleAdvanceClosedLoopTarget(100, 0L));
}

static void test_correctionIdleAdvance_closed_loop_responds_without_output_slew(void) {
    setup_correctionIdleAdvance_closed_loop();
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    //Engaging on target holds the advance the ignition table was already asking
    //for, so entering the loop is not a torque step.
    TEST_ASSERT_EQUAL(10, correctionIdleAdvance(10));

    //A 200 RPM drop must produce the full response on the very next tick.
    //Rate limiting the output would put the ignition, which is the fast
    //actuator, into the same frequency band as the air path.
    //Proportional: (200-15) * 2.0/100 = 3.7 degrees.
    //Derivative:   the 50% filter moves 1000 -> 900 RPM over the 100ms tick,
    //              so -1000 RPM/s, and falling RPM adds 3.0 degrees.
    ++runSecsX10;
    currentStatus.setRpm(800U);
    TEST_ASSERT_EQUAL(17, correctionIdleAdvance(10));
    TEST_ASSERT_EQUAL(17, correctionIdleAdvance(10)); //Same scheduler tick.
}

static void test_correctionIdleAdvance_closed_loop_trim_freezes_in_deadband(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClTrimRate = 10U;            //10s per degree at 100 RPM of error
    configPage15.idleAdvClTrimRequiresIacLimit = 0U;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    (void)correctionIdleAdvance(10); //Engage on target.

    //Hold a 100 RPM error for half of a trim step.
    currentStatus.setRpm(900U);
    for(uint8_t tick = 0U; tick < 5U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(0, idleAdvanceClTrim);

    //Inside the deadband the accumulator must be frozen, not zeroed. Zeroing it
    //is what makes the trim drift to the deadband edge, lose its history and
    //drift back again.
    currentStatus.setRpm(995U);
    for(uint8_t tick = 0U; tick < 3U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(0, idleAdvanceClTrim);

    //So the remaining half of the step still completes in five more ticks.
    currentStatus.setRpm(900U);
    for(uint8_t tick = 0U; tick < 5U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(1, idleAdvanceClTrim);
}

static void test_correctionIdleAdvance_closed_loop_trim_yields_to_iac(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClTrimRate = 1U;             //Fast enough to move every tick
    configPage15.idleAdvClTrimRequiresIacLimit = 1U;
    configPage6.iacAlgorithm = IAC_ALGORITHM_PWM_CL; //The air path owns the steady state offset
    initialiseCorrections();
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    currentStatus.setRpm(900U); //Arms idle advance and holds a persistent error.
    for(uint8_t tick = 0U; tick < 10U; ++tick) {
        (void)correctionIdleAdvance(10);
        ++runSecsX10;
    }

    //The IAC closed loop has authority left, so the ignition loop must not run a
    //second integrator against the same measurement.
    TEST_ASSERT_EQUAL(0, idleAdvanceClTrim);
}

static void test_correctionIdleAdvance_closed_loop_trim_is_bounded(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClTrimRate = 1U;
    configPage15.idleAdvClTrimRange = 2U; //degrees
    configPage15.idleAdvClTrimRequiresIacLimit = 0U;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);

    (void)correctionIdleAdvance(10);
    currentStatus.setRpm(940U); //60 RPM low: outside the deadband, well short of saturating.
    for(uint16_t tick = 0U; tick < 200U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }

    //A persistent error must not be able to drag the center arbitrarily far
    //from its configured value.
    TEST_ASSERT_EQUAL(20, idleAdvanceClTrim);
}

static void setup_correctionIdleAdvance_closed_loop_learning(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClTrimRate = 1U;   //One trim tenth per tick at 100 RPM of error
    configPage15.idleAdvClTrimRequiresIacLimit = 0U;
    configPage15.idleAdvClLearnAuthority = 2U; //degrees per power cycle
    configPage15.idleAdvClLearnMinTemp = temperatureAddOffset(70);
    currentStatus.coolant = 85;
    idleAdvanceClLearnedDelta = 0; //Survives initialiseCorrections() on purpose, so reset it per test
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
    (void)correctionIdleAdvance(10); //Engage on target.
}

//Drive the engine 100 RPM below target long enough to bank one whole degree of trim.
static void run_idleadv_learning_bank_one_degree(void) {
    currentStatus.setRpm(900U);
    for(uint8_t tick = 0U; tick < 10U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(10, idleAdvanceClTrim);
    //Return to target and hold through one full settle window (10s) plus the fold tick.
    currentStatus.setRpm(1000U);
    for(uint8_t tick = 0U; tick < 101U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
}

static void test_correctionIdleAdvance_closed_loop_learn_folds_trim_into_center(void) {
    setup_correctionIdleAdvance_closed_loop_learning();

    run_idleadv_learning_bank_one_degree();

    //The settled degree has moved from the volatile trim into the stored center.
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(11), configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL(0, idleAdvanceClTrim);
    TEST_ASSERT_EQUAL(1, idleAdvanceClLearnedDelta);

    //A fold moves a degree between the two terms of the same sum, so it must
    //never step the commanded advance.
    ++runSecsX10;
    TEST_ASSERT_EQUAL(11, correctionIdleAdvance(10));
}

static void test_correctionIdleAdvance_closed_loop_learn_respects_authority(void) {
    setup_correctionIdleAdvance_closed_loop_learning();
    configPage15.idleAdvClLearnAuthority = 1U;

    //The first fold consumes the whole authority...
    run_idleadv_learning_bank_one_degree();
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(11), configPage15.idleAdvClCenter);

    //...so a second settled degree must stay in the volatile trim: a persistent
    //error is not allowed to keep migrating into the tune.
    run_idleadv_learning_bank_one_degree();
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(11), configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL(10, idleAdvanceClTrim);
    TEST_ASSERT_EQUAL(1, idleAdvanceClLearnedDelta);
}

static void test_correctionIdleAdvance_closed_loop_learn_requires_warm_engine(void) {
    setup_correctionIdleAdvance_closed_loop_learning();
    currentStatus.coolant = 60; //Below the 70C learning threshold

    //A cold engine idles on different advance: the banked degree must stay in
    //the volatile trim and the stored center must not move.
    run_idleadv_learning_bank_one_degree();
    TEST_ASSERT_EQUAL(IGNITION_ADVANCE_LARGE.toRaw(10), configPage15.idleAdvClCenter);
    TEST_ASSERT_EQUAL(10, idleAdvanceClTrim);
    TEST_ASSERT_EQUAL(0, idleAdvanceClLearnedDelta);
}

static void setup_correctionIdleAdvance_gain_autotune(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClLearnMinTemp = temperatureAddOffset(70);
    currentStatus.coolant = 85;
    configPage15.idleAdvClGainAutotuneRequest = 1U;
    idleAdvanceGainTuneAttempts = 0U;      //Per power cycle on purpose, so reset per test
    idleAdvanceGainTuneLastRequest = false;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
    (void)correctionIdleAdvance(10); //Engage on target; first settled tick.

    //Hold a settled idle through the 3s stability window: the relay must not
    //start before it, and starts on the window's last tick.
    for(uint8_t tick = 0U; tick < 30U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RELAY, idleAdvanceGainAutotuneDiag().state);
}

static void test_correctionIdleAdvance_gain_autotune_measures_relay_and_writes_gains(void) {
    setup_correctionIdleAdvance_gain_autotune();

    //While the engine holds the target the relay pushes advance up: center 10 + 3.
    ++runSecsX10;
    TEST_ASSERT_EQUAL(13, correctionIdleAdvance(10));

    //Scripted plant response: a 100 RPM square wave with a 1.2s period. The
    //first two half cycles are the start transient and must be discarded.
    for(uint8_t phase = 0U; phase < 8U; ++phase) {
        currentStatus.setRpm(((phase % 2U) == 0U) ? 1100U : 900U);
        for(uint8_t tick = 0U; tick < 6U; ++tick) {
            ++runSecsX10;
            (void)correctionIdleAdvance(10);
        }
    }

    //Relay describing function: Ku = 4*3/(pi*100) deg/RPM, Tu = 1.2s.
    //Ziegler-Nichols PD: Kp = 0.8*Ku -> raw 61; Kd = Kp*Tu/8 -> raw 91.
    TEST_ASSERT_EQUAL(61, configPage9.idleAdvClKp);
    TEST_ASSERT_EQUAL(91, configPage9.idleAdvClKd);
    TEST_ASSERT_EQUAL(12, idleAdvanceGainAutotuneDiag().periodTenths);
    TEST_ASSERT_EQUAL(100, idleAdvanceGainAutotuneDiag().amplitudeRpm);
    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RESULT_DONE, idleAdvanceGainAutotuneDiag().lastResult);
    //The request is one-shot: the firmware clears it after writing the gains.
    TEST_ASSERT_EQUAL(0, configPage15.idleAdvClGainAutotuneRequest);
}

static void test_correctionIdleAdvance_gain_autotune_uses_configured_setup(void) {
    setup_correctionIdleAdvance_closed_loop();
    configPage15.idleAdvClLearnMinTemp = temperatureAddOffset(70);
    configPage15.idleAdvClGainTuneStep = 5U;
    configPage15.idleAdvClGainTuneSettleTime = 1U;
    configPage15.idleAdvClGainTuneMeasure = 4U;
    currentStatus.coolant = 85;
    configPage15.idleAdvClGainAutotuneRequest = 1U;
    idleAdvanceGainTuneAttempts = 0U;
    idleAdvanceGainTuneLastRequest = false;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
    (void)correctionIdleAdvance(10); //Engage and count the first stable tick.

    for(uint8_t tick = 0U; tick < 10U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }
    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RELAY, idleAdvanceGainAutotuneDiag().state);

    //The configured 5 degree relay step is applied around the 10 degree center.
    ++runSecsX10;
    TEST_ASSERT_EQUAL(15, correctionIdleAdvance(10));

    //Two discarded plus four measured half-cycles complete the configured test.
    for(uint8_t phase = 0U; phase < 6U; ++phase) {
        currentStatus.setRpm(((phase % 2U) == 0U) ? 1100U : 900U);
        for(uint8_t tick = 0U; tick < 6U; ++tick) {
            ++runSecsX10;
            (void)correctionIdleAdvance(10);
        }
    }

    //The same plant with a 5 rather than 3 degree relay produces larger gains.
    TEST_ASSERT_EQUAL(101, configPage9.idleAdvClKp);
    TEST_ASSERT_EQUAL(152, configPage9.idleAdvClKd);
    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RESULT_DONE, idleAdvanceGainAutotuneDiag().lastResult);
}

static void test_correctionIdleAdvance_gain_autotune_aborts_without_oscillation(void) {
    setup_correctionIdleAdvance_gain_autotune();

    //A plant that never crosses the hysteresis is not oscillating: the test
    //must give up rather than hold the relay offset forever.
    for(uint8_t tick = 0U; tick < 51U; ++tick) {
        ++runSecsX10;
        (void)correctionIdleAdvance(10);
    }

    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RESULT_NO_OSCILLATION, idleAdvanceGainAutotuneDiag().lastResult);
    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_WAITING, idleAdvanceGainAutotuneDiag().state);
    //The gains and the request are untouched: it will retry at the next settled idle.
    TEST_ASSERT_EQUAL(40, configPage9.idleAdvClKp);
    TEST_ASSERT_EQUAL(60, configPage9.idleAdvClKd);
    TEST_ASSERT_EQUAL(1, configPage15.idleAdvClGainAutotuneRequest);
}

static void test_correctionIdleAdvance_gain_autotune_aborts_on_disengage(void) {
    setup_correctionIdleAdvance_gain_autotune();

    //Opening the throttle mid test poisons the measurement and must abort it.
    currentStatus.TPS = configPage2.idleAdvTPS + 10U;
    (void)correctionIdleAdvance(10);

    TEST_ASSERT_EQUAL(IDLE_ADV_GAINTUNE_RESULT_DISENGAGED, idleAdvanceGainAutotuneDiag().lastResult);
    TEST_ASSERT_EQUAL(40, configPage9.idleAdvClKp);
    TEST_ASSERT_EQUAL(60, configPage9.idleAdvClKd);
    TEST_ASSERT_EQUAL(1, configPage15.idleAdvClGainAutotuneRequest);
}

static void test_correctionIdleAdvance(void) {
    RUN_TEST_P(test_correctionIdleAdvance_tps_lookup_nodelay);
    RUN_TEST_P(test_correctionIdleAdvance_ctps_lookup_nodelay);
    RUN_TEST_P(test_correctionIdleAdvance_inactive_notrunning);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_modeoff);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_rpmtoohigh);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_vsslimit);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_tpslimit);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_ctpsinactive);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_rundelay);
    RUN_TEST_P(test_correctionIdleAdvance_delay);
    RUN_TEST_P(test_correctionIdleAdvance_delay_updates_once_per_tick);
    RUN_TEST_P(test_correctionIdleAdvance_delay_resets_after_engine_stop);
    RUN_TEST_P(test_correctionIdleAdvance_iac_start_threshold_is_latched);
    RUN_TEST_P(test_correctionIdleAdvance_low_target_does_not_underflow);
    RUN_TEST_P(test_correctionIdleAdvance_delta_does_not_wrap_above_2550rpm);
    RUN_TEST_P(test_correctionIdleAdvance_added_mode_clamps_overflow);
    RUN_TEST_P(test_idleAdvanceClosedLoop_pd_target);
    RUN_TEST_P(test_idleAdvanceClosedLoop_deadband_has_no_step);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_responds_without_output_slew);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_trim_freezes_in_deadband);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_trim_yields_to_iac);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_trim_is_bounded);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_learn_folds_trim_into_center);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_learn_respects_authority);
    RUN_TEST_P(test_correctionIdleAdvance_closed_loop_learn_requires_warm_engine);
    RUN_TEST_P(test_correctionIdleAdvance_gain_autotune_measures_relay_and_writes_gains);
    RUN_TEST_P(test_correctionIdleAdvance_gain_autotune_uses_configured_setup);
    RUN_TEST_P(test_correctionIdleAdvance_gain_autotune_aborts_without_oscillation);
    RUN_TEST_P(test_correctionIdleAdvance_gain_autotune_aborts_on_disengage);
}

extern int8_t correctionSoftRevLimit(int8_t advance);

static void setup_correctionSoftRevLimit(void) {
    initialiseCorrections();

    configPage6.engineProtectType = PROTECT_CUT_IGN;
    configPage4.SoftRevLim = 50;
    configPage4.SoftLimMax = 1;
    configPage4.SoftLimRetard = 5;
    configPage2.SoftLimitMode = SOFT_LIMIT_FIXED;

    currentStatus.setRpm( (configPage4.SoftRevLim + 1U)*100U);
    softLimitTime = 0;

    BIT_CLEAR(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
}

static void assert_correctionSoftRevLimit(int8_t advance) {
    configPage2.SoftLimitMode = SOFT_LIMIT_FIXED;
    TEST_ASSERT_EQUAL(configPage4.SoftLimRetard, correctionSoftRevLimit(advance));
    TEST_ASSERT_TRUE(currentStatus.softLimitActive);

    currentStatus.softLimitActive = false;
    configPage2.SoftLimitMode = SOFT_LIMIT_RELATIVE;
    TEST_ASSERT_EQUAL(advance-configPage4.SoftLimRetard, correctionSoftRevLimit(advance));
    TEST_ASSERT_TRUE(currentStatus.softLimitActive);
}

static void test_correctionSoftRevLimit_modes(void) {
    setup_correctionSoftRevLimit();

    assert_correctionSoftRevLimit(8);
    assert_correctionSoftRevLimit(-3);
}

static void test_correctionSoftRevLimit_inactive_protecttype(void) {
    setup_correctionSoftRevLimit();

    configPage6.engineProtectType = PROTECT_CUT_OFF;
    currentStatus.softLimitActive = true;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_FALSE(currentStatus.softLimitActive);

    configPage6.engineProtectType = PROTECT_CUT_FUEL;
    currentStatus.softLimitActive = true;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_FALSE(currentStatus.softLimitActive);
}

static void test_correctionSoftRevLimit_inactive_rpmtoohigh(void) {
    setup_correctionSoftRevLimit();
    assert_correctionSoftRevLimit(8);

    currentStatus.setRpm( (configPage4.SoftRevLim - 1U)*100U);
    currentStatus.softLimitActive = true;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_FALSE(currentStatus.softLimitActive);
}

static void test_correctionSoftRevLimit_timeout(void) {
    setup_correctionSoftRevLimit();

    configPage4.SoftLimMax = 3;
    configPage2.SoftLimitMode = SOFT_LIMIT_RELATIVE;
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_10HZ);
    TEST_ASSERT_EQUAL(8-configPage4.SoftLimRetard, correctionSoftRevLimit(8));
    TEST_ASSERT_EQUAL(-5-configPage4.SoftLimRetard, correctionSoftRevLimit(-5));
    TEST_ASSERT_EQUAL(23-configPage4.SoftLimRetard, correctionSoftRevLimit(23));
    TEST_ASSERT_EQUAL(-21, correctionSoftRevLimit(-21));
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_EQUAL(0, correctionSoftRevLimit(0));
}

static void test_correctionSoftRevLimit(void) {
    RUN_TEST_P(test_correctionSoftRevLimit_modes);
    RUN_TEST_P(test_correctionSoftRevLimit_inactive_protecttype);
    RUN_TEST_P(test_correctionSoftRevLimit_inactive_rpmtoohigh);
    RUN_TEST_P(test_correctionSoftRevLimit_timeout);
}

extern int8_t correctionNitrous(int8_t advance);

static void test_correctionNitrous_disabled(void) {
    configPage10.n2o_enable = 0;
    TEST_ASSERT_EQUAL(13, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-13, correctionNitrous(-13));
}

static void test_correctionNitrous_stage1(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 5;
    configPage10.n2o_stage2_retard = 0;
    
    currentStatus.nitrous_status = NITROUS_STAGE1;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
    
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
}

static void test_correctionNitrous_stage2(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 0;
    configPage10.n2o_stage2_retard = 5;
    
    currentStatus.nitrous_status = NITROUS_STAGE2;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
    
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
}

static void test_correctionNitrous_stageboth(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 3;
    configPage10.n2o_stage2_retard = 5;
      
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(5, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-21, correctionNitrous(-13));
}

static void test_correctionNitrous(void) {
    RUN_TEST_P(test_correctionNitrous_disabled);
    RUN_TEST_P(test_correctionNitrous_stage1);
    RUN_TEST_P(test_correctionNitrous_stage2);
    RUN_TEST_P(test_correctionNitrous_stageboth);
}

extern int8_t correctionSoftLaunch(int8_t advance);

static void setup_correctionSoftLaunch(void) {
    configPage6.launchEnabled = 1;
    configPage6.flatSArm = 20;
    configPage6.lnchSoftLim = 40;
    configPage10.lnchCtrlTPS = 80;
    configPage10.lnchCtrlVss = 50;
    configPage2.vssMode = 2;
    
    currentStatus.clutchTrigger = 1;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) - 100;
    currentStatus.setRpm( ((configPage6.lnchSoftLim) * 100) + 100);
    currentStatus.TPS = configPage10.lnchCtrlTPS + 1;
    currentStatus.vss = 30;
}

static void test_correctionSoftLaunch_on(void) {
    setup_correctionSoftLaunch();

    configPage6.lnchRetard = -3;
    TEST_ASSERT_EQUAL(configPage6.lnchRetard, correctionSoftLaunch(-8));
    TEST_ASSERT_TRUE(currentStatus.launchingSoft);
    TEST_ASSERT_TRUE(currentStatus.softLaunchActive);

    configPage6.lnchRetard = 3;
    currentStatus.launchingSoft = false;
    currentStatus.softLaunchActive = false;
    TEST_ASSERT_EQUAL(configPage6.lnchRetard, correctionSoftLaunch(8));
    TEST_ASSERT_TRUE(currentStatus.launchingSoft);
    TEST_ASSERT_TRUE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_disabled(void) {
    setup_correctionSoftLaunch();
    configPage6.launchEnabled = 0;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_noclutchtrigger(void) {
    setup_correctionSoftLaunch();
    currentStatus.clutchTrigger = 0;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_clutchrpmlow(void) {
    setup_correctionSoftLaunch();
    currentStatus.clutchEngagedRPM = (configPage6.flatSArm * 100) + 1;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_rpmlimit(void) {
    setup_correctionSoftLaunch();
    currentStatus.setRpm( (configPage6.lnchSoftLim * 100) - 1);
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_tpslow(void) {
    setup_correctionSoftLaunch();
    currentStatus.TPS = configPage10.lnchCtrlTPS - 1;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch_off_vsslimit(void) {
    setup_correctionSoftLaunch();
    currentStatus.vss = 100; //VSS above limit of 80

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_FALSE(currentStatus.softLaunchActive);
}

static void test_correctionSoftLaunch(void) {
    RUN_TEST_P(test_correctionSoftLaunch_on);
    RUN_TEST_P(test_correctionSoftLaunch_off_disabled);
    RUN_TEST_P(test_correctionSoftLaunch_off_noclutchtrigger);
    RUN_TEST_P(test_correctionSoftLaunch_off_clutchrpmlow);
    RUN_TEST_P(test_correctionSoftLaunch_off_rpmlimit);
    RUN_TEST_P(test_correctionSoftLaunch_off_tpslow);
    RUN_TEST_P(test_correctionSoftLaunch_off_vsslimit);
}

extern int8_t correctionSoftFlatShift(int8_t advance);

static void setup_correctionSoftFlatShift(void) {
    configPage6.flatSEnable = 1;
    configPage6.flatSArm = 10;
    configPage6.flatSSoftWin = 10;
    
    currentStatus.clutchTrigger = 1;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) + 500;
    currentStatus.setRpm( currentStatus.clutchEngagedRPM + 600);

    currentStatus.flatShiftSoftCut = false;
}

static void test_correctionSoftFlatShift_on(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;

    TEST_ASSERT_EQUAL(configPage6.flatSRetard, correctionSoftFlatShift(-8));
    TEST_ASSERT_TRUE(currentStatus.flatShiftSoftCut);

    currentStatus.flatShiftSoftCut = false;
    TEST_ASSERT_EQUAL(configPage6.flatSRetard, correctionSoftFlatShift(3));
    TEST_ASSERT_TRUE(currentStatus.flatShiftSoftCut);
}

static void test_correctionSoftFlatShift_off_disabled(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    configPage6.flatSEnable = 0;

    currentStatus.flatShiftSoftCut = true;
    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_FALSE(currentStatus.flatShiftSoftCut);
}

static void test_correctionSoftFlatShift_off_noclutchtrigger(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.clutchTrigger = 0;

    currentStatus.flatShiftSoftCut = true;
    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_FALSE(currentStatus.flatShiftSoftCut);
}

static void test_correctionSoftFlatShift_off_clutchrpmtoolow(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) - 500;

    currentStatus.flatShiftSoftCut = true;
    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_FALSE(currentStatus.flatShiftSoftCut);
}

static void test_correctionSoftFlatShift_off_rpmnotinwindow(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.setRpm( (currentStatus.clutchEngagedRPM - (configPage6.flatSSoftWin * 100) ) - 100);

    currentStatus.flatShiftSoftCut = true;
    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_FALSE(currentStatus.flatShiftSoftCut);
}

static void test_correctionSoftFlatShift(void) {
    RUN_TEST_P(test_correctionSoftFlatShift_on);
    RUN_TEST_P(test_correctionSoftFlatShift_off_disabled);
    RUN_TEST_P(test_correctionSoftFlatShift_off_noclutchtrigger);
    RUN_TEST_P(test_correctionSoftFlatShift_off_clutchrpmtoolow);
    RUN_TEST_P(test_correctionSoftFlatShift_off_rpmnotinwindow);
}

#if 0 // Wait until Noisymime is done with knock implementation
extern int8_t correctionKnock(int8_t advance);

static void setup_correctionKnock(void) {
    configPage10.knock_mode = KNOCK_MODE_DIGITAL;
    configPage10.knock_count = 5U;
    configPage10.knock_firstStep = 3U;
    // knockCounter = configPage10.knock_count + 1;
//   TEST_DATA_P uint8_t startBins[] = { 30, 40, 50, 60, 70, 80 };
//   TEST_DATA_P uint8_t startValues[] = { 30, 25, 20, 15, 10, 5 };
//   populate_2dtable_P(&knockWindowStartTable, startValues, startBins);

//   TEST_DATA_P uint8_t durationBins[] = { 30, 40, 50, 60, 70, 80 };
//   TEST_DATA_P uint8_t durationValues[] = { 30, 25, 20, 15, 10, 5 };
//   populate_2dtable_P(&knockWindowDurationTable, durationValues, durationBins);
}

static void test_correctionKnock_firstStep(void) {
    setup_correctionKnock();

    TEST_ASSERT_EQUAL(-11, correctionKnock(-8));
}

static void test_correctionKnock_disabled_modeoff(void) {
    setup_correctionKnock();
    configPage10.knock_mode = KNOCK_MODE_OFF;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}

static void test_correctionKnock_disabled_counttoolow(void) {
    setup_correctionKnock();
    knockCounter = configPage10.knock_count - 1;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}

static void test_correctionKnock_disabled_knockactive(void) {
    setup_correctionKnock();
    currentStatus.knockActive = true;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}
#endif

static void test_correctionKnock(void) {
}

extern table2D_u8_u8_6 dwellVCorrectionTable; ///< 6 bin dwell voltage correction (2D)

static void setup_correctionsDwell(void) {
    initialiseCorrections();
    BIT_SET(currentStatus.LOOP_TIMER, BIT_TIMER_4HZ);

    configPage4.sparkDur = 10;
    configPage2.perToothIgn = false;
    configPage4.dwellErrCorrect = 0;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.actualDwell = 770;
    currentStatus.battery10 = 95;

    currentStatus.revolutionTime = 666;

    TEST_DATA_P uint8_t bins[] = { 60,  70,  80,  90,  100, 110 };
    TEST_DATA_P uint8_t values[] = { 130, 125, 120, 115, 110, 90 };
    populate_2dtable_P(&dwellVCorrectionTable, values, bins);   
}

extern uint16_t correctDwellClosedLoop(uint16_t computedDwell, uint16_t actualDwell);

static void test_correctDwellClosedLoop_nochange(void) {
    TEST_ASSERT_EQUAL(1000U, correctDwellClosedLoop(1000U, 1000U));
}

static void test_correctDwellClosedLoop_smallError(void) {
    // computed 1000, actual 900 -> error 100 -> returned 1100
    TEST_ASSERT_EQUAL(1100U, correctDwellClosedLoop(1000U, 900U));
}

static void test_correctDwellClosedLoop_largeError(void) {
    // computed 1000, actual 400 -> error 600 > 500 -> doubled to 1200 -> returned 2200
    TEST_ASSERT_EQUAL(2200U, correctDwellClosedLoop(1000U, 400U));
}

static void test_correctDwellClosedLoop_clipComputed(void) {
    // computed value greater than INT16_MAX should be clipped to 32767
    TEST_ASSERT_EQUAL(32767U, correctDwellClosedLoop(40000U, 40000U));
}

static void test_correctDwellClosedLoop_actualGreater(void) {
    // actual value greater than computed value should increase computed value
    TEST_ASSERT_EQUAL(6600U, correctDwellClosedLoop(6600U, 8800U));
}

static void test_correctionsDwell_nopertooth(void) {
    setup_correctionsDwell();

    currentStatus.battery10 = 105;
    configPage2.nCylinders = 8;

    configPage4.sparkMode = IGN_MODE_WASTED;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_SINGLE;
    TEST_ASSERT_EQUAL(74, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_ROTARY;
    configPage10.rotaryType = ROTARY_IGN_RX8;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_ROTARY;
    configPage10.rotaryType = ROTARY_IGN_FC;
    TEST_ASSERT_EQUAL(74, correctionsDwell(800));
}

static void test_correctionsDwell_pertooth(void) {
    setup_correctionsDwell();

    currentStatus.battery10 = 105;
    configPage2.perToothIgn = true;
    configPage4.dwellErrCorrect = 1;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.actualDwell = 200;
    TEST_ASSERT_EQUAL(444, correctionsDwell(800));

    currentStatus.actualDwell = 1400;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));
}

static void test_correctionsDwell_wasted_nopertooth_largerevolutiontime(void) {
    setup_correctionsDwell();

    currentStatus.battery10 = 105;
    currentStatus.revolutionTime = 5000;
    TEST_ASSERT_EQUAL(800, correctionsDwell(800));
}

static void test_correctionsDwell_initialises_current_actualDwell(void) {
    setup_correctionsDwell();

    currentStatus.actualDwell = 0;
    correctionsDwell(777);
    TEST_ASSERT_EQUAL(777, currentStatus.actualDwell);
}

static void test_correctionsDwell_uses_batvcorrection(void) {
    setup_correctionsDwell();

    configPage2.nCylinders = 8;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.battery10 = 105;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    currentStatus.battery10 = 65;
    TEST_ASSERT_EQUAL(337, correctionsDwell(800));
}

static void test_correctionsDwell(void) {
    RUN_TEST_P(test_correctionsDwell_nopertooth);
    RUN_TEST_P(test_correctionsDwell_pertooth);
    RUN_TEST_P(test_correctionsDwell_wasted_nopertooth_largerevolutiontime);
    RUN_TEST_P(test_correctionsDwell_initialises_current_actualDwell);
    RUN_TEST_P(test_correctionsDwell_uses_batvcorrection);
    RUN_TEST_P(test_correctDwellClosedLoop_nochange);
    RUN_TEST_P(test_correctDwellClosedLoop_smallError);
    RUN_TEST_P(test_correctDwellClosedLoop_largeError);
    RUN_TEST_P(test_correctDwellClosedLoop_clipComputed);
    RUN_TEST_P(test_correctDwellClosedLoop_actualGreater);
}

void testIgnCorrections(void) {
    SET_UNITY_FILENAME() {
        RUN_TEST(test_iac_gain_autotune_converts_relay_measurement);
        test_correctionFixedTiming();
        test_correctionCLTadvance();
        test_correctionCrankingFixedTiming();
        test_correctionFlexTiming();
        test_correctionWMITiming();
        test_correctionIATretard();
        test_correctionIdleAdvance();
        test_correctionSoftRevLimit();
        test_correctionNitrous();
        test_correctionSoftLaunch();
        test_correctionSoftFlatShift();
        test_correctionKnock();
        // correctionDFCOignition() is tested in the fueling unit tests, since it is tightly coupled to fuel DFCO
        test_correctionsDwell();
    }
}
