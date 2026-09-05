/** @file
 * @brief Bringing stored settings up to date with the firmware.
 *
 * This used to be a migration ladder: one function per EEPROM data version,
 * each nudging an old tune one step forward, run in sequence so a user who
 * skipped several firmware releases still arrived at a current layout.
 *
 * That ladder is gone. This fork moved every assignable pin out of the spare
 * bits of the feature pages into a page of its own, which is not a change a
 * field-by-field walk can express - the old bits no longer name anything. So
 * the version byte is now only a match/mismatch check: a tune written by this
 * firmware is loaded as-is, and anything else is treated exactly like a blank
 * chip. Retuning is the cost, and it is paid once.
 *
 * What remains here are the default values themselves, which a blank chip
 * needs regardless.
 */
#include "globals.h"
#include "storage.h"
#include "sensors.h"
#include "updates.h"
#include "pages.h"
#include "comms_CAN.h"
#include "scheduler.h"
#include "units.h"
#include "unit_testing.h"
#include "emp_pump.h"
#include "load_source.h"
#include <string.h>

/** @brief Defaults for the closed-loop idle ignition autotunes (all off).
 *
 * The gain autotune request and the spare bits share byte 195 with settings
 * that predate them, so the previously-undefined bits are cleared explicitly:
 * a garbage 1 in the request bit would start a relay test on its own.
 */
TESTABLE_STATIC void setIdleAdvanceClosedLoopLearnDefaults(void) {
    configPage15.idleAdvClLearnAuthority = 0U; //Learning disabled
    configPage15.idleAdvClLearnMinTemp = temperatureAddOffset(70); //degC
    configPage15.idleAdvClGainAutotuneRequest = 0U;
    configPage15.idleAdvClUnused111 = 0U;
}

/** @brief Safe defaults for the closed-loop idle ignition gain relay test. */
TESTABLE_STATIC void setIdleAdvanceGainAutotuneDefaults(void) {
    configPage15.idleAdvClGainTuneStep = 3U;          //degrees either side of center
    configPage15.idleAdvClGainTuneSettleTime = 3U;    //seconds
    configPage15.idleAdvClGainTuneSettleBand = 20U;   //RPM
    configPage15.idleAdvClGainTuneHysteresis = 0U;    //Auto: max(idle deadband, 10 RPM)
    configPage15.idleAdvClGainTuneDiscard = 2U;       //half cycles
    configPage15.idleAdvClGainTuneMeasure = 6U;       //half cycles
    configPage15.idleAdvClGainTuneTimeout = 5U;       //seconds per half cycle
    configPage15.idleAdvClGainTuneRunawayDiv10 = 40U; //400 RPM
    configPage15.idleAdvClGainTuneMinAmplitude = 20U; //RPM
    configPage15.idleAdvClGainTuneMinPeriod = 4U;     //0.4 seconds
    configPage15.idleAdvClGainTuneMaxPeriod = 60U;    //6.0 seconds
    configPage15.idleAdvClGainTuneMaxAttempts = 3U;
}

/** @brief Safe defaults for the closed-loop IAC relay autotune. */
TESTABLE_STATIC void setIacGainAutotuneDefaults(void) {
    configPage15.iacGainAutotuneRequest = 0U;
    configPage15.iacGainAutotuneUnused126 = 0U;
    configPage15.iacGainTuneMinTemp = temperatureAddOffset(70);
    configPage15.iacGainTuneStep = 5U;             //PWM percent or physical stepper steps
    configPage15.iacGainTuneSettleTime = 5U;       //seconds
    configPage15.iacGainTuneSettleBand = 25U;      //RPM
    configPage15.iacGainTuneHysteresis = 0U;       //Auto: 10 RPM
    configPage15.iacGainTuneDiscard = 2U;          //half cycles
    configPage15.iacGainTuneMeasure = 6U;          //half cycles
    configPage15.iacGainTuneTimeout = 10U;         //seconds per half cycle
    configPage15.iacGainTuneRunawayDiv10 = 40U;    //400 RPM
    configPage15.iacGainTuneMinAmplitude = 20U;    //RPM
    configPage15.iacGainTuneMinPeriod = 5U;        //0.5 seconds
    configPage15.iacGainTuneMaxPeriod = 200U;      //20 seconds
    configPage15.iacGainTuneMaxAttempts = 3U;
    configPage15.iacGainTuneMaxTps = 10U;          //5 percent (0.5 percent raw units)
}

/** @brief Safe defaults for the odometer and trip meter. */
TESTABLE_STATIC void setVehicleDistanceDefaults(void) {
    configPage15.vehicleDistanceSource = 0U; //VSS
    configPage15.gpsSpeedAuxChannel = 0U;
    configPage15.vehicleOdometerDeciKm = 0U;
    configPage15.vehicleTripDeciKm = 0U;
}

/** @brief Conservative defaults for sequential fuel-trim autotune.
 *
 * Every channel starts disabled. Channel assignments alternate AFR1/AFR2 so
 * a two-cylinder sequential installation is ready to inspect without enabling
 * a closed loop accidentally. The default objective is cylinder balance and
 * learned maps are stored when the engine stops.
 */
TESTABLE_STATIC void setSequentialTrimAutotuneDefaults(void) {
    constexpr uint8_t DEFAULT_RESISTANCE_PRESET = 4U; //64 s per 1 percent error
    for (uint8_t index = 0U; index < 8U; index++) {
        const uint8_t afr2 = ((index & 1U) != 0U) ? (1U << 2U) : 0U;
        configPage15.seqTrimAutotuneConfig[index] = (uint8_t)((DEFAULT_RESISTANCE_PRESET << 3U) | afr2);
        configPage15.seqTrimAutotuneAuthority[index] = 10U;
    }

    configPage15.seqTrimAutotuneFlags = (1U << 1U); //Balance objective, save at engine stop
    configPage15.seqTrimAutotuneDeadband = 10U;      //1.0 percent lambda
    configPage15.seqTrimAutotuneStableTime = 30U;    //3.0 seconds
    configPage15.seqTrimAutotuneMinClt = temperatureAddOffset(70);
    configPage15.seqTrimAutotuneMinRpmDiv100 = 12U;
    configPage15.seqTrimAutotuneMaxRpmDiv100 = 80U;
    configPage15.seqTrimAutotuneMinLoadDiv2 = 20U;
    configPage15.seqTrimAutotuneMaxLoadDiv2 = 50U;
    configPage15.seqTrimAutotuneSavePeriod = 10U;
}

/** @brief Seed independent AFR1/AFR2 transport and sensor delay maps.
 *
 * Values are stored in 10 ms counts. The generic warm-engine baseline falls
 * with both RPM and load (gas mass flow), but both maps start identical until
 * their individual sensor positions are measured and calibrated.
 */
TESTABLE_STATIC void setAfrDelayTableDefaults(void) {
    constexpr uint8_t DIM = 6U;
    static constexpr uint8_t rpmBins[DIM] = {10U, 15U, 25U, 35U, 50U, 80U};       //RPM / 100
    static constexpr uint8_t delay10ms[DIM][DIM] = {
        {75U, 53U, 35U, 27U, 21U, 16U}, //lowest load bin
        {55U, 40U, 27U, 22U, 18U, 14U},
        {47U, 34U, 24U, 19U, 16U, 13U},
        {42U, 30U, 22U, 18U, 15U, 12U},
        {38U, 28U, 20U, 17U, 14U, 12U},
        {33U, 25U, 18U, 15U, 13U, 11U}, //highest load bin
    };

    uint16_t loadMax = 100U;
    if (configPage2.fuelAlgorithm != LOAD_SOURCE_TPS) {
        loadMax = configPage2.mapMax;
        if (loadMax < 100U) { loadMax = 100U; }
        if (loadMax > 240U) { loadMax = 240U; }
    }
    uint8_t loadBins[DIM];
    for (uint8_t logical = 0U; logical < DIM; logical++) {
        const uint16_t load = 20U + ((loadMax - 20U) * logical) / (DIM - 1U);
        //fuelLoad is MAP/ratio in native units, but TPS is represented as
        //percent * 2. Match the same encoding used by the main fuel tables.
        loadBins[logical] = (configPage2.fuelAlgorithm == LOAD_SOURCE_TPS)
                          ? (uint8_t)(load * 2U)
                          : (uint8_t)(load / 2U);
    }

    for (uint8_t channel = 0U; channel < 2U; channel++) {
        for (uint8_t logical = 0U; logical < DIM; logical++) {
            const uint8_t memory = DIM - 1U - logical;
            afrDelayTables[channel].axisX.axis[memory] = rpmBins[logical];
            afrDelayTables[channel].axisY.axis[memory] = loadBins[logical];
        }
        for (uint8_t loadIndex = 0U; loadIndex < DIM; loadIndex++) {
            const uint8_t memoryRow = DIM - 1U - loadIndex;
            for (uint8_t rpmIndex = 0U; rpmIndex < DIM; rpmIndex++) {
                afrDelayTables[channel].values.values[memoryRow * DIM + rpmIndex] = delay10ms[loadIndex][rpmIndex];
            }
        }
        invalidate_cache(&afrDelayTables[channel].get_value_cache);
    }
}

TESTABLE_STATIC void setAfrDelayConfigDefaults(void) {
    afrDelayConfig.wallWettingOffset10ms = 0;
}

/** @brief Complete EMP defaults for the compact page-15 layout.
 *
 * The global EEPROM data version now owns layout compatibility. EMP no longer
 * consumes a second magic/version/reserved tuple inside the tune page.
 */
TESTABLE_STATIC void setEmpPumpDefaults(void) {
    //Safe default: parameters are ready for inspection, but pump control remains disabled.
    configPage15.empPumpFlags = emp_pump::AFTER_RUN_ENABLED |
                                emp_pump::POWER_HOLD_ENABLED |
                                emp_pump::RUN_DURING_CRANKING |
                                emp_pump::CLOSED_LOOP_ENABLED;
    configPage15.empPumpControllerAddress = 0x96U;
    configPage15.empPumpSourceAddress = 0xA3U;
    configPage15.empPumpStopDebounce100ms = 10U;
    configPage15.empPumpMinimumRunRpm = 1500U;
    configPage15.empPumpMaximumRpm = 6000U;
    configPage15.empPumpAfterRunMinimumRpm = 1800U;
    configPage15.empPumpFailsafeRpm = 3000U;
    configPage15.empPumpRampRpmPerSecond = 2000U;
    configPage15.empPumpAfterRunMaximumSeconds = 180U;
    configPage15.empPumpAfterRunStartTemperature = temperatureAddOffset(95);
    configPage15.empPumpAfterRunStopTemperature = temperatureAddOffset(85);
    configPage15.empPumpBatteryCutoff10 = 115U;
    configPage15.empPumpBatteryResume10 = 120U;

    static constexpr byte temperatureBins[6] = {80U, 110U, 125U, 135U, 145U, 155U};
    static constexpr uint16_t rpmBins[6] = {1500U, 1500U, 2000U, 3000U, 4500U, 6000U};
    for (uint8_t index = 0U; index < 6U; index++) {
        configPage15.empPumpTemperatureBins[index] = temperatureBins[index];
        configPage15.empPumpRpmBins[index] = rpmBins[index];
    }

    configPage15.empPumpManualTestRpm = 2000U;
    configPage15.empPumpManualTestSeconds = 10U;
    configPage15.empPumpStatusTimeoutSeconds = 3U;
    configPage15.empPumpTargetTemperature = temperatureAddOffset(90);
    configPage15.empPumpTemperatureDeadband = 1U;
    configPage15.empPumpProportionalGain = 250U;
    configPage15.empPumpIntegralGain = 12U;
    configPage15.empPumpIntegralLimitRpm = 2000U;
    configPage15.empPumpDerivativeGain = 8U;
    configPage15.empPumpLoadFeedForwardGain = 220U;
    configPage15.empPumpIatReferenceTemperature = temperatureAddOffset(20);
    configPage15.empPumpIatCompensationGain = 18U;
    configPage15.empPumpAirflowFullSpeedKph = 100U;
    configPage15.empPumpAirflowReliefRpm = 600U;
    configPage15.empPumpFanEquivalentSpeedKph = 35U;
    configPage15.empPumpCoolingLimitedDelta = 3U;
    configPage15.empPumpOverloadDelta = 8U;
    configPage15.empPumpOverloadDelaySeconds = 5U;

    static constexpr uint16_t engineRpmBins[4] = {0U, 2000U, 5000U, 9000U};
    static constexpr uint16_t minimumFlowRpmBins[4] = {1500U, 1800U, 2300U, 3000U};
    for (uint8_t index = 0U; index < 4U; index++) {
        configPage15.empPumpEngineRpmBins[index] = engineRpmBins[index];
        configPage15.empPumpMinimumFlowRpmBins[index] = minimumFlowRpmBins[index];
    }

    setVehicleDistanceDefaults();
}

/** @brief Defaults for the closed-loop idle ignition controller.
 *
 * The trim is off by default: with a closed-loop IAC the air path already owns
 * the steady-state offset, and a second integrator on the same measurement is
 * what makes the two loops hunt against each other.
 */
TESTABLE_STATIC void setIdleAdvanceClosedLoopDefaults(void) {
    configPage9.idleAdvClMinAdvance = IGNITION_ADVANCE_LARGE.toRaw(0);
    configPage9.idleAdvClMaxAdvance = IGNITION_ADVANCE_LARGE.toRaw(20);
    configPage9.idleAdvClKp = 40U;  //2.0 degrees per 100 RPM of error
    configPage9.idleAdvClKd = 60U;  //3.0 degrees per 1000 RPM/s

    configPage15.idleAdvClCenter = IGNITION_ADVANCE_LARGE.toRaw(10);
    configPage15.idleAdvClDeadband = 15U;  //RPM
    configPage15.idleAdvClRpmFilter = 50U; //%
    configPage15.idleAdvClTrimRate = 0U;   //Trim disabled
    configPage15.idleAdvClTrimRange = 5U;  //degrees
    configPage15.idleAdvClTrimRequiresIacLimit = 1U;
    setIdleAdvanceClosedLoopLearnDefaults();
    setIdleAdvanceGainAutotuneDefaults();
}

/** @brief The layout this firmware reads and writes.
 *
 * Bump it whenever the stored layout changes in a way that would make an
 * existing tune mean something different. There is no migration path: the
 * bump is what tells an older tune apart, and an older tune is discarded.
 */
#define CURRENT_DATA_VERSION 40

/** @brief Put every page into its as-shipped state. */
static void setTuneToDefaults(void)
{
    setTuneToEmpty();

    configPage9.true_address = 0x200;
    configPage4.FILTER_FLEX = FILTER_FLEX_DEFAULT;
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_OFF;

    setIdleAdvanceClosedLoopDefaults();
    setIacGainAutotuneDefaults();
    setEmpPumpDefaults();
    setVehicleDistanceDefaults();
    setSequentialTrimAutotuneDefaults();
    setAfrDelayTableDefaults();
    setAfrDelayConfigDefaults();

    //Every pin left at zero, which each consumer reads as "keep the board default".
    memset(&configPins, 0, sizeof(configPins));
}

void doUpdates(void)
{
    if(loadEEPROMVersion() != CURRENT_DATA_VERSION)
    {
        setTuneToDefaults();
        saveAllPages();
        saveEEPROMVersion(CURRENT_DATA_VERSION);
    }
}
