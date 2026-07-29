#include "../test_utils.h"
#include "emp_pump.h"

using namespace emp_pump;

static Config makeConfig()
{
    Config config = {};
    config.flags = ENABLED | AFTER_RUN_ENABLED | POWER_HOLD_ENABLED | RUN_DURING_CRANKING;
    config.controllerAddress = 0x96U;
    config.sourceAddress = 0xA3U;
    config.stopDebounce100ms = 0U;
    config.minimumRunRpm = 1500U;
    config.maximumRpm = 6000U;
    config.afterRunMinimumRpm = 1800U;
    config.failsafeRpm = 3000U;
    config.rampRpmPerSecond = 0U;
    config.afterRunMaximumSeconds = 180U;
    config.afterRunStartTemperature = 95;
    config.afterRunStopTemperature = 85;
    config.batteryCutoff10 = 115U;
    config.batteryResume10 = 120U;
    const int16_t temperatures[6] = {40, 70, 85, 95, 105, 115};
    const uint16_t rpms[6] = {1500U, 1500U, 2000U, 3000U, 4500U, 6000U};
    for (uint8_t index = 0U; index < 6U; index++)
    {
        config.temperatureBins[index] = temperatures[index];
        config.rpmBins[index] = rpms[index];
    }
    config.manualTestRpm = 2000U;
    config.manualTestSeconds = 10U;
    config.statusTimeoutSeconds = 3U;
    config.targetTemperature = 90;
    config.temperatureDeadband = 1U;
    config.proportionalGain = 250U;
    config.integralGain = 12U;
    config.integralLimitRpm = 2000U;
    config.derivativeGain = 8U;
    config.loadFeedForwardGain = 220U;
    config.iatReferenceTemperature = 20;
    config.iatCompensationGain = 18U;
    config.airflowFullSpeedKph = 100U;
    config.airflowReliefRpm = 600U;
    config.fanEquivalentSpeedKph = 35U;
    config.coolingLimitedDelta = 3U;
    config.overloadDelta = 8U;
    config.overloadDelaySeconds = 5U;
    const uint16_t engineRpmBins[4] = {0U, 2000U, 5000U, 9000U};
    const uint16_t minimumFlowBins[4] = {1500U, 1800U, 2300U, 3000U};
    for (uint8_t index = 0U; index < 4U; index++)
    {
        config.engineRpmBins[index] = engineRpmBins[index];
        config.minimumFlowRpmBins[index] = minimumFlowBins[index];
    }
    config.valid = true;
    return config;
}

static Inputs makeInputs(int16_t coolant, bool running)
{
    Inputs inputs = {};
    inputs.coolant = coolant;
    inputs.intakeAirTemperature = 20;
    inputs.engineRpm = running ? 3000U : 0U;
    inputs.manifoldPressure = 60U;
    inputs.vehicleSpeedKph = 0U;
    inputs.battery10 = 125U;
    inputs.coolantValid = true;
    inputs.intakeAirTemperatureValid = true;
    inputs.vehicleSpeedValid = true;
    inputs.fanOn = false;
    inputs.airflowAtMaximumCapacity = false;
    inputs.engineRunning = running;
    inputs.engineCranking = false;
    return inputs;
}

static void test_builds_extended_rpm_command_at_two_hertz()
{
    Config config = makeConfig();
    Inputs inputs = makeInputs(95, true);
    reset(0U);
    update(100U, config, inputs);

    Command command = {};
    TEST_ASSERT_TRUE(takeCommand(100U, config, command));
    TEST_ASSERT_EQUAL_HEX32(0x18EF96A3UL, command.id);
    TEST_ASSERT_EQUAL_HEX8(0xF5U, command.data[0]); //Power Hold is armed before a hot engine can be switched off
    TEST_ASSERT_EQUAL_HEX8(0x70U, command.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x17U, command.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, command.data[3]);
    TEST_ASSERT_EQUAL_UINT16(3000U, getTargetRpm());
    TEST_ASSERT_FALSE(takeCommand(599U, config, command));
    TEST_ASSERT_TRUE(takeCommand(600U, config, command));
}

static void test_after_run_uses_power_hold_and_explicitly_releases_it()
{
    Config config = makeConfig();
    Inputs inputs = makeInputs(100, true);
    Command command = {};

    reset(0U);
    update(100U, config, inputs);
    TEST_ASSERT_TRUE(takeCommand(100U, config, command));

    inputs.engineRunning = false;
    update(200U, config, inputs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(State::AfterRun), static_cast<uint8_t>(getState()));
    TEST_ASSERT_TRUE(takeCommand(200U, config, command));
    TEST_ASSERT_EQUAL_HEX8(0xF5U, command.data[0]);
    TEST_ASSERT_TRUE(getAfterRunRemainingSeconds(200U) > 0U);

    inputs.coolant = 85;
    update(300U, config, inputs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(State::Stopped), static_cast<uint8_t>(getState()));
    TEST_ASSERT_TRUE(takeCommand(300U, config, command));
    TEST_ASSERT_EQUAL_HEX8(0xF0U, command.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, command.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, command.data[2]);
}

static void test_invalid_coolant_uses_failsafe_while_engine_runs()
{
    Config config = makeConfig();
    Inputs inputs = makeInputs(95, true);
    inputs.coolantValid = false;

    reset(0U);
    update(100U, config, inputs);
    TEST_ASSERT_EQUAL_UINT16(config.failsafeRpm, getTargetRpm());
    TEST_ASSERT_BITS_HIGH(FAULT_CLT_INVALID, getFaults(100U, config));
}

static void test_decodes_status_two_and_tracks_capability()
{
    Config config = makeConfig();
    CanFrame frame = {};
    frame.id = 0x18FF2396UL;
    frame.extended = true;
    frame.len = 8U;
    frame.data[0] = 0x49U; //Forward, controller status 2, command source 1
    frame.data[1] = 0x70U;
    frame.data[2] = 0x17U; //6000 raw = 3000 RPM
    frame.data[3] = 0x20U;
    frame.data[4] = 0x22U;
    frame.data[5] = 0xD0U;
    frame.data[6] = 0x07U; //2000 raw = 1000 W
    frame.data[7] = 0x09U; //Service 1, operation 2

    reset(0U);
    TEST_ASSERT_TRUE(handleFrame(frame, 250U, config));
    TEST_ASSERT_EQUAL_UINT16(3000U, getActualRpm());
    TEST_ASSERT_EQUAL_UINT16(1000U, getPowerWatts());
    TEST_ASSERT_BITS_HIGH(CAP_STATUS_2, getCapabilities());
    TEST_ASSERT_EQUAL_HEX8(0x95U, getStatusSummary());
    TEST_ASSERT_BITS_HIGH(FAULT_SERVICE_REQUIRED | FAULT_NOT_OPERABLE, getFaults(250U, config));
    TEST_ASSERT_EQUAL_UINT16(0U, getMainStatusAgeMs(250U));
}

static void test_battery_cutoff_stops_after_run_without_ending_timer()
{
    Config config = makeConfig();
    Inputs inputs = makeInputs(100, true);

    reset(0U);
    update(100U, config, inputs);
    inputs.engineRunning = false;
    inputs.battery10 = 110U;
    update(200U, config, inputs);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(State::AfterRun), static_cast<uint8_t>(getState()));
    TEST_ASSERT_EQUAL_UINT16(0U, getTargetRpm());
    TEST_ASSERT_BITS_HIGH(FAULT_BATTERY_LOW, getFaults(200U, config));

    inputs.battery10 = 119U;
    update(300U, config, inputs);
    TEST_ASSERT_EQUAL_UINT16(0U, getTargetRpm());
    inputs.battery10 = 120U;
    update(400U, config, inputs);
    TEST_ASSERT_TRUE(getTargetRpm() >= config.afterRunMinimumRpm);
}

static void test_decodes_external_temperature_from_bytes_two_and_three()
{
    Config config = makeConfig();
    CanFrame frame = {};
    frame.id = 0x18FF4396UL;
    frame.extended = true;
    frame.len = 8U;
    frame.data[0] = 0xFFU;
    frame.data[1] = 0xE0U;
    frame.data[2] = 0x2CU; //0x2CE0 = 86 C with protocol scale/offset

    reset(0U);
    TEST_ASSERT_TRUE(handleFrame(frame, 100U, config));
    TEST_ASSERT_EQUAL_HEX16(0x2CE0U, getExternalTemperatureRaw());
    TEST_ASSERT_BITS_HIGH(CAP_EXTERNAL_TEMPERATURE, getCapabilities());
}

static void test_closed_loop_keeps_safe_minimum_during_warmup()
{
    Config config = makeConfig();
    config.flags |= CLOSED_LOOP_ENABLED;
    Inputs inputs = makeInputs(60, true);
    inputs.engineRpm = 5000U;
    inputs.manifoldPressure = 50U;
    inputs.vehicleSpeedKph = 100U;

    reset(0U);
    update(100U, config, inputs);

    TEST_ASSERT_EQUAL_UINT16(2300U, getMinimumFlowRpm());
    TEST_ASSERT_EQUAL_UINT16(2300U, getTargetRpm());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ThermalState::Warmup),
                            static_cast<uint8_t>(getThermalState()));
}

static void test_closed_loop_feed_forward_uses_iat_and_vehicle_airflow()
{
    Config config = makeConfig();
    config.flags |= CLOSED_LOOP_ENABLED;
    Inputs inputs = makeInputs(90, true);
    inputs.engineRpm = 4000U;
    inputs.manifoldPressure = 100U;
    inputs.intakeAirTemperature = 40;
    inputs.vehicleSpeedKph = 0U;

    reset(0U);
    update(100U, config, inputs);
    const uint16_t hotStationaryRpm = getTargetRpm();

    inputs.intakeAirTemperature = 20;
    inputs.vehicleSpeedKph = 100U;
    reset(0U);
    update(100U, config, inputs);
    const uint16_t coolMovingRpm = getTargetRpm();

    TEST_ASSERT_TRUE(hotStationaryRpm > coolMovingRpm);
    TEST_ASSERT_EQUAL_INT16(90, getTargetTemperature());
    TEST_ASSERT_EQUAL_INT8(0, getTemperatureError());
}

static void test_closed_loop_reports_capacity_limit_and_delayed_overload()
{
    Config config = makeConfig();
    config.flags |= CLOSED_LOOP_ENABLED;
    config.maximumRpm = 3000U;
    config.integralLimitRpm = 1500U;
    for (uint8_t index = 0U; index < 4U; index++)
    {
        if (config.minimumFlowRpmBins[index] > config.maximumRpm)
        {
            config.minimumFlowRpmBins[index] = config.maximumRpm;
        }
    }
    Inputs inputs = makeInputs(105, true);
    inputs.engineRpm = 7000U;
    inputs.manifoldPressure = 120U;
    inputs.intakeAirTemperature = 40;
    inputs.airflowAtMaximumCapacity = true;

    reset(0U);
    update(100U, config, inputs);
    TEST_ASSERT_EQUAL_UINT16(config.maximumRpm, getTargetRpm());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ThermalState::CapacityLimited),
                            static_cast<uint8_t>(getThermalState()));
    TEST_ASSERT_BITS_HIGH(FAULT_COOLING_LIMITED, getFaults(100U, config));

    update(5200U, config, inputs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ThermalState::Overload),
                            static_cast<uint8_t>(getThermalState()));
    TEST_ASSERT_BITS_HIGH(FAULT_THERMAL_OVERLOAD, getFaults(5200U, config));
    TEST_ASSERT_TRUE(getSaturationSeconds(5200U) >= 5U);
}

static void test_invalid_iat_disables_only_its_compensation()
{
    Config config = makeConfig();
    config.flags |= CLOSED_LOOP_ENABLED;
    Inputs inputs = makeInputs(90, true);
    inputs.intakeAirTemperatureValid = false;

    reset(0U);
    update(100U, config, inputs);

    TEST_ASSERT_TRUE(getTargetRpm() >= getMinimumFlowRpm());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ThermalState::ClosedLoop),
                            static_cast<uint8_t>(getThermalState()));
    TEST_ASSERT_BITS_HIGH(FAULT_IAT_INVALID, getFaults(100U, config));
}

void testEmpPump(void)
{
    SET_UNITY_FILENAME()
    {
        RUN_TEST_P(test_builds_extended_rpm_command_at_two_hertz);
        RUN_TEST_P(test_after_run_uses_power_hold_and_explicitly_releases_it);
        RUN_TEST_P(test_invalid_coolant_uses_failsafe_while_engine_runs);
        RUN_TEST_P(test_decodes_status_two_and_tracks_capability);
        RUN_TEST_P(test_battery_cutoff_stops_after_run_without_ending_timer);
        RUN_TEST_P(test_decodes_external_temperature_from_bytes_two_and_three);
        RUN_TEST_P(test_closed_loop_keeps_safe_minimum_during_warmup);
        RUN_TEST_P(test_closed_loop_feed_forward_uses_iat_and_vehicle_airflow);
        RUN_TEST_P(test_closed_loop_reports_capacity_limit_and_delayed_overload);
        RUN_TEST_P(test_invalid_iat_disables_only_its_compensation);
    }
}
