#include <unity.h>
#include "globals.h"
#include "init.h"
#include "../test_utils.h"
#include "storage.h"
#include "resetControl.h"

void prepareForInitialiseAll(uint8_t boardId);

#define UNKNOWN_PIN 0xFF

#if !defined(NOT_A_PIN)
#define NOT_A_PIN 0
#endif

uint8_t getPinMode(uint8_t pin)
{
  auto bit = digitalPinToBitMask(pin);
  auto port = digitalPinToPort(pin);

  // I don't see an option for mega to return this, but whatever...
  if (NOT_A_PIN == port) return UNKNOWN_PIN;

  // Is there a bit we can check?
  if (0 == bit) return UNKNOWN_PIN;

  // Is there only a single bit set?
  if (bit & (bit - 1)) return UNKNOWN_PIN;

  auto reg = portModeRegister(port);
  auto out = portOutputRegister(port);

  if (*reg & bit)
    return OUTPUT;
  else if (*out & bit)
    return INPUT_PULLUP;
  else
    return INPUT;
}

void test_initialisation_complete(void)
{
  prepareForInitialiseAll(3);
  initialiseAll(); //Run the main initialise function
  TEST_ASSERT_EQUAL(true, currentStatus.initialisationComplete);
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V03(void)
{
#if defined(STM32_CORE_VERSION_MAJOR)
  TEST_IGNORE_MESSAGE("Doesn't work on STM32");
#else
  prepareForInitialiseAll(2);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), "Coil1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), "Coil2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), "Coil3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), "Coil4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), "Injector 1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), "Injector 2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), "Injector 3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), "Injector 4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), "Tacho Out");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), "Fuel Pump");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), "Fan");
#endif
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V04(void)
{
#if defined(STM32_CORE_VERSION_MAJOR)
  TEST_IGNORE_MESSAGE("Doesn't work on STM32");
#else
  prepareForInitialiseAll(3);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), "Coil1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), "Coil2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), "Coil3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), "Coil4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), "Injector 1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), "Injector 2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), "Injector 3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), "Injector 4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), "Tacho Out");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), "Fuel Pump");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), "Fan");
  /*
  if(isIdlePWM) 
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle1), "Idle 1");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle2), "Idle 2");
  }
  else if (isIdleStepper)
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperDir), "Stepper Dir");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperStep), "Stepper Step");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperEnable), "Stepper Enable");
  }
  
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), "Fan");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinBoost), "Boost");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_1), "VVT1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_2), "VVT2");
  */
#endif
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_MX5_8995(void)
{
#if defined(STM32_CORE_VERSION_MAJOR)
  TEST_IGNORE_MESSAGE("Doesn't work on STM32");
#else
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), "Coil1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), "Coil2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), "Coil3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), "Coil4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), "Injector 1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), "Injector 2");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), "Injector 3");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), "Injector 4");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), "Tacho Out");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), "Fuel Pump");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), "Fan");
#endif
}

void test_initialisation_outputs_PWM_idle(void)
{
#if defined(CORE_TEENSY) || defined(STM32_CORE_VERSION_MAJOR) // Test hangs under Teensy 4.1. I suspect the PIT based timer
  TEST_IGNORE_MESSAGE("Doesn't work on STM32 or Teensy");
#else
  prepareForInitialiseAll(3);

  //Force 2 channel PWM idle
  configPage6.iacChannels = 1;
  configPage6.iacAlgorithm = 2;

  initialiseAll(); //Run the main initialise function

  bool isIdlePWM = (configPage6.iacAlgorithm > 0) && ((configPage6.iacAlgorithm <= 3) || (configPage6.iacAlgorithm == 6));

  TEST_ASSERT_TRUE_MESSAGE(isPwmIac(configPage6), "Is PWM Idle");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle1), "Idle 1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle2), "Idle 2");
#endif
}

void test_initialisation_outputs_stepper_idle(void)
{
  prepareForInitialiseAll(9);
  bool isIdleStepper = (configPage6.iacAlgorithm > 3) && (configPage6.iacAlgorithm != 6);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_TRUE_MESSAGE(isIdleStepper, "Is Stepper Idle");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperDir), "Stepper Dir");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperStep), "Stepper Step");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperEnable), "Stepper Enable");
}

void test_initialisation_outputs_boost(void)
{
#if defined(STM32_CORE_VERSION_MAJOR)
  TEST_IGNORE_MESSAGE("Doesn't work on STM32");
#else
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinBoost), "Boost");
#endif
}

void test_initialisation_outputs_VVT(void)
{
#if defined(STM32_CORE_VERSION_MAJOR)
  TEST_IGNORE_MESSAGE("Doesn't work on STM32");
#else
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_1), "VVT1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_2), "VVT2");
#endif
}

void test_initialisation_outputs_reset_control_use_board_default(void)
{
#if !defined(NATIVE_BOARD)
  // Board-specific pin numbers: only the native host still runs setPinMapping()
  // without a platform override rewriting these pins.
  TEST_IGNORE_MESSAGE("Pin-number assertions only hold on the native host");
#else
  prepareForInitialiseAll(9);
  configPage4.resetControlConfig = (byte)ResetControlMode::PreventWhenRunning;
  configPins.pin[PIN_ASSIGN_RESET_CONTROL] = PIN_ASSIGNMENT_BOARD_DEFAULT;
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_NOT_EQUAL(0, pinResetControl); 
  TEST_ASSERT_EQUAL(ResetControlMode::PreventWhenRunning, getResetControlMode());
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(pinResetControl));  
#endif
}

void test_initialisation_outputs_reset_control_override_board_default(void)
{
#if !defined(NATIVE_BOARD)
  // Board-specific pin numbers: only the native host still runs setPinMapping()
  // without a platform override rewriting these pins.
  TEST_IGNORE_MESSAGE("Pin-number assertions only hold on the native host");
#else
  prepareForInitialiseAll(9);
  configPage4.resetControlConfig = (byte)ResetControlMode::PreventWhenRunning;
  configPins.pin[PIN_ASSIGN_RESET_CONTROL] = 45; // Use a different pin
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(45, pinResetControl);  
  TEST_ASSERT_EQUAL(ResetControlMode::PreventWhenRunning, getResetControlMode());
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(pinResetControl));
#endif
}

void test_initialisation_user_pin_override_board_default(void)
{
#if !defined(NATIVE_BOARD)
  // Board-specific pin numbers: only the native host still runs setPinMapping()
  // without a platform override rewriting these pins.
  TEST_IGNORE_MESSAGE("Pin-number assertions only hold on the native host");
#else
  prepareForInitialiseAll(3);
  // We do not test all pins, too many & too fragile. So fingers crossed the 
  // same pattern is used for all.
  configPins.pin[PIN_ASSIGN_TACHO] = 15;
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(15, pinTachOut);  
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(pinTachOut));
#endif
}

// Pin assignments are a full byte each now, so an out-of-range assignment is
// finally expressible - and the defensive code that rejects it is testable.
void test_initialisation_user_pin_not_valid_no_override(void)
{
#if !defined(NATIVE_BOARD)
  // Board-specific pin numbers: only the native host still runs setPinMapping()
  // without a platform override rewriting these pins.
  TEST_IGNORE_MESSAGE("Pin-number assertions only hold on the native host");
#else
  prepareForInitialiseAll(3);
  configPins.pin[PIN_ASSIGN_TACHO] = (uint8_t)BOARD_MAX_IO_PINS;
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(49, pinTachOut);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(pinTachOut));
#endif
}

void test_initialisation_input_user_pin_does_not_override_outputpin(void)
{
#if !defined(NATIVE_BOARD)
  // Board-specific pin numbers: only the native host still runs setPinMapping()
  // without a platform override rewriting these pins.
  TEST_IGNORE_MESSAGE("Pin-number assertions only hold on the native host");
#else
  // A user defineable input pin should not overwrite any output pins.
  prepareForInitialiseAll(3);
  configPins.pin[PIN_ASSIGN_LAUNCH] = 49; // 49 is the default tacho output
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(49, pinTachOut);  
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(pinTachOut));
#endif
}

void testInitialisation()
{
  SET_UNITY_FILENAME() {

  RUN_TEST_P(test_initialisation_complete);
  RUN_TEST_P(test_initialisation_outputs_V03);
  RUN_TEST_P(test_initialisation_outputs_V04);
  RUN_TEST_P(test_initialisation_outputs_MX5_8995);
  RUN_TEST_P(test_initialisation_outputs_PWM_idle);
  RUN_TEST_P(test_initialisation_outputs_boost);
  RUN_TEST_P(test_initialisation_outputs_VVT);
  RUN_TEST_P(test_initialisation_outputs_reset_control_use_board_default);
  RUN_TEST_P(test_initialisation_outputs_reset_control_override_board_default);
  RUN_TEST_P(test_initialisation_user_pin_override_board_default);
  RUN_TEST_P(test_initialisation_user_pin_not_valid_no_override);
  RUN_TEST_P(test_initialisation_input_user_pin_does_not_override_outputpin);
  }
}