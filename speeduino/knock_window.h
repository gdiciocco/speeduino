#pragma once

/** @file
 * @brief Optional knock-window GPIO output.
 *
 * When KNOCK_WINDOW_OUTPUT_PIN is defined, a GPIO is driven active for a
 * tune-defined crank angle window after each spark. An external knock
 * acquisition device can use this signal to gate its sampling.
 *
 * The window is attached to the ignition schedule state machine as two extra
 * post-spark states (see @ref ScheduleStatus). All degree/time conversion
 * happens in the main loop; the timer ISRs only load pre-computed compare
 * deltas.
 */

#include "opf_core.h" // Board specific KNOCK_WINDOW_OUTPUT_* selection

#if defined(KNOCK_WINDOW_OUTPUT_PIN)

#include "board_definition.h" // COMPARE_TYPE

struct statuses;

/** @brief true when the output pin was successfully claimed at startup */
extern volatile bool knockWindowOutputEnabled;
/** @brief Spark end to window open delay (timer ticks). 0 = open at spark */
extern volatile COMPARE_TYPE knockWindowDelayCompare;
/** @brief Window open time (timer ticks). 0 = no window scheduled */
extern volatile COMPARE_TYPE knockWindowDurationCompare;

/** @brief Claim & initialise the output pin. Call once during initialiseAll() */
void initialiseKnockWindowOutput(void);

/**
 * @brief Convert the tune knock window tables to timer compare deltas.
 *
 * Must be called from the main loop whenever the ignition angles are
 * recalculated, so the ISRs never perform the conversion themselves.
 */
void updateKnockWindowSchedule(const statuses &current);

/** @brief Open the (shared, refcounted) knock window output. ISR context */
void knockWindowOutputOn(void);

/** @brief Close the (shared, refcounted) knock window output. ISR context */
void knockWindowOutputOff(void);

/** @brief Is a knock window currently requested by the tune? */
static inline bool isKnockWindowScheduled(void)
{
  return knockWindowOutputEnabled && (knockWindowDurationCompare > 0U);
}

#endif // KNOCK_WINDOW_OUTPUT_PIN
