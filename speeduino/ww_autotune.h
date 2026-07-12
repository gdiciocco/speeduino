/** @file ww_autotune.h
 * @brief Wall wetting (X-Tau) table autotune from wideband AFR feedback.
 *
 * Learns the wall wetting "added to wall" (X) and "removed from wall" (Y)
 * coefficient tables by observing the de-lagged AFR error following throttle
 * tip-in transients. See ww_autotune.cpp for the algorithm description.
 *
 * Enabled when configPage2.aeMode == AE_MODE_WALL_WETTING and
 * configPage2.wallWettingFuel > 0. The wallWettingFuel value doubles as the
 * learning authority: the maximum number of table counts a single cell may be
 * moved away from its value at boot, per drive cycle.
 */
#ifndef WW_AUTOTUNE_H
#define WW_AUTOTUNE_H

#include <stdint.h>

/** @brief Initialise the autotune state. Call once at startup, after the tune has been loaded.
 *
 * Seeds physically plausible baseline values into the wall wetting tables if
 * they are empty/invalid (fresh page, never tuned).
 */
void wwAutotuneInit(void);

/** @brief Run one autotune step. Call at 30Hz (BIT_TIMER_30HZ), after corrections have run. */
void wwAutotuneUpdate(void);

/** @brief Seed baseline wall wetting tables derived from the engine parameters.
 *
 * @param force When false, tables are only seeded if their current content is
 * empty or degenerate (axis not strictly increasing). When true, always seeds.
 * @return true if the tables were (re)seeded
 */
bool wwSeedBaselineTables(bool force);

/** @brief Number of transient events that resulted in a table adjustment (diagnostics) */
uint16_t wwAutotuneLearnedEventCount(void);

#endif
