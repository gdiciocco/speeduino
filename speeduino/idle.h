#ifndef IDLE_H
#define IDLE_H

#include <stdint.h>

extern uint16_t idle_pwm_max_count; //Used for variable PWM frequency

void initialiseIdle(bool forcehoming);
void idleControl(void);
void disableIdle(void);
void idleInterrupt(void);

/** @brief True when a closed loop IAC algorithm has driven its output to either
 * configured limit, meaning the air path has no authority left over idle RPM.
 * The idle ignition closed loop only trims its learned center while this holds,
 * so that two integrators never act on the same measurement at the same time.
 */
bool isIdleClosedLoopAtLimit(void);

#endif
