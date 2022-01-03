#ifndef BTTSKR__H
#define BTTSKR

#include "main.h"

#define AXES 4

#define MOTOR_STEP_ANGLE 1.8f // deg
#define MICROSTEPS 8
#define DEGREES_PER_STEP (MOTOR_STEP_ANGLE / MICROSTEPS)
#define CLOCK_FREQ 72000000                       // Hz
#define STEPPER_TIMER_FREQ 1000000                // Hz
#define MIN_STEP_PERIOD 80                        // us
#define MAX_VEL * 10.0f * 360.0f / DEGREES_PER_STEP // stp/s
#define ANG_ACCEL 2000000                          // deg/s^2
#define STEP_ACCEL (ANG_ACCEL / DEGREES_PER_STEP) // stp/s^2

#endif //BTTSKR_H