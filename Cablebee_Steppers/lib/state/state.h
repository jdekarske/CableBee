#ifndef STATE_H
#define STATE_H

#include "main.h"
#include <traj_gen.h>
#include "stm32f1xx_hal.h"
#include "BTTSKR.h"

// speed control params, one for each axis
static Movement_FSMTypeDef mode;
static float count;     // init for the count counter
static float lastcount; // c_i-1
static float nextcount; // c_i
static int32_t step;    // the current step we are on (from 0)
static int32_t n_step;  // this one is decremented for deceleration
static int32_t goalsteps;
static Trapezoidal_MoveTypeDef current_move;

Trapezoidal_MoveTypeDef command_buffer[100];
uint8_t read_ptr;
uint8_t write_ptr;

void nextMove(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base);

void stepMotor();

void initState();

#endif //STATE_H
