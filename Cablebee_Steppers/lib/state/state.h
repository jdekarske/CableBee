#ifndef STATE_H
#define STATE_H

#include "main.h"
#include <traj_gen.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "BTTSKR.h"

typedef struct
{
    GPIO_TypeDef *EN_GPIOx;
    uint16_t EN_GPIO_Pin;
    GPIO_TypeDef *DIR_GPIOx;
    uint16_t DIR_GPIO_Pin;
    GPIO_TypeDef *STP_GPIOx;
    uint16_t STP_GPIO_Pin;
    float count;     // init for the count counter
    float nextcount; // c_i
    int32_t step;    // the current step we are on (from 0)
    int32_t n_step;  // this one is decremented for deceleration
    int32_t goalsteps;
    Movement_FSMTypeDef mode;
} Motor_ChannelTypeDef;

#define NUM_CHANNELS 4
Motor_ChannelTypeDef allchannels[NUM_CHANNELS];

Trapezoidal_MoveTypeDef command_buffer[100][NUM_CHANNELS]; //circular buffer 100 commands of 4 channels each
uint8_t read_ptr;
uint8_t write_ptr;

void nextMove();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base);

void stepMotor();

void initMotorState();

void initState();

#endif //STATE_H
