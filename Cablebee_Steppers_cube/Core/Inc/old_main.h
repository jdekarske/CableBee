#include "main.h"
#include <traj_gen.h>

#define MOTOR_STEP_ANGLE 1.8f // deg
#define MICROSTEPS 8
#define DEGREES_PER_STEP (MOTOR_STEP_ANGLE / MICROSTEPS)
#define CLOCK_FREQ 72000000                       // Hz
#define STEPPER_TIMER_FREQ 1000000                // Hz
#define MIN_STEP_PERIOD 80                        // us
#define ANG_ACCEL 500000                          // deg/s^2
#define STEP_ACCEL (ANG_ACCEL * MOTOR_STEP_ANGLE) // stp/s^2

// speed control params
static Movement_FSMTypeDef mode;
static float count;     // init for the count counter
static float lastcount; // c_i-1
static float nextcount; // c_i
static int32_t step;    // the current step we are on (from 0)
static int32_t n_step;  // this one is decremented for deceleration
static int32_t goalsteps;
static Trapezoidal_MoveTypeDef current_move;

int stepcommands[10];

int error = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (mode != STOP)
    {
        count++;
        if (count >= nextcount)
        {
            // do a step
            HAL_GPIO_WritePin(XSTP_GPIO_Port, XSTP_Pin, GPIO_PIN_SET);
            // at 72Mhz 8 NOPs= 112ns > 100ns min step.
            // okay it even works with none... is HAL GPIO really that slow? I wish I could find my oscilloscope.
            // for (size_t i = 0; i < 8; i++)
            // {
            //   __ASM volatile("NOP");
            // }
            HAL_GPIO_WritePin(XSTP_GPIO_Port, XSTP_Pin, GPIO_PIN_RESET);

            step++;

            // change our goal if necessary
            if (mode == ACCEL)
            {
                if (step >= goalsteps)
                {
                    goalsteps = current_move.decel_steps;
                    mode = RUN;
                }

                n_step++;

                // calculate when to step next
                // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
                nextcount = nextcount - ((2 * nextcount) / (4 * n_step + 1));
            }
            else if (mode == RUN)
            {
                if (step >= goalsteps)
                {
                    mode = DECEL;
                    goalsteps = current_move.steps;
                }

                // use the same count as last step
                nextcount = count;
            }
            else if (mode == DECEL)
            {
                if (step >= goalsteps)
                {
                    mode = STOP;
                }

                n_step--;
                nextcount = nextcount - ((2 * nextcount) / (4 * -n_step + 1));
            }

            count = 0;
        }
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

int old_main(void)
{

    HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, GPIO_PIN_RESET);

    Speed_Profile_ParamsTypeDef profile_params = {
        .current_speed = 0,
        .max_speed = 15000,
        .final_speed = 0,
        .acceleration = ANG_ACCEL,
        .deceleration = ANG_ACCEL,
        .steps = 10 * 360 / DEGREES_PER_STEP,
        .degreesperstep = DEGREES_PER_STEP,
        .counter_freq = STEPPER_TIMER_FREQ,
    };

    generate_trap_profile(profile_params, &current_move);

    goalsteps = current_move.accel_steps;
    lastcount = current_move.starting_count;
    count = 0;
    nextcount = lastcount;
    step = 1;
    n_step = 1;

    HAL_Delay(1000);
    mode = ACCEL; // this should probably be more generic, then let the planner figure it out.

    while (1)
    {
    }
}