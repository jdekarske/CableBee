#include "state.h"

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
                    goalsteps = command_buffer[read_ptr].decel_steps;
                    mode = RUN;
                }
                else
                {
                    n_step++;
                    // calculate when to step next
                    // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
                    nextcount = nextcount - ((2 * nextcount) / (4 * n_step + 1));
                }
            }
            else if (mode == RUN)
            {
                if (step >= goalsteps)
                {
                    mode = DECEL;
                    goalsteps = command_buffer[read_ptr].steps;
                }

                // use the same count as last step
                // nextcount = count;
            }
            else if (mode == DECEL)
            {
                if (step >= goalsteps)
                {
                    mode = STOP;

                    // finished this move so read the next one
                    read_ptr++;
                    nextMove();
                    return;
                }

                n_step--;
                nextcount = nextcount - ((2 * nextcount) / (4 * -n_step + 1));
            }

            count = 0;
        }
    }
}

void nextMove(void)
{
    // move to next command
    // stop if we are out of moves
    if (read_ptr != write_ptr)
    {
        goalsteps = command_buffer[read_ptr].accel_steps;
        nextcount = command_buffer[read_ptr].starting_count;
        count = 0;
        step = 1;
        n_step = 1;
        mode = ACCEL;

        return;
    }
}

void initState(void)
{
    read_ptr = 0;
    write_ptr = 0;

    HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, GPIO_PIN_RESET);

    Speed_Profile_ParamsTypeDef profile_params;
    for (size_t i = 2; i < 10; i++)
    {
        profile_params = (Speed_Profile_ParamsTypeDef){
            .current_speed = 0,
            .max_speed = i * 0.5 * 10.0f * 360.0f / DEGREES_PER_STEP,
            .final_speed = 0,
            .acceleration = ANG_ACCEL,
            .deceleration = ANG_ACCEL,
            .steps = 5 * 360.0f / DEGREES_PER_STEP, // DONT USE ZERO
            .degreesperstep = DEGREES_PER_STEP,
            .counter_freq = STEPPER_TIMER_FREQ,
        };
        generate_trap_profile(profile_params, &command_buffer[write_ptr]);
        write_ptr++; //TODO redundant now, but not when we use the buffer
    }

    nextMove();
}