#include "state.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // loop through channels
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++)
    {
        if (allchannels[channel].mode != STOP)
        {
            allchannels[channel].count++;
            if (allchannels[channel].count >= allchannels[channel].nextcount)
            {
                // do a step
                HAL_GPIO_WritePin(allchannels[channel].STP_GPIOx, allchannels[channel].STP_GPIO_Pin, GPIO_PIN_SET);
                // at 72Mhz 8 NOPs= 112ns > 100ns min step.
                // okay it even works with none... is HAL GPIO really that slow? I wish I could find my oscilloscope.
                // for (size_t i = 0; i < 8; i++)
                // {
                //   __ASM volatile("NOP");
                // }
                HAL_GPIO_WritePin(allchannels[channel].STP_GPIOx, allchannels[channel].STP_GPIO_Pin, GPIO_PIN_RESET);

                allchannels[channel].step++;

                // change our goal if necessary
                if (allchannels[channel].mode == ACCEL)
                {
                    if (allchannels[channel].step >= allchannels[channel].goalsteps)
                    {
                        allchannels[channel].goalsteps = command_buffer[read_ptr][channel].decel_steps;
                        allchannels[channel].mode = RUN;
                    }
                    else
                    {
                        allchannels[channel].n_step++;
                        // calculate when to step next
                        // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
                        allchannels[channel].nextcount = allchannels[channel].nextcount - ((2 * allchannels[channel].nextcount) / (4 * allchannels[channel].n_step + 1));
                    }
                }
                else if (allchannels[channel].mode == RUN)
                {
                    if (allchannels[channel].step >= allchannels[channel].goalsteps)
                    {
                        allchannels[channel].mode = DECEL;
                        allchannels[channel].goalsteps = command_buffer[read_ptr][channel].steps;
                    }

                    // use the same count as last step
                    // nextcount = count;
                }
                else if (allchannels[channel].mode == DECEL)
                {
                    if (allchannels[channel].step >= allchannels[channel].goalsteps)
                    {
                        allchannels[channel].mode = STOP;

                        // check if other channels are stopped
                        uint8_t all_stopped = 0;
                        for (size_t i = 0; i < NUM_CHANNELS; i++)
                        {
                            if (allchannels[i].mode)
                            {
                                all_stopped++;
                            }
                        }
                        if (all_stopped == NUM_CHANNELS)
                        {
                            // finished this move so read the next one
                            read_ptr++;
                            nextMove();
                        }
                    }

                    allchannels[channel].n_step--;
                    allchannels[channel].nextcount = allchannels[channel].nextcount - ((2 * allchannels[channel].nextcount) / (4 * -allchannels[channel].n_step + 1));
                }

                allchannels[channel].count = 0;
            }
        }
    }
}

void nextMove()
{
    // stop if we are out of moves
    if (read_ptr != write_ptr)
    {
        for (size_t i = 0; i < NUM_CHANNELS; i++)
        {
            allchannels[i].goalsteps = command_buffer[read_ptr][i].accel_steps;
            allchannels[i].nextcount = command_buffer[read_ptr][i].starting_count;
            allchannels[i].count = 0;
            allchannels[i].step = 1;
            allchannels[i].n_step = 1;
            allchannels[i].mode = ACCEL;
        }
    }
}

void initMotorState(void)
{
    allchannels[0].EN_GPIOx = XEN_GPIO_Port;
    allchannels[0].EN_GPIO_Pin = XEN_Pin;
    allchannels[0].DIR_GPIOx = XDIR_GPIO_Port;
    allchannels[0].DIR_GPIO_Pin = XDIR_Pin;
    allchannels[0].STP_GPIOx = XSTP_GPIO_Port;
    allchannels[0].STP_GPIO_Pin = XSTP_Pin;

    allchannels[1].EN_GPIOx = YEN_GPIO_Port;
    allchannels[1].EN_GPIO_Pin = YEN_Pin;
    allchannels[1].DIR_GPIOx = YDIR_GPIO_Port;
    allchannels[1].DIR_GPIO_Pin = YDIR_Pin;
    allchannels[1].STP_GPIOx = YSTP_GPIO_Port;
    allchannels[1].STP_GPIO_Pin = YSTP_Pin;

    allchannels[2].EN_GPIOx = ZEN_GPIO_Port;
    allchannels[2].EN_GPIO_Pin = ZEN_Pin;
    allchannels[2].DIR_GPIOx = ZDIR_GPIO_Port;
    allchannels[2].DIR_GPIO_Pin = ZDIR_Pin;
    allchannels[2].STP_GPIOx = ZSTP_GPIO_Port;
    allchannels[2].STP_GPIO_Pin = ZSTP_Pin;

    allchannels[3].EN_GPIOx = E0EN_GPIO_Port;
    allchannels[3].EN_GPIO_Pin = E0EN_Pin;
    allchannels[3].DIR_GPIOx = E0DIR_GPIO_Port;
    allchannels[3].DIR_GPIO_Pin = E0DIR_Pin;
    allchannels[3].STP_GPIOx = E0STP_GPIO_Port;
    allchannels[3].STP_GPIO_Pin = E0STP_Pin;

    for (size_t i = 0; i < NUM_CHANNELS; i++)
    {
        allchannels[i].step = 1;
        allchannels[i].n_step = 1;

        // enable all channels
        HAL_GPIO_WritePin(allchannels[i].EN_GPIOx, allchannels[i].EN_GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(allchannels[i].DIR_GPIOx, allchannels[i].DIR_GPIO_Pin, GPIO_PIN_RESET);
    }
}

void initState(void)
{

    read_ptr = 0;
    write_ptr = 0;

    Speed_Profile_ParamsTypeDef profile_params;
    for (size_t i = 2; i < 10; i++)
    {
        for (size_t j = 0; j < NUM_CHANNELS; j++)
        {

            profile_params = (Speed_Profile_ParamsTypeDef){
                .current_speed = 0,
                .max_speed = i * 0.5 * 10.0f * 360.0f / DEGREES_PER_STEP,
                .final_speed = 0,
                .acceleration = ANG_ACCEL,
                .deceleration = ANG_ACCEL,
                .steps = 4 * 360.0f / DEGREES_PER_STEP, // DONT USE ZERO
                .degreesperstep = DEGREES_PER_STEP,
                .counter_freq = STEPPER_TIMER_FREQ,
            };
            generate_trap_profile(profile_params, &command_buffer[write_ptr][j]);
        }
        write_ptr++; //TODO redundant now, but not when we use the buffer
    }

    nextMove();
}