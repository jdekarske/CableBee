#include "traj_gen.h"

void generate_trap_profile(Speed_Profile_ParamsTypeDef params, Trapezoidal_MoveTypeDef *out)
{
    // the number of steps needed to accelerate to the desired speed
    float max_speed_lim = (params.max_speed * params.max_speed) / (2 * params.acceleration);
    // the number of steps before deceleration starts
    float accel_lim = (params.steps * params.deceleration) / (params.acceleration + params.deceleration);

    out->steps = params.steps;

    if (max_speed_lim < accel_lim)
    {
        // limit the speed
        out->decel_steps = max_speed_lim * params.acceleration / params.deceleration;
        out->accel_steps = max_speed_lim;
    }
    else
    {
        // otherwise slow down early!
        out->decel_steps = params.steps - accel_lim;
        out->accel_steps = out->decel_steps;
    }
}