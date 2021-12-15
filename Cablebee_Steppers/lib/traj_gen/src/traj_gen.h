#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

typedef enum
{
    STOP,
    ACCEL,
    RUN,
    DECEL,
} Movement_FSMTypeDef;

typedef struct
{
    // the speed that the motor will start moving from (deg*s^-1)
    float current_speed;
    // the max speed of the movement, this may or not be used (deg*s^-1)
    float max_speed;
    // the speed to end at (deg*s^-1) (TODO I think we assume it is zero)
    float final_speed;
    // (deg*s^-2)
    float acceleration;
    // (deg*s^-2)
    float deceleration;
    // the number of steps we want to move
    float steps;
    // This is dependent on the motor, include microstepping
    float degreesperstep;
    // (Hz)
    float counter_freq;
} Speed_Profile_ParamsTypeDef;

typedef struct
{
    // the total number of steps to move
    float steps;
    // the number of steps to be accelerating for to reach the desired velocity
    float accel_steps;
    // the number of steps to decelerate (right now, to zero)
    float decel_steps;
    // the first delay, (TODO) I think this may need work starting from arbitrary speeds
    float starting_count;
} Trapezoidal_MoveTypeDef;

void generate_trap_profile(Speed_Profile_ParamsTypeDef params, Trapezoidal_MoveTypeDef *out);

#endif // TRAJ_GEN_H
