#include "pid_controller.h"
#include <math.h>

void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max)
{
    if (pid == NULL)
        return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integrator_min = out_min;
    pid->integrator_max = out_max;

    float tau = 10.0f * dt;
    pid->d_filter_alpha = dt / (tau + dt);

    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL)
        return;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d_term = 0.0f;
    pid->prev_measurement = 0.0f;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement)
{
    if (pid == NULL)
        return 0.0f;

    /* 1. Error calculation */
    float error = setpoint - measurement;

    /* 2. Proportional term */
    float p_term = pid->Kp * error;

    /* 3. Integral term with Anti-Windup conditional clamping */
    pid->integrator += pid->Ki * error * pid->dt;
    if (pid->integrator > pid->integrator_max)
    {
        pid->integrator = pid->integrator_max;
    }
    else if (pid->integrator < pid->integrator_min)
    {
        pid->integrator = pid->integrator_min;
    }

    /* 4. Derivative on Measurement (Derivative Kick Elimination) + Low-Pass Filter */
    float raw_derivative = -(measurement - pid->prev_measurement) / pid->dt;
    float filtered_d = pid->prev_d_term + pid->d_filter_alpha * (raw_derivative - pid->prev_d_term);
    float d_term = pid->Kd * filtered_d;

    /* 5. Output sum */
    float output = p_term + pid->integrator + d_term;

    /* 6. Output Saturation Clamping */
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    /* 7. Update history */
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->prev_d_term = filtered_d;

    return output;
}