#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Discrete PID Configuration and State Parameters
 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float d_filter_alpha;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
    float integrator;
    float prev_error;
    float prev_d_term;
    float prev_measurement;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void pid_reset(pid_controller_t *pid);
float pid_update(pid_controller_t *pid, float setpoint, float measurement);

#endif /* PID_CONTROLLER_H */