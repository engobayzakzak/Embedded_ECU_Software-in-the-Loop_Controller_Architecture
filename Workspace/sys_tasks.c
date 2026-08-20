#include "sys_tasks.h"
#include "fsm.h"
#include <stdio.h>

/**
 * @brief Supervisory System Management Task (50 Hz / 20 ms)
 */
void task_sys_mgmt_step(system_shared_context_t *sys_ctx, rtos_queue_t *cmd_q)
{
    if (sys_ctx == NULL || cmd_q == NULL)
        return;

    /* 1. Process incoming commands from IPC queue */
    cmd_message_t cmd;
    while (rtos_queue_receive(cmd_q, &cmd, 0))
    {
        if (cmd.event != EVENT_NONE)
        {
            fsm_dispatch_event(&sys_ctx->fsm, cmd.event);
        }
        if (cmd.target_setpoint != 0.0f)
        {
            sys_ctx->active_setpoint = cmd.target_setpoint;
        }
    }

    /* 2. Step the state machine */
    fsm_step(&sys_ctx->fsm);
}

/**
 * @brief Real-Time Motion Control Task (1000 Hz / 1 ms)
 */
void task_control_loop_step(system_shared_context_t *sys_ctx, rtos_queue_t *telem_q)
{
    if (sys_ctx == NULL)
        return;

    static float simulated_plant_pos = 0.0f;
    static float simulated_plant_vel = 0.0f;
    const float dt = 0.001f;

    /* If system is not in STATE_ACTIVE, output 0 control effort */
    if (sys_ctx->fsm.current_state != STATE_ACTIVE)
    {
        sys_ctx->latest_telemetry.control_effort = 0.0f;
    }
    else
    {
        /* Discrete Proportional-Derivative Loop */
        const float Kp = 15.0f;
        const float Kd = 0.8f;
        float error = sys_ctx->active_setpoint - simulated_plant_pos;
        float derivative = -simulated_plant_vel;

        float effort = (Kp * error) + (Kd * derivative);

        /* Saturation clamping [-1.0, 1.0] */
        if (effort > 1.0f)
        {
            effort = 1.0f;
            sys_ctx->latest_telemetry.saturation_flag = true;
        }
        else if (effort < -1.0f)
        {
            effort = -1.0f;
            sys_ctx->latest_telemetry.saturation_flag = true;
        }
        else
        {
            sys_ctx->latest_telemetry.saturation_flag = false;
        }

        sys_ctx->latest_telemetry.control_effort = effort;

        /* Physics integration (SIL Plant Digital Twin) */
        simulated_plant_vel += (effort * 10.0f) * dt;
        simulated_plant_pos += simulated_plant_vel * dt;
    }

    sys_ctx->latest_telemetry.timestamp_ms = rtos_get_tick_count_ms();
    sys_ctx->latest_telemetry.actual_position = simulated_plant_pos;
    sys_ctx->latest_telemetry.actual_velocity = simulated_plant_vel;

    /* Publish to telemetry IPC queue */
    if (telem_q != NULL && (sys_ctx->latest_telemetry.timestamp_ms % 10 == 0))
    {
        rtos_queue_send(telem_q, &sys_ctx->latest_telemetry, 0);
    }
}

/**
 * @brief Communications & Telemetry Task (100 Hz / 10 ms)
 */
void task_telemetry_step(system_shared_context_t *sys_ctx, rtos_queue_t *telem_q, rtos_queue_t *cmd_q)
{
    (void)sys_ctx;
    (void)cmd_q;
    control_telemetry_t telem;

    while (rtos_queue_receive(telem_q, &telem, 0))
    {
        /* Encodes CAN / UART streaming frame */
    }
}