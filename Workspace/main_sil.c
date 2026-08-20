#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "fsm.h"
#include "rtos_port.h"
#include "rtos_ipc_types.h"
#include "can_hal.h"
#include "pid_controller.h"

#define CAN_ID_CMD_RECEIVE 0x110U
#define CAN_ID_TELEM_TRANSMIT 0x120U

/* System State Context */
static system_shared_context_t g_system_ctx;
static pid_controller_t g_pid_inst;

/* Message Queues */
static rtos_queue_t *g_cmd_queue = NULL;
static rtos_queue_t *g_telem_queue = NULL;

/* Plant Physics States */
static float g_plant_pos = 0.0f;
static float g_plant_vel = 0.0f;

/* Task 1: CAN Communication & Telemetry Gateway (100 Hz / 10 ms) */
static void task_can_telemetry_step(void)
{
    can_frame_t rx_frame;

    /* 1. Ingest CAN Bus frames */
    while (can_hal_receive(&rx_frame) == CAN_STATUS_OK)
    {
        if (rx_frame.id == CAN_ID_CMD_RECEIVE && rx_frame.dlc >= 5)
        {
            cmd_message_t cmd;
            cmd.event = (system_event_t)rx_frame.data[0];

            /* Unpack 32-bit float target setpoint from 4 bytes */
            uint32_t raw_float = ((uint32_t)rx_frame.data[1]) |
                                 ((uint32_t)rx_frame.data[2] << 8) |
                                 ((uint32_t)rx_frame.data[3] << 16) |
                                 ((uint32_t)rx_frame.data[4] << 24);
            memcpy(&cmd.target_setpoint, &raw_float, sizeof(float));
            cmd.command_id = 0x01;

            rtos_queue_send(g_cmd_queue, &cmd, 0);
        }
    }

    /* 2. Transmit outgoing telemetry over CAN */
    control_telemetry_t telem;
    while (rtos_queue_receive(g_telem_queue, &telem, 0))
    {
        can_frame_t tx_frame;
        tx_frame.id = CAN_ID_TELEM_TRANSMIT;
        tx_frame.id_type = CAN_ID_STANDARD;
        tx_frame.dlc = 8;
        tx_frame.timestamp_us = rtos_get_tick_count_ms() * 1000U;

        /* Pack position & control effort into CAN payload */
        int16_t pos_fixed = (int16_t)(telem.actual_position * 100.0f);
        int16_t effort_fixed = (int16_t)(telem.control_effort * 1000.0f);

        tx_frame.data[0] = (uint8_t)(pos_fixed & 0xFF);
        tx_frame.data[1] = (uint8_t)((pos_fixed >> 8) & 0xFF);
        tx_frame.data[2] = (uint8_t)(effort_fixed & 0xFF);
        tx_frame.data[3] = (uint8_t)((effort_fixed >> 8) & 0xFF);
        tx_frame.data[4] = (uint8_t)g_system_ctx.fsm.current_state;
        tx_frame.data[5] = telem.saturation_flag ? 1 : 0;
        tx_frame.data[6] = 0xAA;
        tx_frame.data[7] = 0x55;

        can_hal_transmit(&tx_frame);
    }
}

/* Task 2: Supervisory State Machine & Safety Monitor (50 Hz / 20 ms) */
static void task_supervisory_step(void)
{
    cmd_message_t cmd;
    while (rtos_queue_receive(g_cmd_queue, &cmd, 0))
    {
        if (cmd.event != EVENT_NONE)
        {
            fsm_dispatch_event(&g_system_ctx.fsm, cmd.event);
        }
        if (cmd.target_setpoint != 0.0f)
        {
            g_system_ctx.active_setpoint = cmd.target_setpoint;
        }
    }
    fsm_step(&g_system_ctx.fsm);
}

/* Task 3: Hard Real-Time Motion Control Loop (1000 Hz / 1 ms) */
static void task_control_loop_step(void)
{
    const float dt = 0.001f;
    float effort = 0.0f;

    if (g_system_ctx.fsm.current_state == STATE_ACTIVE)
    {
        effort = pid_update(&g_pid_inst, g_system_ctx.active_setpoint, g_plant_pos);

        /* 2nd-order Plant Dynamics: accel = 25*u - 2*v */
        float accel = (25.0f * effort) - (2.0f * g_plant_vel);
        g_plant_vel += accel * dt;
        g_plant_pos += g_plant_vel * dt;
    }
    else
    {
        pid_reset(&g_pid_inst);
        effort = 0.0f;
    }

    g_system_ctx.latest_telemetry.timestamp_ms = rtos_get_tick_count_ms();
    g_system_ctx.latest_telemetry.actual_position = g_plant_pos;
    g_system_ctx.latest_telemetry.actual_velocity = g_plant_vel;
    g_system_ctx.latest_telemetry.control_effort = effort;
    g_system_ctx.latest_telemetry.saturation_flag = (fabsf(effort) >= 1.0f);

    /* Enqueue telemetry every 10 ms (100 Hz) */
    static uint32_t sub_sample_count = 0;
    if (++sub_sample_count >= 10)
    {
        sub_sample_count = 0;
        rtos_queue_send(g_telem_queue, &g_system_ctx.latest_telemetry, 0);
    }
}

int main(void)
{
    printf("===============================================================\n");
    printf("     INTEGRATED EMBEDDED ECU SIL CONTROLLER ARCHITECTURE       \n");
    printf("===============================================================\n\n");

    /* 1. Hardware Drivers & IPC Initialization */
    can_hal_init();
    can_hal_set_filter(0, 0x100, 0x700); /* Accept IDs 0x100 - 0x1FF */

    g_cmd_queue = rtos_queue_create(8, sizeof(cmd_message_t));
    g_telem_queue = rtos_queue_create(16, sizeof(control_telemetry_t));

    fsm_init(&g_system_ctx.fsm, &g_system_ctx);
    pid_init(&g_pid_inst, 20.0f, 3.0f, 1.2f, 0.001f, -1.0f, 1.0f);

    /* 2. Boot Phase */
    printf("[1] ECU Booting Peripherals...\n");
    for (int i = 0; i < 15; i++)
    {
        task_supervisory_step();
    }
    printf("    Boot Status: OK | System State: %s\n\n",
           fsm_get_state_name(g_system_ctx.fsm.current_state));
    assert(g_system_ctx.fsm.current_state == STATE_STANDBY);

    /* 3. Remote Injection: Transmit CAN Frame to Arm and Set Target to 8.0 rad */
    printf("[2] Injecting External CAN Command Frame (ID: 0x110, Target: 8.0 rad)...\n");
    float commanded_target = 8.0f;
    uint32_t target_raw;
    memcpy(&target_raw, &commanded_target, sizeof(float));

    can_frame_t cmd_frame = {
        .id = CAN_ID_CMD_RECEIVE,
        .id_type = CAN_ID_STANDARD,
        .dlc = 5,
        .data = {
            (uint8_t)EVENT_START_CMD,
            (uint8_t)(target_raw & 0xFF),
            (uint8_t)((target_raw >> 8) & 0xFF),
            (uint8_t)((target_raw >> 16) & 0xFF),
            (uint8_t)((target_raw >> 24) & 0xFF)},
        .timestamp_us = 10000};
    can_hal_transmit(&cmd_frame);

    /* 4. Run Multi-Rate FreeRTOS SIL Loop for 3000 ms (3.0 s) */
    printf("[3] Running Multi-Rate SIL Scheduling Loop (3.0 seconds / 3000 cycles)...\n");
    for (int ms = 0; ms < 3000; ms++)
    {
        /* Task 3: 1000 Hz Motion Control */
        task_control_loop_step();

        /* Task 2: 50 Hz Supervisory FSM (every 20 ms) */
        if (ms % 20 == 0)
        {
            task_supervisory_step();
        }

        /* Task 1: 100 Hz CAN Telemetry (every 10 ms) */
        if (ms % 10 == 0)
        {
            task_can_telemetry_step();
        }
    }

    /* 5. Validation Assertions */
    printf("\n=== Real-Time Performance Benchmarks ===\n");
    printf("  Target Setpoint         : %.3f rad\n", g_system_ctx.active_setpoint);
    printf("  Final Plant Position    : %.3f rad\n", g_system_ctx.latest_telemetry.actual_position);
    printf("  Settling Error          : %.4f rad\n",
           fabsf(g_system_ctx.active_setpoint - g_system_ctx.latest_telemetry.actual_position));
    printf("  Final Control Effort    : %.4f (Normalized)\n", g_system_ctx.latest_telemetry.control_effort);
    printf("  System Operational State: %s\n", fsm_get_state_name(g_system_ctx.fsm.current_state));

    assert(fabsf(g_system_ctx.active_setpoint - g_system_ctx.latest_telemetry.actual_position) < 0.03f);

    printf("\n>> SUCCESS: Complete Integrated ECU SIL Firmware Suite fully verified.\n");
    return 0;
}