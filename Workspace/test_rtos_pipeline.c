#include "sys_tasks.h"
#include "fsm.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

int main(void)
{
    system_shared_context_t sys_ctx;
    memset(&sys_ctx, 0, sizeof(sys_ctx));

    /* Initialize State Machine */
    fsm_init(&sys_ctx.fsm, &sys_ctx);

    /* Allocate Thread-Safe Queues */
    rtos_queue_t *cmd_queue = rtos_queue_create(8, sizeof(cmd_message_t));
    rtos_queue_t *telem_queue = rtos_queue_create(16, sizeof(control_telemetry_t));
    assert(cmd_queue != NULL && telem_queue != NULL);

    printf("=== Real-Time FreeRTOS Multi-Task SIL Harness ===\n");

    /* 1. Boot sequence */
    for (int i = 0; i < 15; i++)
    {
        task_sys_mgmt_step(&sys_ctx, cmd_queue);
    }
    printf("[1] Boot completed. State: %s\n", fsm_get_state_name(sys_ctx.fsm.current_state));
    assert(sys_ctx.fsm.current_state == STATE_STANDBY);

    /* 2. Dispatch Start Command + Setpoint to 5.0 radians via IPC Queue */
    cmd_message_t start_cmd = {
        .event = EVENT_START_CMD,
        .command_id = 0xA1,
        .target_setpoint = 5.0f,
        .crc32_checksum = 0};
    rtos_queue_send(cmd_queue, &start_cmd, 0);

    /* Step System Management Task to consume command */
    task_sys_mgmt_step(&sys_ctx, cmd_queue);
    printf("[2] Command processed via Queue. State: %s, Setpoint: %.2f\n",
           fsm_get_state_name(sys_ctx.fsm.current_state), sys_ctx.active_setpoint);
    assert(sys_ctx.fsm.current_state == STATE_ACTIVE);

    /* 3. Run 1 kHz Control Loop for 1000 ms (1 second) */
    printf("[3] Running 1 kHz Closed-Loop SIL Simulation for 1000 cycles (1.0 s)...\n");
    for (int ms = 0; ms < 1000; ms++)
    {
        task_control_loop_step(&sys_ctx, telem_queue);
        if (ms % 20 == 0)
        {
            task_sys_mgmt_step(&sys_ctx, cmd_queue);
        }
        if (ms % 10 == 0)
        {
            task_telemetry_step(&sys_ctx, telem_queue, cmd_queue);
        }
    }

    printf("    Final Tracked Position : %.3f rad (Target: %.3f rad)\n",
           sys_ctx.latest_telemetry.actual_position, sys_ctx.active_setpoint);
    printf("    Final Control Effort   : %.3f (Normalized)\n",
           sys_ctx.latest_telemetry.control_effort);

    /* Over 1 second, position should have tracked closely toward setpoint */
    assert(sys_ctx.latest_telemetry.actual_position > 4.0f);

    printf("\n>> SUCCESS: Module 2 FreeRTOS IPC & Multi-Task Loop fully validated.\n");
    return 0;
}