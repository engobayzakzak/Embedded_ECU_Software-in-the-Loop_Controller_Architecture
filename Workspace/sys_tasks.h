#ifndef SYS_TASKS_H
#define SYS_TASKS_H

#include "rtos_ipc_types.h"
#include "rtos_port.h"

/* Task Entry Signatures */
void task_sys_mgmt_step(system_shared_context_t *sys_ctx, rtos_queue_t *cmd_q);
void task_control_loop_step(system_shared_context_t *sys_ctx, rtos_queue_t *telem_q);
void task_telemetry_step(system_shared_context_t *sys_ctx, rtos_queue_t *telem_q, rtos_queue_t *cmd_q);

#endif /* SYS_TASKS_H */