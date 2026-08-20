#ifndef RTOS_IPC_TYPES_H
#define RTOS_IPC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "fsm_types.h"

/**
 * @brief Command messages routed from Comm Task -> Supervisory FSM Task
 */
typedef struct
{
    system_event_t event;
    uint32_t command_id;
    float target_setpoint;
    uint32_t crc32_checksum;
} cmd_message_t;

/**
 * @brief Control loop state broadcasted to Supervisory & Telemetry
 */
typedef struct
{
    uint32_t timestamp_ms;
    float actual_position;
    float actual_velocity;
    float control_effort; /* Normalized -1.0f to +1.0f */
    uint32_t execution_jitter_us;
    bool saturation_flag;
} control_telemetry_t;

/**
 * @brief Global Shared Hardware Context across RTOS Tasks
 */
typedef struct
{
    fsm_context_t fsm;
    control_telemetry_t latest_telemetry;
    float active_setpoint;
    uint32_t task_heartbeats[STATE_COUNT];
    bool emergency_shutdown;
} system_shared_context_t;

#endif /* RTOS_IPC_TYPES_H */