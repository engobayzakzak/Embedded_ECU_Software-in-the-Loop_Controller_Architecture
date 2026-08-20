#ifndef FSM_TYPES_H
#define FSM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief System Operating States
 */
typedef enum
{
    STATE_INIT = 0,
    STATE_STANDBY,
    STATE_ACTIVE,
    STATE_FAULT,
    STATE_COUNT
} system_state_t;

/**
 * @brief System Events triggering state transitions
 */
typedef enum
{
    EVENT_NONE = 0,
    EVENT_INIT_SUCCESS,
    EVENT_INIT_FAILED,
    EVENT_START_CMD,
    EVENT_STOP_CMD,
    EVENT_FAULT_TRIGGERED,
    EVENT_FAULT_CLEARED,
    EVENT_COUNT
} system_event_t;

/**
 * @brief Latching Fault Bitmask
 */
typedef enum
{
    FAULT_NONE = 0x00000000U,
    FAULT_OVER_CURRENT = (1U << 0),
    FAULT_OVER_VOLTAGE = (1U << 1),
    FAULT_UNDER_VOLTAGE = (1U << 2),
    FAULT_OVER_TEMPERATURE = (1U << 3),
    FAULT_SENSOR_COMM_LOST = (1U << 4),
    FAULT_WATCHDOG_TIMEOUT = (1U << 5),
    FAULT_ESTOP_ASSERTED = (1U << 6)
} fault_mask_t;

/* Forward declaration */
struct fsm_context;

/**
 * @brief State Action Callback Signatures
 */
typedef void (*state_action_t)(struct fsm_context *ctx);
typedef bool (*state_guard_t)(struct fsm_context *ctx, system_event_t event);

/**
 * @brief State Interface Structure
 */
typedef struct
{
    system_state_t state_id;
    const char *name;
    state_action_t on_entry;
    state_action_t on_run;
    state_action_t on_exit;
} state_interface_t;

/**
 * @brief Main FSM Context Instance
 */
typedef struct fsm_context
{
    system_state_t current_state;
    system_state_t previous_state;
    uint32_t fault_flags;       /* Active latched faults */
    uint32_t state_timer_ticks; /* Time spent in current state */
    bool state_changed;
    const state_interface_t *states[STATE_COUNT];
    void *user_data; /* Hook for hardware/telemetry handles */
} fsm_context_t;

#endif /* FSM_TYPES_H */