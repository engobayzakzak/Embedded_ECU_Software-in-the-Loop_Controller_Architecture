#include "fsm.h"
#include <stdio.h>

/* --- Forward Declarations of State Action Handlers --- */
static void on_init_entry(fsm_context_t *ctx);
static void on_init_run(fsm_context_t *ctx);
static void on_init_exit(fsm_context_t *ctx);

static void on_standby_entry(fsm_context_t *ctx);
static void on_standby_run(fsm_context_t *ctx);
static void on_standby_exit(fsm_context_t *ctx);

static void on_active_entry(fsm_context_t *ctx);
static void on_active_run(fsm_context_t *ctx);
static void on_active_exit(fsm_context_t *ctx);

static void on_fault_entry(fsm_context_t *ctx);
static void on_fault_run(fsm_context_t *ctx);
static void on_fault_exit(fsm_context_t *ctx);

/* --- State Interface Descriptors --- */
static const state_interface_t g_state_init = {
    .state_id = STATE_INIT,
    .name = "STATE_INIT",
    .on_entry = on_init_entry,
    .on_run = on_init_run,
    .on_exit = on_init_exit};

static const state_interface_t g_state_standby = {
    .state_id = STATE_STANDBY,
    .name = "STATE_STANDBY",
    .on_entry = on_standby_entry,
    .on_run = on_standby_run,
    .on_exit = on_standby_exit};

static const state_interface_t g_state_active = {
    .state_id = STATE_ACTIVE,
    .name = "STATE_ACTIVE",
    .on_entry = on_active_entry,
    .on_run = on_active_run,
    .on_exit = on_active_exit};

static const state_interface_t g_state_fault = {
    .state_id = STATE_FAULT,
    .name = "STATE_FAULT",
    .on_entry = on_fault_entry,
    .on_run = on_fault_run,
    .on_exit = on_fault_exit};

/* --- Internal State Transition Processor --- */
static void transition_to(fsm_context_t *ctx, system_state_t new_state)
{
    if (ctx == NULL || new_state >= STATE_COUNT)
    {
        return;
    }

    if (ctx->states[ctx->current_state]->on_exit)
    {
        ctx->states[ctx->current_state]->on_exit(ctx);
    }

    ctx->previous_state = ctx->current_state;
    ctx->current_state = new_state;
    ctx->state_timer_ticks = 0;
    ctx->state_changed = true;

    if (ctx->states[ctx->current_state]->on_entry)
    {
        ctx->states[ctx->current_state]->on_entry(ctx);
    }
}

/* --- Public Core API --- */
void fsm_init(fsm_context_t *ctx, void *user_data)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->states[STATE_INIT] = &g_state_init;
    ctx->states[STATE_STANDBY] = &g_state_standby;
    ctx->states[STATE_ACTIVE] = &g_state_active;
    ctx->states[STATE_FAULT] = &g_state_fault;

    ctx->current_state = STATE_INIT;
    ctx->previous_state = STATE_INIT;
    ctx->fault_flags = FAULT_NONE;
    ctx->state_timer_ticks = 0;
    ctx->state_changed = true;
    ctx->user_data = user_data;

    if (ctx->states[STATE_INIT]->on_entry)
    {
        ctx->states[STATE_INIT]->on_entry(ctx);
    }
}

void fsm_raise_fault(fsm_context_t *ctx, fault_mask_t fault)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->fault_flags |= (uint32_t)fault;

    /* Immediate deterministic transition to safe-state */
    if (ctx->current_state != STATE_FAULT)
    {
        transition_to(ctx, STATE_FAULT);
    }
}

void fsm_clear_fault(fsm_context_t *ctx, fault_mask_t fault)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->fault_flags &= ~((uint32_t)fault);

    if (ctx->current_state == STATE_FAULT && ctx->fault_flags == FAULT_NONE)
    {
        fsm_dispatch_event(ctx, EVENT_FAULT_CLEARED);
    }
}

bool fsm_is_fault_active(const fsm_context_t *ctx, fault_mask_t fault)
{
    if (ctx == NULL)
    {
        return false;
    }
    return (ctx->fault_flags & (uint32_t)fault) != 0U;
}

void fsm_dispatch_event(fsm_context_t *ctx, system_event_t event)
{
    if (ctx == NULL)
    {
        return;
    }

    /* Asynchronous fault trigger overrides everything */
    if (event == EVENT_FAULT_TRIGGERED)
    {
        fsm_raise_fault(ctx, FAULT_ESTOP_ASSERTED);
        return;
    }

    switch (ctx->current_state)
    {
    case STATE_INIT:
        if (event == EVENT_INIT_SUCCESS)
        {
            transition_to(ctx, STATE_STANDBY);
        }
        else if (event == EVENT_INIT_FAILED)
        {
            fsm_raise_fault(ctx, FAULT_SENSOR_COMM_LOST);
        }
        break;

    case STATE_STANDBY:
        if (event == EVENT_START_CMD && ctx->fault_flags == FAULT_NONE)
        {
            transition_to(ctx, STATE_ACTIVE);
        }
        break;

    case STATE_ACTIVE:
        if (event == EVENT_STOP_CMD)
        {
            transition_to(ctx, STATE_STANDBY);
        }
        break;

    case STATE_FAULT:
        if (event == EVENT_FAULT_CLEARED && ctx->fault_flags == FAULT_NONE)
        {
            transition_to(ctx, STATE_STANDBY);
        }
        break;

    default:
        /* Undefined recovery -> force safe-state */
        fsm_raise_fault(ctx, FAULT_WATCHDOG_TIMEOUT);
        break;
    }
}

void fsm_step(fsm_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->state_timer_ticks++;

    if (ctx->states[ctx->current_state]->on_run)
    {
        ctx->states[ctx->current_state]->on_run(ctx);
    }
}

const char *fsm_get_state_name(system_state_t state)
{
    switch (state)
    {
    case STATE_INIT:
        return "STATE_INIT";
    case STATE_STANDBY:
        return "STATE_STANDBY";
    case STATE_ACTIVE:
        return "STATE_ACTIVE";
    case STATE_FAULT:
        return "STATE_FAULT";
    default:
        return "STATE_UNKNOWN";
    }
}

/* --- State Action Callbacks --- */
static void on_init_entry(fsm_context_t *ctx)
{
    (void)ctx;
}

static void on_init_run(fsm_context_t *ctx)
{
    /* Emulate boot validation: e.g., self-test completes after 10 ticks */
    if (ctx->state_timer_ticks >= 10)
    {
        fsm_dispatch_event(ctx, EVENT_INIT_SUCCESS);
    }
}

static void on_init_exit(fsm_context_t *ctx)
{
    (void)ctx;
}

static void on_standby_entry(fsm_context_t *ctx)
{
    (void)ctx;
    /* Safe stage: Gate drivers disabled, zero PWM duty cycle */
}

static void on_standby_run(fsm_context_t *ctx)
{
    (void)ctx;
    /* Telemetry heartbeat broadcasting */
}

static void on_standby_exit(fsm_context_t *ctx)
{
    (void)ctx;
}

static void on_active_entry(fsm_context_t *ctx)
{
    (void)ctx;
    /* Enable power stages, engage closed-loop control */
}

static void on_active_run(fsm_context_t *ctx)
{
    (void)ctx;
    /* High-frequency control task execution */
}

static void on_active_exit(fsm_context_t *ctx)
{
    (void)ctx;
    /* Soft deceleration ramp and power stage isolation */
}

static void on_fault_entry(fsm_context_t *ctx)
{
    (void)ctx;
    /* HARDWARE SAFE STATE: Trip gate drivers, engage dynamic braking */
}

static void on_fault_run(fsm_context_t *ctx)
{
    (void)ctx;
    /* Broadcast high-priority fault frame over CAN */
}

static void on_fault_exit(fsm_context_t *ctx)
{
    (void)ctx;
}