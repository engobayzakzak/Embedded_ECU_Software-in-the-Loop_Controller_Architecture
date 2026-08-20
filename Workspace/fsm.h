#ifndef FSM_H
#define FSM_H

#include "fsm_types.h"

void fsm_init(fsm_context_t *ctx, void *user_data);
void fsm_dispatch_event(fsm_context_t *ctx, system_event_t event);
void fsm_step(fsm_context_t *ctx);
void fsm_raise_fault(fsm_context_t *ctx, fault_mask_t fault);
void fsm_clear_fault(fsm_context_t *ctx, fault_mask_t fault);
bool fsm_is_fault_active(const fsm_context_t *ctx, fault_mask_t fault);
const char *fsm_get_state_name(system_state_t state);

#endif /* FSM_H */