#include "fsm.h"
#include <stdio.h>
#include <assert.h>

int main(void)
{
    fsm_context_t fsm;
    fsm_init(&fsm, NULL);

    printf("1. Initial state: %s\n", fsm_get_state_name(fsm.current_state));
    assert(fsm.current_state == STATE_INIT);

    /* Step until boot self-test passes */
    for (int i = 0; i < 15; i++)
    {
        fsm_step(&fsm);
    }
    printf("2. Post-boot state: %s\n", fsm_get_state_name(fsm.current_state));
    assert(fsm.current_state == STATE_STANDBY);

    /* Command system to ACTIVE */
    fsm_dispatch_event(&fsm, EVENT_START_CMD);
    printf("3. Commanded start: %s\n", fsm_get_state_name(fsm.current_state));
    assert(fsm.current_state == STATE_ACTIVE);

    /* Inject Over-Current Hardware Fault */
    printf("4. Injecting OVER_CURRENT fault...\n");
    fsm_raise_fault(&fsm, FAULT_OVER_CURRENT);
    printf("   Current state: %s, Latched Faults: 0x%08X\n",
           fsm_get_state_name(fsm.current_state), fsm.fault_flags);
    assert(fsm.current_state == STATE_FAULT);

    /* Try to force start during fault -> Must remain blocked */
    fsm_dispatch_event(&fsm, EVENT_START_CMD);
    assert(fsm.current_state == STATE_FAULT);

    /* Clear fault and verify safe return to STANDBY */
    fsm_clear_fault(&fsm, FAULT_OVER_CURRENT);
    printf("5. Fault cleared. Recovered state: %s\n", fsm_get_state_name(fsm.current_state));
    assert(fsm.current_state == STATE_STANDBY);

    printf("\n>> SUCCESS: FSM engine passed all deterministic assertion tests.\n");
    return 0;
}