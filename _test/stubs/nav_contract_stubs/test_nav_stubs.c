#include <stddef.h>
#include <time.h>

#include "test_nav_stubs.h"
#include "nav_contract.h"

NAV_STATUS stub__state_reserve = NAV_OK;
NAV_STATUS stub__state_release = NAV_OK;
int stub__pof_transitions_logged = 0;

NAV_STATUS nav_reserve_state
    (
    NAV_PUBLIC_STATE* nav_state
    )
{
return stub__state_reserve;

}

NAV_STATUS nav_release_state
    (
    NAV_PUBLIC_STATE* nav_state
    )
{
return stub__state_release;

}

void nav_pof_transition_event
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_POF original_state,
    NAV_POF new_state
    )
{
stub__pof_transitions_logged++;

}