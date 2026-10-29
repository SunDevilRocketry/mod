#include <stddef.h>
#include <time.h>

#include "test_nav_stubs.h"
#include "timer.h"
#include "nav_contract.h"

NAV_STATUS stub__state_reserve = NAV_OK;
NAV_STATUS stub__state_release = NAV_OK;

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

SYSTEM_TIME get_system_time
    (
    void
    )
{
SYSTEM_TIME to_return;
to_return.secs = 10;

return to_return;

}