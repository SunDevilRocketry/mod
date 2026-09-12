#include "test_telemetry_stubs.h"

#include "main.h"
#include "sensor.h"
#include "telemetry.h"

static unsigned int error_fail_fast_calls;
static ERROR_CODE reported_error;

void stubs_reset(void)
{
    error_fail_fast_calls = 0;
    reported_error = ERROR_UNKNOWN_FATAL_ERROR;
}

unsigned int get_error_fail_fast_calls(void)
{
    return error_fail_fast_calls;
}

ERROR_CODE get_reported_error(void)
{
    return reported_error;
}

void error_fail_fast(volatile ERROR_CODE error_code)
{
    error_fail_fast_calls++;
    reported_error = error_code;
}

uint32_t HAL_GetTick(void)
{
    return 1234;
}

uint32_t HAL_GetUIDw0(void)
{
    return 0;
}

uint32_t HAL_GetUIDw1(void)
{
    return 0;
}

uint32_t HAL_GetUIDw2(void)
{
    return 0;
}

FLIGHT_COMP_STATE_TYPE get_fc_state(void)
{
    return FC_STATE_IDLE;
}

void dashboard_construct_dump(DASHBOARD_DUMP_TYPE* dump_buffer_ptr)
{
    (void)dump_buffer_ptr;
}

void sensor_baro_alt(SENSOR_DATA* sensor_data_ptr)
{
    (void)sensor_data_ptr;
}
