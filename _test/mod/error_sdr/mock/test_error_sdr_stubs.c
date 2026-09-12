#include "test_error_sdr_stubs.h"

static unsigned int callback_calls;
static ERROR_CODE callback_error;

void stubs_reset(void)
{
    callback_calls = 0;
    callback_error = ERROR_UNKNOWN_FATAL_ERROR;
}

unsigned int get_callback_calls(void)
{
    return callback_calls;
}

ERROR_CODE get_callback_error(void)
{
    return callback_error;
}

void test_error_callback(ERROR_CODE error_code)
{
    callback_calls++;
    callback_error = error_code;
}

uint32_t HAL_GetTick(void)
{
    return 9876;
}
