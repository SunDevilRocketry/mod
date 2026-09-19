#include <string.h>

#include "test_debug_sdr_stubs.h"
#include "timer.h"

static unsigned int write_calls;
static unsigned int write_callback_limit;
static size_t last_write_size;
static uint8_t last_write_buffer[128];
static unsigned int overflow_calls;
static size_t last_overflow_size;

void stubs_reset(void)
{
    write_calls = 0;
    write_callback_limit = 0;
    last_write_size = 0;
    memset(last_write_buffer, 0, sizeof(last_write_buffer));
    overflow_calls = 0;
    last_overflow_size = 0;
}

void set_write_callback_limit(unsigned int limit)
{
    write_callback_limit = limit;
}

unsigned int get_write_calls(void)
{
    return write_calls;
}

size_t get_last_write_size(void)
{
    return last_write_size;
}

const uint8_t* get_last_write_buffer(void)
{
    return last_write_buffer;
}

unsigned int get_overflow_calls(void)
{
    return overflow_calls;
}

size_t get_last_overflow_size(void)
{
    return last_overflow_size;
}

void test_write_callback(void* buffer, size_t size)
{
    write_calls++;
    last_write_size = size;
    memcpy(last_write_buffer, buffer, size);
    if( write_callback_limit != 0 && write_calls >= write_callback_limit )
        {
        }
}

void test_overflow_callback(const char* message, size_t size)
{
    (void)message;
    overflow_calls++;
    last_overflow_size = size;
}

SYSTEM_TIME get_system_time(void)
{
    SYSTEM_TIME time = { 1, 2, 3, 4 };
    return time;
}
