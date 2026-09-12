#ifndef TEST_DEBUG_SDR_STUBS_H
#define TEST_DEBUG_SDR_STUBS_H

#include <stddef.h>
#include <stdint.h>

void stubs_reset(void);
void set_write_callback_limit(unsigned int limit);
unsigned int get_write_calls(void);
size_t get_last_write_size(void);
const uint8_t* get_last_write_buffer(void);
unsigned int get_overflow_calls(void);
size_t get_last_overflow_size(void);

#endif
