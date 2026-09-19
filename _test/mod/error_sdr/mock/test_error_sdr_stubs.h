#ifndef TEST_ERROR_SDR_STUBS_H
#define TEST_ERROR_SDR_STUBS_H

#include <stdint.h>
#include "error_sdr.h"

void stubs_reset(void);
unsigned int get_callback_calls(void);
ERROR_CODE get_callback_error(void);
void test_error_callback(ERROR_CODE error_code);

#endif
