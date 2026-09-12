#ifndef TEST_SENSOR_STUBS_H
#define TEST_SENSOR_STUBS_H

#include <stdbool.h>

#include "baro.h"
#include "imu.h"
#include "usb.h"

void stubs_reset(void);
void set_start_imu_status(IMU_STATUS status);
void set_start_baro_status(BARO_STATUS status);
unsigned int get_start_imu_calls(void);
unsigned int get_start_baro_calls(void);

#endif
