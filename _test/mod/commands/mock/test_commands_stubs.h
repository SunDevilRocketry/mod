#ifndef TEST_COMMANDS_STUBS_H
#define TEST_COMMANDS_STUBS_H

#include <stddef.h>
#include <stdint.h>

#include "sensor.h"
#include "usb.h"

void stubs_reset(void);
void set_sensor_dump_status(SENSOR_STATUS status);
void set_usb_transmit_status(USB_STATUS status);
size_t get_usb_transmit_size(void);
uint32_t get_usb_transmit_timeout(void);
unsigned int get_usb_transmit_calls(void);
void* get_usb_transmit_buffer(void);

#endif
