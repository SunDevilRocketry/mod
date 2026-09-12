#include <string.h>

#include "test_commands_stubs.h"

static SENSOR_STATUS sensor_dump_status;
static USB_STATUS usb_transmit_status;
static uint8_t usb_transmit_buffer[256];
static size_t usb_transmit_size;
static uint32_t usb_transmit_timeout;
static unsigned int usb_transmit_calls;

void stubs_reset(void)
{
    sensor_dump_status = SENSOR_OK;
    usb_transmit_status = USB_OK;
    memset(usb_transmit_buffer, 0, sizeof(usb_transmit_buffer));
    usb_transmit_size = 0;
    usb_transmit_timeout = 0;
    usb_transmit_calls = 0;
}

void set_sensor_dump_status(SENSOR_STATUS status)
{
    sensor_dump_status = status;
}

void set_usb_transmit_status(USB_STATUS status)
{
    usb_transmit_status = status;
}

size_t get_usb_transmit_size(void)
{
    return usb_transmit_size;
}

uint32_t get_usb_transmit_timeout(void)
{
    return usb_transmit_timeout;
}

unsigned int get_usb_transmit_calls(void)
{
    return usb_transmit_calls;
}

void* get_usb_transmit_buffer(void)
{
    return usb_transmit_buffer;
}

SENSOR_STATUS sensor_dump(SENSOR_DATA* sensor_data_ptr)
{
    (void)sensor_data_ptr;
    return sensor_dump_status;
}

USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout)
{
    usb_transmit_calls++;
    usb_transmit_size = tx_data_size;
    usb_transmit_timeout = timeout;
    memcpy(usb_transmit_buffer, tx_data_ptr, tx_data_size);
    return usb_transmit_status;
}
