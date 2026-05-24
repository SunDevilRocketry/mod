/**
  ******************************************************************************
  * @file           : debug_sdr.c
  * @brief          : Interfaces for debugging SDR embedded firmware.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the   
  * BSD-3-Clause.                                                          
  *                                                                              
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                      ##### Integration Guide #####
  ==============================================================================
  [..]
  (+) Provide an async debug write callback using debug_init(). If your debug
      write callback is not asynchronous, you risk blocking in the middle of
      a real-time event or otherwise hiding the true performance of the system.
  (+) Ensure you call debug_init() in your initialization sequence before ever 
      logging to it. This can be initialized as soon as the debug write interface 
      is initialized, so consider HAL -> debug write intf -> debug module -> rest
      of init to ensure you get useful logging throughout.
  (+) Register [ETS TEMP] (the callback handler) to your async write function's
      callback handler.
  (+) Optionally, register an overflow handler callback to handle circular buffer
      overflows. This takes the full, constructed message that triggered the 
      overflow as an argument. Pass NULL to use the default handler.
  (+) Do not use the _debug functions directly! Use the macros so that you have
      protection against release builds.
  (+) If you want to log in release builds, use the error_sdr logging functions.
  ******************************************************************************
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "main.h"
#include "debug_sdr.h"
#include "error_sdr.h"
#include "timer.h"

/* Static Variables ----------------------------------------------------------*/
static DEBUG_HANDLE debug_handle; /* This module may be conditionally compiled 
                                     for debug mode only. Holding the handle 
                                     internally helps avoid issues in debug
                                     versus release since we aren't passing
                                     around this giant buffer for no reason. */


/* Private function prototypes -----------------------------------------------*/
static void default_overflow_handler(const char* trigger_msg, size_t len);
static DEBUG_STATUS start_tx(void);

/* Procedures ----------------------------------------------------------------*/

/**
  * @brief Initializes the debug module
  * 
  * @details This function should be called before any other in the
  * debug module, as it initializes pointers and buffers to their
  * correct values.
  *
  * @param write_function Callback to the function that writes over the debug
  *        interface.
  * @param overflow_function Callback to the function that handles the buffer
  *        running out of space.
  * 
  *        Pass (void*)NULL as the overflow callback if you want to use
  *        the default overflow handler.
  * 
  * @retval The status of the debug module.
  */
DEBUG_STATUS debug_init
    (
    debug_write_callback write_function, 
    overflow_callback overflow_function
    )
{
/* Begin with an invariant check */
assert_return( write_function, DEBUG_FAIL ); /* exit if nullptr */

/* Initialize the internal debug handle */
memset( &debug_handle, 0, sizeof( DEBUG_HANDLE ) );

/* Set the callbacks */
debug_handle.write_function = write_function; /* already null checked */
if( overflow_function == 0 )
    {
    debug_handle.overflow_function = default_overflow_handler;
    }
else
    {
    debug_handle.overflow_function = overflow_function;
    }

/* Set the circular buffer's pointers */
debug_handle.buf_put_head = debug_handle.tx_buf;
debug_handle.buf_tx_head = debug_handle.tx_buf;
debug_handle.buffer_size_cnt = 0;

/* Clear the transmitting flag */
debug_handle.transmitting = false;

return DEBUG_OK;

} /* debug_init */

/**
 * @brief Reset the circular buffer.
 */
void debug_clear
    (
    void
    )
{
/* Reset the circular buffer's pointers */
debug_handle.buf_put_head = debug_handle.tx_buf;
debug_handle.buf_tx_head = debug_handle.tx_buf;
debug_handle.buffer_size_cnt = 0;
} /* debug_clear */


/**
  * @brief Add to the debug buffer.
  * 
  * @note Use the macros unless you know what you're doing!
  *
  * @param message A C-string with the message you want to send.
  * @param len The length of the message you want to send.
  * @param log_level The severity of the debug message.
  * 
  * @retval The status of the debug module.
  */
DEBUG_STATUS debug_log
    (
    const char* message,
    size_t len,
    DEBUG_LEVEL log_level
    )
{
/* Local Variables */
char temp_msg[DEBUG_MSG_MAX_LEN];
size_t true_len = 0;
memset(temp_msg, 0, DEBUG_MSG_MAX_LEN);

/* Begin with an invariant check */
assert_return( message, DEBUG_FAIL ); /* exit if nullptr */
assert_return( len <= DEBUG_MSG_USER_MAX_LEN, DEBUG_FAIL ); /* exit if msg too long */

/* Construct the message to be logged */
SYSTEM_TIME time = get_system_time();
true_len = snprintf(temp_msg, DEBUG_MSG_MAX_LEN, "[%02d:%02d:%02d.%03d ", (time.hours % 24), time.mins, time.secs, time.millis);
switch( log_level )
    {
    case LOG_LVL_INFO:
        true_len += snprintf( (temp_msg+true_len), DEBUG_MSG_MAX_LEN - true_len, "INFO]: " );
        break;
    case LOG_LVL_WARN:
        true_len += snprintf( (temp_msg+true_len), DEBUG_MSG_MAX_LEN - true_len, "WARN]: " );
        break;
    case LOG_LVL_ERROR:
        true_len += snprintf( (temp_msg+true_len), DEBUG_MSG_MAX_LEN - true_len, "ERROR]: " );
        break;
    default:
        return DEBUG_FAIL;
    }
strncpy(temp_msg+true_len, message, len);
true_len += len;
temp_msg[true_len] = '\n';
true_len++;
assert_return( true_len <= DEBUG_MSG_MAX_LEN, DEBUG_FAIL );

/* Check the overflow condition */
if( debug_handle.buffer_size_cnt + true_len > DEBUG_INTF_BUF_SIZE )
    {
    debug_handle.overflow_function( temp_msg, true_len );
    return DEBUG_OVERFLOW;
    }

/* Put the message in the buffer */
size_t space_to_end = debug_handle.tx_buf + DEBUG_INTF_BUF_SIZE - debug_handle.buf_put_head;
if( true_len <= space_to_end )
    {
    memset( debug_handle.buf_put_head, 0, true_len );
    memcpy( debug_handle.buf_put_head, temp_msg, true_len );
    }
else
    {
    /* two copies to handle wraparound */
    memset( debug_handle.buf_put_head, 0, space_to_end );
    memset( debug_handle.tx_buf, 0, true_len - space_to_end );
    memcpy( debug_handle.buf_put_head, temp_msg, space_to_end );
    memcpy( debug_handle.tx_buf, temp_msg + space_to_end, true_len - space_to_end );
    }
debug_handle.buffer_size_cnt += true_len;
/* Gross pointer arithmetic to set the buffer put head */
debug_handle.buf_put_head = debug_handle.tx_buf + ((debug_handle.buf_put_head - debug_handle.tx_buf) + true_len) % DEBUG_INTF_BUF_SIZE;

/* Kick off a transmission if one's not already going */
if( !debug_handle.transmitting )
    {
    return start_tx();
    }
else
    {
    return DEBUG_OK;
    }

} /* debug_log */


void debug_callback_handler
    (
    void
    )
{
debug_handle.transmitting = false;
if( debug_handle.buffer_size_cnt > 0 )
    {
    start_tx();
    }

} /* debug_callback_handler */


static DEBUG_STATUS start_tx
    (
    void
    )
{
/* Local variables */
size_t msg_len = 0;
uint8_t* tx_addr = debug_handle.buf_tx_head;
size_t space_to_end = debug_handle.tx_buf + DEBUG_INTF_BUF_SIZE - tx_addr;
static uint8_t staging_buf[DEBUG_MSG_MAX_LEN];
memset( staging_buf, 0, DEBUG_MSG_MAX_LEN );

/* Get the length of the string to be written */
for( uint16_t i = 0; i < DEBUG_MSG_MAX_LEN; i++ )
    {
    uint8_t* search_addr = debug_handle.tx_buf + ((tx_addr - debug_handle.tx_buf) + i) % DEBUG_INTF_BUF_SIZE;
    if( (char)*(search_addr) == '\n' )
        {
        msg_len = i + 1;
        break;
        }
    }

/* Check invariants */
assert_return( msg_len != 0, DEBUG_FAIL );
assert_return( msg_len <= DEBUG_MSG_MAX_LEN, DEBUG_FAIL );
assert_return( msg_len <= debug_handle.buffer_size_cnt, DEBUG_FAIL );

/* Kick off write */
if( msg_len <= space_to_end )
    {
    memcpy( staging_buf, tx_addr, msg_len );
    memset( tx_addr, 0, msg_len );
    }
else
    {
    /* Message wraps: write in two halves */
    memcpy( staging_buf, tx_addr, space_to_end );
    memset( tx_addr, 0, space_to_end );
    memcpy( staging_buf + space_to_end, debug_handle.tx_buf, msg_len - space_to_end );
    memset( debug_handle.tx_buf, 0, msg_len - space_to_end );
    }

/* Kick off write */
debug_handle.transmitting = true;
debug_handle.buf_tx_head = debug_handle.tx_buf + ((debug_handle.buf_tx_head - debug_handle.tx_buf) + msg_len) % DEBUG_INTF_BUF_SIZE;
debug_handle.buffer_size_cnt -= msg_len;
debug_handle.write_function( staging_buf, msg_len );

return DEBUG_OK;

} /* start_tx */


static void default_overflow_handler
    (
    const char* trigger_msg, 
    size_t len
    )
{
const char* msg = "Debug buffer overflowed!";
/* Clear the buffer and put a new message */
debug_clear();
debug_log(msg, strlen( msg ), LOG_LVL_WARN);

} /* default_overflow_handler */