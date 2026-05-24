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
                      ##### Debug module features #####
  ==============================================================================
  [..]
  (+) Write to a logging interface
  ******************************************************************************
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H 
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>

#include "main.h"

/* Constants -----------------------------------------------------------------*/
#ifndef DEBUG_INTF_BUF_SIZE /* allow the project to override the buffer length if needed */
#define DEBUG_INTF_BUF_SIZE 2048 /* bytes */
#endif

#define DEBUG_MSG_PREAMBLE_MAX_LEN  22    /* "[XX:XX:XX.XXX ERROR]: "           */
#define DEBUG_MSG_MAX_LEN           128   /* total length allowed incl newline    */
#define DEBUG_MSG_USER_MAX_LEN      (DEBUG_MSG_MAX_LEN - DEBUG_MSG_PREAMBLE_MAX_LEN)
                                          /* max length of the user provided msg  */

/* Types ---------------------------------------------------------------------*/
typedef void (*debug_write_callback)(void*, size_t);
typedef void (*overflow_callback)(const char*, size_t);

/* this handle is exposed here, but will only be modified internally */
typedef struct _DEBUG_HANDLE {
    debug_write_callback  write_function;               /* callback to a function to write to the log (async)      */
    overflow_callback     overflow_function;            /* callback to a function to handle overflow events        */
                                                        /* the default handling is to clear the debug buffer and   */
                                                        /* then add a message indicating that it has overflowed    */

    uint8_t               tx_buf[DEBUG_INTF_BUF_SIZE];  /* the log message buffer                                  */
    uint8_t*              buf_tx_head;                  /* the address of the next message to be sent over the wire*/
    uint8_t*              buf_put_head;                 /* the address to put the next message                     */
    size_t                buffer_size_cnt;              /* the amount of unsent data in the buffer                 */
    bool                  transmitting;                 /* flag to indicate whether we're currently transmitting   */
} DEBUG_HANDLE;

typedef enum _DEBUG_STATUS {
  DEBUG_OK = 0,
  DEBUG_FAIL,
  DEBUG_OVERFLOW
} DEBUG_STATUS;

typedef enum _DEBUG_LEVEL {
  LOG_LVL_INFO,
  LOG_LVL_WARN,
  LOG_LVL_ERROR /* note: "debug" errors will not trigger in release builds */
} DEBUG_LEVEL;

/* Exported functions prototypes ---------------------------------------------*/

DEBUG_STATUS debug_init
    (
    debug_write_callback write_function, 
    overflow_callback overflow_function
    );

void debug_clear
    (
    void
    );

DEBUG_STATUS debug_log
    (
    const char* message,
    size_t len,
    DEBUG_LEVEL log_level
    );

void debug_callback_handler
    (
    void
    );

/* Alias Macros --------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       debug_log_assert                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		  Checks condition, if false logs a message at the given severity. Debug *
*       mode only.                                                             *
*                                                                              *
*******************************************************************************/
#if defined(DEBUG) || !defined(RELBLD)
    #define debug_log_assert( condition, msg, level ) \
        do { if ( !(condition) ) debug_log(msg, sizeof(msg), level); } while(0)
#else
    #define debug_log_assert( condition, msg, level ) \
        do { } while(0)
#endif


/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       debug_log_msg                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		  Logs a message at the given severity. Debug mode only.                 *
*                                                                              *
*******************************************************************************/
#if defined(DEBUG) || !defined(RELBLD)
    #define debug_log_msg( msg, level ) \
        do { debug_log(msg, sizeof(msg), level); } while(0)
#else
    #define debug_log_msg( msg, level ) \
        do { } while(0)
#endif


/*******************************************************************************
*                                                                              *
* MACRO:                                                                       * 
*       debug_is_connected                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		  Checks if the debug interface is connected. HW dependent.              *
*                                                                              *
*******************************************************************************/
#ifdef STM32H7XX
#define debug_is_connected() (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
#else
#define debug_is_connected() (false)
#endif


#endif /* DEBUG_H */