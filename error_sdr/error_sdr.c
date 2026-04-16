/*******************************************************************************
*
* FILE: 
* 		error_sdr.c
*
* DESCRIPTION: 
* 		Contains error-related functions for SDR code.
*
* COPYRIGHT:                                                                   
*       Copyright (c) 2025 Sun Devil Rocketry.                                 
*       All rights reserved.                                                   
*                                                                              
*       This software is licensed under terms that can be found in the LICENSE 
*       file in the root directory of this software component.                 
*       If no LICENSE file comes with this software, it is covered under the   
*       BSD-3-Clause.                                                          
*                                                                              
*       https://opensource.org/license/bsd-3-clause          
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"
#include "error_sdr.h"

#ifdef STM32H750xx
#include "led.h"
#include "stm32h7xx_hal.h"
#else if defined( F1_TESTBED )
// LEDs not supported. Provide a stub.
typedef enum {
    LED_RED,
    LED_BLUE,
    LED_GREEN,
    LED_PURPLE,
    LED_CYAN,
    LED_YELLOW,
    LED_WHITE
} LED_COLOR;

void led_set_color( LED_COLOR color )
{} // stub
#endif

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/
#ifdef USE_CALLBACK_TABLE
static volatile ERROR_CALLBACK* callback_table_lookup
    (
    volatile ERROR_CODE error_code
    );
#endif

static void dflt_error_handler
    (
    volatile ERROR_CODE error_code
    );

/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/

/* Contract: Callback table must be defined by the project. 
   Implementation not provided by the module. */
#ifdef USE_CALLBACK_TABLE
extern volatile ERROR_CALLBACK error_callback_table[];
extern uint16_t error_callback_table_size;
#endif

/*------------------------------------------------------------------------------
 Static Variables  
------------------------------------------------------------------------------*/

TEXT_MESSAGE last_warning;
bool is_pending_warning = false;

TEXT_MESSAGE last_info;
bool is_pending_info = false;

/* Default error handler for table misses or undefined tables. Can be overridden. */
volatile ERROR_CALLBACK default_error_handler = { 0, dflt_error_handler };

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_fail_fast                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		This handles errors by matching an error code to its callback in the   *
*       table.                                                                 *
*                                                                              *
*******************************************************************************/
void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
#if defined(USE_CALLBACK_TABLE)
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
volatile ERROR_CALLBACK* error_callback_ptr;

/*------------------------------------------------------------------------------
 Implementation
------------------------------------------------------------------------------*/
error_callback_ptr = callback_table_lookup( error_code );

/* If error is in table, use its callback and return. */
if( error_callback_ptr != NULL)
    {
    error_callback_ptr->error_callback( error_code );
    return;
    }
#endif

/*------------------------------------------------------------------------------
 Default to Legacy Handling (table not defined or callback not found)
------------------------------------------------------------------------------*/
default_error_handler.error_callback( error_code );

} /* error_fail_fast */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_log_warning                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Place a warning message in the buffer.                                 *
*                                                                              *
*******************************************************************************/
void error_log_warning
    (
    char* message
    )
{
last_warning.systick = HAL_GetTick();
memcpy( last_warning.message, message, 72 );
is_pending_warning = true;

} /* error_log_warning */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_log_info                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Place a warning message in the buffer.                                 *
*                                                                              *
*******************************************************************************/
void error_log_info
    (
    char* message
    )
{
last_info.systick = HAL_GetTick();
memcpy( last_info.message, message, 72 );
is_pending_info = true;

} /* error_log_info */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_get_warning                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Get the latest warning message if one exists.                          *
*                                                                              *
*******************************************************************************/
bool error_get_warning
    (
    TEXT_MESSAGE* buffer
    )
{
if ( !is_pending_warning )
    {
    return false;
    }

memcpy( buffer, &last_warning, sizeof( TEXT_MESSAGE ) );
is_pending_warning = false;
return true;

} /* error_get_warning */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_is_pending_warning                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Get the latest warning message if one exists.                          *
*                                                                              *
*******************************************************************************/
bool error_is_pending_warning
    (
    void
    )
{
return is_pending_warning;

} /* error_is_pending_warning */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_get_info                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Get the latest info message if one exists.                             *
*                                                                              *
*******************************************************************************/
bool error_get_info
    (
    TEXT_MESSAGE* buffer
    )
{
if ( !is_pending_info )
    {
    return false;
    }

memcpy( buffer, &last_info, sizeof( TEXT_MESSAGE ) );
is_pending_info = false;
return true;

} /* error_get_info */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_is_pending_info                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Get the latest info message if one exists.                             *
*                                                                              *
*******************************************************************************/
bool error_is_pending_info
    (
    void
    )
{
return is_pending_info;

} /* error_is_pending_info */


#if defined(USE_CALLBACK_TABLE)
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		callback_table_lookup                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Match an error code to its callback table entry. Returns null if       *
*       a callback is not found.                                               *
*                                                                              *
*******************************************************************************/
static volatile ERROR_CALLBACK* callback_table_lookup
    (
    volatile ERROR_CODE error_code
    )
{
for( uint8_t i = 0; i < error_callback_table_size; i++ )
    {
    if( error_callback_table[i].error_code == error_code )
        {
        return &( error_callback_table[i] );
        }
    }

return NULL;

} /* callback_table_lookup */
#endif


/**
 * GCOVR_EXCL_START
 * 
 * This whole function is excluded due to the control flow trap. It is impossible
 * to return from this function. This case can be reached, but the whole point of
 * getting here is that an error is unrecoverable, so via analysis we can prove
 * that this will not return.
 */
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		dflt_error_handler                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Legacy style error trap function. Default error handler.               *
*                                                                              *
*******************************************************************************/
static void dflt_error_handler
    (
    volatile ERROR_CODE error_code
    )
{
led_set_color( LED_RED );
while(1); /* Control flow trap */

} /* dflt_error_handler */
/**
 * GCOVR_EXCL_STOP
 */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/