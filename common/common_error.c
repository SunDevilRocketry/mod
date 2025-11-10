/*******************************************************************************
*
* FILE: 
* 		common_error.c
*
* DESCRIPTION: 
* 		Contains error-related functions for SDR code.
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
#include "common.h"
#include "sdr_error.h"
#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Static Variables  
------------------------------------------------------------------------------*/
TEXT_MESSAGE last_warning;
bool is_pending_warning = false;

TEXT_MESSAGE last_info;
bool is_pending_info = false;

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_fail_fast                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		In case of error occurrence, this function passes the error            *
*       code to the error handler                                              *
*                                                                              *
*******************************************************************************/
void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
Error_Handler(error_code);

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

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/