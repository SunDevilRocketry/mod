/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                               
------------------------------------------------------------------------------*/
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#ifdef USE_RS485
    #include "rs485.h"
#endif
#include "usb.h"
#ifdef VALVE_CONTROLLER
    #include "valve.h"
#endif


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ping                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sends a 1 byte response back to host PC to signal a functioning        * 
*       serial connection                                                      *
*                                                                              *
*******************************************************************************/
void ping
    (
    #ifndef VALVE_CONTROLLER
        void
    #else
        CMD_SOURCE cmd_source
    #endif
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t    response;   /* A0002 Response Code */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
response = PING_RESPONSE_CODE; /* Code specific to board and revision */


/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
#ifdef VALVE_CONTROLLER 
    if ( cmd_source == CMD_SOURCE_USB )
        {
        usb_transmit( &response         , 
                      sizeof( response ), 
                      HAL_DEFAULT_TIMEOUT );
        }
    else
        {
        valve_transmit( &response         , 
                        sizeof( response ), 
                        HAL_DEFAULT_TIMEOUT );
        }
#endif
#ifdef ENGINE_CONTROLLER
    #if defined( USE_RS485 )
        rs485_transmit( &response, sizeof( response ), RS485_DEFAULT_TIMEOUT );
    #else
        usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );
    #endif
#endif /* #ifdef ENGINE_CONTROLLER */

} /* ping */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/