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
 Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#include "usb.h"


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
	void
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t    response;   /* A0002 Response Code */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
response = A0002_PING_RESPONSE_CODE; /* Code specific to board and revision */


/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );

} /* ping */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/