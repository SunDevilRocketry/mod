/**
  ******************************************************************************
  * @file           : commands.c
  * @brief          : Contains general command functions common to all embedded controllers
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */


/*------------------------------------------------------------------------------
 Standard Includes                                                               
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "commands.h"
#include "usb.h"

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/**
  * @brief Sends a 1 byte response back to host PC to signal a functioning
  * serial connection
  */
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
response = PING_RESPONSE_CODE; /* Code specific to board and revision */

/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
usb_transmit( &response, sizeof( response ), USB_DEFAULT_TIMEOUT );

} /* ping */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/