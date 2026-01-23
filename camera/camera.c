/*******************************************************************************
*
* FILE: 
* 		camera.c
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer camera control 
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
 Project Includes
------------------------------------------------------------------------------*/
#include "camera.h"


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		set_camera_state                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Turns the selected camera on or off                                    *
*                                                                              *
*******************************************************************************/
CAMERA_STATUS set_camera_state
    (
    CAMERA_SELECTION camera, 
    CAMERA_STATE state
    )
{
GPIO_PinState pin_state;

if ( state == CAMERA_ON )
    {
    pin_state = GPIO_PIN_SET;
    }
else
    {
    pin_state = GPIO_PIN_RESET;
    }

/* Write pin state to selected GPIO pin */
switch ( camera )
    {
    case CAMERA_MAIN:
        {
	    HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, pin_state );
        break;
        }

    case CAMERA_DROGUE:
        {
        HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, pin_state );
        break;
        }

    default:
        {
        return CAMERA_INVALID_SELECTION;
        }
    }

return CAMERA_SUCCESS;

} /* set_camera_state */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/