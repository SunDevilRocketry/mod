/*******************************************************************************
*
* FILE: 
* 		camera.c
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer camera control 
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
void set_camera_state
    (
    CAMERA_SELECTION camera, CAMERA_STATE state
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
        /* Error? */
        }
    }

} /* set_camera_state */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/