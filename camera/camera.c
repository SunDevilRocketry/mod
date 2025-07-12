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
* 		set_camera_main                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Turns the camera connected to the main terminal on or off              *
*                                                                              *
*******************************************************************************/
void set_camera_main
    (
    CAMERA_STATE state
    )
{
if ( state ) /* on */
    {
    HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_SET ); 
    }
else         /* off */
    {
    HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_RESET );
    }

} /* set_camera_drogue */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		set_camera_drogue                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Turns the camera connected to the drougue terminal on or off           *
*                                                                              *
*******************************************************************************/
void set_camera_drogue
    (
    CAMERA_STATE state
    )
{
if ( state ) /* on */
    {
    HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_SET ); 
    }
else         /* off */
    {
    HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_RESET );
    }

} /* set_camera_drogue */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/