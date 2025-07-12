/*******************************************************************************
*
* FILE: 
* 		camera.h
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer camera control 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Which camera to use based on parachute terminals */
/* May or may not use something like this */
// typedef enum _CAMERA_SELECTION
// {
//     CAMERA_MAIN    = MAIN_GPIO_PORT,
//     CAMERA_DROGUE  = DROGUE_GPIO_PORT
// } CAMERA_SELECTION;


/* Whether the camera is powered on or off */
typedef enum _CAMERA_STATE 
{
    CAMERA_OFF = 0,
    CAMERA_ON
} CAMERA_STATE;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
void set_camera_main
(
    CAMERA_STATE state
);


void set_camera_drogue
(
    CAMERA_STATE state
);


#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/