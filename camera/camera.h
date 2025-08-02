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

/* Camera selection */
typedef enum _CAMERA_SELECTION
    {
    CAMERA_MAIN,
    CAMERA_DROGUE
    } CAMERA_SELECTION;


/* Whether the camera is powered on or off */
typedef enum _CAMERA_STATE 
    {
    CAMERA_OFF = 0,
    CAMERA_ON
    } CAMERA_STATE;


/* Status response for error handling */
typedef enum _CAMERA_STATUS 
    {
    CAMERA_SUCCESS = 0,
    CAMERA_INVALID_SELECTION
    } CAMERA_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Turns the selected camera on or off */
CAMERA_STATUS set_camera_state
    (
    CAMERA_SELECTION camera, 
    CAMERA_STATE state
    );


#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/