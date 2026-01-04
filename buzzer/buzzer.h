/*******************************************************************************
*
* FILE: 
* 		buzzer.h
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer buzzer 
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUZZER_H 
#define BUZZER_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Buzzer beep durations */
#define BUZZ_BEEP_DURATION     ( 50  )
#define BUZZ_STOP_DURATION     ( 75  )
#define BUZZ_SEQUENCE_DELAY    ( 3000 )


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Buzzer API return codes */
typedef enum _BUZZ_STATUS
	{
	BUZZ_OK    = 0,
	BUZZ_HAL_ERROR,
	BUZZ_FAIL
	} BUZZ_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Beep the flight computer buzzer */
BUZZ_STATUS buzzer_beep
	(
	uint32_t duration /* Length of beep in milliseconds */
	);

/* Beep the flight computer buzzer a specified number of times (blocking) */
BUZZ_STATUS buzzer_multi_beeps
	(
	uint32_t beep_duration, 		/* Length of beep in milliseconds */
	uint32_t time_between_beeps,	/* How long to wait between beeps in ms */
	uint8_t	 num_beeps 				/* How many times to repeat */
	);

/* Beep the flight computer buzzer specified number of times */
BUZZ_STATUS buzzer_num_beeps
	(
	uint8_t num_beeps /* Number of beeps */
	);

#ifdef __cplusplus
}
#endif
#endif /* BUZZER_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/