/*******************************************************************************
*
* FILE: 
* 		buzzer.h
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer buzzer 
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