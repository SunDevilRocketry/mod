/*******************************************************************************
*
* FILE: 
* 		buzzer.c
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer buzzer 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "sdr_pin_defines_A0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#else
	#error No buzzer compatible device specified
#endif
#include "buzzer.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       buzzer_beep                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Beep the flight computer buzzer                                        *
*                                                                              *
*******************************************************************************/
BUZZ_STATUS buzzer_beep 
	(
	uint32_t duration /* Length of beep in milliseconds */
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return codes from HAL API */

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Start generating PWM pulses */
hal_status = HAL_TIM_PWM_Start( &(BUZZ_TIM), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}

/* Wait for beep duration */
HAL_Delay( duration );

/* Stop the PWM pulses */
hal_status = HAL_TIM_PWM_Stop( &( BUZZ_TIM ), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}
else
	{
	return BUZZ_OK;
	}

} /* buzzer_beep */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       buzzer_num_beeps                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Beep the flight computer buzzer specified number of times              *
*                                                                              *
*******************************************************************************/
BUZZ_STATUS buzzer_num_beeps
	(
	uint8_t num_beeps /* Number of beeps */
	)
{
/* This function is implemented as a non-blocking function that uses the 
   sysClock to detemine when to take an action. The function is meant to be 
   used to loops with processing times less than the beep and intermittent 
   durations. The static variable num_calls is used to maintain memory of 
   how many times the function has been called. The static variables 
   init_beep_time and stop_beep_time are used to keep track of the time of 
   starting and stopping a beep between subsequenct function calls. The static 
   variable last_end_time is used to add a delay between subsequent sequences 
   of beeps so the beep count can be discerned */
static uint8_t   num_calls      = 0;
static uint32_t  init_beep_time = 0;
static uint32_t  stop_beep_time = 0;
static uint32_t  last_end_time  = 0;

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;      /* Return codes from HAL API      */
uint8_t           total_num_calls; /* Number of times the function must be 
                                      called to beep num_beeps times */
uint8_t           is_odd;          /* 1 when num_calls is odd        */
uint32_t          current_tick;    /* Current Sys tick               */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status      = HAL_OK;
total_num_calls = 2*num_beeps;
is_odd          = num_calls%2;
current_tick    = HAL_GetTick();


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Check that enough time has elapsed since last beep sequence */
if ( ( current_tick - last_end_time ) <= BUZZ_SEQUENCE_DELAY )
	{
	return BUZZ_OK;
	}

/* Check for 0 beeps */
if ( num_beeps == 0 )
	{
	return BUZZ_OK;
	}

/* Remaining beeps */
if ( is_odd )
	{
	/* Stop Beep */
	if ( ( ( current_tick - init_beep_time ) >=  BUZZ_BEEP_DURATION ) )
		{
		hal_status = HAL_TIM_PWM_Stop( &( BUZZ_TIM ), BUZZ_TIM_CHANNEL );
		if ( hal_status != HAL_OK )
			{
			return BUZZ_HAL_ERROR;
			}
		else
			{
			stop_beep_time = current_tick;
			num_calls++;
			/* Termination condition */
			if ( num_calls == total_num_calls )
				{
				num_calls = 0;
				last_end_time = current_tick;
				}
			return BUZZ_OK;
			}
		}
	else
		{
		return BUZZ_OK;
		}
	
	}
else
	{
	/* Start beep */
	if ( ( ( current_tick - stop_beep_time ) >= BUZZ_STOP_DURATION ) )
		{
		hal_status = HAL_TIM_PWM_Start( &(BUZZ_TIM), BUZZ_TIM_CHANNEL );
		if ( hal_status != HAL_OK )
			{
			return BUZZ_HAL_ERROR;
			}
		else
			{
			init_beep_time = current_tick;
			num_calls++;
			return BUZZ_OK;
			}
		}
	else
		{
		return BUZZ_OK;
		}
	}
} /* buzzer_num_beeps */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/