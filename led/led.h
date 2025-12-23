/*******************************************************************************
*
* FILE: 
* 		led.h
*
* DESCRIPTION: 
* 		Contains API functions to set the behavior of the on-board rgb led
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
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* LED Color Codes */
typedef enum LED_COLOR_CODES
	{
	LED_GREEN = 1,
    LED_RED      ,
    LED_BLUE     ,
    LED_CYAN     ,
    LED_PURPLE   ,
    LED_YELLOW   ,
    LED_WHITE
	} LED_COLOR_CODES;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Display Red to indicate software exception */
void led_error_assert
	(
    void
    );

/* Reset the led */
void led_reset
	(
    void
    );

/* Flash Red to indicate that the code hit a block of code not meant to be run
   without blocking the program from running  */
void led_error_flash
	(
    void
    );

/* Sets the LED to a color from the LED_COLOR_CODES enum */
void led_set_color
	(
	LED_COLOR_CODES color
	);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/