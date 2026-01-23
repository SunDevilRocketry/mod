/*******************************************************************************
*
* FILE: 
* 		power.c
*
* DESCRIPTION: 
* 		Contains API functions to manage the engine controller power supply
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
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_L0002.h"
#include "power.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pwr_get_source                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Reads the 5V power multiplexor source pin to determine if the MCU is   *
*       being powered by USB or by the buck converter                          *
*                                                                              *
*******************************************************************************/
PWR_SRC pwr_get_source
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint8_t pwr_source; /* USB or buck converter power supply */


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Read the MCU pin */
pwr_source = HAL_GPIO_ReadPin(PWR_SRC_GPIO_PORT, PWR_SRC_PIN);

/* Return the corresponding code */
if (pwr_source == GPIO_PIN_RESET)
	{
	/* Buck converter 5V source */
	return BUCK_5V_SRC; 
    }
else
	{
	/* USB 5V source */
	return USB_5V_SRC;
	}

}



/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
