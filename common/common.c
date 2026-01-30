/*******************************************************************************
*
* FILE: 
* 		common.c
*
* DESCRIPTION: 
* 		Contains utility functions for SDR code.
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
#include "cmsis_gcc.h"
#include "main.h"
#include "common.h"
#include "stm32h7xx_hal.h"


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
* 		disable_irq()                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Disables IRQ interrupts                                        *
* 		Wrapper for __disable_irq(); see cmsis_gcc.h                   *
*                                                                              *
*******************************************************************************/
void disable_irq
    (
    void
    ) 
{
        __disable_irq();
} /* disable_irq */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		enable_irq()                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Enables IRQ interrupts                                         *
* 		Wrapper for __enable_irq(); see cmsis_gcc.h                    *
*                                                                              *
*******************************************************************************/
void enable_irq
    (
    void
    ) 
{
        __enable_irq();
} /* enable_irq */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crc32                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns a 32bit checksum from the given data.                          *
*                                                                              *
*******************************************************************************/
uint32_t crc32
    (
    const uint8_t *data, 
    size_t len
    ) 
{
uint32_t crc = 0xFFFFFFFF;
while (len--) 
    {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i)
        crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
return ~crc;

} /* crc32 */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_fail_fast                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		In case of error occurrence, this function passes the error            *
*       code to the error handler                                              *
*                                                                              *
*******************************************************************************/
void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
Error_Handler(error_code);

} /* error_fail_fast */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		delay_ms                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Minimum delay in miliseconds                                           *
*                                                                              *
*******************************************************************************/
void delay_ms
    (
    uint32_t delay
    )
{
HAL_Delay(delay);

} /* delay_ms */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
