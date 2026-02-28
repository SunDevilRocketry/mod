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
#include "main.h"
#include "common.h"
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/
static const IRQn_Type non_critical_irqs[] = { BARO_I2C_EV_IRQn, IMU_I2C_EV_IRQn, GPS_UART_IRQn }; /* LoRa eventually too */

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       disable_irq                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Disables IRQ interrupts that can be safely disabled                    *
*                                                                              *
*******************************************************************************/
void disable_irq
    (
    void
    ) 
{
for ( int i = 0; i < array_size( non_critical_irqs ); i++ )
    {
    HAL_NVIC_DisableIRQ( non_critical_irqs[i] );
    }

} /* disable_irq */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       enable_irq                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Enables IRQ interrupts that can be safely disabled                     *
*                                                                              *
*******************************************************************************/
void enable_irq
    (
    void
    ) 
{
for ( int i = 0; i < array_size( non_critical_irqs ); i++ )
    {
    HAL_NVIC_EnableIRQ( non_critical_irqs[i] );
    }

} /* enable_irq */


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
