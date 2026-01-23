/*******************************************************************************
*
* FILE: 
* 		wireless.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data wirelessly using the XBee and 
*       and LoRa modules 
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
#if defined( GROUND_STATION )
	#include "sdr_pin_defines_A0005.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#endif

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "wireless.h"


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rf_xbee_transmit_byte                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		transmits a byte wirelessly using the xbee module                      *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_transmit_byte 
	(
    uint8_t tx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Wait for Clear to send signal */
#ifdef GROUND_STATION
while ( HAL_GPIO_ReadPin( XBEE_CTS_GPIO_PORT, XBEE_CTS_PIN ) != GPIO_PIN_RESET )
	{
	}
#endif

/* Transmit byte */
hal_status = HAL_UART_Transmit( &( XBEE_HUART )  ,
                                &tx_byte         , 
                                sizeof( tx_byte ), 
                                RF_POLL_TIMEOUT );

/* Return HAL status */
if ( hal_status != HAL_OK )
	{
	return RF_ERROR;
	}
else
	{
	return RF_OK;
	}

} /* rf_xbee_transmit_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rf_xbee_transmit                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		transmits a buffer of bytes wirelessly using the xbee module           *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_transmit
	(
    void*  tx_buffer_ptr,   /* Pointer to buffer data    */
	size_t buffer_size	    /* Number of bytes in buffer */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */
uint32_t          timeout;       /* UART transmit timeout */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;
timeout    = RF_POLL_TIMEOUT*buffer_size;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
hal_status = HAL_UART_Transmit( &( XBEE_HUART ),
                                tx_buffer_ptr  , 
                                buffer_size    , 
                                timeout );

/* Return HAL status */
if ( hal_status != HAL_OK )
	{
	return RF_ERROR;
	}
else
	{
	return RF_OK;
	}

} /* rf_xbee_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rf_xbee_receieve_byte                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Receives a byte from the xbee module                                   *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_receive_byte 
	(
	uint8_t* p_rx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Set the Ready to Send Signal */
#ifdef GROUND_STATION
HAL_GPIO_WritePin( XBEE_RTS_GPIO_PORT, XBEE_RTS_PIN, GPIO_PIN_RESET );
#endif

/* Receive byte */
hal_status = HAL_UART_Receive( &( XBEE_HUART )  ,
                               p_rx_byte        , 
                               sizeof( uint8_t ), 
                               RF_POLL_TIMEOUT );

/* Reset the Read to Send signal */
#ifdef GROUND_STATION
HAL_GPIO_WritePin( XBEE_RTS_GPIO_PORT, XBEE_RTS_PIN, GPIO_PIN_SET );
#endif

/* Return HAL status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return RF_TIMEOUT;
		}
	case HAL_OK:
		{
		return RF_OK;
		}
	default:
		{
		return RF_ERROR;
        }
	}
} /* rf_xbee_receive_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rf_xbee_receive                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Receives data from the xbee module and outputs to a buffer             *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_receive
	(
	void*  rx_buffer_ptr,   /* Pointer to output data buffer */	
	size_t rx_buffer_size   /* Number of bytes to recevie    */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */
uint32_t          timeout;       /* UART receive timeout  */


/*------------------------------------------------------------------------------
 Initialization 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;
timeout    = rx_buffer_size*RF_POLL_TIMEOUT;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Receive data */
hal_status = HAL_UART_Receive( &( XBEE_HUART ),
                               rx_buffer_ptr  , 
                               timeout        , 
                               RF_POLL_TIMEOUT );

/* Return HAL status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return RF_TIMEOUT;
		}
	case HAL_OK:
		{
		return RF_OK;
		}
	default:
		{
		return RF_ERROR;
        }
	}
} /* rf_xbee_receive */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/