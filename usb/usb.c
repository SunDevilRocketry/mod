/*******************************************************************************
*
* FILE: 
* 		usb.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER     )
	#include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION       )
	#include "sdr_pin_defines_A0005.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"


/*------------------------------------------------------------------------------
 Preprocesor Directives 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		usb_transmit_bytes                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over USB                         *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Transmit( &( USB_HUART ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( usb_status != HAL_OK )
	{
	return usb_status;
	}
else
	{
	return USB_OK;
	}

} /* usb_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_receieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Receive( &( USB_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );

/* Return HAL status */
switch ( usb_status )
	{
	case HAL_TIMEOUT:
		{
		return USB_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return USB_OK;
		break;
		}
	default:
		{
		return USB_FAIL;
		break;
        }
	}

} /* usb_receive */


#if defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
	defined( L0005_REV3           )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_detect                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Detect a USB connection by checking the power on the USB 5V line       *
*                                                                              *
*******************************************************************************/
bool usb_detect
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t usb_detect_pinstate;    /* USB detect state, return value from HAL    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
usb_detect_pinstate = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Read voltage on usb detect pin */
usb_detect_pinstate = HAL_GPIO_ReadPin( USB_DETECT_GPIO_PORT, USB_DETECT_PIN );

/* Set return value */
if ( usb_detect_pinstate == 0 )
	{
	return false;
	}
else
	{
	return true;
	}
} /* usb_detect */
#endif /* #if defined( A0002_REV2 ) || defined( FLIGHT_COMPUTER_LITE ) */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/