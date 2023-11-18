/*******************************************************************************
*
* FILE: 
* 		gps.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over GPS 
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
#include "gps.h"


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
* 		gps_transmit_bytes                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over GPS                         *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Transmit( &( GPS_HUART ),
                                tx_data_ptr   , 
                                tx_data_size , 
                                timeout );

/* Return HAL status */
if ( gps_status != HAL_OK )
	{
	return gps_status;
	}
else
	{
	return GPS_OK;
	}

} /* gps_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_receieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the GPS port                                       *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Receive( &( GPS_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );
                               
/* Return HAL status */
switch ( gps_status )
	{
	case HAL_TIMEOUT:
		{
		return GPS_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return GPS_OK;
		break;
		}
	default:
		{
		return GPS_FAIL;
		break;
        }
	}

} /* gps_receive */


#if defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
	defined( L0005_REV3           )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_detect                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Detect a GPS connection by checking the power on the GPS 5V line       *
*                                                                              *
*******************************************************************************/
bool gps_detect
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t gps_detect_pinstate;    /* GPS detect state, return value from HAL    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
gps_detect_pinstate = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Read voltage on gps detect pin */
gps_detect_pinstate = HAL_GPIO_ReadPin( GPS_DETECT_GPIO_PORT, GPS_DETECT_PIN );

/* Set return value */
if ( gps_detect_pinstate == 0 )
	{
	return false;
	}
else
	{
	return true;
	}
} /* gps_detect */
#endif /* #if defined( A0002_REV2 ) || defined( FLIGHT_COMPUTER_LITE ) */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/