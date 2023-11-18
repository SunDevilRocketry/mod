/*******************************************************************************
*
* FILE: 
* 		gps.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over GPS 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum GPS_STATUS
	{
	GPS_OK = 0,
    GPS_FAIL  ,
	GPS_TIMEOUT
	} GPS_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* transmits bytes over GPS */
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	);

/* Receives bytes from the GPS port */
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	);

/* Checks for an active GPS connection */
#if defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
	defined( L0005_REV3 )
bool gps_detect
	(
	void
	);
#endif /* #if defined( A0002_REV2 ) || defined( FLIGHT_COMPUTER_LITE ) */

