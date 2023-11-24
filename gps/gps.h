/*******************************************************************************
*
* FILE: 
* 		gps.h
*
* DESCRIPTION: 
* 		Contains API functions for gps
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

/* transmits bytes over USB */
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	);

/* Receives bytes from the USB port */
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	);

GPS_STATUS gps_receive_IT
	(
	uint8_t*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size /* Size of the data to be received */
	);

#ifdef __cplusplus
}
#endif

#endif /* USB_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/