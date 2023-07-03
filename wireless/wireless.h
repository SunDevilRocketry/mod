/*******************************************************************************
*
* FILE: 
* 		wireless.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data wirelessly using the XBee and 
*       and LoRa modules 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WIRELESS_H
#define WIRELESS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum RF_STATUS
	{
	RF_OK = 0,
    RF_ERROR ,
	RF_TIMEOUT
	} RF_STATUS;

/* Wireless Module Codes */
typedef enum WIRELESS_MOD_CODES
	{
	LORA = 0,
    XBEE
	} WIRELESS_MOD_CODES;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* HAL Settings*/
#define RF_POLL_TIMEOUT		( 100 )


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/


/* Transmits a byte wirelessly using the xbee module */
RF_STATUS rf_xbee_transmit_byte 
	(
    uint8_t tx_byte	
	);


/* Receives a byte from the xbee module */
RF_STATUS rf_xbee_receive_byte 
	(
	uint8_t* p_rx_byte	
	);

/* Transmits a buffer of bytes wirelessly using the xbee module */
RF_STATUS rf_xbee_transmit
	(
    void*  tx_buffer_ptr,   /* Pointer to buffer data    */
	size_t buffer_size	    /* Number of bytes in buffer */
	);

/* Receives data from the xbee module and outputs to a buffer */
RF_STATUS rf_xbee_receive
	(
	void*  rx_buffer_ptr,   /* Pointer to output data buffer */	
	size_t rx_buffer_size   /* Number of bytes to recevie    */
	);


#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/