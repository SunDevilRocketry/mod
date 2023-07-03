/*******************************************************************************
*
* FILE: 
* 		rs485.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over RS485 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RS485_H 
#define RS485_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum RS485_STATUS 
	{
	RS485_OK = 0,
    RS485_ERROR,
	RS485_TIMEOUT	
	} RS485_STATUS;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Timeouts */
#define RS485_POLL_TIMEOUT		( 1000 )
#ifndef SDR_DEBUG
	#define RS485_DEFAULT_TIMEOUT  ( 20  ) /* Default RS485 Timeout           */
#else
	/* Disable timeouts when debugging */
	#define RS485_DEFAULT_TIMEOUT  ( 0xFFFFFFFF )
#endif /* SDR_DEBUG */


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* transmits a byte over RS485 */
RS485_STATUS rs485_transmit_byte 
	(
    uint8_t tx_byte	
	);

/* transmits a buffer of bytes over RS485 */
RS485_STATUS rs485_transmit
	(
    void*    tx_buffer_ptr,   /* Pointer to buffer data    */
	size_t   buffer_size  ,   /* Number of bytes in buffer */
	uint32_t timeout          /* Timeout in ms             */
	);

/* Receives a byte from the RS485 interface */
RS485_STATUS rs485_receive_byte 
	(
	uint8_t* p_rx_byte	
	);


/* Receives data from the RS485 interface and outputs to a buffer */
RS485_STATUS rs485_receive
	(
	void*    rx_buffer_ptr,   /* Pointer to output data buffer */	
	size_t   rx_buffer_size,  /* Number of bytes to recevie    */
	uint32_t timeout          /* Timeout in ms                 */
	);

/* Receives data from the RS485 interface in interrupt mode and outputs to 
   a buffer */
RS485_STATUS rs485_receive_IT
	(
	void*    rx_buffer_ptr,   /* Pointer to output data buffer */	
	size_t   rx_buffer_size   /* Number of bytes to recevie    */
	);


#ifdef __cplusplus
}
#endif

#endif /* RS485_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/