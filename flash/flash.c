/*******************************************************************************
*
* FILE: 
* 		flash.c
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "usb.h"
#include "led.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
static bool write_enabled = false;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Converts a flash memory address in uint32_t format to a byte array */
static void address_to_bytes
	(
	uint32_t address,
	uint8_t* address_bytes
	);

/* Converts a flash memory address in byte format to uint32_t format */
static inline uint32_t bytes_to_address 
	(
	uint8_t address_bytes[3]
	);

/* Enable writing to the external flash chip hardware */
static FLASH_STATUS write_enable
    (
    void 
    );

/* Disable writing to the external flash chip hardware */
static FLASH_STATUS write_disable 
    (
    void 
    );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a flash subcommand based on input from the sdec terminal      *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_cmd_execute
	(
    uint8_t             subcommand   ,
	HFLASH_BUFFER*      pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint8_t          opcode;              /* Subcommand opcode                    */
uint8_t          num_bytes;           /* Number of bytes on which to 
                                         operate                              */
uint8_t          address[3];          /* flash address in byte form           */
uint8_t*         pbuffer;             /* Position within flash buffer         */
uint8_t          buffer[512];         /* buffer (flash extract)               */
FLASH_STATUS     flash_status;        /* Return value of flash API calls      */
USB_STATUS       usb_status;          /* Return value of USB API calls        */


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/
opcode    = ( subcommand & FLASH_SUBCMD_OP_BITMASK ) >>  5;
num_bytes = ( subcommand & FLASH_NBYTES_BITMASK    ); 
pflash_handle -> num_bytes = num_bytes;
pbuffer   = &buffer[0];
memset( pbuffer, 0, sizeof( buffer ) );
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
switch ( opcode )
	{
    /*-----------------------------READ Subcommand----------------------------*/
    case FLASH_SUBCMD_READ:
        {

		/* Get flash address from USB */
		usb_status = usb_receive( &( address[0] )  , 
                                  sizeof( address ), 
                                  HAL_DEFAULT_TIMEOUT );
		
		if ( usb_status != USB_OK )
			{
			return FLASH_USB_ERROR;
			}
		else
			{
			/* Call API Function */
			pflash_handle -> address = bytes_to_address( address );
			flash_status = flash_read( pflash_handle, num_bytes );
			
			/* Check for flash error */
			if ( flash_status != FLASH_OK )
				{
				/* Bytes not read */
				return FLASH_FAIL;
				}

			/* Transmit bytes from pbuffer over USB */
			usb_status = usb_transmit( pflash_handle -> pbuffer,
                                       num_bytes               ,
									   HAL_FLASH_TIMEOUT );

			if ( usb_status != USB_OK )
				{
				/* Bytes not transimitted */
				return FLASH_USB_ERROR;
				}

			}

		/* Bytes read and transimitted back sucessfully */
		return FLASH_OK;

		} /* FLASH_SUBCMD_READ */

    /*------------------------------ENABLE Subcommand-----------------------------*/
    case FLASH_SUBCMD_ENABLE:
        {
		flash_write_enable();
		return FLASH_OK;
        } /* FLASH_SUBCMD_ENABLE */

    /*------------------------------DISABLE Subcommand----------------------------*/
    case FLASH_SUBCMD_DISABLE:
        {
		flash_write_disable();
		return FLASH_OK;
        } /* FLASH_SUBCMD_DISABLE */

    /*------------------------------WRITE Subcommand------------------------------*/
    case FLASH_SUBCMD_WRITE:
        {
		/* Get Address bits */
		usb_status = usb_receive( &( address[0] )  ,
                                  sizeof( address ),
                                  HAL_DEFAULT_TIMEOUT );

		if ( usb_status != USB_OK )	
			{
			/* Address not recieved */
			return FLASH_USB_ERROR;
            }
		else
			{
			/* Convert flash address to uint32_t */
			pflash_handle -> address = bytes_to_address( address );

			/* Get bytes to be written to flash */
			for ( int i = 0; i < num_bytes; i++ )
				{
				pbuffer = ( pflash_handle -> pbuffer ) + i;
				flash_status = usb_receive( pbuffer          , 
                                            sizeof( uint8_t ),
                                            HAL_DEFAULT_TIMEOUT );

				/* Return if usb call failed */
				if ( usb_status != USB_OK )
					{
					/* Bytes not received */
				    return FLASH_USB_ERROR;	
                    }

				}
            }

		/* Call API function */
		flash_status = flash_write( pflash_handle );

	    return flash_status;	
        } /* FLASH_SUBCMD_WRITE */

    /*------------------------------ERASE Subcommand------------------------------*/
    case FLASH_SUBCMD_ERASE:
        {
		/* Call API Function*/
		flash_status = flash_erase( pflash_handle );

		return flash_status;
        } /* FLASH_SUBCMD_ERASE */

    /*------------------------------STATUS Subcommand-----------------------------*/
    case FLASH_SUBCMD_STATUS:
        {
		/* Call API function */
		flash_status = flash_get_status( pflash_handle );

		/* Send status register contents back to PC */
		usb_status = usb_transmit( &( pflash_handle -> status_register ),
                                   sizeof( uint8_t )                    ,
                                   HAL_DEFAULT_TIMEOUT );

		/* Return status code */
		if      ( usb_status   != USB_OK   )
			{
			return FLASH_USB_ERROR;
			}
		else if ( flash_status != FLASH_OK )
			{
			return FLASH_SPI_ERROR;
			}
		else
			{
			return FLASH_OK;	
			}
        } /* FLASH_SUBCMD_STATUS */

    /*-----------------------------EXTRACT Subcommand-----------------------------*/
    case FLASH_SUBCMD_EXTRACT:
        {
		/* Extracts the entire flash chip, flash chip address from 0 to 0x7FFFF */
		pflash_handle->pbuffer = &buffer[0];
		pflash_handle->address = 0;
		while( pflash_handle->address <= FLASH_MAX_ADDR )
			{
			flash_status = flash_read( pflash_handle, sizeof( buffer ) );
			if( flash_status == FLASH_OK )
				{
				usb_transmit( &buffer, sizeof( buffer ), HAL_FLASH_TIMEOUT );
				}
			else
				{
				/* Extract Failed */
				return FLASH_EXTRACT_ERROR;
				}

			/* Read from next address */
			(pflash_handle->address) += sizeof( buffer ) ;
			}

		return FLASH_OK;
        } /* FLASH_SUBCMD_EXTRACT */

    /*---------------------------Unrecognized Subcommand--------------------------*/
	default:
        {
	    return FLASH_UNRECOGNIZED_OP;	
        }

    }
} /* flash_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_init                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Initializes the flash chip                                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_init 
	(
	HFLASH_BUFFER* pflash_handle  /* Flash handle */
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS flash_status;    /* Flash API function return codes        */
uint8_t      status_register; /* Desired status register contents       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status    = FLASH_OK;
status_register = FLASH_REG_RESET_VAL;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check for invalid BPL setting */
if ( pflash_handle -> bpl_bits > FLASH_BPL_ALL )
	{
	return FLASH_INVALID_INPUT; 
	}

/* Determine the desired status register contents */
status_register &= pflash_handle -> bpl_bits;
status_register |= pflash_handle -> bpl_write_protect;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Configure write protection */
if ( pflash_handle -> write_protected )
	{
	flash_write_disable();
	return FLASH_OK;
	}
else
	{
	flash_write_enable();
	}

/* Check the flash chip status register to confirm chip can be reached */
flash_status = flash_get_status( pflash_handle );
if ( flash_status != FLASH_OK )
	{
	return flash_status;
	}
else if ( pflash_handle -> status_register == 0xFF )
	{
	return FLASH_INIT_FAIL;
	}

/* Disable writing to the chip in case of prior interrupted write operation */
if ( write_disable() != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_DISABLE;
	}

/* Set the bpl bits in the flash chip */
flash_status = flash_set_status( status_register );

/* Confirm Status register contents */
while( flash_is_flash_busy() == FLASH_BUSY ){}
flash_status = flash_get_status( pflash_handle );
if ( pflash_handle -> status_register != status_register )
	{
	return FLASH_INIT_FAIL;
	}
else
	{
	return flash_status;
	}

} /* flash_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_get_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the status register of the flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_get_status
	(
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[2];  /* Status codes returned by SPI HAL         */
uint8_t           flash_opcode;   /* Data to be transmitted over SPI          */

/*------------------------------------------------------------------------------
 Initialiazations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_RDSR;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select line low */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );

/* Send RDSR code to flash chip */
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
                                  &flash_opcode         ,
                                  sizeof( flash_opcode ),
                                  HAL_DEFAULT_TIMEOUT );

/* Recieve status code */
hal_status[1] = HAL_SPI_Receive( &( FLASH_SPI ),
                                 &( pflash_handle      -> status_register ),
                                 sizeof( pflash_handle -> status_register ),
							     HAL_DEFAULT_TIMEOUT );

/* Drive slave select line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Return flash status code */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_get_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_set_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Writes to the status register                                          *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_set_status
	(
	uint8_t        flash_status	
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status   [3];   /* Status code returned by hal spi functions      */
uint8_t flash_opcodes[2];   /* Flash instruction cycle bytes                  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcodes[0] = FLASH_OP_HW_EWSR;
flash_opcodes[1] = FLASH_OP_HW_WRSR;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/
if ( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Enable write to status register */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )   ,
                                  &flash_opcodes[0],
                                  sizeof( uint8_t ),
                                  HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Write the Data to the status register */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI )   ,
                                  &flash_opcodes[1],
                                  sizeof( uint8_t ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
                                  &flash_status         ,
                                  sizeof( flash_status ),
							      HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Return flash status */
if ( hal_status[0] != HAL_OK ||
     hal_status[1] != HAL_OK ||
	 hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_set_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_is_flash_busy                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Polls the flash status register to see if the chip is busy. False      *
*       boolean return value indicates the chip is ready for write operations  *
*                                                                              *
*******************************************************************************/
bool flash_is_flash_busy
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HFLASH_BUFFER flash_handle; /* Flash handle to store status reg contents      */
FLASH_STATUS  flash_status; /* Flash API return codes                         */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status                 = FLASH_OK;
flash_handle.status_register = 0xFF;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read status register */
flash_status = flash_get_status( &flash_handle );
if      ( flash_status != FLASH_OK                          )
	{
	return FLASH_BUSY;
	}
else if ( flash_handle.status_register & FLASH_BUSY_BITMASK )
	{
	return FLASH_BUSY;
	}
else
	{
	return FLASH_READY;
	}

} /* flash_is_flash_busy */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Enable writing to the external flash chip by configuring the global    *
*       write protection variable                                              *
*                                                                              *
*******************************************************************************/
void flash_write_enable
	(
	void
	)
{
write_enabled = true;
} /* flash_write_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Disable writing to the external flash chip                             *
*                                                                              *
*******************************************************************************/
void flash_write_disable
    (
    void 
    )
{
write_enabled = false;
} /* flash_write_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_byte                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       writes a byte to the external flash                                    *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write_byte 
    (
	HFLASH_BUFFER* pflash_handle,
	uint8_t        byte
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[3];    /* Status code return by hal functions    */
FLASH_STATUS      flash_status;     /* Status code returned by flash API      */
uint8_t           flash_opcode;     /* Data to be transmitted over SPI        */
uint8_t           address[3];       /* Flash memory address in byte form      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status  = FLASH_OK;
flash_opcode  = FLASH_OP_HW_BYTE_PROGRAM;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable the chip for writing */
flash_status = write_enable();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_ENABLE;
	}

/* Send the byte program command to the flash over SPI */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )       ,
							      &flash_opcode        ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );

/* Send address bytes */
hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI )   ,
							      &address[0]      ,
							      sizeof( address ),
							      HAL_DEFAULT_TIMEOUT );

/* Write bytes */
hal_status[2] = HAL_SPI_Transmit( &( FLASH_SPI ),
							     &byte          ,
							     sizeof( byte ) ,
							     HAL_FLASH_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return flash status */
if ( hal_status[0] != HAL_OK ||
     hal_status[1] != HAL_OK ||
	 hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_write_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       writes bytes from a flash buffer to the external flash                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write 
    (
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[3];    /* Status codes returned by HAL           */
FLASH_STATUS      flash_status;     /* Status codes returned by flash API     */
uint8_t           flash_opcode;     /* Opcode for flash instructions          */
uint32_t          timeout;          /* Timeout for flash write calls          */
uint32_t          timeout_ctr;      /* Counter to trigger timeout             */
uint8_t*          pbuffer;          /* Pointer to data in flash buffer        */
uint8_t           address_bytes[3]; /* Flash memory address in byte form      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode  = FLASH_OP_HW_AAI_PROGRAM; 
flash_status  = FLASH_OK;
timeout       = pflash_handle -> num_bytes; /* 1 ms/byte timeout */
timeout_ctr   = 0;
pbuffer       = pflash_handle -> pbuffer;
address_to_bytes( pflash_handle -> address, &address_bytes[0] );


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}

/* Check for 1/0 byte edge case */
if      ( pflash_handle -> num_bytes == 1 )
	{
	return flash_write_byte( pflash_handle, *( pflash_handle -> pbuffer ) );
	}
else if ( pflash_handle -> num_bytes == 0 )
	{
	return FLASH_ERROR_MISSING_DATA;
	}


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable the chip for writing */
flash_status = write_enable();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_ENABLE;
	}

/* Initial SPI transmission */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI )         ,
							      &address_bytes[0]      ,
							      sizeof( address_bytes ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( FLASH_SPI ), 
                                  pbuffer       ,
								  2             , 
								  HAL_DEFAULT_TIMEOUT );						
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Check for SPI errors */
if ( hal_status[0] != HAL_OK ||
     hal_status[1] != HAL_OK ||
	 hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}

/* Transmit remaining data */
for ( int i = 2; i < pflash_handle -> num_bytes; i += 2 )
	{
	/* Setup buffer pointer  */
	pbuffer += 2;

	/* Wait for flash to be ready */
	while( flash_is_flash_busy() == FLASH_BUSY )
		{
		if ( timeout_ctr >= timeout )
			{
			return FLASH_TIMEOUT;
			}
		else
			{
			timeout_ctr++;
			HAL_Delay( 1 );
			}
		}

	/* SPI Transmission */	
	HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
	hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
									&flash_opcode         ,
									sizeof( flash_opcode ),
									HAL_DEFAULT_TIMEOUT );
	hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI ), 
									pbuffer       ,
									2             , 
									HAL_DEFAULT_TIMEOUT );						
	HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

	/* Check for errors */
	if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
		{
		return FLASH_WRITE_ERROR;
		}
	} /* for ( i < pflash_handle -> num_bytes )*/

/* Wait for AAI to complete */
while ( flash_is_flash_busy() == FLASH_BUSY ){}

/* Terminate AAI Programming */
flash_status = write_disable();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_EXIT_AAI;
	}

/* Wait for WRDI to complete */
while ( flash_is_flash_busy() == FLASH_BUSY ){}

/* Transmit final byte if num_bytes is odd */
if ( ( pflash_handle -> num_bytes ) %2 == 1 )
	{
	pbuffer += 1;
	return flash_write_byte( pflash_handle, *pbuffer );
	}
else
	{
	/* Flash write successful */
	return FLASH_OK;
	}

} /* flash_write */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_read                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads a specified number of bytes using a flash buffer                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint32_t       num_bytes
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[4]; /* Status code return by hal spi functions   */
uint8_t           flash_opcode;  /* Data to be transmitted over SPI           */
uint8_t           address[3];    /* Flash address in byte format              */
uint8_t*          pbuffer;       /* Pointer to position in output buffer      */
uint8_t           dummy_byte;    /* Dummy byte needed for high speed read     */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_READ_HS;
dummy_byte   = 0;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Initiate SPI transmission */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );

/* Command opcode */
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );

/* Address */
hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI )   ,
							      &( address[0] )  ,
							      sizeof( address ),
							      HAL_DEFAULT_TIMEOUT );

/* Dummy Cycle */
hal_status[2] = HAL_SPI_Transmit( &( FLASH_SPI )   ,
							      &( dummy_byte )  ,
							      sizeof( dummy_byte ),
							      HAL_DEFAULT_TIMEOUT );

/* Check for SPI errors */
if ( hal_status[0] != HAL_OK || 
     hal_status[1] != HAL_OK || 
	 hal_status[2] != HAL_OK )
	{   
	return FLASH_SPI_ERROR;
	}

/* Recieve output into buffer*/
for ( int i = 0; i < num_bytes; ++i )
	{
	pbuffer = ( pflash_handle -> pbuffer ) + i;
	hal_status[3] = HAL_SPI_Receive( &( FLASH_SPI )      ,
								     pbuffer             ,
								     sizeof( uint8_t )   ,
								     HAL_DEFAULT_TIMEOUT );

	if ( hal_status[3] != HAL_OK )
		{
		return FLASH_SPI_ERROR;
		}
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Flash read successful */
return FLASH_OK;

} /* flash_read */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_erase                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       erases the entire flash chip                                           *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
int8_t       hal_status;    /* Status code return by hal spi functions        */
uint8_t      flash_opcode;  /* Data to be transmitted over SPI                */
FLASH_STATUS flash_status;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_FULL_ERASE;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
    {
    return FLASH_WRITE_PROTECTED;
    }


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable writing to flash */
flash_status = write_enable();

/* Full chip erase */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( FLASH_SPI )        ,
							   &flash_opcode         ,
							   sizeof( flash_opcode ),
							   HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return status code */
if      ( hal_status   != HAL_OK   )
	{
	return FLASH_SPI_ERROR;
	}
else if ( flash_status != FLASH_OK )
	{
	return FLASH_FAIL;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_erase */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_block_erase                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Erase a block of flash                                                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_block_erase
	(
	FLASH_BLOCK      flash_block_num, /* Block of flash to erase */
	FLASH_BLOCK_SIZE size             /* Size of block           */
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
int8_t       hal_status[2];       /* Status code return by hal spi functions  */
uint8_t      flash_opcode;        /* Data to be transmitted over SPI          */
FLASH_STATUS flash_status;        /* Return codes from flash API              */
uint32_t     flash_addr;          /* Address of block to erase                */
uint8_t      flash_addr_bytes[3]; /* Address of block to erase in byte form   */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status = FLASH_OK;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Determine block setting */
switch( size )
	{
	case FLASH_BLOCK_4K:
		{
		flash_opcode = FLASH_OP_HW_4K_ERASE;
		flash_addr   = flash_block_num*(0x1000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );
		break;
		}

	case FLASH_BLOCK_32K:
		{
		flash_opcode = FLASH_OP_HW_32K_ERASE;
		flash_addr   = flash_block_num*(0x8000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );
		break;
		}

	case FLASH_BLOCK_64K:
		{
		flash_opcode = FLASH_OP_HW_64K_ERASE;
		flash_addr   = flash_block_num*(0x10000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );

		/* Error check */
		if ( flash_block_num >= FLASH_BLOCK_8 )
			{
			return FLASH_ADDR_OUT_OF_BOUNDS;
			}
		break;
		}
	}

/* Check if write_enabled */
if( !( write_enabled ) )
    {
    return FLASH_WRITE_PROTECTED;
    }


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable writing to flash */
flash_status = write_enable();

/* Sector erase sequence */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( FLASH_SPI )        ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( FLASH_SPI )            , 
                                  &flash_addr_bytes[0]      ,
								  sizeof( flash_addr_bytes ), 
								  HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return status code */
if      ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else if ( flash_status != FLASH_OK )
	{
	return FLASH_FAIL;
	}
else
	{
	return FLASH_OK;
	}
} /* flash_block_erase */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		address_to_bytes                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in uint32_t format to a byte array     *
*                                                                              *
*******************************************************************************/
static void address_to_bytes
	(
	uint32_t address,
	uint8_t* address_bytes
	)
{
address_bytes[0] = (address >> 16) & 0xFF;
address_bytes[1] = (address >> 8 ) & 0xFF;
address_bytes[2] =  address        & 0xFF;
} /* address_to_bytes */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		bytes_to_address                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in byte format to uint32_t format      *
*                                                                              *
*******************************************************************************/
static inline uint32_t bytes_to_address 
	(
	uint8_t address_bytes[3]
	)
{
return ( (uint32_t) address_bytes[0] << 16 ) |
	   ( (uint32_t) address_bytes[1] << 8  ) |
	   ( (uint32_t) address_bytes[2] << 0  );
} /* address_to_bytes */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_enable                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable writing to the external flash chip hardware                     *
*                                                                              *
*******************************************************************************/
static FLASH_STATUS write_enable
    (
    void 
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;        /* Status code return by hal spi functions         */
uint8_t flash_opcode;      /* Flash operation/instruction cyle byte           */


/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
flash_opcode  = FLASH_OP_HW_WREN;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Send the write enable instruction to the flash over SPI */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( FLASH_SPI )        ,
                               &flash_opcode         ,
                               sizeof( flash_opcode ),
                               HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Set write enabled bit in flash buffer handle */
if ( hal_status != HAL_OK )
	{
	return FLASH_SPI_ERROR;
    }
else
	{
	return FLASH_OK;
    }

} /* write_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_disable                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable writing to the external flash chip hardware                    *
*                                                                              *
*******************************************************************************/
static FLASH_STATUS write_disable 
    (
    void 
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;        /* Status code return by hal spi functions         */
uint8_t flash_opcode;      /* Flash operation/instruction cyle byte           */


/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
flash_opcode  = FLASH_OP_HW_WRDI;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Send the write enable instruction to the flash over SPI */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( FLASH_SPI )        ,
                               &flash_opcode         ,
                               sizeof( flash_opcode ),
                               HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Return results of SPI HAL calls */
if ( hal_status != HAL_OK )
	{
	return FLASH_SPI_ERROR;
    }
else
	{
	return FLASH_OK;
    }

} /* write_disable */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/