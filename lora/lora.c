/*******************************************************************************
*
* FILE: 
* 		lora.c
*
* DESCRIPTION: 
* 		Contains API functions for transmating and receiving from the board's
*       built-in LoRa module.
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( GROUND_STATION    )
    #include "sdr_pin_defines_A0005.h"
#endif

/*------------------------------------------------------------------------------
 Project Inlcudes
------------------------------------------------------------------------------*/
#include "lora.h"
#include "main.h"

/*------------------------------------------------------------------------------
 Helper functions for various pin functions on the LoRa modem.
------------------------------------------------------------------------------*/
LORA_STATUS LORA_SPI_Receive( uint8_t* read_buffer_ptr ) {
    HAL_StatusTypeDef status;

    /* Takes pointer to the read buffer. and puts output there */
    status = HAL_SPI_Receive( &(LORA_SPI), read_buffer_ptr, 1, 2000 );

    if (status == HAL_OK){
        return LORA_OK;
    } else 
        return LORA_FAIL;
}

LORA_STATUS LORA_SPI_Transmit_Single( LORA_REGISTER_ADDR reg ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write (1 byte) and writes that register. */
    // uint8_t transmitBuffer[2] = { reg, data };
    uint8_t transmitBuffer = reg;
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer, 1, 2000);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

LORA_STATUS LORA_SPI_Transmit_Double( LORA_REGISTER_ADDR reg, uint8_t data ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write (1 byte) and writes that register. */
    uint8_t transmitBuffer[2] = { reg, data };
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer, 1, 2000);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

LORA_STATUS lora_read_register( LORA_REGISTER_ADDR lora_register, uint8_t* pRegData) {
    LORA_STATUS transmit_status, receive_status;

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );
    
    transmit_status = LORA_SPI_Transmit_Single( (lora_register & 0x7F) );
    receive_status = LORA_SPI_Receive( pRegData );

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

    if (transmit_status + receive_status == 0){
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}


LORA_STATUS lora_write_register( LORA_REGISTER_ADDR lora_register, uint8_t data ) {
    LORA_STATUS status;

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

    status = LORA_SPI_Transmit_Double( (lora_register | 0x80), data );
    
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

    if ( status == LORA_OK )
        return LORA_OK;
    else return LORA_FAIL;
}

// Get the device chip ID
LORA_STATUS lora_get_device_id(uint8_t* buffer_ptr) {
    LORA_STATUS status;

    status = lora_read_register( LORA_REG_ID_VERSION, buffer_ptr );

    if ( status == LORA_OK )
        return LORA_OK;
    else return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Function to set the mode of the chip
------------------------------------------------------------------------------*/
LORA_STATUS lora_set_chip_mode( LORA_CHIPMODE chip_mode ) {
    /* chip_mode should be one of the following 3-bit variables defined in lora.h
            #define LORA_SLEEP_MODE            0b000
            #define LORA_STANDBY_MODE          0b001
            #define LORA_FREQ_SYNTH_TX_MODE    0b010
            #define LORA_TRANSMIT_MODE         0b011
            #define LORA_FREQ_SYNTH_RX_MODE    0b100
            #define LORA_RX_CONTINUOUS_MODE    0b101            #define LORA_RX_SINGLE_MODE        0b111
    */
    // I may give this function a return type in the future to check success of operation.

    // Get initial value of the operation mode register
    uint8_t operation_mode_register;
    LORA_STATUS read_status = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );
    if( read_status & 0b00000001 != 0b00000001 ) { // Make function automatically fail if chip is not in LoRa mode
        return LORA_FAIL;
    }

    // Shift the chip mode bits to be the first 3 bits of the sequence
    uint8_t shifted_chip_mode = (chip_mode << 0b00000);

    // Change the value of the chip register to set it to the suggested chip mode
    uint8_t new_opmode_register = (operation_mode_register | shifted_chip_mode);

    // Write new byte
    LORA_STATUS write_status = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

    if ( write_status + read_status == 0){
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}

LORA_STATUS lora_init() {
    LORA_STATUS set_sleep_status = lora_set_chip_mode( LORA_SLEEP_MODE ); // Switch to sleep mode to enable LoRa bit (datasheeet page 102)
    // Get initial value of the operation mode register
    uint8_t operation_mode_register;
    LORA_STATUS read_status = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );
    uint8_t new_opmode_register = ( operation_mode_register | 0b00000001 ); // Toggle the LoRa bit
    // Write new byte
    LORA_STATUS write_status = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

    LORA_STATUS standby_status = lora_set_chip_mode( LORA_STANDBY_MODE ); // Switch it into standby mode, which is what's convenient.

    if( set_sleep_status + read_status + write_status + standby_status == 0 ) {
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}
/*
LORA_STATUS lora_transmit( uint8_t data ) {
    LORA_STATUS data_write = lora_write_register( LORA_REG_FIFO_RW, 255 );

    LORA_STATUS chip_mode_status = lora_set_chip_mode( LORA_TRANSMIT_MODE );

    uint8_t transmit_status = 0b00000000; // This variable holds the IRQ register, which is checked for transmit done flag
    while( ( transmit_status & 0b00010000 ) != 0b00010000 ) { // We use this bitmask to check the 4th bit to see if transmission is done
        lora_read_register( LORA_REG_LORA_STATE_FLAGS, &transmit_status );
    }

    if ( data_write + chip_mode_status == 0){
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}
*/