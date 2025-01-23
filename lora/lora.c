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
    status = HAL_SPI_Receive( &(LORA_SPI), &read_buffer_ptr[0], 2, 2000 );

    if (status == HAL_OK){
        return LORA_OK;
    } else 
        return LORA_FAIL;
}

LORA_STATUS LORA_SPI_Transmit( LORA_REGISTER_ADDR reg, uint8_t data ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write (1 byte) and writes that register. */
    uint8_t transmitBuffer[2] = { reg, data };
    // uint8_t transmitBuffer = reg;
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer[0], 2, 2000);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}


LORA_STATUS lora_read_register( LORA_REGISTER_ADDR lora_register, uint8_t* regData) {
    LORA_STATUS transmit_status, receive_status;

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );
    
    transmit_status = LORA_SPI_Transmit( (lora_register & 0x7F), 0x00 ); // The problem starts here
    receive_status = LORA_SPI_Receive( &regData[0] );

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

    status = LORA_SPI_Transmit( (lora_register | 0x80), data );
    
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

    if ( status == LORA_OK )
        return LORA_OK;
    else return LORA_FAIL;
}

// Get the device chip ID
LORA_STATUS lora_get_device_id(uint8_t* buffer_ptr) {
    LORA_STATUS status;

    status = lora_read_register( LORA_REG_ID_VERSION, &buffer_ptr[0] );

    if ( status == LORA_OK )
        return LORA_OK;
    else return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Function to set the mode of the chip
------------------------------------------------------------------------------*/
// void lora_set_chip_mode( LORA_CHIPMODE chip_mode ) {
//     /* chip_mode should be one of the following 3-bit variables defined in lora.h
//             #define LORA_SLEEP_MODE            0b000
//             #define LORA_STANDBY_MODE          0b001
//             #define LORA_FREQ_SYNTH_TX_MODE    0b010
//             #define LORA_TRANSMIT_MODE         0b011
//             #define LORA_FREQ_SYNTH_RX_MODE    0b100
//             #define LORA_RX_CONTINUOUS_MODE    0b101
//             #define LORA_RX_SINGLE_MODE        0b111
//     */
//     // I may give this function a return type in the future to check success of operation.
//     // Also, may need to be updated based on pin definitions after those are fixed.

//     // Pull chip select low
//     lora_write_cs_pin( GPIO_PIN_SET );

//     // Get initial value of the operation mode register
//     uint8_t operation_mode_register = lora_spi_get_register( LORA_REG_OPERATION_MODE );

//     // Shift the chip mode bits to be the first 3 bits of the sequence
//     uint8_t shifted_chip_mode = (chip_mode << 0b00000);

//     // Change the value of the chip register to set it to the suggested chip mode
//     uint8_t new_opmode_register = (operation_mode_register | chip_mode);

//     // Write new bit
//     lora_spi_set_register( LORA_REG_OPERATION_MODE, new_opmode_register );LORA_REG_

//     // Bring chip select back high
//     lora_write_cs_pin( GPIO_PIN_RESET );
// }