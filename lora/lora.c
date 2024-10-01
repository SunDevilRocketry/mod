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

#include "stm32h7xx_hal.h"
/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/
#include <stdbool.h>


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
#include lora.h

/*------------------------------------------------------------------------------
 Helper functions for various pin functions on the LoRa modem.
------------------------------------------------------------------------------*/
//TODO: Switch pinState to a good enum
void lora_write_cs_pin( bool pinState ) {
    /* Takes either GPIO_PIN_SET or GPIO_PIN_RESET to bring the chip select pin
       high or low.*/
    HAL_GPIO_WritePin( LORA_NSS_PORT, pinState );
}

void lora_spi_receive( uint8_t *read_buffer[2] ) {
    /* Takes pointer to the read buffer. and puts output there */
    read_buffer = HAL_SPI_Receive( &(hspi4), &(read_buffer), 2, HAL_DEFAULT_TIMEOUT  );
}

void lora_spi_transmit( LORA_REGISTER_ADDR register, uint8_t data ) {
    // TODO: Actually make this return something
    /* Takes register and data to write (1 byte) and writes that register. */
    uint8_t transmitBuffer[2] = { register, data };
    HAL_SPI_Transmit( &(hspi4),  &transmitBuffer, 2, HAL_DEFAULT_TIMEOUT);
}

uint8_t lora_spi_get_register( LORA_REGISTER_ADDR lora_register ) {
    uint8_t read_buffer[2];

    lora_spi_transmit( (lora_register | 0b00000000), 0b00000000 );

    lora_spi_receive( &read_buffer );

    return read_buffer[2];
}

void lora_spi_set_register( LORA_REGISTER_ADDR lora_register, uint8_t data ) {
    lora_spi_transmit( (lora_register | 0b10000000), data );
}

/*------------------------------------------------------------------------------
 Function to set the mode of the chip
------------------------------------------------------------------------------*/

void set_lora_chip_mode( LORA_CHIPMODE chip_mode ) {
    /* chip_mode should be one of the following 3-bit variables defined in lora.h
            #define LORA_SLEEP_MODE            0b000
            #define LORA_STANDBY_MODE          0b001
            #define LORA_FREQ_SYNTH_TX_MODE    0b010
            #define LORA_TRANSMIT_MODE         0b011
            #define LORA_FREQ_SYNTH_RX_MODE    0b100
            #define LORA_RX_CONTINUOUS_MODE    0b101
            #define LORA_RX_SINGLE_MODE        0b111
    */
    // I may give this function a return type in the future to check success of operation.
    // Also, may need to be updated based on pin definitions after those are fixed.

    // Pull chip select low
    lora_write_cs_pin( GPIO_PIN_SET );

    // Get initial value of the operation mode register
    uint8_t operation_mode_register = lora_spi_get_register( &(LORA_REG_OPERATION_MODE) );

    // Shift the chip mode bits to be the first 3 bits of the sequence
    uint8_t shifted_chip_mode = (chip_mode << 0b00000);

    // Change the value of the chip register to set it to the suggested chip mode
    uint8_t new_opmode_register = (operation_mode_register | chip_mode);

    // Write new bit
    lora_spi_set_register( LORA_REG_OPERATION_MODE, new_opmode_register );

    // Bring chip select back high
    lora_write_cs_pin( GPIO_PIN_RESET );
}