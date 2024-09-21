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
 Function to set the mode of the chip
------------------------------------------------------------------------------*/

void set_chip_mode( uint8_t chip_mode ) {
    // I may give this function a return type in the future to check success of operation.
    // Also, may need to be updated based on pin definitions after those are fixed.

    // Pull chip select low
    HAL_GPIO_WritePin( LORA_SS_GPIO_PORT, LOSS_SS_GPIO_PIN, GPIO_PIN_RESET );

    // Get initial value of the operation mode register
    HAL_StatusTypeDef operation_mode_register = HAL_SPI_Receive( &(LORA_SPI), &(LORA_REG_OPERATION_MODE), 1, HAL_DEFAULT_TIMEOUT  );

    // Shift the chip mode bits to be the first 3 bits of the sequence
    uint8_t shifted_chip_mode = (chip_mode << 0b00000);

    // Change the value of the chip register to set it to the suggested chip mode
    uint8_t new_opmode_register = (operation_mode_register | chip_mode);

    // Write new bit
    HAL_SPI_Transmit( &(LORA_SPI),  &new_opmode_register, 1, HAL_DEFAULT_TIMEOUT);

    // Bring chip select back high
    HAL_GPIO_WritePin( LORA_SS_GPIO_PORT, LOSS_SS_GPIO_PIN, GPIO_PIN_SET );
}