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
    status = HAL_SPI_Receive( &(LORA_SPI), read_buffer_ptr, 1, LORA_TIMEOUT );

    if (status == HAL_OK){
        return LORA_OK;
    } else 
        return LORA_FAIL;
}

LORA_STATUS LORA_SPI_Transmit_Byte( LORA_REGISTER_ADDR reg ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write (1 byte) and writes that register. */
    uint8_t transmitBuffer = reg;
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer, 1, LORA_TIMEOUT);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

LORA_STATUS LORA_SPI_Transmit_Data( LORA_REGISTER_ADDR reg, uint8_t data ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write and writes that register. */
    uint8_t transmitBuffer[2] = { reg, data };
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer, 1, LORA_TIMEOUT);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

LORA_STATUS lora_read_register( LORA_REGISTER_ADDR lora_register, uint8_t* pRegData) {
    LORA_STATUS transmit_status, receive_status;

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );
    
    transmit_status = LORA_SPI_Transmit_Byte( (lora_register & 0x7F) );
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

    status = LORA_SPI_Transmit_Data( (lora_register | 0x80), data );
    
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
    // Get initial value of the operation mode register
    uint8_t operation_mode_register;
    LORA_STATUS read_status = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

    if (read_status != LORA_OK)
    {
        return LORA_FAIL;
    }

    // Fail if not in LORA Mode 
    if ( !( operation_mode_register & (1<<7) ) ){
        return LORA_FAIL;
    }

    // Change the value of the chip register to set it to the suggested chip mode
    uint8_t new_opmode_register = (operation_mode_register | chip_mode);

    // Write new byte
    LORA_STATUS write_status = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

    if ( write_status + read_status == 0){
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}

LORA_STATUS lora_init( LORA_CONFIG *lora_config_ptr ) {
    LORA_STATUS set_sleep_status = lora_set_chip_mode( LORA_SLEEP_MODE ); // Switch to sleep mode to enable LoRa bit (datasheeet page 102)
    // Get initial value of the operation mode register
    uint8_t operation_mode_register;
    LORA_STATUS read_status1 = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

    uint8_t new_opmode_register;
    new_opmode_register = ( operation_mode_register | 0b00000001 ); // Toggle the LoRa bit

    // Write new byte
    LORA_STATUS write_status1 = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

    // Get initial value of config register 2
    uint8_t modem_config2_register;
    LORA_STATUS read_status2 = lora_read_register( LORA_REG_RX_HEADER_INFO, &modem_config2_register );

    uint8_t new_config2_register = modem_config2_register & 0x0F; // Erase spread factor bits
    new_config2_register = ( modem_config2_register | ( lora_config_ptr->lora_spread << 4 ) ); // Set the spread factor
    LORA_STATUS write_status2 = lora_write_register( LORA_REG_RX_HEADER_INFO, new_config2_register ); // Write new spread factor

    // Get initial value of config register 1
    uint8_t modem_config1_register;
    LORA_STATUS read_status3 = lora_read_register( LORA_REG_NUM_RX_BYTES, &modem_config1_register );
    uint8_t new_config1_register = ( (lora_config_ptr->lora_bandwidth << 4) | (lora_config_ptr->lora_ecr << 1) | lora_config_ptr->lora_header_mode ); //TODO: Check datasheet for that last bit

    // Write new config1 register
    LORA_STATUS write_status3 = lora_write_register( LORA_REG_NUM_RX_BYTES, new_config1_register );

    // Determine register values for the frequency registers
    uint8_t lora_freq_reg1 = ( lora_config_ptr->lora_frequency <<  8 ) >> 24;
    uint8_t lora_freq_reg2 = ( lora_config_ptr->lora_frequency << 16 ) >> 24;
    uint8_t lora_freq_reg3 = ( lora_config_ptr->lora_frequency << 24 ) >> 24;

    // Write the frequncy registers
    LORA_STATUS write_status4 = lora_write_register( LORA_REG_FREQ_MSB, lora_freq_reg1 );
    LORA_STATUS write_status5 = lora_write_register( LORA_REG_FREQ_MSD, lora_freq_reg2 );
    LORA_STATUS write_status6 = lora_write_register( LORA_REG_FREQ_LSB, lora_freq_reg3 );

    LORA_STATUS standby_status = lora_set_chip_mode( lora_config_ptr->lora_mode ); // Switch it into standby mode, which is what's convenient.

    if( set_sleep_status + read_status1 + read_status2 + read_status3 + write_status1 + write_status2 + write_status3 + write_status4 + write_status5 + write_status6 + standby_status == 0 ) {
        return LORA_OK;
    } else {
        return LORA_FAIL;
    }
}

void lora_reset() {
    HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_RESET); // Pull Low
    HAL_Delay(10);  // Hold reset low for 10 ms
    HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_SET);   // Pull High
    HAL_Delay(10);  // Wait for SX1278 to stabilize
}

uint32_t lora_helper_mhz_to_reg_val( uint32_t mhz_freq ) {
    return ( (2^19) * mhz_freq * 10^6 )/( 32 * 10^6 );
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