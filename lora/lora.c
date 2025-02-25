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

// Debugging purposes
#include "led.h"

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
    // if( read_status & 0b00000001 != 0b00000001 ) { // Make function automatically fail if chip is not in LoRa mode
    //     return LORA_FAIL;
    // }

    if (read_status != LORA_OK)
    {
        return LORA_FAIL;
    }

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


    uint8_t modem_config2_register;
    LORA_STATUS read_status2 = lora_read_register( LORA_REG_RX_HEADER_INFO, &modem_config2_register );

    uint8_t new_config2_register = modem_config2_register & 0x0F; // Erase spread factor bits
    uint8_t new_config2_register = ( modem_config2_register | ( lora_config_ptr->lora_spread << 4 ) ); // Set the spread factor
    LORA_STATUS write_status2 = lora_write_register( LORA_REG_RX_HEADER_INFO, new_config2_register ); // Write new spread factor

    LORA_STATUS standby_status = lora_set_chip_mode( lora_config_ptr->lora_mode ); // Switch it into standby mode, which is what's convenient.

    if( set_sleep_status + read_status1 + read_status2 + write_status1 + write_status2 + standby_status == 0 ) {
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

// =============================================================================
// lora_transmit: transmit a buffer through lora fifo
// =============================================================================
LORA_STATUS lora_transmit(uint8_t* buffer_ptr, uint8_t buffer_len){
    // Mode request STAND-BY
    LORA_STATUS status = lora_set_chip_mode(LORA_STANDBY_MODE);

    // TX Init TODO

    // Write data to LoRA FIFO
    uint8_t fifo_ptr_addr;
    LORA_STATUS status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
    if (status != LORA_OK){
        // Error handler
        led_set_color(LED_RED);
    }
    LORA_STATUS status = lora_write_register(LORA_REG_FIFO_TX_BASE_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
    if (status != LORA_OK){
        // Error handler
        led_set_color(LED_RED);
    }
    // Write buffer length to fifo_rw
    LORA_STATUS status = lora_write_register(LORA_REG_FIFO_RW, buffer_len); 
    LORA_STATUS status = lora_set_chip_mode(LORA_TRANSMIT_MODE);
    while (1){
        uint8_t lora_op
        LORA_STATUS status = lora_read_register(LORA_REG_OPERATION_MODE, &lora_op);
        if ((lora_op & 0b111) == LORA_STANDBY_MODE){
            break;
        } 
    }
    // Send each byte of the buffer
    for (int i = 0; i<buffer_len; i++){
        LORA_STATUS status = lora_write_register(LORA_REG_FIFO_RW, buffer_ptr[i]);
        LORA_STATUS status = lora_set_chip_mode(LORA_TRANSMIT_MODE);
        while (1){
            uint8_t lora_op
            LORA_STATUS status = lora_read_register(LORA_REG_OPERATION_MODE, &lora_op);
            if ((lora_op & 0b111) == LORA_STANDBY_MODE){
                break;
            } 
        }   
    }
    return LORA_OK;
}

// =============================================================================
// lora_receive: receive a buffer from lora fifo with single mode
// =============================================================================
LORA_STATUS lora_receive(uint8_t* buffer_ptr, uint8_t* buffer_len_ptr){
    uint8_t timeout_flag;
    uint8_t rx_done;
    
    // Mode request STAND-BY
    LORA_STATUS status = lora_set_chip_mode(LORA_STANDBY_MODE);

    // RX Init TODO
    
    // Set lora fifo pointer to the RX base address
    uint8_t fifo_ptr_addr;
    LORA_STATUS status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
    if (status != LORA_OK){
        // Error handler
        led_set_color(LED_RED);
    }
    LORA_STATUS status = lora_write_register(LORA_REG_FIFO_RX_BASE_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
    if (status != LORA_OK){
        // Error handler
        led_set_color(LED_RED);
    }

    // Send request for RX Single mode
    LORA_STATUS status = lora_set_chip_mode(LORA_RX_SINGLE_MODE);

    // Wait for LoRA IRQ
    uint16_t timeout = 0;
    while(timeout<10000){
        uint8_t irq_flag;
        LORA_STATUS status = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
        timeout_flag = (irq_flag & (1<<7)) >> 7;
        rx_done = (irq_flag & (1<<6)) >> 6;
        
        if (timeout_flag){
            return LORA_TIMEOUT;
        } else if (rx_done) {
            break;
        }
        timeout++;
    }

    if (rx_done){
        LORA_STATUS status = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
        uint8_t crc_err = (irq_flag & (1<<5)) >> 5;

        if (!crc_err){
            // Read received number of bytes
            uint8_t num_bytes;
            LORA_STATUS status = lora_read_register(LORA_REG_FIFO_RX_NUM_BYTES, &num_bytes);

            // Set lora fifo pointer to the RX base current address
            uint8_t fifo_ptr_addr;
            LORA_STATUS status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
            if (status != LORA_OK){
                // Error handler
                led_set_color(LED_RED);
            }
            LORA_STATUS status = lora_write_register(LORA_REG_FIFO_RX_BASE_CUR_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
            if (status != LORA_OK){
                // Error handler
                led_set_color(LED_RED);
            }
            // Begin extracting payload
            for (int i = 0; i < num_bytes; i++){
                uint8_t packet;
                LORA_STATUS status = lora_read_register(LORA_REG_FIFO_RW, &packet);  // Access LoRA FIFO data buffer pointer
                buffer_ptr[i] = packet;
            }
            *buffer_len_ptr = num_bytes;
        } 
        return LORA_OK;
    }
    return LORA_FAIL;
}