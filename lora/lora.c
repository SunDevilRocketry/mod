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
 Project Includes
------------------------------------------------------------------------------*/
#include "lora.h"
#include "main.h"

// Debugging purposes
#include "led.h"

/*------------------------------------------------------------------------------
 Internal Use Functions 
------------------------------------------------------------------------------*/

/* Wrapper function for HAL SPI Receive Call,  */
LORA_STATUS LORA_SPI_Receive( uint8_t* read_buffer_ptr ) {
    HAL_StatusTypeDef status;

    /* Takes pointer to the read buffer. and puts output there */
    status = HAL_SPI_Receive( &(LORA_SPI), read_buffer_ptr, 1, LORA_TIMEOUT );

    if (status == HAL_OK){
        return LORA_OK;
    } else 
        return LORA_FAIL;
}

/* Wrapper function for HAL SPI Transmit Call, Multiple Bytes */
LORA_STATUS LORA_SPI_Transmit_Byte( LORA_REGISTER_ADDR reg ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write (1 byte) and writes that register. */
    uint8_t transmitBuffer = reg;
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer, 1, LORA_TIMEOUT);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

/* Wrapper function for HAL SPI Transmit Call, Single Byte */
LORA_STATUS LORA_SPI_Transmit_Data( LORA_REGISTER_ADDR reg, uint8_t data ) {
    HAL_StatusTypeDef status;

    /* Takes register and data to write and writes that register. */
    uint8_t transmitBuffer[2] = { reg, data };
    status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer[0], 2, LORA_TIMEOUT);

    if (status == HAL_OK){
        return LORA_OK;
    } else return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Internal Use/Testing/Debugging Functions 
------------------------------------------------------------------------------*/

/* Read LoRa register value */
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

/* Write value to LoRa register */
LORA_STATUS lora_write_register( LORA_REGISTER_ADDR lora_register, uint8_t data ) {
    LORA_STATUS status;

    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

    status = LORA_SPI_Transmit_Data( (lora_register | 0x80), data );
    
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

    if ( status == LORA_OK )
        return LORA_OK;
    else return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		lora_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Get the sillicon revesion ID of the chip                               *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_get_device_id
    (
    uint8_t* buffer_ptr
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
LORA_STATUS status;

/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
status = lora_read_register( LORA_REG_ID_VERSION, buffer_ptr );

if ( status == LORA_OK )
    return LORA_OK;
else return LORA_FAIL;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		lora_set_chip_mode                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a flash subcommand based on input from the sdec terminal      *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    )
{
/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
/* Get initial value of the operation mode register */
uint8_t operation_mode_register;
LORA_STATUS read_status = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

if (read_status != LORA_OK)
{
    return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
/* Form new operation mode register byte */
uint8_t new_opmode_register = (operation_mode_register & ~(0x7));
new_opmode_register = (new_opmode_register | chip_mode);

/* Write new byte */
LORA_STATUS write_status = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

if ( write_status + read_status == 0){
    return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		lora_init                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initialize the LoRa modem based on the provided configuration.         *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_init
    (
    LORA_CONFIG *lora_config_ptr
    )
{
/*------------------------------------------------------------------------------
 Get Initial Register Values 
------------------------------------------------------------------------------*/
/* Get initial value of the operation mode register */
uint8_t opmode_register;
LORA_STATUS op_read = lora_read_register( LORA_REG_OPERATION_MODE, &opmode_register );

/* Get initial value of config register 1 */
uint8_t modem_config1_register;
LORA_STATUS m1_read = lora_read_register( LORA_REG_MODEM_CONFIG_1, &modem_config1_register );

/* Get initial value of config register 2 */
uint8_t modem_config2_register;
LORA_STATUS m2_read = lora_read_register( LORA_REG_MODEM_CONFIG_2, &modem_config2_register );

/* Check if any of these reads failed */
if(
    m2_read |
    op_read |
    m1_read
) {
    return LORA_FAIL;
}

/*------------------------------------------------------------------------------
 Calculate New Register Values
------------------------------------------------------------------------------*/
/* Toggle the LoRa bit (We set the mode later using existing set opmode function) */
uint8_t new_opmode_register = ( opmode_register | 0b10000000 );

/* Generate new Modem Config Register 1 value from config */
uint8_t new_config1_register = 
    ( 
    (lora_config_ptr->lora_bandwidth << 4) |
    (lora_config_ptr->lora_ecr << 1) |
    lora_config_ptr->lora_header_mode 
    );

/* Generate new Modem Config Register 2 value from config */
uint8_t new_config2_register = 
    (
    ( modem_config2_register & 0x0F ) | /* Erase spread factor bits */
    ( lora_config_ptr->lora_spread << 4 ) /* Set the spread factor */
    );

/* Generate new Modem Config Register 2 value from config */
/* Convert integer MHz value to internal chip unit */
/* Note that while this is a uint32_t, this value should only be 24 bits large */
uint32_t frf_reg = lora_config_ptr->lora_frequency * 524288 / 32;

/* Split up frequency into 3 bytes, one for each frequency register */
uint8_t lora_freq_msb = ( frf_reg <<  8 ) >> 24; /* Most significant byte */
uint8_t lora_freq_msd = ( frf_reg << 16 ) >> 24; /* Middle significant byte */
uint8_t lora_freq_lsb = ( frf_reg << 24 ) >> 24; /* Least significant byte */

/*------------------------------------------------------------------------------
 Write Generated Register Values
------------------------------------------------------------------------------*/
/* Opmode registers */
LORA_STATUS op_write = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

/* Modem Config Registers */
LORA_STATUS m1_write = lora_write_register( LORA_REG_MODEM_CONFIG_1, new_config1_register );
LORA_STATUS m2_write = lora_write_register( LORA_REG_MODEM_CONFIG_2, new_config2_register );

/* Frequency Registers */
LORA_STATUS freq_msb_write = lora_write_register( LORA_REG_FREQ_MSB, lora_freq_msb );
LORA_STATUS freq_msd_write = lora_write_register( LORA_REG_FREQ_MSD, lora_freq_msd );
LORA_STATUS freq_lsb_write = lora_write_register( LORA_REG_FREQ_LSB, lora_freq_lsb );

/* Check Write Status */
if(
    op_write       |
    m1_write       |
    m2_write       |
    freq_msb_write |
    freq_msd_write |
    freq_lsb_write
) {
    return LORA_FAIL;
}

/* Bring chip into convient mode for usage */
if(  lora_set_chip_mode( LORA_STANDBY_MODE ) ) {
    return LORA_FAIL;
} else {
    return LORA_OK;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		lora_rest                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Reset the LoRa modem (Must run lora_init again after)                  *
*                                                                              *
*******************************************************************************/
void lora_reset
    (
    void
    )
{
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_RESET); /* Pull Low */
HAL_Delay(10);  /* Hold reset low for 10 ms */
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_SET);   // Pull High
HAL_Delay(10);  /* Wait for RFM95 to stabilize */
}

// =============================================================================
// lora_transmit: transmit a buffer through lora fifo
// ================v=============================================================
LORA_STATUS lora_transmit
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len
    )
{
/* uint8_t fifo_ptr_addr_test; // Testing fifo increment */

/* Mode request STAND-BY */
LORA_STATUS standby_status = lora_set_chip_mode(LORA_STANDBY_MODE);

// Write data to LoRA FIFO
uint8_t fifo_ptr_addr;
LORA_STATUS ptr_status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
if (ptr_status + standby_status != LORA_OK){
    // Error handler
    led_set_color(LED_RED);
    return LORA_FAIL;
}
LORA_STATUS tx_base_status = lora_write_register(LORA_REG_FIFO_TX_BASE_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
if (tx_base_status != LORA_OK){
    // Error handler
    led_set_color(LED_RED);
    return LORA_FAIL;
}

// Write buffer length to fifo_rw
LORA_STATUS fifo_status = lora_write_register(LORA_REG_PAYLOAD_LENGTH, buffer_len); 

// Send byte to byte to the fifo buffer
LORA_STATUS sendbyte_status;
for (int i = 0; i<buffer_len; i++){
    sendbyte_status = lora_write_register(LORA_REG_FIFO_RW, buffer_ptr[i]);
    
    /* TESTING PURPOSE */
    /* LORA_STATUS ptr_status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr_test);  // Access LoRA FIFO data buffer pointer */
}

/* TESTING PURPOSE: Read payload length */
/*
uint8_t test_buf;
lora_read_register( LORA_REG_PAYLOAD_LENGTH, &test_buf );
*/

LORA_STATUS tmode_status = lora_set_chip_mode(LORA_TRANSMIT_MODE);

/* TESTING PURPOSE: Test op mode transition */
/*
uint8_t operation_mode_register;
lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );
*/


uint8_t lora_op;
LORA_STATUS regop_status;
while (1){ // TODO Add a timeout here
    regop_status = lora_read_register(LORA_REG_OPERATION_MODE, &lora_op);
    if ((lora_op & 0b111) == LORA_STANDBY_MODE){
        break;
    } 
}
if( fifo_status + tmode_status + regop_status + sendbyte_status == 0 ) {
        return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		lora_receive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Receive a buffer from LoRa FIFO with single mode                       *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_receive(uint8_t* buffer_ptr, uint8_t* buffer_len_ptr){
/* TODO: Refactor this
   I'm holding off for now because this still needs to be tested,
   and I'd rather wait on refactoring something we don't know to
   work yet.
*/
uint8_t timeout_flag;
uint8_t rx_done;

// Mode request STAND-BY
LORA_STATUS standby_status = lora_set_chip_mode(LORA_STANDBY_MODE);

// RX Init TODO

    // Set lora fifo pointer to the RX base address
uint8_t fifo_ptr_addr;
LORA_STATUS ptr_status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
if (standby_status + ptr_status != LORA_OK){
    // Error handler
    led_set_color(LED_RED);
    return LORA_FAIL;
}
LORA_STATUS fifo_status = lora_write_register(LORA_REG_FIFO_RX_BASE_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
if (fifo_status != LORA_OK){
    // Error handler
    led_set_color(LED_RED);
    return LORA_FAIL;
}

// Send request for RX Single mode
LORA_STATUS rmode_status = lora_set_chip_mode(LORA_RX_SINGLE_MODE);

// Wait for LoRA IRQ
uint16_t timeout = 0;
uint8_t irq_flag;
LORA_STATUS irq_status;
while(timeout<10000){
    irq_status = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
    timeout_flag = (irq_flag & (1<<7)) >> 7;
    rx_done = (irq_flag & (1<<6)) >> 6;
    
    if (timeout_flag){
        return LORA_TIMEOUT_FAIL;
    } else if (rx_done) {
        break;
    }
    timeout++;
}

if (rx_done){
    LORA_STATUS irq_status2 = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
    uint8_t crc_err = (irq_flag & (1<<5)) >> 5;

    if (!crc_err){
        // Read received number of bytes
        uint8_t num_bytes;
        LORA_STATUS fifo2_status = lora_read_register(LORA_REG_FIFO_RX_NUM_BYTES, &num_bytes);

        // Set lora fifo pointer to the RX base current address
        uint8_t fifo_ptr_addr;
        LORA_STATUS ptr2_status = lora_read_register(LORA_REG_FIFO_SPI_POINTER, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
        if (ptr2_status != LORA_OK){
            // Error handler
            led_set_color(LED_RED);
            return LORA_FAIL;
        }
        LORA_STATUS base_adr_status = lora_write_register(LORA_REG_FIFO_RX_BASE_CUR_ADDR, fifo_ptr_addr); // Set fifo data pointer to TX base address
        if (base_adr_status != LORA_OK){
            // Error handler
            led_set_color(LED_RED);
            return LORA_FAIL;
        }
        // Begin extracting payload
        LORA_STATUS pld_xtr_status;
        for (int i = 0; i < num_bytes; i++){
            uint8_t packet;
            pld_xtr_status = lora_read_register(LORA_REG_FIFO_RW, &packet);  // Access LoRA FIFO data buffer pointer
            buffer_ptr[i] = packet;
        }
        *buffer_len_ptr = num_bytes;
        if( pld_xtr_status == LORA_OK ) {
            return LORA_OK;
        } else {
            return LORA_FAIL;
        }
    }
    return LORA_OK;
}
return LORA_FAIL;
}