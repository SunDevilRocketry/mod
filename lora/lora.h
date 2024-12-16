/*******************************************************************************
*
* FILE: 
* 		LoRa.h
*
* DESCRIPTION: 
* 		Contains register definitions and functions for controlling the LoRa
*       radio module.
*
*******************************************************************************/

#ifndef LORA_H
#define LORA_H

/* Project includes */

/* Operation Mode Register Values */
/* These are just random parts of the operation register that may or not get used
To avoid any chance they're in the final binary without being used,
they are commented out for now and will be uncommented as they're needed.

#define LORA_LORA_MODE             0b1
#define LORA_LORA_REGISTER_PAGE    0b0
#define LORA_FSK_REGISTER_PAGE     0b1
#define LORA_HIGH_FREQ_MODE        0b1
#define LORA_LOW_FREQ_MODE         0b0
#define LORA_OPERATION_RESERVED    0b00
*/

typedef enum LORA_CHIPMODE {
   LORA_SLEEP_MODE = 0b000,
   LORA_STANDBY_MODE = 0b001,
   LORA_FREQ_SYNTH_TX_MODE = 0b010,
   LORA_TRANSMIT_MODE = 0b011,
   LORA_FREQ_SYNTH_RX_MODE = 0b100,
   LORA_RX_CONTINUOUS_MODE = 0b101,
   LORA_RX_SINGLE_MODE = 0b111,
} LORA_CHIPMODE;

typedef enum LORA_STATUS {
   LORA_OK = 0,
   LORA_FAIL,
   // Temporary enum additions to distinguish between transmit and receive failures
   LORA_TRANSMIT_FAIL,
   LORA_RECEIVE_FAIL
} LORA_STATUS;

typedef enum _CS_STATUS {
   CS_LOW = 0,
   CS_HIGH = 1
} CS_STATUS;

/* Radio register addresses from datasheet (https://www.mouser.com/datasheet/2/975/1463993415RFM95_96_97_98W-1858106.pdf)
   Note: as we are using LoRa, the FSK opcodes are not included*/
typedef enum LORA_REGISTER_ADDR {
   LORA_REG_FIFO_RW                    = 0x00,
   LORA_REG_OPERATION_MODE             = 0x01,
   LORA_REG_FREQ_MSB                   = 0x06,
   LORA_REG_FREQ_MSD                   = 0x07,
   LORA_REG_FREQ_LSB                   = 0x08,
   LORA_REG_PA_CONFIG                  = 0x09,
   LORA_REG_PA_RAMP                    = 0x0A,
   LORA_REG_OVER_CURRENT_PROT_CTRL     = 0x0B,
   LORA_REG_LNA_SETTINGS               = 0x0C,
   LORA_REG_FIFO_SPI_POINTER           = 0x0D,
   LORA_REG_FIFO_TX_DATA               = 0x0E,
   LORA_REG_FIFO_RX_DATA               = 0x0F,
   LORA_REG_LORA_STATE_FLAGS           = 0x10,
   LORA_REG_LORA_FLAG_MASK             = 0x11,
   LORA_REG_INTERMEDIATE_FREQ_MSB      = 0x12,
   LORA_REG_INTERMEDIATE_FREQ_LSB      = 0x13,
   LORA_REG_RCV_TIMEOUT_MSB            = 0x14,
   LORA_REG_RCV_TIMEOUT_LSB            = 0x15,
   LORA_REG_TRANS_CONFIG               = 0x16,
   LORA_REG_TRANS_PAYLOAD_LENGTH       = 0x17,
   LORA_REG_PREAMBLE_SIZE_MSB          = 0x18,
   LORA_REG_PREAMBLE_SIZE_LSB          = 0x19,
   LORA_REG_MODULATION_CONFIG          = 0x1A,
   LORA_REG_RF_MODE                    = 0x1B,
   LORA_REG_FHSS_HOP_PERIOD            = 0x1C,
   LORA_REG_NUM_RX_BYTES               = 0x1D,
   LORA_REG_RX_HEADER_INFO             = 0x1E,
   LORA_REG_NUM_RX_VALID_HEADERS       = 0x1F,
   LORA_REG_NUM_RX_VALID_PACKETS       = 0x20,
   LORA_REG_MODEM_STATUS               = 0x21,
   LORA_REG_SIGNAL_TO_NOISE            = 0x22,
   LORA_REG_CURRENT_RSSI               = 0x23,
   LORA_REG_LAST_PACKET_RSSI           = 0x24,
   LORA_REG_FREQ_HOP_START_CHANNEL     = 0x25,
   LORA_REG_RX_DATA_POINTER            = 0x26,
   LORA_REG_DIO_MAPPING_MODE_1         = 0x40,
   LORA_REG_DIO_MAPPING_MODE_2         = 0x41,
   LORA_REG_ID_VERSION                 = 0x42,
   LORA_REG_TCXO_OR_XTAL               = 0x4B,
   LORA_REG_PA_SETTINGS                = 0x4D,
   LORA_REG_FORMER_TEMP                = 0x5B,
   LORA_REG_AGC_REFERENCE              = 0x61,
   LORA_REG_AGC_THRESHOLD_1            = 0x62,
   LORA_REG_AGC_THRESHOLD_2            = 0x63,
   LORA_REG_AGC_THRESHOLD_4            = 0x64
} LORA_REGISTER_ADDR;

LORA_STATUS lora_spi_receive( uint8_t read_buffer[] );

LORA_STATUS lora_spi_transmit( LORA_REGISTER_ADDR register, uint8_t data );

void lora_get_device_id(uint8_t* packet);

#endif