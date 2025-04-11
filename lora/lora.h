/*******************************************************************************
*
* FILE: 
* 		LoRa.h
*
* DESCRIPTION: 
* 		Contains register definitions and functions for controlling the LoRa
*     radio module.
*
*     Datasheet:
*     https://www.mouser.com/datasheet/2/975/1463993415RFM95_96_97_98W-1858106.pdf
*     
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LORA_H
#define LORA_H

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

#define LORA_TIMEOUT                2000

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Device Modes (See Pg. 102, Reg 0x01, Bits 0-2) */
typedef enum LORA_CHIPMODE {
   LORA_SLEEP_MODE = 0x00,
   LORA_STANDBY_MODE = 0x01,
   LORA_FREQ_SYNTH_TX_MODE = 0x02,
   LORA_TRANSMIT_MODE = 0x03,
   LORA_FREQ_SYNTH_RX_MODE = 0x04,
   LORA_RX_CONTINUOUS_MODE = 0x05,
   LORA_RX_SINGLE_MODE = 0x06,
   LORA_RX_CAD         = 0x07
} LORA_CHIPMODE;

/* LoRa return value codes */
typedef enum LORA_STATUS {
   LORA_OK = 0,
   LORA_FAIL,
   LORA_TIMEOUT_FAIL,
} LORA_STATUS;

/* Radio register addresses (Pg. 86-87, 102-107)*/
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
   LORA_REG_FIFO_TX_BASE_ADDR          = 0x0E,
   LORA_REG_FIFO_RX_BASE_ADDR          = 0x0F,
   LORA_REG_FIFO_RX_BASE_CUR_ADDR      = 0x10,
   LORA_REG_IRQ_FLAGS_MASK             = 0x11,
   LORA_REG_IRQ_FLAGS                  = 0x12,
   LORA_REG_FIFO_RX_NUM_BYTES          = 0x13,
   LORA_REG_RX_HEADER_CNT_MSB          = 0x14,
   LORA_REG_RX_HEADER_CNT_LSB          = 0x15,
   LORA_REG_RX_PACKET_CNT_MSB          = 0x16,
   LORA_REG_RX_PACKET_CNT_LSB          = 0x17,
   LORA_REG_MODEM_STATUS               = 0x18,
   LORA_REG_PACKET_SNR                 = 0x19,
   LORA_REG_PACKET_RSSI                = 0x1A,
   LORA_REG_CURRENT_RSSI               = 0x1B,
   LORA_REG_HOP_CHANNEL                = 0x1C,
   LORA_REG_MODEM_CONFIG_1             = 0x1D,
   LORA_REG_MODEM_CONFIG_2             = 0x1E,
   LORA_REG_SYMBOL_TIMOUT_LSB          = 0x1F,
   LORA_REG_PREAMBLE_MSB               = 0x20,
   LORA_REG_PREAMBLE_LSB               = 0x21,
   LORA_REG_PAYLOAD_LENGTH             = 0x22,
   LORA_REG_MAX_PAYLOAD_LENGTH         = 0x23,
   LORA_REG_HOP_PERIOD                 = 0x24,
   LORA_REG_FIFO_RX_BYTE_ADDR          = 0x25,
   LORA_REG_MODEM_CONFIG_3             = 0x26,
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

/* Bandwith, Pg. 106 */
typedef enum LORA_BANDWIDTH {
   LORA_BANDWIDTH_7_8_KHZ   = 0x00,
   LORA_BANDWIDTH_10_4_KHZ  = 0x01,
   LORA_BANDWIDTH_15_6_KHZ  = 0x02,
   LORA_BANDWIDTH_20_8_KHZ  = 0x03,
   LORA_BANDWIDTH_31_25_KHZ = 0x04,
   LORA_BANDWIDTH_41_7_KHZ  = 0x05,
   LORA_BANDWIDTH_62_5_KHZ  = 0x06,
   LORA_BANDWIDTH_125_KHZ   = 0x07,
   LORA_BANDWIDTH_250_KHZ   = 0x08,
   LORA_BANDWIDTH_500_KHZ   = 0x09
} LORA_BANDWIDTH;

/* Error Coding Rate, Pg. 106  */
typedef enum LORA_ERROR_CODING {
   LORA_ECR_4_5 = 0x01,
   LORA_ECR_4_6 = 0x02,
   LORA_ECR_4_7 = 0x03,
   LORA_ECR_4_8 = 0x04
} LORA_ERROR_CODING;

// Explicit or Implicit Header, Pg. 106 (More info 26-27)
typedef enum LORA_HEADER_MODE {
   LORA_IMPLICIT_HEADER = 0b1,
   LORA_EXPLICIT_HEADER = 0b0
} LORA_HEADER_MODE;

/* Spreading factor, Pg. 107 */
typedef enum LORA_SPREADING_FACTOR {
   LORA_SPREAD_6 = 6,
   LORA_SPREAD_7 = 7,
   LORA_SPREAD_8 = 8,
   LORA_SPREAD_9 = 9,
   LORA_SPREAD_10 = 10,
   LORA_SPREAD_11 = 11,
   LORA_SPREAD_12 = 12
} LORA_SPREADING_FACTOR;

/* LORA CONFIG SETTINGS */
typedef struct _LORA_CONFIG {
   LORA_CHIPMODE lora_mode; // Current LORA Chipmode
   LORA_SPREADING_FACTOR lora_spread; // LoRa Spread factor
   LORA_BANDWIDTH lora_bandwidth; // Signal bandwith
   LORA_ERROR_CODING lora_ecr; // Data Error coding
   LORA_HEADER_MODE lora_header_mode; // LORA Header mode
   uint32_t lora_frequency; // The LORA carrier frequency in MHz
} LORA_CONFIG;

/*------------------------------------------------------------------------------
 Functions 
------------------------------------------------------------------------------*/

/* Wrapper function for HAL SPI Receive Call, 
   For Library Development Convenience */
LORA_STATUS LORA_SPI_Receive
   (
   uint8_t* read_buffer_ptr
   );

/* Wrapper function for HAL SPI Transmit Call, 
   For Library Development Convenience,
   Multiple Bytes */
LORA_STATUS LORA_SPI_Transmit_Data
   (
   LORA_REGISTER_ADDR reg,
   uint8_t data
   );

/* Wrapper function for HAL SPI Transmit Call, 
   For Library Development Convenience,
   Single Byte */
LORA_STATUS LORA_SPI_Transmit_Byte
   (
   LORA_REGISTER_ADDR reg
   );

/* Read LoRa register value */
LORA_STATUS lora_read_register
   (
   LORA_REGISTER_ADDR lora_register,
   uint8_t* regData
   );

/* Write value to LoRa register */
LORA_STATUS lora_write_register
   (
   LORA_REGISTER_ADDR lora_register,
   uint8_t data
   );

/* Get sillicon revision ID */
LORA_STATUS lora_get_device_id
   (
   uint8_t* buffer_ptr
   );

LORA_STATUS lora_set_chip_mode
   (
   LORA_CHIPMODE chip_mode
   );

/* Initialize LoRa chip with LoRa config */
LORA_STATUS lora_init
   (
   LORA_CONFIG *lora_config_ptr
   );

/* Reset LoRa chip */
void lora_reset
   (
   void
   );

/* Transmit Data Over LoRa Protocol */
LORA_STATUS lora_transmit
   (
   uint8_t* buffer_ptr, uint8_t buffer_len
   );

/* Receive Data over LoRa Protocol */
LORA_STATUS lora_receive
   (
   uint8_t* buffer_ptr,
   uint8_t* buffer_len_ptr
   );

#endif