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

/* Standard includes */
#include <stdbool.h>

/* Project includes */
#include "sensor.h"

/* Radio register addresses from datasheet (https://www.mouser.com/datasheet/2/975/1463993415RFM95_96_97_98W-1858106.pdf)
   Note: as we are using LoRa, the FSK opcodes are not included*/
#define LORA_REG_FIFO_RW                   0x00
#define LORA_REG_OPERATION_MODE            0x01
#define LORA_REG_FREQ_MSB                  0x06
#define LORA_REG_FREQ_MSD                  0x07
#define LORA_REG_FREQ_LSB                  0x08
#define LORA_REG_PA_CONFIG                 0x09
#define LORA_REG_PA_RAMP                   0x0A
#define LORA_REG_OVER_CURRENT_PROT_CTRL    0x0B
#define LORA_REG_LNA_SETTINGS              0x0C
#define LORA_REG_FIFO_SPI_POINTER          0x0D
#define LORA_REG_FIFO_TX_DATA              0x0E
#define LORA_REG_FIFO_RX_DATA              0x0F
#define LORA_REG_LORA_STATE_FLAGS          0x10
#define LORA_REG_LORA_FLAG_MASK            0x11
#define LORA_REG_INTERMEDIATE_FREQ_MSB     0x12
#define LORA_REG_INTERMEDIATE_FREQ_LSB     0x13
#define LORA_REG_RCV_TIMEOUT_MSB           0x14
#define LORA_REG_RCV_TIMEOUT_LSB           0x15
#define LORA_REG_TRANS_CONFIG              0x16
#define LORA_REG_TRANS_PAYLOAD_LENGTH      0x17
#define LORA_REG_PREAMBLE_SIZE_MSB         0x18
#define LORA_REG_PREAMBLE_SIZE_LSB         0x19
#define LORA_REG_MODULATION_CONFIG         0x1A
#define LORA_REG_RF_MODE                   0x1B
#define LORA_REG_FHSS_HOP_PERIOD           0x1C
#define LORA_REG_NUM_RX_BYTES              0x1D
#define LORA_REG_RX_HEADER_INFO            0x1E
#define LORA_REG_NUM_RX_VALID_HEADERS      0x1F
#define LORA_REG_NUM_RX_VALID_PACKETS      0x20
#define LORA_REG_MODEM_STATUS              0x21
#define LORA_REG_SIGNAL_TO_NOISE           0x22
#define LORA_REG_CURRENT_RSSI              0x23
#define LORA_REG_LAST_PACKET_RSSI          0x24
#define LORA_REG_FREQ_HOP_START_CHANNEL    0x25
#define LORA_REG_RX_DATA_POINTER           0x26
#define LORA_REG_DIO_MAPPING_MODE_1        0x40
#define LORA_REG_DIO_MAPPING_MODE_2        0x41
#define LORA_REG_ID_VERSION                0x42
#define LORA_REG_TCXO_OR_XTAL              0x4B
#define LORA_REG_PA_SETTINGS               0x4D
#define LORA_REG_FORMER_TEMP               0x5B
#define LORA_REG_AGC_REFERENCE             0x61
#define LORA_REG_AGC_THRESHOLD_1           0x62
#define LORA_REG_AGC_THRESHOLD_2           0x63
#define LORA_REG_AGC_THRESHOLD_4           0x64