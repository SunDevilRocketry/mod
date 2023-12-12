// Copyright (c) Konstantin Belyalov. All rights reserved.
// Licensed under the MIT license.

#ifndef __LORA_H
#define __LORA_H

#include "main.h"


// sx1276 registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_SYMB_TIMEOUT_LSB     0x1f
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define OPMODE_SLEEP             0x00
#define OPMODE_STDBY             0x01
#define OPMODE_TX                0x03
#define OPMODE_RX_CONTINUOUS     0x05
#define OPMODE_RX_SINGLE         0x06
#define OPMODE_LONG_RANGE_MODE   0x80  // (1 << 7)

// Power Amplifier (PA_DAC) settings
#define PA_DAC_HIGH_POWER        0x87
#define PA_DAC_HALF_POWER        0x84

// Over Current Protection (OCP) config
#define OCP_ON                      (1 << 5)

// Modem config register parameters
#define MC1_IMPLICIT_HEADER_MODE    (1 << 0)

#define MC2_CRC_ON                  (1 << 2)

#define MC3_AGCAUTO                 (1 << 2)
#define MC3_MOBILE_NODE             (1 << 3)

// IRQs
#define IRQ_FLAGS_RX_TIMEOUT        (1 << 7)
#define IRQ_FLAGS_RX_DONE           (1 << 6)
#define IRQ_FLAGS_PAYLOAD_CRC_ERROR (1 << 5)
#define IRQ_FLAGS_VALID_HEADER      (1 << 4)
#define IRQ_FLAGS_TX_DONE           (1 << 3)
#define IRQ_FLAGS_CAD_DONE          (1 << 2)
#define IRQ_FLAGS_FHSSCHANGECHANNEL (1 << 1)
#define IRQ_FLAGS_CAD_DETECTED      (1 << 0)
#define IRQ_FLAGS_RX_ALL            0xf0

// Just to make it readable
#define BIT_7                       (1 << 7)

#define TRANSFER_MODE_DMA           1
#define TRANSFER_MODE_BLOCKING      2



/* LORA SETTING CONFIGURATION */

#define LORA_MAX_PACKET_SIZE               128

// Operational frequency
#define MHZ                                1000000LLU
#define LORA_BASE_FREQUENCY_US             (915LLU*MHZ)
#define LORA_BASE_FREQUENCY_EU             (868LLU*MHZ)

// Default settings
#define LORA_DEFAULT_TX_POWER              17
#define LORA_DEFAULT_SF                    7
#define LORA_DEFAULT_PREAMBLE_LEN          10
#define LORA_DEFAULT_RX_ADDR               0
#define LORA_DEFAULT_TX_ADDR               0
#define LORA_DEFAULT_SPI_TIMEOUT           1000 // ms

#define LORA_COMPATIBLE_VERSION            0x12U

// LORA return codes
#define LORA_OK                            0
#define LORA_CRC_ERROR                     1
#define LORA_TIMEOUT                       2
#define LORA_INVALID_HEADER                3
#define LORA_ERROR                         4
#define LORA_BUSY                          5
#define LORA_EMPTY                         6

// TX power mode select
#define LORA_PA_OUTPUT_RFO                 0
#define LORA_PA_OUTPUT_PA_BOOST            1

// Coding rate
#define LORA_CODING_RATE_4_5               0x08
#define LORA_CODING_RATE_4_6               0x10
#define LORA_CODING_RATE_4_7               0x18
#define LORA_CODING_RATE_4_8               0x20

// Signal bandwidth ("spread factor")
enum {
  LORA_BANDWIDTH_7_8_KHZ = 0,
  LORA_BANDWIDTH_10_4_KHZ,
  LORA_BANDWIDTH_15_6_KHZ,
  LORA_BANDWIDTH_20_8_KHZ,
  LORA_BANDWIDTH_31_25_KHZ,
  LORA_BANDWIDTH_41_7_KHZ,
  LORA_BANDWIDTH_62_5_KHZ,
  LORA_BANDWIDTH_125_KHZ,  // default SF - 7
  LORA_BANDWIDTH_250_KHZ,
  LORA_BANDWIDTH_500_KHZ,
  LORA_BW_LAST,
};

// LORA definition
typedef struct {
  // SPI parameters
  SPI_HandleTypeDef  *spi;
  GPIO_TypeDef       *nss_port;
  uint32_t            spi_timeout;
  // Operating frequency, in Hz
  uint32_t            frequency;
  // Output PIN (module internal, not related to your design)
  // Can be one of PA_OUTPUT_PA_BOOST / PA_OUTPUT_RFO
  uint32_t            pa_mode;
  // Base FIFO addresses for RX/TX
  uint8_t             tx_base_addr;
  uint8_t             rx_base_addr;

  uint16_t            nss_pin;
} lora_sx1276;


// LORA Module setup //

// Initialize LORA with default parameters:
// Params:
//  - `lora` LoRa definition to be initialized
//  - `spi` SPI HAL bus (`hspi1`, `hspi2`, etc)
//  - `nss_port` - GPIO port where `NSS` pin connected to
//  - `nss_pin` - GPIO pin number in `nss_port`
//  - `freq` - operating frequency. In Hz
// Returns:
//  - `LORA_OK` - modem initialized successfully
//  - `LORA_ERROR` - initialization failed (e.g. no modem present on SPI bus / wrong NSS port/pin)
uint8_t  lora_init(lora_sx1276 *lora, SPI_HandleTypeDef *spi, GPIO_TypeDef *nss_port,
                   uint16_t nss_pin, uint64_t freq);

// Returns LoRa modem version number (usually 0x12)
uint8_t  lora_version(lora_sx1276 *lora);


// LORA mode selection //

// Put radio into SLEEP mode:
// In this mode only SPI and configuration registers are accessible.
// LoRa FIFO is not accessible.
void     lora_mode_sleep(lora_sx1276 *lora);

// Put radio into standby (idle) mode:
// Both Crystal Oscillator and LoRa baseband blocks are turned on.
// RF part and PLLs are disabled.
void     lora_mode_standby(lora_sx1276 *lora);

// Put radio into continuous receive mode:
// When activated the RFM95/96/97/98(W) powers all remaining blocks required for reception,
// processing all received data until a new user request is made to change operating mode.
void     lora_mode_receive_continuous(lora_sx1276 *lora);

// Put radio into single receive mode:
// When activated the RFM95/96/97/98(W) powers all remaining blocks required for reception, remains in
// this state until a valid packet has been received and then returns to Standby mode.
void     lora_mode_receive_single(lora_sx1276 *lora);


// LORA signal / transmission parameters //

// Sets LoRa transmit power.
// Params:
//  - `level` - TX power in dBm. Valid range from 2dBm to 20dBm
void     lora_set_tx_power(lora_sx1276 *lora, uint8_t level);

// Set operational frequency.
// Params:
//  - `freq` - frequency in Hz
void     lora_set_frequency(lora_sx1276 *lora, uint64_t freq);

// Set signal bandwidth.
// Params:
//  - `bw` - desired bandwidth, from LORA_BANDWIDTH_7_8_KHZ to LORA_BANDWIDTH_500_KHZ
// For more information refer to section 4.1 of datasheet.
void     lora_set_signal_bandwidth(lora_sx1276 *lora, uint64_t bw);

// Set signal spreading factor.
// Params:
//  - `sf` - spreading factor. Value from 6 to 12
// For more information refer to section 4.1 of datasheet.
void     lora_set_spreading_factor(lora_sx1276 *lora, uint8_t sf);

// Set coding rate.
//  - `rate` - coding rate. Use any of LORA_CODING_RATE* constants.
// For more information refer to section 4.1 of datasheet.
void     lora_set_coding_rate(lora_sx1276 *lora, uint8_t rate);

// Enable / disable CRC
// Params:
//  - `enable` - set to 0 to disable CRC, any other value enables CRC.
void     lora_set_crc(lora_sx1276 *lora, uint8_t enable);

// Set length of packet preamble.
// Params:
//  - `len` - length of packet preamble
// For more information refer to section 4.1.1.6 of datasheet
void     lora_set_preamble_length(lora_sx1276 *lora, uint16_t len);

// Set "implicit header" mode, meaning no packet header at all.
// Refer to section 4.1.1.6 of datasheet
void     lora_set_implicit_header_mode(lora_sx1276 *lora);

// Set "explicit", i.e. always add packet header with various system information.
// Refer to section 4.1.1.6 of datasheet
void     lora_set_explicit_header_mode(lora_sx1276 *lora);


// Received packet information //

// Returns RSSI of last received packet
int8_t  lora_packet_rssi(lora_sx1276 *lora);

// Returns SNR of last received packet
uint8_t  lora_packet_snr(lora_sx1276 *lora);


// SEND packet routines //

// Query modem for any ongoing packet transmission.
// Returns 0 if no active transmission present
uint8_t  lora_is_transmitting(lora_sx1276 *lora);

// Send packet in non-blocking mode
// Params:
//  - `data` - pointer to buffer to be transmitted
//  - `data_len` - how many bytes to transmit from `data` buffer.
// Returns:
//  - `LORA_BUSY` in case of active transmission ongoing
//  - `LORA_OK` packet scheduled to be sent.
//     Check state with `lora_is_transmitting()` or by interrupt.
uint8_t  lora_send_packet(lora_sx1276 *lora, uint8_t *data, uint8_t data_len);

// Send packet using DMA mode.
// You must call lora_send_packet_dma_complete() from DMA transfer complete callback
// Params:
//  - `data` - pointer to buffer to be transmitted
//  - `data_len` - how many bytes to transmit from `data` buffer.
// Returns:
//  - `LORA_BUSY` in case of active transmission ongoing
//  - `LORA_TIMEOUT` packet wasn't transmitted in given time frame.
//  - `LORA_OK` packet scheduled to be sent.
uint8_t  lora_send_packet_dma_start(lora_sx1276 *lora, uint8_t *data, uint8_t data_len);

// Finish packet send initiated by lora_send_packet_dma_start()
void     lora_send_packet_dma_complete(lora_sx1276 *lora);

// Send packet and returns only when packet sent / error occurred (blocking mode).
// Params:
//  - `data` - pointer to buffer to be transmitted
//  - `data_len` - how many bytes to transmit from `data` buffer.
//  - `timeout` - maximum wait time to finish transmission.
// Returns:
//  - `LORA_BUSY` in case of active transmission ongoing
//  - `LORA_TIMEOUT` packet wasn't transmitted in given time frame.
//  - `LORA_OK` packet scheduled to be sent.
uint8_t  lora_send_packet_blocking(lora_sx1276 *lora, uint8_t *data, uint8_t data_len, uint32_t timeout);


// RECEIVE packet routines //

// Checks if packet modem has packet awaiting to be received
// Returns 0 if no packet is available, or any positive integer in case packet is ready
uint8_t  lora_is_packet_available(lora_sx1276 *lora);

// If modem has packet awaiting to be received - returns it's length.
uint8_t  lora_pending_packet_length(lora_sx1276 *lora);

// Receives packet from LoRa modem
// Params:
//  - `buffer` - pointer to buffer where copy packet to.
//  - `buffer_len` - length of `buffer`. If incoming packet greater than `buffer_len` it will be truncated
//    to fit `buffer_len`.
//  - `error` - pointer to `uint8_t` to store error. Can be NULL - so no error information will be stored.
// Returns actual packet length stored into `buffer`.
//
// `error` is one of:
//  - `LORA_OK` - packet successfully received.
//  - `LORA_EMPTY` - no packet received at the moment (check for packet by `lora_is_packet_available()` before).
//  - `LORA_TIMEOUT` - timeout while receiving packet (only for single receive mode).
//  - `LORA_INVALID_HEADER` - packet with malformed header received.
//  - `LORA_CRC_ERROR` - malformed packet received (CRC failed). Please note that you need to enable
//    this functionality explicitly, it is disabled by default.
uint8_t  lora_receive_packet(lora_sx1276 *lora, uint8_t *buffer, uint8_t buffer_len, uint8_t *error);

// Start receiving packet from LoRa modem in DMA mode.
// 1. In case of single receive mode: when packet arrived or timeout occurred.
//    Please note that it is not enough to set `timeout` to something positive -
//    you also need to set timeout in LoRa modem by calling `lora_set_rx_symbol_timeout` before.
// 2. For continuous receiving mode will wait until packet arrived / timeout occurred.
// Params:
//  - `buffer` - pointer to buffer where copy packet to.
//  - `buffer_len` - length of `buffer`. If incoming packet greater than `buffer_len` it will be truncated
//    to fit `buffer_len`.
//  - `error` - pointer to `uint8_t` to store error. Can be NULL - so no error information will be stored.
// Returns actual packet length stored into `buffer`.
//
// `error` is one of:
//  - `LORA_OK` - packet successfully received.
//  - `LORA_EMPTY` - no packet received at the moment (check for packet by `lora_is_packet_available()` before).
//  - `LORA_TIMEOUT` - timeout while receiving packet (only for single receive mode).
//  - `LORA_INVALID_HEADER` - packet with malformed header received.
//  - `LORA_CRC_ERROR` - malformed packet received (CRC failed). Please note that you need to enable
//    this functionality explicitly, it is disabled by default.
uint8_t  lora_receive_packet_dma_start(lora_sx1276 *lora, uint8_t *buffer, uint8_t buffer_len,
                                       uint8_t *error);

// Finish receive packet in DMA dome
void     lora_receive_packet_dma_complete(lora_sx1276 *lora);


// Receive packet in "blocking mode" i.e. function return only when packet:
// 1. In case of single receive mode: when packet arrived or timeout occurred.
//    Please note that it is not enough to set `timeout` to something positive -
//    you also need to set timeout in LoRa modem by calling `lora_set_rx_symbol_timeout` before.
// 2. For continuous receiving mode will wait until packet arrived / timeout occurred.
// Params:
//  - `buffer` - pointer to buffer where copy packet to.
//  - `buffer_len` - length of `buffer`. If incoming packet greater than `buffer_len` it will be truncated
//    to fit `buffer_len`.
//  - `error` - pointer to `uint8_t` to store error. Can be NULL - so no error information will be stored.
// Returns actual packet length stored into `buffer`.
//
// `error` is one of:
//  - `LORA_OK` - packet successfully received.
//  - `LORA_EMPTY` - no packet received at the moment (check for packet by `lora_is_packet_available()` before).
//  - `LORA_TIMEOUT` - timeout while receiving packet (only for single receive mode).
//  - `LORA_INVALID_HEADER` - packet with malformed header received.
//  - `LORA_CRC_ERROR` - malformed packet received (CRC failed). Please note that you need to enable
//    this functionality explicitly, it is disabled by default.
uint8_t  lora_receive_packet_blocking(lora_sx1276 *lora, uint8_t *buffer, uint8_t buffer_len,
                                      uint32_t timeout, uint8_t *error);

// Sets timeout for `lora_mode_receive_single()` in symbols.
// Params:
//  - `symbols` - timeout value. Valid from `4` to `1024` symbols.
// For more information refer to datasheet section 4.1.5
void     lora_set_rx_symbol_timeout(lora_sx1276 *lora, uint16_t symbols);


// Enables interrupt on DIO0 when packet received
// SX1276 module will pull DIO0 line high
void     lora_enable_interrupt_rx_done(lora_sx1276 *lora);

// Enables interrupt on DIO0 when transmission is done
// SX1276 module will pull DIO0 line high
void     lora_enable_interrupt_tx_done(lora_sx1276 *lora);

// Clears all RX interrupts on DIO0 (done, timeout, crc, etc)
void lora_clear_interrupt_rx_all(lora_sx1276 *lora);

// Clears TX interrupt on DIO0
void     lora_clear_interrupt_tx_done(lora_sx1276 *lora);

#endif
