/**
  ******************************************************************************
  * @file           : telemetry.h
  * @brief          : Definitions for the telemetry data structures.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEM_H
#define __TELEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Standard Includes                                                                    
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes  
------------------------------------------------------------------------------*/
#include "commands.h"

/*------------------------------------------------------------------------------
 Constants
------------------------------------------------------------------------------*/
#define LORA_INTERNAL_HEADER_SIZE 8U
#define LORA_PAYLOAD_SIZE 40U
#define TELEMETRY_MESSAGE_SIZE (LORA_INTERNAL_HEADER_SIZE + LORA_PAYLOAD_SIZE)
#define DASHBOARD_DUMP_SIZE 36U

/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/

/* Aliased types for platform independence */
typedef uint8_t FLIGHT_COMP_STATE_TYPE_OPAQUE;

/* Opaque structs to allow FC dependencies to be used on non-FC platforms*/
typedef struct {
    float imu_offset[ 6 ];
} IMU_OFFSET_OPAQUE;

typedef struct {
    float baro_preset[ 2 ];
} BARO_PRESET_OPAQUE;

typedef struct {
    uint8_t servo_preset[ 4 ];
} SERVO_PRESET_OPAQUE;

/* Dashboard dump data structure */
typedef struct __attribute__((packed)) _DASHBOARD_DUMP_TYPE
	{
	QUAT attitude;
	float alt;
	float latitude;
	float longitude;
	float acc_x;
	float roll_rate;
	} DASHBOARD_DUMP_TYPE;
	_Static_assert( sizeof(DASHBOARD_DUMP_TYPE) == DASHBOARD_DUMP_SIZE, "DASHBOARD_DUMP_TYPE size invalid.");

/* Telemetry type definitions */

typedef uint32_t VERSION_INFO_TYPE; /* hw version : fw version : fw patch : fw prerelease */
									/* msb									lsb			  */

typedef enum _TELEMETRY_MESSAGE_TYPES
    {
    TELEMETRY_MSG_VEHICLE_ID = 0x00000001,
    TELEMETRY_MSG_DASHBOARD_DATA = 0x00000002,
    TELEMETRY_MSG_CALIBRATION = 0x00000003,
    __TELEMETRY_MSG_FORCE_32BIT = 0xFFFFFFFF /* used to force this type size to 32 bits */
    } TELEMETRY_MESSAGE_TYPES;
    _Static_assert( sizeof(TELEMETRY_MESSAGE_TYPES) == 4, "TELEMETRY_MESSAGE_TYPES size invalid.");

/* UID serial number (packed struct inhibits padding) */
typedef struct __attribute__((packed)) _ST_UID_TYPE 
    {
    uint32_t wafer_coords;
    char lot_num_1[3];
    uint8_t wafer_num;
    char lot_num_2[4];
    } ST_UID_TYPE;
_Static_assert( sizeof(ST_UID_TYPE) == 12, "ST_UID_TYPE packing incorrect." );

typedef struct __attribute__((packed)) _LORA_INTERNAL_HEADER_TYPE
    {
    TELEMETRY_MESSAGE_TYPES mid; /* message identifier -- currently 32 bits but will reserve command/control bits later*/
    uint32_t timestamp; /* systick in ms */
    } LORA_INTERNAL_HEADER_TYPE;
    _Static_assert( sizeof(LORA_INTERNAL_HEADER_TYPE) == LORA_INTERNAL_HEADER_SIZE, "LORA_INTERNAL_HEADER size invalid.");

typedef struct __attribute((packed)) _TELEMETRY_MSG_VEHICLE_ID_TYPE
    {
    uint32_t uid[3]; /* unique identifier per stm32 MCU */
    uint8_t hw_opcode; /* hardware identifier as defined by connect command */
	uint8_t fw_opcode; /* firmware identifier as defined by connect command */
	VERSION_INFO_TYPE version; /* version string defined above */
	char flight_id[16]; /* a 16 character c-string for the current flight (future: configurable) */
    uint8_t explicit_padding[6]; /* pad the end of this struct so the union behaves as expected */
    } TELEMETRY_MSG_VEHICLE_ID_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_VEHICLE_ID_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_VEHICLE_ID_TYPE size invalid.");

typedef struct __attribute__((packed)) _TELEMETRY_MSG_DASHBOARD_DUMP_TYPE
    {
    uint8_t fsm_state; /* current state of the flight computer */
    DASHBOARD_DUMP_TYPE data; /* the data used by the dashboard for location/orientation */
    uint8_t explicit_padding[3]; /* pad the end of this struct so the union behaves as expected */
    } TELEMETRY_MSG_DASHBOARD_DUMP_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_DASHBOARD_DUMP_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_DASHBOARD_DUMP_TYPE size invalid.");

typedef struct __attribute__((packed)) _TELEMETRY_MSG_CALIBRATION_TYPE
    {
    float imu_offset[6];
    float baro_preset[2];
    float qfe_elevation;
    uint8_t servo_preset[4];
    } TELEMETRY_MSG_CALIBRATION_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_CALIBRATION_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_CALIBRATION size invalid." );

/* struct is packed to inhibit padding */
typedef struct __attribute__((packed)) _TELEMETRY_MESSAGE
	{
	LORA_INTERNAL_HEADER_TYPE header; /* data common to every message */
    union _payload
        {
        TELEMETRY_MSG_VEHICLE_ID_TYPE vehicle_id; /* provide information about the transmitting device */
        TELEMETRY_MSG_DASHBOARD_DUMP_TYPE dashboard_dump; /* gives vehicle state info (location, orientation) */
        TELEMETRY_MSG_CALIBRATION_TYPE calibration;
        } payload;
	} TELEMETRY_MESSAGE;
	_Static_assert( sizeof(TELEMETRY_MESSAGE) == TELEMETRY_MESSAGE_SIZE, "LORA_PAYLOAD size invalid.");


/*------------------------------------------------------------------------------
 Wireless Transmission: Contract Functions
 
 The project must provide an implementation of the function prototypes defined
 below.
------------------------------------------------------------------------------*/

/**
  * @brief Constructs the next telemetry message.
  *
  * @param payload Telemetry message buffer to fill.
  */
void telemetry_get_next_message
    (
    TELEMETRY_MESSAGE* payload
    );

/**
  * @brief Builds a telemetry payload of the requested type.
  *
  * @param msg_buf Telemetry message buffer to fill.
  * @param message_type Type of telemetry message to build.
  */
void telemetry_build_payload
    (
    TELEMETRY_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    TELEMETRY_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    );

#ifdef __cplusplus
}
#endif

#endif /* __TELEM_H */


/**
  * END OF FILE
  */