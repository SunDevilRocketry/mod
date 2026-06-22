/*******************************************************************************
*
* FILE: 
* 		telemetry.h
*
* DESCRIPTION: 
* 		Definitions for the telemetry data structures.
*
* NOTE:
*       Every data structure given here is incredibly regression sensitive. All
*       changes to structs must also change their factory/constructor procedures
*       and ensure compatibility through the flight computer software stack
*       (SDEC/API/CLI & Dashboard).
*                                                                              
* COPYRIGHT:                                                                   
*       Copyright (c) 2025 Sun Devil Rocketry.                                 
*       All rights reserved.                                                   
*                                                                              
*       This software is licensed under terms that can be found in the LICENSE 
*       file in the root directory of this software component.                 
*       If no LICENSE file comes with this software, it is covered under the   
*       BSD-3-Clause.                                                          
*
*       https://opensource.org/license/bsd-3-clause
*
*******************************************************************************/

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
#include "math_sdr.h"
#include "error_sdr.h"
#include "commands.h"
#include "main.h"

#ifndef F1_TESTBED
#include "stm32h7xx_hal.h"
#include "sensor.h"
#endif

/*------------------------------------------------------------------------------
 Constants
------------------------------------------------------------------------------*/
#define LORA_INTERNAL_HEADER_SIZE 8U
#define LORA_PAYLOAD_SIZE 40U
#define TELEMETRY_MESSAGE_SIZE (LORA_INTERNAL_HEADER_SIZE + LORA_PAYLOAD_SIZE)

/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/


/* Aliased type if it doesn't exist on this platform */
#ifndef FLIGHT_COMPUTER
typedef uint8_t FLIGHT_COMP_STATE_TYPE;
#endif

typedef uint32_t VERSION_INFO_TYPE; /* hw version : fw version : fw patch : fw prerelease */
									/* msb									lsb			  */

typedef enum _TELEMETRY_MESSAGE_TYPES
    {
    TELEMETRY_MSG_VEHICLE_ID = 0x00000001,
    TELEMETRY_MSG_DASHBOARD_DATA = 0x00000002,
    TELEMETRY_MSG_WARNING_MESSAGE = 0x00000003, /* ETS TODO */
    TELEMETRY_MSG_INFO_MESSAGE = 0x00000004,    /* ETS TODO */
    __TELEMETRY_MSG_FORCE_32BIT = 0xFFFFFFFF /* used to force this type size to 32 bits */
    } TELEMETRY_MESSAGE_TYPES;
    _Static_assert( sizeof(TELEMETRY_MESSAGE_TYPES) == 4, "TELEMETRY_MESSAGE_TYPES size invalid.");

typedef struct __attribute__((packed)) _LORA_INTERNAL_HEADER_TYPE
    {
    TELEMETRY_MESSAGE_TYPES mid; /* message identifier -- currently 32 bits but will reserve command/control bits later*/
    uint32_t timestamp; /* systick in ms */
    } LORA_INTERNAL_HEADER_TYPE;
    _Static_assert( sizeof(LORA_INTERNAL_HEADER_TYPE) == LORA_INTERNAL_HEADER_SIZE, "LORA_INTERNAL_HEADER size invalid.");

typedef struct __attribute((packed)) _TELEMETRY_MSG_VEHICLE_ID_TYPE
    {
    ST_UID_TYPE uid; /* unique identifier per stm32 MCU */
    uint8_t hw_opcode; /* hardware identifier as defined by connect command */
	uint8_t fw_opcode; /* firmware identifier as defined by connect command */
	VERSION_INFO_TYPE version; /* version string defined above */
	char flight_id[16]; /* a 16 character c-string for the current flight (future: configurable) */
    uint8_t explicit_padding[6]; /* pad the end of this struct so the union behaves as expected */
    } TELEMETRY_MSG_VEHICLE_ID_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_VEHICLE_ID_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_VEHICLE_ID_TYPE size invalid.");

typedef struct __attribute__((packed)) _TELEMETRY_MSG_DASHBOARD_DUMP_TYPE
    {
    FLIGHT_COMP_STATE_TYPE fsm_state; /* current state of the flight computer */
    DASHBOARD_DUMP_TYPE data; /* the data used by the dashboard for location/orientation */
    uint8_t explicit_padding[3]; /* pad the end of this struct so the union behaves as expected */
    } TELEMETRY_MSG_DASHBOARD_DUMP_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_DASHBOARD_DUMP_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_DASHBOARD_DUMP_TYPE size invalid.");

/* maps to warning and info messages */
typedef struct __attribute__((packed)) _TELEMETRY_MSG_TEXT_MESSAGE_TYPE
    {
    TEXT_MESSAGE msg; /* encodes the systick where it was generated + a short message */
    } TELEMETRY_MSG_TEXT_MESSAGE_TYPE;
    _Static_assert( sizeof(TELEMETRY_MSG_TEXT_MESSAGE_TYPE) == LORA_PAYLOAD_SIZE, "TELEMETRY_MSG_TEXT_MESSAGE_TYPE size invalid.");

/* struct is packed to inhibit padding */
typedef struct __attribute__((packed)) _TELEMETRY_MESSAGE
	{
	LORA_INTERNAL_HEADER_TYPE header; /* data common to every message */
    union _payload
        {
        TELEMETRY_MSG_VEHICLE_ID_TYPE vehicle_id; /* provide information about the transmitting device */
        TELEMETRY_MSG_DASHBOARD_DUMP_TYPE dashboard_dump; /* gives vehicle state info (location, orientation) */
        TELEMETRY_MSG_TEXT_MESSAGE_TYPE text_message; /* TODO: a short message generated by the firmware */
        } payload;
	} TELEMETRY_MESSAGE;
	_Static_assert( sizeof(TELEMETRY_MESSAGE) == TELEMETRY_MESSAGE_SIZE, "LORA_PAYLOAD size invalid.");


/*------------------------------------------------------------------------------
 Function prototypes                                             
------------------------------------------------------------------------------*/

/* telemetry.c */
void telemetry_get_next_message
    (
    TELEMETRY_MESSAGE* payload
    );

void telemetry_build_payload
    (
    TELEMETRY_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    TELEMETRY_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    );

#ifdef __cplusplus
}
#endif

#endif /* __TELEM_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/