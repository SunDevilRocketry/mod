/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		telemetry.c                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Module for LoRa (wireless) communication.                              *
*                                                                              *
* COPYRIGHT:                                                                   *
*       Copyright (c) 2025 Sun Devil Rocketry.                                 *
*       All rights reserved.                                                   *
*                                                                              *
*       This software is licensed under terms that can be found in the LICENSE *
*       file in the root directory of this software component.                 *
*       If no LICENSE file comes with this software, it is covered under the   *
*       BSD-3-Clause.                                                          *
*                                                                              *
*       https://opensource.org/license/bsd-3-clause                            *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------ 
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------ 
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"
#include "commands.h"
#include "error_sdr.h"
#include "telemetry.h"
#include "lora.h"

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/
extern PRESET_DATA   preset_data;      /* Struct with preset data   */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor    */

static TELEMETRY_FSM_STATE telemetry_state 
                                 = TELEMETRY_STATE_BLOCKING;
static uint8_t       register_contents[2] = {0x00, 0x00};
static LORA_STATUS   lora_status = LORA_OK;
static uint32_t      message_idx = 0;
static LORA_MESSAGE  payload;
static uint8_t       burst_write_buf[LORA_MESSAGE_SIZE + 1];

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/

static void telemetry_build_msg_vehicle_id
    (
    LORA_MESSAGE* msg_buf
    );


static void telemetry_build_msg_dashboard_dump
    (
    LORA_MESSAGE* msg_buf
    );


static void telemetry_build_text_message
    (
    LORA_MESSAGE* msg_buf,
    LORA_MESSAGE_TYPES message_type
    );


/*------------------------------------------------------------------------------ 
 Public APIs                                                                    
------------------------------------------------------------------------------*/

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_update                                                         *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Update the telemetry system. Holds state; consider this the "main"       *
*       function for the telemetry system.                                       *
*                                                                                *
*********************************************************************************/
void telemetry_update
    (
    TELEMETRY_EVENT update_cause /* i: which kind of event triggered this update */
    )
{
/* Local Variables */

/* precondition: lora is not broken */
if( ( lora_status & ( LORA_FAIL | LORA_TRANSMIT_FAIL | LORA_TIMEOUT_FAIL ) )
 || ( update_cause == TELEMETRY_EVENT_CANCEL ) )
    {
    telemetry_state = TELEMETRY_STATE_BLOCKING; /* cancel telemetry FSM */
    return;
    }

/* update the current telemetry state */
switch( telemetry_state )
    {
    case TELEMETRY_STATE_BLOCKING: /* FSM start */
        {
        /* Assumptions: LoRa is initialized with valid configs, telem should only initialize
           with a synchronous event from main loop. */
        if( update_cause != TELEMETRY_EVENT_SYNCHRONOUS_UPDATE )
            {
            /* do nothing */
            return;
            }
        telemetry_state = TELEMETRY_STATE_STATUS_CHECK;
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case TELEMETRY_STATE_STATUS_CHECK: /* check if in standby mode */
        {
        if( update_cause != TELEMETRY_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }

        /* check return. if not standby, cancel telem fsm and go back to blocking */
        if ((register_contents[1] & 0b111) != LORA_STANDBY_MODE)
            {
            telemetry_state = TELEMETRY_STATE_BLOCKING;
            lora_status = lora_write_register_IT(LORA_REG_OPERATION_MODE, LORA_STANDBY_MODE);
            return;
            }
        else /* success: go to next state*/
            {
            telemetry_state = TELEMETRY_STATE_GETTING_BUF;
            lora_status = lora_read_register_IT(LORA_REG_FIFO_TX_BASE_ADDR, register_contents);
            }
        return;
        }
    
    case TELEMETRY_STATE_GETTING_BUF: /* get fifo base ptr */
        {
        if( update_cause != TELEMETRY_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }

        telemetry_state = TELEMETRY_STATE_SETTING_TX_BASE;
        lora_status = lora_write_register_IT(LORA_REG_FIFO_SPI_POINTER, register_contents[1]);
        return;
        }

    case TELEMETRY_STATE_SETTING_TX_BASE: /* set tx base ptr */
        {
        if( update_cause != TELEMETRY_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }

        telemetry_state = TELEMETRY_STATE_WRITING_MSG_LEN;
        telemetry_get_next_message();
        lora_status = lora_write_register_IT(LORA_REG_SIGNAL_TO_NOISE, LORA_MESSAGE_SIZE);
        return;
        }

    case TELEMETRY_STATE_WRITING_MSG_LEN: /* write the length of the lora message */
        {
        if( update_cause != TELEMETRY_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }

        telemetry_state = TELEMETRY_STATE_WRITING_MSG;
        burst_write_buf[0] = (LORA_REG_FIFO_RW | 0x80); /* set up reg write */
        memcpy(&(burst_write_buf[1]), &payload, LORA_MESSAGE_SIZE);
        lora_status = lora_write_IT(burst_write_buf, LORA_MESSAGE_SIZE + 1);
        return;
        }
    
    case TELEMETRY_STATE_WRITING_MSG: /* writing the message itself */
        {
        if( update_cause != TELEMETRY_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* check status register */
        telemetry_state = TELEMETRY_STATE_PRE_TX_STATUS_CHECK;
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case TELEMETRY_STATE_PRE_TX_STATUS_CHECK: /* writing the message itself */
        {
        if( update_cause != TELEMETRY_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* switch to TX mode */
        telemetry_state = TELEMETRY_STATE_STARTING_TRANSMISSION;
        uint8_t new_opmode_register = (register_contents[1] & ~(0x7));
        new_opmode_register = (new_opmode_register | LORA_TRANSMIT_MODE);
        lora_status = lora_write_register_IT( LORA_REG_OPERATION_MODE, new_opmode_register );
        return;
        }
    
    case TELEMETRY_STATE_STARTING_TRANSMISSION:
        {
        if( update_cause != TELEMETRY_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* opmode change complete, we are now transmitting */
        telemetry_state = TELEMETRY_STATE_TRANSMITTING;
        register_contents[1] = 0xFF; /* set this to FF so we can detect when the contents have changed */
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case TELEMETRY_STATE_TRANSMITTING:
        {
        if( ( update_cause != TELEMETRY_EVENT_EXTI_RAISED )
         && ( update_cause != TELEMETRY_EVENT_REG_READ_CPLT ) )
            {
            /* do nothing */
            return;
            }
        
        if( (register_contents[1] & 0b111) == LORA_STANDBY_MODE ) 
            {
            /* transmission is complete! start the buffer retrieval operation and jump higher on the FSM */
            telemetry_state = TELEMETRY_STATE_GETTING_BUF;
            lora_status = lora_read_register_IT(LORA_REG_FIFO_TX_BASE_ADDR, register_contents);
            }
        else
            {
            lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
            }
        return;
        }
        
    }


} /* telemetry_update */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_get_next_message                                               *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Construct a new telemetry message.                                       *
*                                                                                *
* NOTE:                                                                          *
*       Future implementations might benefit from contracting this function so   *
*       each app can decide. For now (and for simplicity) we will choose.        *
*                                                                                *
*********************************************************************************/
void telemetry_get_next_message
    (
    void
    )
{
LORA_MESSAGE_TYPES msg_type;

/* Determine which payload to send */
if( ( message_idx % 2 == 0 )
    && ( get_fc_state() == FC_STATE_LAUNCH_DETECT ) )
    {
    msg_type = LORA_MSG_VEHICLE_ID;
    }
else
    {
    msg_type = LORA_MSG_DASHBOARD_DATA;
    }
telemetry_build_payload(&payload, msg_type);

} /* telemetry_get_next_message */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_payload                                                  *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Builds a LoRa payload.                                                   *
*                                                                                *
*********************************************************************************/
void telemetry_build_payload
    (
    LORA_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    LORA_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    )
{
/*------------------------------------------------------------------------------ 
 Construct Header                                                                    
------------------------------------------------------------------------------*/
memset(msg_buf, 0, LORA_MESSAGE_SIZE);
get_uid( &(msg_buf->header.uid) );
msg_buf->header.mid = message_type;
msg_buf->header.timestamp = HAL_GetTick();

/*------------------------------------------------------------------------------ 
 Build Payload
------------------------------------------------------------------------------*/
switch( message_type )
    {
    case LORA_MSG_VEHICLE_ID:
        {
        telemetry_build_msg_vehicle_id(msg_buf);
        break;
        }
    case LORA_MSG_DASHBOARD_DATA:
        {
        telemetry_build_msg_dashboard_dump(msg_buf);
        break;
        }
    case LORA_MSG_WARNING_MESSAGE: /* intentional fallthrough */
    case LORA_MSG_INFO_MESSAGE:
        {
        telemetry_build_text_message(msg_buf, message_type);
        break;
        }
    default:
        {
        error_fail_fast( ERROR_RECORD_FLIGHT_EVENTS_ERROR );
        break;
        }
    }

/*------------------------------------------------------------------------------ 
 Encrypt Message
------------------------------------------------------------------------------*/
/* not yet ready */

// ETS Temp: This may not be done by v2.6.0. I'm happy to commit to dropping
// encryption for this release since our comms are one-way
// flight critical --> non-critical, so the flight critical element is never
// receiving outside data in-flight.

} /* telemetry_build_payload */

/*------------------------------------------------------------------------------ 
 State Functions                                                                    
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
 Telemetry Message Constructors                                                                    
------------------------------------------------------------------------------*/

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_msg_vehicle_id                                           *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the vehicle id payload. Assume header is filled by caller.         *
*                                                                                *
*********************************************************************************/
static void telemetry_build_msg_vehicle_id
    (
    LORA_MESSAGE* msg_buf
    )
{
/* hardware & firmware identifiers */
msg_buf->payload.vehicle_id.hw_opcode = PING_RESPONSE_CODE;
msg_buf->payload.vehicle_id.fw_opcode = FIRMWARE_APPA;

/* version string */
msg_buf->payload.vehicle_id.version |= ( VERSION_HARDWARE << 24 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_MAJOR << 16 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_PATCH << 8 );
msg_buf->payload.vehicle_id.version |= ( VERSION_PRERELEASE_NUMBER );

/* flight id (not yet implemented) */
strncpy( msg_buf->payload.vehicle_id.flight_id, "AVIONICS_TEST", 16 );

} /* telemetry_build_msg_vehicle_id */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_msg_dashboard_dump                                       *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the dashboard dump payload. Assume header is filled by caller.     *
*                                                                                *
*********************************************************************************/
static void telemetry_build_msg_dashboard_dump
    (
    LORA_MESSAGE* msg_buf
    )
{
msg_buf->payload.dashboard_dump.fsm_state = get_fc_state();
dashboard_construct_dump( &(msg_buf->payload.dashboard_dump.data) );

} /* telemetry_build_msg_dashboard_dump */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_text_message                                             *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the payload for warning and info messages. Assume header is filled *
*       by caller.                                                               *
*                                                                                *
*********************************************************************************/
static void telemetry_build_text_message
    (
    LORA_MESSAGE* msg_buf,
    LORA_MESSAGE_TYPES message_type
    )
{
TEXT_MESSAGE text_message; /* extra copy is required due to packed struct */
switch( message_type )
    {
    case LORA_MSG_WARNING_MESSAGE:
        {
        /* return discarded; presence of warning checked earlier */
        error_get_warning( &text_message );
        break;
        }
    case LORA_MSG_INFO_MESSAGE:
        {
        /* return discarded; presence of warning checked earlier */
        error_get_info( &text_message );
        break;
        }
    /**
     * GCOVR_EXCL_START
     * 
     * Protective default case to prevent programmer error. Called by one function that will fall into one
     * of the above two cases.
     * 
     * ETS TEMP: Whoever writes this test should evaluate whether this covex works correctly. The default
     * branch should be excluded from coverage.
     */
    default:
        {
        /* shouldn't get here */
        error_fail_fast( ERROR_UNSUPPORTED_OP_ERROR );
        break;
        }
    /**
     * GCOVR_EXCL_STOP
     */
    }

memcpy( &(msg_buf->payload.text_message.msg), &text_message, sizeof( TEXT_MESSAGE ) );

} /* telemetry_build_text_message */