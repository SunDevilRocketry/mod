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
#include <string.h>

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

static uint32_t      message_idx = 0;

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/

static void telemetry_build_msg_vehicle_id
    (
    TELEMETRY_MESSAGE* msg_buf
    );


static void telemetry_build_msg_calibration
    (
    TELEMETRY_MESSAGE* msg_buf
    );


static void telemetry_build_msg_dashboard_dump
    (
    TELEMETRY_MESSAGE* msg_buf
    );

/*------------------------------------------------------------------------------ 
 Public APIs                                                                    
------------------------------------------------------------------------------*/


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
    TELEMETRY_MESSAGE* payload /* o: constructed telemetry message */
    )
{
TELEMETRY_MESSAGE_TYPES msg_type;

/* Determine which payload to send */
if( ( message_idx % 3 == 0 )
    && ( get_fc_state() == FC_STATE_LAUNCH_DETECT ) )
    {
    msg_type = TELEMETRY_MSG_VEHICLE_ID;
    }
else if( ( message_idx % 3 == 1 )
    && ( get_fc_state() == FC_STATE_LAUNCH_DETECT ) )
    {
    msg_type = TELEMETRY_MSG_CALIBRATION;
    }
else
    {
    msg_type = TELEMETRY_MSG_DASHBOARD_DATA;
    }
telemetry_build_payload(payload, msg_type);
message_idx++;

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
    TELEMETRY_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    TELEMETRY_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    )
{
/*------------------------------------------------------------------------------ 
 Construct Header                                                                    
------------------------------------------------------------------------------*/
memset(msg_buf, 0, TELEMETRY_MESSAGE_SIZE);
msg_buf->header.mid = message_type;
msg_buf->header.timestamp = HAL_GetTick();

/*------------------------------------------------------------------------------ 
 Build Payload
------------------------------------------------------------------------------*/
switch( message_type )
    {
    case TELEMETRY_MSG_VEHICLE_ID:
        {
        telemetry_build_msg_vehicle_id(msg_buf);
        break;
        }
    case TELEMETRY_MSG_DASHBOARD_DATA:
        {
        telemetry_build_msg_dashboard_dump(msg_buf);
        break;
        }
    case TELEMETRY_MSG_CALIBRATION:
        {
        telemetry_build_msg_calibration(msg_buf);
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
    TELEMETRY_MESSAGE* msg_buf
    )
{
/* hardware & firmware identifiers */
msg_buf->payload.vehicle_id.hw_opcode = PING_RESPONSE_CODE;
msg_buf->payload.vehicle_id.fw_opcode = FIRMWARE_APPA;
get_uid( &(msg_buf->payload.vehicle_id.uid) );

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
* 		telemetry_build_msg_calibration                                          *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the calibration payload. Assume header is filled by caller.        *
*                                                                                *
*********************************************************************************/
static void telemetry_build_msg_calibration
    (
    TELEMETRY_MESSAGE* msg_buf
    )
{
/*------------------------------------------------------------------------------ 
 Local Variables                                    
------------------------------------------------------------------------------*/
SENSOR_DATA qfe_sensor_data;

memset( &qfe_sensor_data, 0, sizeof( SENSOR_DATA ) );

/*------------------------------------------------------------------------------ 
 Construct known elements from preset data                                      
------------------------------------------------------------------------------*/
msg_buf->payload.calibration.imu_offset = preset_data.imu_offset;
msg_buf->payload.calibration.baro_preset = preset_data.baro_preset;
msg_buf->payload.calibration.servo_preset = preset_data.servo_preset;

/*------------------------------------------------------------------------------ 
 Determine QFE reference elevation
------------------------------------------------------------------------------*/
qfe_sensor_data.baro_pressure = preset_data.baro_preset.baro_pres;
qfe_sensor_data.baro_temp = preset_data.baro_preset.baro_temp;
sensor_baro_alt( &qfe_sensor_data );

msg_buf->payload.calibration.qfe_elevation = qfe_sensor_data.baro_alt;

} /* telemetry_build_msg_calibration */


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
    TELEMETRY_MESSAGE* msg_buf
    )
{
msg_buf->payload.dashboard_dump.fsm_state = get_fc_state();
dashboard_construct_dump( &(msg_buf->payload.dashboard_dump.data) );

} /* telemetry_build_msg_dashboard_dump */
