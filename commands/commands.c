/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                               
------------------------------------------------------------------------------*/
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#ifdef USE_RS485
    #include "rs485.h"
#endif
#include "usb.h"
#include "sensor.h"
#ifdef VALVE_CONTROLLER
    #include "valve.h"
#endif

/*------------------------------------------------------------------------------
 Globals 
------------------------------------------------------------------------------*/
extern SENSOR_DATA sensor_data;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ping                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sends a 1 byte response back to host PC to signal a functioning        * 
*       serial connection                                                      *
*                                                                              *
*******************************************************************************/
void ping
    (
    #ifndef VALVE_CONTROLLER
        void
    #else
        CMD_SOURCE cmd_source
    #endif
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t    response;   /* A0002 Response Code */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
response = PING_RESPONSE_CODE; /* Code specific to board and revision */



/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
#ifdef VALVE_CONTROLLER 
    if ( cmd_source == CMD_SOURCE_USB )
        {
        usb_transmit( &response         , 
                      sizeof( response ), 
                      HAL_DEFAULT_TIMEOUT );
        }
    else
        {
        valve_transmit( &response         , 
                        sizeof( response ), 
                        HAL_DEFAULT_TIMEOUT );
        }

#elifdef ENGINE_CONTROLLER
    #if defined( USE_RS485 )
        rs485_transmit( &response, sizeof( response ), RS485_DEFAULT_TIMEOUT );
    #else
        usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );
    #endif

#else

    usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );

#endif


} /* ping */


#ifdef A0002_REV2
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		dashboard_dump                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sends the data required by the dashboard.                              *
*                                                                              *
*******************************************************************************/
USB_STATUS dashboard_dump
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t buffer[DASHBOARD_DUMP_SIZE];
uint8_t idx = 0;
SENSOR_STATUS sensor_status = SENSOR_OK;

sensor_status = sensor_dump( &sensor_data );

if ( !( sensor_status == SENSOR_OK ) )
    {
    return USB_FAIL;
    }

/* IMU (6 axes) */
memcpy( &buffer[idx],
    &(sensor_data.imu_data.imu_converted),
    sizeof( IMU_CONVERTED ));
idx += sizeof( IMU_CONVERTED );

/* Baro */
memcpy( &buffer[idx], &(sensor_data.baro_pressure), sizeof(float));
idx += 4;
memcpy( &buffer[idx], &(sensor_data.baro_temp), sizeof(float));
idx += 4;
memcpy( &buffer[idx], &(sensor_data.baro_alt), sizeof(float));
idx += 4;
memcpy( &buffer[idx], &(sensor_data.baro_velo), sizeof(float));
idx += 4;

/* Roll/Pitch + Rates */
memcpy( &buffer[idx],
        &(sensor_data.imu_data.state_estimate),
        4 * sizeof( FLOAT )); /* just the first 4 */
idx += 4 * sizeof( FLOAT );

/* GPS */
memcpy( &buffer[idx], &(sensor_data.gps_dec_longitude), sizeof(float));
idx += 4;
memcpy( &buffer[idx], &(sensor_data.gps_dec_latitude), sizeof(float));
idx += 4;
memcpy( &buffer[idx], &(sensor_data.gps_ns), sizeof(char));
idx++;
memcpy( &buffer[idx], &(sensor_data.gps_ew), sizeof(char));
idx++;

// assert_fail_fast( idx == DASHBOARD_DUMP_SIZE )

return usb_transmit( buffer, 
                        idx, 
                        HAL_SENSOR_TIMEOUT /* more forgiving HW timeout */ );
} /* dashboard_dump */
#endif


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/