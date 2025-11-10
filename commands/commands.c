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
#include <string.h>

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
DASHBOARD_DUMP_TYPE buffer;
SENSOR_STATUS sensor_status = SENSOR_OK;

sensor_status = sensor_dump( &sensor_data );

if ( !( sensor_status == SENSOR_OK ) )
    {
    return USB_FAIL;
    }

dashboard_construct_dump( &buffer );

return usb_transmit( &buffer, 
                        DASHBOARD_DUMP_SIZE, 
                        HAL_SENSOR_TIMEOUT /* more forgiving HW timeout */ );
} /* dashboard_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		dashboard_construct_dump                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Fill the buffer with the dashboard dump.                               *
*                                                                              *
*******************************************************************************/
void dashboard_construct_dump
    (
    DASHBOARD_DUMP_TYPE* dump_buffer_ptr /* must be DASHBOARD_DUMP_SIZE */
    )
{

/* IMU (6 axes) */
memcpy( dump_buffer_ptr,
    &(sensor_data.imu_data.imu_converted),
    sizeof( float ) * 6 );

/* Roll/Pitch + Rates */
dump_buffer_ptr->pitch_angle = sensor_data.imu_data.state_estimate.pitch_angle;
dump_buffer_ptr->roll_angle = sensor_data.imu_data.state_estimate.roll_angle;
dump_buffer_ptr->yaw_angle = sensor_data.imu_data.state_estimate.yaw_angle;
dump_buffer_ptr->pitch_rate = sensor_data.imu_data.state_estimate.pitch_rate;
dump_buffer_ptr->roll_rate = sensor_data.imu_data.state_estimate.roll_rate;
dump_buffer_ptr->yaw_rate = sensor_data.imu_data.state_estimate.yaw_rate;

/* Baro */
dump_buffer_ptr->baro_pressure = sensor_data.baro_pressure;
dump_buffer_ptr->baro_temp = sensor_data.baro_temp;
dump_buffer_ptr->baro_alt = sensor_data.baro_alt;
dump_buffer_ptr->baro_velo = sensor_data.baro_velo;

/* GPS */
dump_buffer_ptr->gps_dec_longitude = sensor_data.gps_dec_longitude;
dump_buffer_ptr->gps_dec_latitude = sensor_data.gps_dec_latitude;

}
#endif


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/