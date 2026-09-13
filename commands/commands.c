/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
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
#include "usb.h"
#include "sensor.h"

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
    void
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
usb_transmit( &response, sizeof( response ), HAL_DEFAULT_TIMEOUT );

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
/* Quats */
dump_buffer_ptr->attitude = sensor_data.state_estimate.attitude;

/* Baro */
dump_buffer_ptr->alt = sensor_data.baro_alt;

/* GPS */
dump_buffer_ptr->longitude = sensor_data.gps_dec_longitude;
dump_buffer_ptr->latitude = sensor_data.gps_dec_latitude;

/* Controls */
dump_buffer_ptr->acc_x = sensor_data.imu_converted.accel_x;
dump_buffer_ptr->roll_rate = sensor_data.state_estimate.roll_rate;

}
#endif


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/