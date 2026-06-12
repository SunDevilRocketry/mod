/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
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
#include <string.h>
#include <stdbool.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"
#include "baro.h"
#include "timer.h"
#include "usb.h"
#include "sensor.h"
#include "math_sdr.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Timing (sensors) */
extern volatile uint32_t tdelta, previous_time;
uint64_t baro_velo_tick = 0;
uint64_t imu_velo_tick = 0;

extern GPS_DATA gps_data;
extern IMU_OFFSET imu_offset;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

static SENSOR_STATUS sensor_get_it_ready
	(
	uint32_t timeout
	);

static void sensor_conv_mag
	(
	IMU_DATA* imu_data, 
	IMU_RAW* imu_raw
	);


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
	uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_status;                         /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
USB_STATUS    usb_status;                            /* USB return codes      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
uint8_t       num_sensor_bytes = SENSOR_DATA_SIZE;   /* Size of data in bytes */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
usb_status      = USB_OK;
sensor_status   = SENSOR_OK;
memset( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset( &sensor_data         , 0, sizeof( sensor_data       ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{
	/*--------------------------------------------------------------------------
	 SENSOR DUMP 
	--------------------------------------------------------------------------*/
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		usb_status = usb_transmit( &num_sensor_bytes,
								   sizeof( num_sensor_bytes ), 
								   HAL_DEFAULT_TIMEOUT );

		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Get the sensor readings */
	    sensor_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_status == SENSOR_OK )
			{
			usb_transmit( &sensor_data_bytes[0], 
						sizeof( sensor_data_bytes ), 
						HAL_SENSOR_TIMEOUT );
			return ( sensor_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/*--------------------------------------------------------------------------
	 UNRECOGNIZED SUBCOMMAND 
	--------------------------------------------------------------------------*/
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
SENSOR_STATUS parallel_status; 
IMU_STATUS    imu_status;
BARO_STATUS   baro_status;
IMU_RAW       imu_raw;

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
parallel_status = SENSOR_OK;
imu_status      = IMU_OK;
baro_status     = BARO_OK;

/* Poll Sensors  */

/*Call sensor API functions*/

/* check that IMU & BARO are ready to be read */
parallel_status = sensor_get_it_ready( HAL_DEFAULT_TIMEOUT );

if( parallel_status != SENSOR_OK ) 
	{
	return parallel_status; 
	}

/* Disabling interrupts to avoid race conditions */
sensor_mutex_reserve();

/* CRITICAL SECTION BEGIN */

memset( &(imu_raw), 0, sizeof( IMU_RAW ) );

/* GPS sensor */
sensor_data_ptr->gps_altitude_ft	= gps_data.altitude_ft;
sensor_data_ptr->gps_speed_kmh		= gps_data.speed_km;
sensor_data_ptr->gps_utc_time 		= gps_data.utc_time;
sensor_data_ptr->gps_dec_longitude 	= gps_data.dec_longitude;
sensor_data_ptr->gps_dec_latitude 	= gps_data.dec_latitude;
sensor_data_ptr->gps_ns		        = gps_data.ns;
sensor_data_ptr->gps_ew				= gps_data.ew;
sensor_data_ptr->gps_gll_status		= gps_data.gll_status;
sensor_data_ptr->gps_rmc_status		= gps_data.rmc_status;

/* IMU Read */
imu_status = get_imu_it( &imu_raw );

/* Baro Read */
baro_status = get_baro_it( &(sensor_data_ptr->baro_pressure), &(sensor_data_ptr->baro_temp) );

/*Compute State Estimations*/

/* Calculated and retrieve converted IMU data */
sensor_conv_imu( &(sensor_data_ptr->imu_data), &imu_raw );

/* Calculated to get body state */
sensor_body_state( &(sensor_data_ptr->imu_data) );

/* Calculated velocity and position */
sensor_imu_velo( &(sensor_data_ptr->imu_data) );

/* Calculated velocity from barometer */
sensor_baro_velo( sensor_data_ptr );

/* CRITICAL SECTION END */

/* Re-enabling interrupts after potentially dangerous reads/writes occur */
sensor_mutex_release();

/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/

/* Start next measurement and return status */
parallel_status |= sensor_start_IT( sensor_data_ptr );

if( imu_status != IMU_OK )
	{
	return SENSOR_IMU_FAIL;
	}
else if ( baro_status != BARO_OK)
	{
	return SENSOR_BARO_ERROR;
	}
else if ( parallel_status != SENSOR_OK )
	{
	return SENSOR_IT_TIMEOUT;
	}
else
	{
	return SENSOR_OK;
	}
} /* sensor_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_initialize_tick                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the initial values for baro and imu tick at calibration            *
*                                                                              *
*******************************************************************************/
void sensor_initialize_tick
	(
	void
	)
{
baro_velo_tick = get_us_tick();
imu_velo_tick = baro_velo_tick;

} /* sensor_initialize_tick */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_conv_imu                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Conversion of IMU raw chip readouts into 9-axis Accelerometer and Gyro.*
*                                                                              *
*******************************************************************************/
void sensor_conv_imu
	(
	IMU_DATA* imu_data, 
	IMU_RAW* imu_raw
	)
{
/* Convert raw accel values */
imu_data->imu_converted.accel_x = sensor_acc_conv(imu_raw->accel_x);
imu_data->imu_converted.accel_y = sensor_acc_conv(imu_raw->accel_y);
imu_data->imu_converted.accel_z = sensor_acc_conv(imu_raw->accel_z);

/* Do not use offset compensation for accel to preserve gravity */
/*
imu_data->imu_converted.accel_x -= imu_offset.accel_x;
imu_data->imu_converted.accel_y -= imu_offset.accel_y;
imu_data->imu_converted.accel_z -= imu_offset.accel_z;
*/

/* Convert raw gyroscope values to deg/s */
imu_data->imu_converted.gyro_x = sensor_gyro_conv(imu_raw->gyro_x);
imu_data->imu_converted.gyro_y = sensor_gyro_conv(imu_raw->gyro_y);
imu_data->imu_converted.gyro_z = sensor_gyro_conv(imu_raw->gyro_z);

/* Remove gyro bias */
imu_data->imu_converted.gyro_x -= imu_offset.gyro_x;
imu_data->imu_converted.gyro_y -= imu_offset.gyro_y;
imu_data->imu_converted.gyro_z -= imu_offset.gyro_z;

sensor_conv_mag(imu_data, imu_raw);
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_body_state                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Perform sensor fusion on imu converted data to get body rate           *
*                                                                              *
*******************************************************************************/
static uint32_t last_tick = 0;
void sensor_body_state
	(
	IMU_DATA* imu_data
	)
{
/* Determine delta T */
uint32_t now_tick = HAL_GetTick();
float dt = (now_tick - last_tick) / 1000.0f;
if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
last_tick = now_tick;

/* Copy IMU data for readability */
float ax = imu_data->imu_converted.accel_x;
float ay = imu_data->imu_converted.accel_y;
float az = imu_data->imu_converted.accel_z;

float gx = imu_data->imu_converted.gyro_x;
float gy = imu_data->imu_converted.gyro_y;
float gz = imu_data->imu_converted.gyro_z;

/* Compute pitch/roll from accelerometer */
float acc_roll  = -rad_to_deg(atan2f(ay, ax));
float acc_pitch = rad_to_deg(atan2f(-az, sqrtf(ax * ax + ay * ay)));

/* Integrate gyro data */
static float roll = 0.0f;
static float pitch = 0.0f;
static float yaw = 0.0f;

roll  += gx * dt;
pitch += gy * dt;
yaw   += gz * dt;

/* Wrap yaw to -180..180 degrees */
if (yaw > 180.0f)  yaw -= 360.0f;
if (yaw < -180.0f) yaw += 360.0f;

/* Complementary filter fusion */
roll  = COMP_ALPHA * roll  + (1.0f - COMP_ALPHA) * acc_roll;
pitch = COMP_ALPHA * pitch + (1.0f - COMP_ALPHA) * acc_pitch;
// yaw uses gyro data only

/* Compute angular rates (deg/s) */
float roll_r = deg_to_rad(roll);
float pitch_r = deg_to_rad(pitch);

float roll_rate  = gx + sinf(roll_r) * tanf(pitch_r) * gy + cosf(roll_r) * tanf(pitch_r) * gz;
float pitch_rate = cosf(roll_r) * gy - sinf(roll_r) * gz;
float yaw_rate   = (sinf(roll_r) / cosf(pitch_r)) * gy + (cosf(roll_r) / cosf(pitch_r)) * gz;

/* Store results (angles & rates in degrees / deg/s) */
imu_data->state_estimate.roll_angle  = roll;
imu_data->state_estimate.pitch_angle = pitch;
imu_data->state_estimate.yaw_angle   = yaw;
imu_data->state_estimate.roll_rate   = roll_rate;
imu_data->state_estimate.pitch_rate  = pitch_rate;
imu_data->state_estimate.yaw_rate    = yaw_rate;

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_acc_conv                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert Acc readouts to m/s^2                                          *
*                                                                              *
*******************************************************************************/
float sensor_acc_conv
	(
	int16_t readout
	)
{
/* Scale readout value from integer limits to +/- the accelerometer measurement range */
const float accel_step = (2.0f * ACCEL_G_RANGE * GRAVITY ) / UINT16_MAX;

return accel_step * readout;
 
} /* sensor_acc_conv */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_gyro_conv                                                       *   
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert gyro readouts to deg/s                                         *
*                                                                              *
*******************************************************************************/
float sensor_gyro_conv
	(
	int16_t readout
	)
{
const float gyro_sens = UINT16_MAX / ( 2.0f * GYRO_RANGE );

return readout / gyro_sens;

} /* sensor_gyro_conv */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_imu_velo                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate the velocity depending on accel 							   *
*                                                                              *
*******************************************************************************/
float velo_x_prev, velo_y_prev, velo_z_prev = 0.0;
void sensor_imu_velo(IMU_DATA* imu_data){
	float velo_x, velo_y, velo_z, velocity;

	float accel_x = imu_data->imu_converted.accel_x;
	float accel_y = imu_data->imu_converted.accel_y;
	float accel_z = imu_data->imu_converted.accel_z;

	float ts_delta;
	
	uint64_t current_tick = get_us_tick();
	uint64_t imu_tdelta = current_tick - imu_velo_tick;
	ts_delta = imu_tdelta / MICROSEC_PER_SEC;

	// Calculate 3 velocity vectors using motion equations
	velo_x = velo_x_prev + accel_x*ts_delta;
	velo_y = velo_y_prev + accel_y*ts_delta;
	velo_z = velo_z_prev + accel_z*ts_delta;

	// Calculate the velocity scalar
	velocity = sqrtf(powf(velo_x, 2.0) + powf(velo_y, 2.0) + powf(velo_z, 2.0));

	/* Update state estimations*/
	imu_data->state_estimate.velo_x = velo_x;
	imu_data->state_estimate.velo_y = velo_y;
	imu_data->state_estimate.velo_z = velo_z;

	imu_data->state_estimate.velocity = velocity;

	// Save current velocity for next computation
	velo_x_prev = velo_x;
	velo_y_prev = velo_y;
	velo_z_prev = velo_z;

	imu_data->state_estimate.position = 0; //TODO: Implement position

	imu_velo_tick = current_tick;

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_baro_velo                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate the velocity from pressure readings 						   *
*                                                                              *
*******************************************************************************/
float velo_prev, alt_prev = 0.0;
void sensor_baro_velo(SENSOR_DATA* sen_data)
{
	float velocity;

	float pressure = sen_data->baro_pressure;
	float temp = sen_data->baro_temp;
	// conv pressure to pascal for equation
	// pressure *= 6894.76;
	uint64_t current_tick = get_us_tick();
	uint64_t baro_tdelta = current_tick - baro_velo_tick;
	float ts_delta = baro_tdelta / MICROSEC_PER_SEC;

	// calc altitude
	float PRESSURE_SEA_LEVEL = 101325;
    float EXP = 0.190294958;
    float TEMP_LAPSE_RATE = 0.0065;

    float alt = (pow(PRESSURE_SEA_LEVEL / pressure, EXP) - 1) * (temp + 273.15) / TEMP_LAPSE_RATE;


	// Calculate the velocity scalar
	velocity = (alt-alt_prev)/ts_delta;
	alt_prev = alt;
	velo_prev = velocity;

	sen_data->baro_alt = alt;
	sen_data->baro_velo = velocity;

	baro_velo_tick = current_tick;

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_reset_velo                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Reset velocity values to prevent accumulation of drift                 *
*                                                                              *
*******************************************************************************/
void sensor_reset_velo
	(
	void
	)
{
velo_prev = 0;
velo_x_prev = 0;
velo_y_prev = 0;
velo_z_prev = 0;

} /* sensor_reset_velo */


#if defined( A0002_REV2 )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_start_IT                                                   	   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Signal IT enabled peripherals to collect data.                         *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_start_IT
	( 
	SENSOR_DATA* sensor_data_ptr
	)
{
if( start_imu_read_IT() != IMU_OK )
	{
	return SENSOR_IMU_FAIL;
	}
if( start_baro_read_IT() != BARO_OK )
	{
	return SENSOR_BARO_ERROR;
	}
return SENSOR_OK;

} /* sensor_start_IT */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       sensor_mutex_reserve                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Reserve the sensor data struct mutex and disable interrupts            *
*       to ISRs that will check out the mutex.                                 *
*                                                                              *
*******************************************************************************/
void sensor_mutex_reserve
    (
    void
    ) 
{
HAL_NVIC_DisableIRQ( GPS_UART_IRQn );
/* HAL_NVIC_DisableIRQ( [lora placeholder] ); */

} /* sensor_mutex_reserve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       sensor_mutex_release                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Release the sensor data struct mutex and enable interrupts             *
*       to ISRs that will check out the mutex.                                 *
*                                                                              *
*******************************************************************************/
void sensor_mutex_release
    (
    void
    ) 
{
HAL_NVIC_EnableIRQ( GPS_UART_IRQn );
/* HAL_NVIC_EnableIRQ( [lora placeholder] ); */

} /* sensor_mutex_release */

#endif


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


#ifdef A0002_REV2
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_get_it_ready                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Ensures baro & mag & imu are ready to be read from.                          *
*                                                                              *
*******************************************************************************/
static SENSOR_STATUS sensor_get_it_ready
	(
	uint32_t timeout
	)
{
/* set up timeout */
uint32_t starting_time = HAL_GetTick();
uint32_t curr_time = HAL_GetTick();
while( curr_time <= starting_time + timeout )
	{
	/* Ensure both the IMU and barometer are ready to be read */                
	if ( imu_get_imu_data_ready() 
	  && imu_get_mag_data_ready() 
	  && baro_get_baro_data_ready() ) 
		{
		return SENSOR_OK;
		}

	/* update timeout poll */
	curr_time = HAL_GetTick();
	}

return SENSOR_IT_TIMEOUT;

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_conv_mag											           	   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert raw magnetometer values to useful magnetic field data.         *
*                                                                              *
* COPYRIGHT:                                                                   *
*       This function is heavily derived from the official Bosch BMM150        *
*       driver, which is protected by the BSD-3-Clause license. This function  *
*		is exempt from any licensing that may be applied to a current/future   * 
*		Sun Devil Rocketry project. Per the terms of the BSD-3-Clause license, *
*		the following notice is retained from the source project and applies   *
*		to the procedure below.                                                *
*	              							                                   *
*		Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.		   *
*																			   *
*		BSD-3-Clause														   *
*																			   *
*		Redistribution and use in source and binary forms, with or without	   *
*		modification, are permitted provided that the following conditions are *
*		met:																   *
*																			   *
*		1. Redistributions of source code must retain the above copyright      *
*	    notice, this list of conditions and the following disclaimer.		   *
*																			   *
*		2. Redistributions in binary form must reproduce the above copyright   *
*	    notice, this list of conditions and the following disclaimer in the    *
*	    documentation and/or other materials provided with the distribution.   *
*																			   *
*		3. Neither the name of the copyright holder nor the names of its       *
*	    contributors may be used to endorse or promote products derived from   *
*	    this software without specific prior written permission. 			   *
*																			   *
*		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS	   *
*		"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT	   *
*		LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS	   *
*		FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE		   *
*		COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,   *
*		INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES			   *
*		(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR	   *
*		SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)	   *
*		HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,	   *
*		STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  *
*		IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE	   *
*		POSSIBILITY OF SUCH DAMAGE.											   *
*                                                                              *
*******************************************************************************/
static void sensor_conv_mag
	(
	IMU_DATA* imu_data, 
	IMU_RAW* imu_raw
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
MAG_TRIM mag_trim;
float mag_x;
float mag_y;
float mag_z;

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
mag_trim = imu_get_mag_trim();

/*------------------------------------------------------------------------------
 Apply Bosch compensation using factory trim values
------------------------------------------------------------------------------*/
float rhall = (imu_raw->mag_hall == 0) ? mag_trim.dig_xyz1 : imu_raw->mag_hall;

/* ---- X compensation ---- */
float process_comp_x0 = ((float)mag_trim.dig_xyz1) * 16384.0f / rhall;
float process_comp_x1 = process_comp_x0 - 16384.0f;
float process_comp_x2 = ((float)mag_trim.dig_xy2) * (process_comp_x1 * process_comp_x1 / 268435456.0f);
float process_comp_x3 = process_comp_x2 + process_comp_x1 * ((float)mag_trim.dig_xy1) / 16384.0f;
float process_comp_x4 = ((float)mag_trim.dig_x2) + 160.0f;
float process_comp_x5 = ((float)imu_raw->mag_x) * (((process_comp_x3 + 256.0f) * process_comp_x4) / 8192.0f);
mag_x = (process_comp_x5 / 16.0f) / 10.0f;  // µT

/* ---- Y compensation ---- */
float process_comp_y0 = ((float)mag_trim.dig_xyz1) * 16384.0f / rhall;
float process_comp_y1 = process_comp_y0 - 16384.0f;
float process_comp_y2 = ((float)mag_trim.dig_xy2) * (process_comp_y1 * process_comp_y1 / 268435456.0f);
float process_comp_y3 = process_comp_y2 + process_comp_y1 * ((float)mag_trim.dig_xy1) / 16384.0f;
float process_comp_y4 = ((float)mag_trim.dig_y2) + 160.0f;
float process_comp_y5 = ((float)imu_raw->mag_y) * (((process_comp_y3 + 256.0f) * process_comp_y4) / 8192.0f);
mag_y = (process_comp_y5 / 16.0f) / 10.0f;  // µT

/* ---- Z compensation ---- */
float process_comp_z0 = ((float)imu_raw->mag_z) - ((float)mag_trim.dig_z4) * 128.0f;
float process_comp_z1 = ((float)mag_trim.dig_z3) * (rhall - ((float)mag_trim.dig_xyz1)) / 4.0f;
float process_comp_z2 = (mag_trim.dig_z2 == 0) ? 0.0f :
    (((process_comp_z0 - process_comp_z1) * ((float)mag_trim.dig_z1)) /
     (((float)mag_trim.dig_z2) + ((float)mag_trim.dig_z1) * (rhall - (float)mag_trim.dig_xyz1) / 32768.0f));
mag_z = process_comp_z2 / 4.0f / 10.0f;  // µT

/*------------------------------------------------------------------------------
 Store converted field data
------------------------------------------------------------------------------*/
imu_data->imu_converted.mag_x = mag_x;
imu_data->imu_converted.mag_y = mag_y;
imu_data->imu_converted.mag_z = mag_z;
} /* sensor_conv_mag */
#endif


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/