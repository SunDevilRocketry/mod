/**
  ******************************************************************************
  * @file           : sensor.c
  * @brief          : Contains functions to interface between SDEC terminal commands and SDR sensor APIs
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
#include "mahony.h"
#include "error_sdr.h"
#include "debug_sdr.h"

/*------------------------------------------------------------------------------
 Private Macros
------------------------------------------------------------------------------*/

/*
 * Initial Mahony gains for firmware integration.
 *
 * Proportional correction is enabled conservatively. Integral correction
 * remains disabled until the gains are tuned using stationary and flight data.
 *
 *
 * Proportional gain (KP) corrects gyro drift using the accelerometer. It is
 * deliberately low to reduce overcorrection during flight.
 *
 * Integral gain (KI) learns persistent gyro bias over time. It remains
 * disabled because an untuned integral term can accumulate incorrect
 * corrections during vibration, launch acceleration, or invalid accelerometer
 * data.
 */
#define SENSOR_MAHONY_KP    1.0f
#define SENSOR_MAHONY_KI    0.0f

/* Barometer EMA Alphas */
#define BARO_PRESS_ALPHA (0.7f)
#define BARO_TEMP_ALPHA (0.7f)


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern GPS_DATA gps_data;
extern IMU_OFFSET imu_offset;

/* Timing (sensors) */
uint64_t imu_velo_tick = 0;

/* IMU */
float velo_x_prev = 0.0f;
float velo_y_prev = 0.0f;
float velo_z_prev = 0.0f;

/* State estimation */
QUAT attitude = { 1.0f, 0.0f, 0.0f, 0.0f };


/*------------------------------------------------------------------------------
 Static Variables 
------------------------------------------------------------------------------*/
static MOUNT_ORIENTATION mount_orientation = MOUNT_ORIENTATION_IMU_NORMAL;

/*
 * Persistent attitude filter state. This instance retains the quaternion and
 * integral correction between consecutive IMU updates.
 */
static MAHONY_FILTER mahony_filter;

/* Timestamp of the previous Mahony update in microseconds. */
static uint64_t mahony_tick = 0;

static float ema_press_prev = 0.0f;
static float ema_temp_prev = 0.0f;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

static SENSOR_STATUS sensor_get_it_ready
	(
	uint32_t timeout
	);

static QUAT quat_grav_attitude
	(
	float ax,
	float ay,
	float az,
	QUAT attitude
	);

static float quat_to_yaw
	(
	QUAT q
	);

static void sensor_baro_ema
	(
	SENSOR_DATA* sen_data_ptr
	);


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Executes a sensor subcommand and transmits the requested readings.
  * @param subcommand Sensor subcommand code.
  * @return Sensor operation status.
  */
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



/**
  * @brief Reads the available sensors and fills the sensor data structure.
  * @param sensor_data_ptr Pointer to the sensor data structure to fill.
  * @return Sensor operation status.
  */
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
SENSOR_STATUS body_state_status;
IMU_STATUS    imu_status;
BARO_STATUS   baro_status;
IMU_RAW       imu_raw;

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
parallel_status 	= SENSOR_OK;
body_state_status 	= SENSOR_OK;
imu_status      	= IMU_OK;
baro_status     	= BARO_OK;

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
sensor_conv_imu( &(sensor_data_ptr->imu_converted), &imu_raw );

/* Calculated to get body state */
body_state_status = sensor_body_state( &(sensor_data_ptr->imu_converted), &(sensor_data_ptr->state_estimate) );

/* Calculated velocity and position */
sensor_imu_velo( &(sensor_data_ptr->imu_converted), &(sensor_data_ptr->state_estimate) );

/* Calculated baro exponential moving average */
sensor_baro_ema( sensor_data_ptr );

/* Calculated altitude from barometer */
sensor_baro_alt( sensor_data_ptr );

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
else if ( body_state_status != SENSOR_OK )
    {
    return body_state_status;
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


/**
  * @brief Initializes sensor timing and resets velocity state.
  * @param preset_data Pointer to the preset calibration data.
  */
SENSOR_STATUS sensor_init
    (
    PRESET_DATA* preset_data
    )
{
float ax = preset_data->imu_offset.accel_x;
float ay = preset_data->imu_offset.accel_y;
float az = preset_data->imu_offset.accel_z;

QUAT initial_attitude = quat_grav_attitude
    (
    ax,
    ay,
    az,
    IDENTITY_QUAT
    );

imu_velo_tick = get_us_tick();
mahony_tick = imu_velo_tick;

sensor_reset_velo();

MAHONY_STATUS mahony_status = mahony_init
    (
    &mahony_filter,
    initial_attitude,
    SENSOR_MAHONY_KP,
    SENSOR_MAHONY_KI
    );

if ( mahony_status != MAHONY_OK )
    {
    return( SENSOR_FAIL );
    }

return( SENSOR_OK );

} /* sensor_init */



/**
  * @brief Converts raw IMU readings into calibrated, remapped sensor data.
  * @param imu_converted Converted IMU data to fill.
  * @param imu_raw Raw IMU readouts.
  */
void sensor_conv_imu
	(
	IMU_CONVERTED* imu_converted, 
	IMU_RAW* imu_raw
	)
{
/* Convert raw accel values */ 
imu_converted->accel_x = sensor_acc_conv(imu_raw->accel_x);
imu_converted->accel_y = sensor_acc_conv(imu_raw->accel_y);
imu_converted->accel_z = sensor_acc_conv(imu_raw->accel_z);

sensor_axis_remap( &(imu_converted->accel_x), &(imu_converted->accel_y), &(imu_converted->accel_z) );

/* Do not use offset compensation for accel to preserve gravity */
/*
imu_converted.accel_x -= imu_offset.accel_x;
imu_converted.accel_y -= imu_offset.accel_y;
imu_converted.accel_z -= imu_offset.accel_z;
*/

/* Convert raw gyroscope values to deg/s and remap axes */
imu_converted->gyro_x = sensor_gyro_conv(imu_raw->gyro_x);
imu_converted->gyro_y = sensor_gyro_conv(imu_raw->gyro_y);
imu_converted->gyro_z = sensor_gyro_conv(imu_raw->gyro_z);

/* Remove gyro bias BEFORE applying axis conversion */
imu_converted->gyro_x -= imu_offset.gyro_x;
imu_converted->gyro_y -= imu_offset.gyro_y;
imu_converted->gyro_z -= imu_offset.gyro_z;

sensor_axis_remap( &(imu_converted->gyro_x), &(imu_converted->gyro_y), &(imu_converted->gyro_z) );

mag_conv_raw(imu_converted, imu_raw);
}


/**
 * @brief Get the mount orientation
 * 
 * @return The configured mount orientation 
 */
MOUNT_ORIENTATION sensor_get_mount_orientation
	(
	void
	)
{
return mount_orientation;

} /* get_mount_orientation */


/**
 * @brief Set the mount orientation
 * 
 * @param orientation The new mount orientation
 */
void sensor_set_mount_orientation
	(
	MOUNT_ORIENTATION orientation
	)
{
mount_orientation = orientation;

} /* set_mount_orientation */


/**
  * @brief Integrates gyro data to update the estimated body attitude and rate.
  * @param imu_converted Converted IMU data.
  * @param state_estimate State estimate to update.
  */
SENSOR_STATUS sensor_body_state
    (
    const IMU_CONVERTED* imu_converted,
    STATE_ESTIMATION* state_estimate
    )
{
uint64_t current_tick;
uint64_t imu_tdelta;

float delta_time_s;

bool use_accel;

VECTOR_3F gyro_body_rad_s;
VECTOR_3F accel_body_m_s2;

/* Calculate elapsed time between attitude updates using the microsecond timer. */
current_tick = get_us_tick();
imu_tdelta = current_tick - mahony_tick;

delta_time_s = (float)imu_tdelta / (float)MICROSEC_PER_SEC;

if ( mahony_tick == 0 
	|| delta_time_s <= 0.0f 
	|| delta_time_s > 1.0f )
    {
    delta_time_s = 0.01f;
    }

mahony_tick = current_tick;

/*
 * Converted gyro data is in degrees per second. Mahony requires radians per
 * second.
 */
gyro_body_rad_s.x = deg_to_rad(imu_converted->gyro_x);
gyro_body_rad_s.y = deg_to_rad(imu_converted->gyro_y);
gyro_body_rad_s.z = deg_to_rad(imu_converted->gyro_z);

/*
 * Converted accelerometer data is already in meters per second squared.
 */
accel_body_m_s2.x = imu_converted->accel_x;
accel_body_m_s2.y = imu_converted->accel_y;
accel_body_m_s2.z = imu_converted->accel_z;

/*
 * Permit accelerometer correction only before powered flight. The Mahony
 * filter still performs its own magnitude and finite-value checks.
 */
use_accel = get_fc_state() <= FC_STATE_LAUNCH_DETECT;

MAHONY_STATUS mahony_status = mahony_update_imu
    (
    &mahony_filter,
    gyro_body_rad_s,
    accel_body_m_s2,
    delta_time_s,
    use_accel
    );

if ( mahony_status != MAHONY_OK )
    {
    return SENSOR_IMU_FAIL;
    }

/*
 * Store the filter's body-to-world quaternion as the system attitude estimate.
 */
state_estimate->attitude = mahony_filter.attitude;

/*
 * Preserve the existing public roll-rate units of degrees per second.
 */
state_estimate->roll_rate = imu_converted->gyro_x;

return SENSOR_OK;

} /* sensor_body_state */



/**
  * @brief Remaps sensor axes according to the flight-computer mounting orientation.
  * @param x X-axis value to remap.
  * @param y Y-axis value to remap.
  * @param z Z-axis value to remap.
  */
void sensor_axis_remap
	(
	float* x,
	float* y,
	float* z
	)
{
*x *= mount_orientation;
(void)y;
*z *= mount_orientation;
}



/**
  * @brief Converts a raw accelerometer reading to meters per second squared.
  * @param readout Raw accelerometer readout.
  * @return Acceleration in meters per second squared.
  */
float sensor_acc_conv
	(
	int16_t readout
	)
{
/* Scale readout value from integer limits to +/- the accelerometer measurement range */
const float accel_step = (2.0f * ACCEL_G_RANGE * GRAVITY ) / UINT16_MAX;

return accel_step * readout;
 
} /* sensor_acc_conv */


/**
  * @brief Converts a raw gyro reading to degrees per second.
  * @param readout Raw gyro readout.
  * @return Angular rate in degrees per second.
  */
float sensor_gyro_conv
	(
	int16_t readout
	)
{
const float gyro_sens = UINT16_MAX / ( 2.0f * GYRO_RANGE );

return readout / gyro_sens;

} /* sensor_gyro_conv */


/**
  * @brief Calculates velocity from converted accelerometer measurements.
  * @param imu_converted Converted IMU data.
  * @param state_estimate State estimate to update.
  */
void sensor_imu_velo
	(
	const IMU_CONVERTED* imu_converted,
	STATE_ESTIMATION* state_estimate
	)
{
float velo_x, velo_y, velo_z, velocity;

/*
 * The world frame uses North-East-Down (NED) coordinates,
 * so gravity points in the world-frame +Z direction.
 */
const QUAT gravity_world =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = -GRAVITY
    };

/*
 * The attitude quaternion is body-to-world, so rotate world gravity
 * into the body frame before subtracting it from the accelerometer.
 */
QUAT gravity_body = quat_rotate_world_to_body
    (
    state_estimate->attitude,
    gravity_world
    );

QUAT linear_accel_body =
    {
    .w = 0.0f,
    .x = imu_converted->accel_x - gravity_body.x,
    .y = imu_converted->accel_y - gravity_body.y,
    .z = imu_converted->accel_z - gravity_body.z
    };

/*
 * Rotate gravity-compensated acceleration into the world frame so the
 * integrated velocity components remain in a fixed coordinate frame.
 */
QUAT linear_accel_world = quat_rotate_body_to_world
    (
    state_estimate->attitude,
    linear_accel_body
    );

float accel_world_x = linear_accel_world.x;
float accel_world_y = linear_accel_world.y;
float accel_world_z = linear_accel_world.z;

float ts_delta;

uint64_t current_tick = get_us_tick();
uint64_t imu_tdelta = current_tick - imu_velo_tick;
ts_delta = (float) imu_tdelta / (float) MICROSEC_PER_SEC;

// Calculate 3 velocity vectors using motion equations
velo_x = velo_x_prev + accel_world_x * ts_delta;
velo_y = velo_y_prev + accel_world_y * ts_delta;
velo_z = velo_z_prev + accel_world_z * ts_delta;

// Calculate the velocity scalar
velocity = sqrtf
    (
    velo_x * velo_x +
    velo_y * velo_y +
    velo_z * velo_z
    );

/* Update state estimations*/
state_estimate->velo_x = velo_x;
state_estimate->velo_y = velo_y;
state_estimate->velo_z = velo_z;

state_estimate->velocity = velocity;

// Save current velocity for next computation
velo_x_prev = velo_x;
velo_y_prev = velo_y;
velo_z_prev = velo_z;

imu_velo_tick = current_tick;

}


/**
  * @brief Calculates altitude from the current barometric pressure and temperature.
  * @param sen_data Sensor data structure to update.
  */
void sensor_baro_alt(SENSOR_DATA* sen_data)
{
	float pressure = sen_data->baro_pressure;
	float temp = sen_data->baro_temp;
	// conv pressure to pascal for equation
	// pressure *= 6894.76;

	// calc altitude
	float PRESSURE_SEA_LEVEL = 101325;
    float EXP = 0.190294958;
    float TEMP_LAPSE_RATE = 0.0065;

    float alt = (pow(PRESSURE_SEA_LEVEL / pressure, EXP) - 1) * (temp + 273.15) / TEMP_LAPSE_RATE;

	sen_data->baro_alt = alt;

}


/**
  * @brief Resets accumulated velocity values to prevent drift.
  */
void sensor_reset_velo
	(
	void
	)
{
velo_x_prev = 0;
velo_y_prev = 0;
velo_z_prev = 0;

} /* sensor_reset_velo */


#if defined( A0002_REV2 )

/**
  * @brief Starts interrupt-driven measurements for the enabled sensors.
  * @param sensor_data_ptr Sensor data structure associated with the readings.
  * @return Sensor operation status.
  */
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



/**
  * @brief Disables sensor-related interrupts while sensor data is accessed.
  */
void sensor_mutex_reserve
    (
    void
    ) 
{
HAL_NVIC_DisableIRQ( GPS_UART_IRQn );
/* HAL_NVIC_DisableIRQ( [lora placeholder] ); */

} /* sensor_mutex_reserve */



/**
  * @brief Re-enables sensor-related interrupts after sensor data access.
  */
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



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		quat_grav_attitude                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Computes quaternion attitude from static accelerometer data            *
*       Experiences gimbal lock at pitch = +/- 90 degrees                      *
*                                                                              *
*******************************************************************************/
static QUAT quat_grav_attitude
	(
	float ax,
	float ay,
	float az,
	QUAT attitude
	)
{
/* Compute pitch/roll from accelerometer */
float grav_pitch = atan2f(ax, sqrtf(ay * ay + az * az));
float grav_roll  = atan2f(ay, az);

float yaw = quat_to_yaw(attitude);

return eul_to_quat(yaw, grav_pitch, grav_roll);

}

/* Comment deferred until mod#132 
 Formula in https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
static float quat_to_yaw
	(
	QUAT q
	)
{
float y = 2.0f * (q.w * q.z + q.x * q.y);
float x = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);

return atan2f(y, x);
}

#ifdef A0002_REV2

/**
  * @brief Waits until the interrupt-driven sensors report ready status.
  * @param timeout Maximum wait time in milliseconds.
  * @return Sensor operation status.
  */
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
#endif


/**
  * @brief Updates the barometer exponential moving average (EMA) with the latest sensor readings.
  * @param sen_data_ptr Pointer to the sensor data structure containing barometer readings.
  */
static void sensor_baro_ema
	(
	SENSOR_DATA* sen_data_ptr
	)
{
if (ema_press_prev == 0.0f || ema_temp_prev == 0.0f)
	{
	ema_press_prev = sen_data_ptr->baro_pressure;
	ema_temp_prev = sen_data_ptr->baro_temp;
	return;

	}

ema_press_prev = ( BARO_PRESS_ALPHA * sen_data_ptr->baro_pressure) + ( (1-BARO_PRESS_ALPHA) * ema_press_prev);
ema_temp_prev = ( BARO_TEMP_ALPHA * sen_data_ptr->baro_temp) + ( (1-BARO_TEMP_ALPHA) * ema_temp_prev);

} /* sensor_baro_ema*/

/**
  * END OF FILE
  */