/*******************************************************************************
*
* FILE: 
* 		sensor_fusion.c
*
* DESCRIPTION: 
* 		Contains functions to obtain sensor fusion and post processing
*       calculation of the sensor data
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
	#include "baro.h"
#endif
#include "usb.h"
#include "sensor.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
#ifdef FLIGHT_COMPUTER
const float PRESSURE_SEA_LEVEL = 101325;
const float EXP = 0.1903;
const float TEMP_LAPSE_RATE = 0.0065;

extern uint32_t tdelta, previous_time;
extern IMU_OFFSET imu_offset;

// float velo_x_prev, velo_y_prev, velo_z_prev = 0.0;
extern float velo_x_prev, velo_y_prev, velo_z_prev ;
float baro_velo_prev, baro_alt_prev = 0.0;
#endif


/*------------------------------------------------------------------------------
 API Functions for Flight Computer
------------------------------------------------------------------------------*/
#ifdef FLIGHT_COMPUTER

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_eliminate_accel_offset                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Eliminate accleration initial state for sensor ground calibration      *
*                                                                              *
*******************************************************************************/
void sf_eliminate_accel_offset(IMU_DATA* imu_data_ptr){
	imu_data_ptr->imu_converted.accel_x = imu_data_ptr->imu_converted.accel_x - imu_offset.accel_x;
	imu_data_ptr->imu_converted.accel_y = imu_data_ptr->imu_converted.accel_y - imu_offset.accel_y;
	imu_data_ptr->imu_converted.accel_z = imu_data_ptr->imu_converted.accel_z - imu_offset.accel_z;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_eliminate_gyro_offset                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Eliminate gyro initial state for sensor ground calibration             *
*                                                                              *
*******************************************************************************/
void sf_eliminate_gyro_offset(IMU_DATA* imu_data_ptr){
    imu_data_ptr->imu_converted.gyro_x = imu_data_ptr->imu_converted.gyro_x - imu_offset.gyro_x;
	imu_data_ptr->imu_converted.gyro_y = imu_data_ptr->imu_converted.gyro_y - imu_offset.gyro_y;
	imu_data_ptr->imu_converted.gyro_z = imu_data_ptr->imu_converted.gyro_z - imu_offset.gyro_z;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_roll_angle                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain roll angle                                        *           
*                                                                              *
*******************************************************************************/
void sf_get_roll_angle(IMU_DATA* imu_data_ptr){
    float roll;
    
    // Calculate roll angle
    roll = atanf( imu_data_ptr->imu_converted.accel_z / imu_data_ptr->imu_converted.accel_y );

    // Store calculated data
    imu_data_ptr->state_estimate.roll_angle = roll;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_pitch_angle                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain pitch angle                                       *           
*                                                                              *
*******************************************************************************/
void sf_get_pitch_angle(IMU_DATA* imu_data_ptr){
    float pitch;
    float g = 9.8;
    
    // Calculate pitch angle
	pitch = atanf( imu_data_ptr->imu_converted.accel_x / g );

    // Store calculated data
    imu_data_ptr->state_estimate.pitch_angle = pitch;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_roll_rate                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain roll angle rate                                   * 
*       Note: this requires pitch and roll values                              *           
*                                                                              *
*******************************************************************************/
void sf_get_roll_rate(IMU_DATA* imu_data_ptr){
    float roll_rate;
    float roll, pitch;
    float g = 9.8;
    
    roll = atanf( imu_data_ptr->imu_converted.accel_z / imu_data_ptr->imu_converted.accel_y );
	pitch = atanf( imu_data_ptr->imu_converted.accel_x / g );

    // Calculate roll rate
	roll_rate = imu_data_ptr->imu_converted.gyro_x + 									\
        imu_data_ptr->imu_converted.gyro_y * ( sinf(roll)*tanf(pitch) ) +	 		\
        imu_data_ptr->imu_converted.gyro_z * ( cosf(roll)*tanf(pitch) );

    // Store calculated data
    imu_data_ptr->state_estimate.roll_rate = roll_rate;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_pitch_rate                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain pitch angle rate                                  * 
*       Note: this requires roll values                                        *           
*                                                                              *
*******************************************************************************/
void sf_get_pitch_rate(IMU_DATA* imu_data_ptr){
    float pitch_rate;
    float roll;
    
    roll = atanf( imu_data_ptr->imu_converted.accel_z / imu_data_ptr->imu_converted.accel_y );

    // Calculate pitch rate
    pitch_rate = imu_data_ptr->imu_converted.gyro_y * ( cosf(roll) ) - imu_data_ptr->imu_converted.gyro_z * ( sinf(roll) );

    // Store calculated data
    imu_data_ptr->state_estimate.pitch_rate = pitch_rate;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_imu_velo_x                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain accleration derived x-axis velocity               * 
*                                                                              *
*******************************************************************************/
void sf_get_imu_velo_x(IMU_DATA* imu_data_ptr){
    float velo_x, accel_x;
    accel_x = imu_data_ptr->imu_converted.accel_x;

    // Calculate velo_x
    velo_x = velo_x_prev + accel_x*tdelta/1000;

    velo_x_prev = velo_x;

    // Store calculated data
    imu_data_ptr->state_estimate.velo_x = velo_x;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_imu_velo_y                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain accleration derived y-axis velocity               * 
*                                                                              *
*******************************************************************************/
void sf_get_imu_velo_y(IMU_DATA* imu_data_ptr){
    float velo_y, accel_y;
    accel_y = imu_data_ptr->imu_converted.accel_y;

    // Calculate velo_x
    velo_y = velo_y_prev + accel_y*tdelta/1000;

    velo_y_prev = velo_y;

    // Store calculated data
    imu_data_ptr->state_estimate.velo_y = velo_y;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_imu_velo_z                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain accleration derived z-axis velocity               * 
*                                                                              *
*******************************************************************************/
void sf_get_imu_velo_z(IMU_DATA* imu_data_ptr){
    float velo_z, accel_z;
    accel_z = imu_data_ptr->imu_converted.accel_z;

    // Calculate velo_x
    velo_z = velo_z_prev + accel_z*tdelta/1000;

    velo_z_prev = velo_z;

    // Store calculated data
    imu_data_ptr->state_estimate.velo_z = velo_z;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_imu_velocity                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain accleration derived scalar velocity               * 
*                                                                              *
*******************************************************************************/
void sf_get_imu_velocity(IMU_DATA* imu_data_ptr){
    float velo_x, velo_y, velo_z, velocity;

    // Calculate all axis of velocity
    sf_get_imu_velo_x(imu_data_ptr);
    sf_get_imu_velo_y(imu_data_ptr);
    sf_get_imu_velo_z(imu_data_ptr);
    
    velo_x = imu_data_ptr->state_estimate.velo_x;
    velo_y = imu_data_ptr->state_estimate.velo_y;
    velo_z = imu_data_ptr->state_estimate.velo_z;

    // Calculate the velocity scalar
    velocity = sqrtf(powf(velo_x, 2.0) + powf(velo_y, 2.0) + powf(velo_z, 2.0));

    // Store calculated data
    imu_data_ptr->state_estimate.velocity = velocity;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_baro_alt                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain barometric altitude                               * 
*                                                                              *
*******************************************************************************/
void sf_get_baro_alt(SENSOR_DATA* sensor_data_ptr){
    float pres, temp, alt;

    pres = sensor_data_ptr->baro_pressure;
    temp = sensor_data_ptr->baro_temp;

    alt = (powf(PRESSURE_SEA_LEVEL / pres, EXP) - 1) * (temp + 273.15) / TEMP_LAPSE_RATE;

    sensor_data_ptr->baro_alt = alt;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sf_get_baro_velo                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate and obtain barometric velocity                               * 
*                                                                              *
*******************************************************************************/
void sf_get_baro_velo(SENSOR_DATA* sensor_data_ptr){
    float alt, velo;

    sf_get_baro_alt(sensor_data_ptr);

    alt = sensor_data_ptr->baro_alt;

    velo = (alt - baro_alt_prev)/(tdelta/1000);

    baro_alt_prev = alt;

    sensor_data_ptr->baro_velo = velo;
}

#endif