/*******************************************************************************
*
* FILE: 
* 		sensor.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "imu.h"
#include "gps.h"

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif

typedef struct _PRESET_DATA PRESET_DATA; /* From main.h */


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE        ( 0x01 )


#if defined( A0002_REV2 )
	#define ACCEL_G_RANGE ( 16 ) /* Accelerometer measurement range */
	#define GYRO_RANGE ( 2000 )  /* Gyroscope sensitivity in degrees/sec */
#endif

/* General */
#define NUM_SENSORS         ( 38   )
#define SENSOR_DATA_SIZE	( 128   )

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Sensor status return codes */
typedef enum 
	{
    SENSOR_OK = 0                ,
	SENSOR_UNRECOGNIZED_OP       ,
	SENSOR_UNSUPPORTED_OP        ,
	SENSOR_IMU_FAIL              ,
	SENSOR_PT_ERROR              ,
	SENSOR_TC_ERROR              ,
	SENSOR_LC_ERROR              ,
	SENSOR_ACCEL_ERROR           ,
    SENSOR_GYRO_ERROR            ,
	SENSOR_MAG_ERROR             ,
	SENSOR_BARO_ERROR            ,
	SENSOR_USB_FAIL              ,
	SENSOR_UNRECOGNIZED_SENSOR_ID,
    SENSOR_FAIL   				 ,
	SENSOR_IT_TIMEOUT
    } SENSOR_STATUS;

/* Mount configuration of FC */
typedef enum 
	{
	MOUNT_ORIENTATION_Z_DOWN = -1,
	MOUNT_ORIENTATION_Z_UP	 = 1
	} MOUNT_ORIENTATION;

/* State estimation from processed sensors */
typedef struct _STATE_ESTIMATION {
    QUAT attitude;
	float roll_rate;
    float velocity;
    float velo_x;
    float velo_y;
    float velo_z;     
} STATE_ESTIMATION;

/* Sensor Data */
typedef struct SENSOR_DATA 
	{
	IMU_CONVERTED imu_converted;
    STATE_ESTIMATION state_estimate;
	float    baro_pressure; 
	float    baro_temp;	
	float	 baro_alt;
	float 	 baro_velo;
	float	 gps_altitude_ft;
	float 	 gps_speed_kmh;
	float 	 gps_utc_time;
	float	 gps_dec_longitude;
	float	 gps_dec_latitude;
	char	 gps_ns;
	char	 gps_ew;
	char	 gps_gll_status;
	char 	 gps_rmc_status;
	} SENSOR_DATA;

/* Baro Preset data */
typedef struct _BARO_PRESET
	{
	float baro_pres;
	float baro_temp;
	} BARO_PRESET;

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/* Execute a sensor subcommand */
SENSOR_STATUS sensor_cmd_execute
	(
	uint8_t subcommand
    );

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );

void sensor_init
	(
	PRESET_DATA* preset_data
	);

MOUNT_ORIENTATION get_mount_orientation
	(
	void
	);

void set_mount_orientation
	(
	MOUNT_ORIENTATION orientation
	);

/* Reset velocity values to prevent accumulation of drift */
void sensor_reset_velo
	(
	void
	);

/* Perform sensor fusion on imu converted data to get body rate */
void sensor_body_state
	(
	const IMU_CONVERTED* imu_converted,
	STATE_ESTIMATION* state_estimate
	);

/* Calculate the velocity depending on accel */
void sensor_imu_velo
	(
	const IMU_CONVERTED* imu_converted,
	STATE_ESTIMATION* state_estimate
	);

/* Conversion of IMU raw chip readouts into 9-axis Accelerometer and Gyro. */
void sensor_conv_imu
	(
	IMU_CONVERTED* imu_converted,
	IMU_RAW* imu_raw
	);

/* Convert Acc readouts to m/s^2 */
float sensor_acc_conv
	(
	int16_t readout
	);

/* Convert gyro readouts to deg/s */
float sensor_gyro_conv
	(
	int16_t readout
	);

/* Calculate the velocity from pressure readings */
void sensor_baro_velo
	(
	SENSOR_DATA* sensor_data_ptr
	);

#ifdef A0002_REV2 
/* Signal IT enabled peripherals to collect data. */
SENSOR_STATUS sensor_start_IT
	( 
	SENSOR_DATA* sensor_data_ptr 
	);

/* Reserve the sensor data struct mutex and disable interrupts to ISRs that will check out the mutex. */
void sensor_mutex_reserve
    (
    void
    );

/* Release the sensor data struct mutex and enable interrupts to ISRs that will check out the mutex. */
void sensor_mutex_release
	(
	void
	);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
