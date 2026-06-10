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


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE        ( 0x01 )
#define SENSOR_POLL_CODE        ( 0x02 )

/* Max allowed number of sensors for polling */
#define SENSOR_MAX_NUM_POLL     ( 5    )

#if defined( A0002_REV2 )
	#define ACCEL_G_RANGE ( 16 ) /* Accelerometer measurement range */
	#define GYRO_RANGE ( 2000 )  /* Gyroscope sensitivity in degrees/sec */
#endif

/* General */
#define NUM_SENSORS         ( 38   )
// #define IMU_DATA_SIZE       ( 20   )
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
	SENSOR_POLL_FAIL_TO_START    ,
	SENSOR_POLL_FAIL             ,
	SENSOR_POLL_UNRECOGNIZED_CMD ,
	SENSOR_VALVE_UART_ERROR      ,
	SENSOR_ADC_POLL_ERROR        ,
    SENSOR_FAIL   				 ,
	SENSOR_IT_TIMEOUT
    } SENSOR_STATUS;

/* Sensor poll command codes */
typedef enum
	{
	SENSOR_POLL_START   = 0xF3,
	SENSOR_POLL_REQUEST = 0x51,
	SENSOR_POLL_WAIT    = 0x44,
	SENSOR_POLL_RESUME  = 0xEF,
	SENSOR_POLL_STOP    = 0x74
	} SENSOR_POLL_CMD;

/* Sensor idenification code instance*/
typedef uint8_t SENSOR_ID;

/* Sensor Names/codes */
typedef enum
	{
	SENSOR_ACCX_CONV 	= 0x00,
	SENSOR_ACCY_CONV 	= 0x01,
	SENSOR_ACCZ_CONV 	= 0x02,
	SENSOR_GYROX_CONV 	= 0x03,
	SENSOR_GYROY_CONV 	= 0x04,
	SENSOR_GYROZ_CONV 	= 0x05,
	SENSOR_MAGX_CONV 	= 0x06,
	SENSOR_MAGY_CONV 	= 0x07,
	SENSOR_MAGZ_CONV 	= 0x08,
	SENSOR_ROLL_DEG 	= 0x09,
	SENSOR_PITCH_DEG 	= 0x0A,
	SENSOR_YAW_DEG		= 0x0B,
	SENSOR_ROLL_RATE 	= 0x0C,
	SENSOR_PITCH_RATE 	= 0x0D,
	SENSOR_YAW_RATE		= 0x0E,
	SENSOR_VELOCITY 	= 0x0F,
	SENSOR_VELO_X		= 0x10,
	SENSOR_VELO_Y		= 0x11,
	SENSOR_VELO_Z		= 0x12,
	SENSOR_POSITION 	= 0x13,
	SENSOR_PRES  		= 0x14,
	SENSOR_TEMP  		= 0x15,
	SENSOR_BARO_ALT		= 0x16,
	SENSOR_BARO_VELO	= 0x17,
	SENSOR_GPS_ALT		= 0x18,
	SENSOR_GPS_SPEED	= 0x19,
	SENSOR_GPS_TIME		= 0x1A,
	SENSOR_GPS_DEC_LONG	= 0x1B,
	SENSOR_GPS_DEC_LAT 	= 0x1C,
	SENSOR_GPS_NS		= 0x1D,
	SENSOR_GPS_EW		= 0x1E,
	SENSOR_GPS_GLL		= 0x1F,
	SENSOR_GPS_RMC		= 0x20,
	} SENSOR_IDS;

/* Sensor Data */
typedef struct SENSOR_DATA 
	{
	IMU_DATA imu_data;
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


/* Sensor Data sizes and offsets */
typedef struct SENSOR_DATA_SIZE_OFFSETS
	{
	uint8_t offset;  /* Offset of sensor readout in SENSOR_DATA struct  */
	size_t  size;    /* Size of readout in bytes                        */
	} SENSOR_DATA_SIZE_OFFSETS;


/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/* Initialize the sensor module */
void sensor_init 
	(
	void
	);

/* Execute a sensor subcommand */
SENSOR_STATUS sensor_cmd_execute
	(
	uint8_t subcommand
    );

/* Poll specific sensors on the board */
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID* sensor_ids_ptr  ,
	uint8_t    num_sensors
	);

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );

#ifdef FLIGHT_COMPUTER
void sensor_initialize_tick(void);
void sensor_reset_velo(void);
void sensor_body_state(IMU_DATA* imu_data);
void sensor_imu_velo(IMU_DATA* imu_data);
void sensor_conv_imu(IMU_DATA* imu_data, IMU_RAW* imu_raw);
float sensor_acc_conv(int16_t readout);
float sensor_gyro_conv(int16_t readout);
void sensor_baro_velo(SENSOR_DATA* sensor_data_ptr);
#endif

#ifdef A0002_REV2 
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
