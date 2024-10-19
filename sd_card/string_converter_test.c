
/*******************************************************************************
*
* FILE: 
* 		string_converter_test.c
*
* DESCRIPTION: 
* 		Testing string converter 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER  )
	#include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION    )
	#include "sdr_pin_defines_A0005.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "string_converter_test.h"
#include "sensor.h"
#include <string.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		reverseString                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Reverse string created by the String converter functions to be in      *
*       the correct order                                                      *
*                                                                              *
*******************************************************************************/

void reverseString
    (
    char* pnum_str, /* Pointer to the string of a number */
    int len         /* The actual length of the string's length */
    )
{
for(int i = 0; i<len/2; i++)
    {
    char temp = pnum_str[i];
    pnum_str[i] = pnum_str[len-i-1];
    pnum_str[len-i-1] = temp;
    }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		int2str                                                                *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Function that converts integer to string and returns the size          *
*       length of the string                                                   *
*                                                                              *
*******************************************************************************/

int int2str
    (
    int num,        /* Integer number to be converted */
    char num_str[]  /* String of number to be returned by this function */
    )
{
int index = 0;
while ( num/10 != 0 )
    {
    num_str[index] = ( num%10 ) + '0';
    num = num / 10;
    index++;
    }
num_str[index] = ( num%10 ) + '0';
index++;
reverseString( num_str, index );        /* Reverse string to be in the correct order*/

return index;                          /* String size = last index + 1 */;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		float2str                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Function that converts float to string                                 *
*                                                                              *
*******************************************************************************/

void float2str
    (
    float float_num,    /* Float number to be converted to string */
    char* pnum_str,     /* Pointer to the string of a number */
    int str_len,        /* Length (size) of the string */
    int decimal         /* Numbers after a decimal point */
    )
{
memset(pnum_str, 0, sizeof(char) * str_len ); /* Clean string array every convert */
int integer_num = (int) float_num;
float fractional_part = float_num - (float) integer_num;

fractional_part = fractional_part * pow( 10, decimal );

int i = int2str( integer_num, pnum_str );
pnum_str[i] = '.';

int2str( (int) fractional_part, pnum_str + i + 1 );
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		dataframe_to_string                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Convert sensor dataframe to string	                                   *
*                                                                              *
*******************************************************************************/

void dataframe_to_string
	(
	SENSOR_DATA*  sensor_data_ptr, /* Sensor dataframe to be converted */
	uint32_t	  time,			   /* Extracted time value */
	char*		  pbuffer_str  	   /* Converted data to be returned */
	)
{
uint16_t accel_x 		= sensor_data_ptr->imu_data.accel_x;
uint16_t accel_y 		= sensor_data_ptr->imu_data.accel_y;
uint16_t accel_z 		= sensor_data_ptr->imu_data.accel_z;
uint16_t gyro_x 		= sensor_data_ptr->imu_data.gyro_x;
uint16_t gyro_y 		= sensor_data_ptr->imu_data.gyro_y;
uint16_t gyro_z 		= sensor_data_ptr->imu_data.gyro_z;
uint16_t mag_x 			= sensor_data_ptr->imu_data.mag_x;
uint16_t mag_y 			= sensor_data_ptr->imu_data.mag_y;
uint16_t mag_z 			= sensor_data_ptr->imu_data.mag_z;
// uint32_t baro_pressure  = (uint32_t) sensor_data.baro_pressure;
// uint32_t baro_temp		= (uint32_t) sensor_data.baro_temp;
float 	 baro_pressure  = sensor_data_ptr->baro_pressure;
float	 baro_temp		= sensor_data_ptr->baro_temp;

char	 baro_temp_str[11];

float2str(baro_temp, baro_temp_str, sizeof(baro_temp_str) / sizeof(baro_temp_str[0]) , 2);
char	 baro_pressure_str[11];

float2str(baro_pressure, baro_pressure_str, sizeof(baro_pressure_str) / sizeof(baro_pressure_str[0]), 2);
sprintf(
		pbuffer_str, 
		"time: %d,accelX: %d,accelY: %d,accelZ: %d,"
        "gyroX: %d,gyroY: %d,gyroZ: %d,"
        "magX: %d,magY: %d,magZ: %d,"
        "baro_pres: %s,baro_temp: %s",
		time, accel_x, accel_y, accel_z,
		gyro_x, gyro_y, gyro_z, 
		mag_x, mag_y, mag_z, 
		baro_pressure_str, baro_temp_str
		);
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/