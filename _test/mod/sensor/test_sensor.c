/*******************************************************************************
*
* FILE:
*      test_sensor.c
*
* DESCRIPTION:
*      Unit tests for error handling in the sensor module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdint.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "sensor.h"
#include "test_sensor_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
GPS_DATA gps_data;
IMU_OFFSET imu_offset;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_start_it                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test every condition and decision in sensor_start_IT.                   *
*                                                                              *
*******************************************************************************/
void test_sensor_start_it
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data;

/*------------------------------------------------------------------------------
Case 1: IMU start failure
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_FAIL );
set_start_baro_status( BARO_OK );

TEST_ASSERT_EQ_UINT( "IMU start failure returns SENSOR_IMU_FAIL.", sensor_start_IT( &sensor_data ), SENSOR_IMU_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer is not started after IMU failure.", get_start_baro_calls(), 0 );

/*------------------------------------------------------------------------------
Case 2: IMU succeeds and barometer start fails
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_OK );
set_start_baro_status( BARO_FAIL );

TEST_ASSERT_EQ_UINT( "Barometer start failure returns SENSOR_BARO_ERROR.", sensor_start_IT( &sensor_data ), SENSOR_BARO_ERROR );
TEST_ASSERT_EQ_UINT( "Barometer is started after IMU success.", get_start_baro_calls(), 1 );

/*------------------------------------------------------------------------------
Case 3: IMU and barometer start successfully
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_OK );
set_start_baro_status( BARO_OK );

TEST_ASSERT_EQ_UINT( "Successful sensor start returns SENSOR_OK.", sensor_start_IT( &sensor_data ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "IMU is started for the success path.", get_start_imu_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Barometer is started for the success path.", get_start_baro_calls(), 1 );

} /* test_sensor_start_it */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the sensor error tests.          *
*                                                                              *
*******************************************************************************/
int main
	(
	void
	)
{
/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "Sensor: Start IT Error Handling", test_sensor_start_it }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "sensor", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
