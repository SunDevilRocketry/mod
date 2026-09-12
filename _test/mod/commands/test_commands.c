/*******************************************************************************
*
* FILE:
*      test_commands.c
*
* DESCRIPTION:
*      Unit tests for functions in the commands module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "commands.h"
#include "main.h"
#include "test_commands_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_commands_ping                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the board-specific ping response.                                 *
*                                                                              *
*******************************************************************************/
void test_commands_ping
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
uint8_t expected_response = PING_RESPONSE_CODE;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
ping();

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Ping transmits one response byte.", get_usb_transmit_size(), sizeof( expected_response ) );
TEST_ASSERT_EQ_UINT( "Ping uses the default timeout.", get_usb_transmit_timeout(), HAL_DEFAULT_TIMEOUT );
TEST_ASSERT_EQ_MEMORY( "Ping transmits the board response code.", get_usb_transmit_buffer(), &expected_response, sizeof( expected_response ) );

} /* test_commands_ping */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_commands_dashboard_construct_dump                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test construction of the packed dashboard data payload.                *
*                                                                              *
*******************************************************************************/
void test_commands_dashboard_construct_dump
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
DASHBOARD_DUMP_TYPE actual;
DASHBOARD_DUMP_TYPE expected;

/*------------------------------------------------------------------------------
Set up sensor data
------------------------------------------------------------------------------*/
memset( &sensor_data, 0, sizeof( sensor_data ) );
sensor_data.state_estimate.attitude = ( QUAT ){ 1.0f, 2.0f, 3.0f, 4.0f };
sensor_data.baro_alt = 1234.5f;
sensor_data.gps_dec_latitude = 33.4255f;
sensor_data.gps_dec_longitude = -111.9400f;
sensor_data.imu_converted.accel_x = 9.81f;
sensor_data.state_estimate.roll_rate = -2.5f;

expected.attitude = sensor_data.state_estimate.attitude;
expected.alt = sensor_data.baro_alt;
expected.latitude = sensor_data.gps_dec_latitude;
expected.longitude = sensor_data.gps_dec_longitude;
expected.acc_x = sensor_data.imu_converted.accel_x;
expected.roll_rate = sensor_data.state_estimate.roll_rate;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_construct_dump( &actual );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump has the declared packed size.", sizeof( actual ), DASHBOARD_DUMP_SIZE );
TEST_ASSERT_EQ_MEMORY( "Dashboard dump contains the selected sensor fields.", &actual, &expected, sizeof( actual ) );

} /* test_commands_dashboard_construct_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_commands_dashboard_dump                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test dashboard transmission and sensor failure handling.                *
*                                                                              *
*******************************************************************************/
void test_commands_dashboard_dump
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
DASHBOARD_DUMP_TYPE expected;
USB_STATUS dashboard_status;

/*------------------------------------------------------------------------------
Set up sensor data
------------------------------------------------------------------------------*/
memset( &sensor_data, 0, sizeof( sensor_data ) );
sensor_data.state_estimate.attitude = ( QUAT ){ 0.1f, 0.2f, 0.3f, 0.4f };
sensor_data.baro_alt = 250.0f;
sensor_data.gps_dec_latitude = 40.0f;
sensor_data.gps_dec_longitude = -111.0f;
sensor_data.imu_converted.accel_x = 16.0f;
sensor_data.state_estimate.roll_rate = 7.0f;

expected.attitude = sensor_data.state_estimate.attitude;
expected.alt = sensor_data.baro_alt;
expected.latitude = sensor_data.gps_dec_latitude;
expected.longitude = sensor_data.gps_dec_longitude;
expected.acc_x = sensor_data.imu_converted.accel_x;
expected.roll_rate = sensor_data.state_estimate.roll_rate;

/*------------------------------------------------------------------------------
Case 1: Successful dashboard dump
------------------------------------------------------------------------------*/
stubs_reset();
set_sensor_dump_status( SENSOR_OK );
set_usb_transmit_status( USB_OK );

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_status = dashboard_dump();

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump returns the USB result.", dashboard_status, USB_OK );
TEST_ASSERT_EQ_UINT( "Dashboard dump transmits the complete payload.", get_usb_transmit_size(), DASHBOARD_DUMP_SIZE );
TEST_ASSERT_EQ_UINT( "Dashboard dump uses the sensor timeout.", get_usb_transmit_timeout(), HAL_SENSOR_TIMEOUT );
TEST_ASSERT_EQ_MEMORY( "Dashboard dump transmits constructed sensor data.", get_usb_transmit_buffer(), &expected, sizeof( expected ) );

/*------------------------------------------------------------------------------
Case 2: Sensor failure
------------------------------------------------------------------------------*/
stubs_reset();
set_sensor_dump_status( SENSOR_FAIL );

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_status = dashboard_dump();

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump rejects sensor failure.", dashboard_status, USB_FAIL );
TEST_ASSERT_EQ_UINT( "Dashboard dump does not transmit after sensor failure.", get_usb_transmit_calls(), 0 );

} /* test_commands_dashboard_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the commands tests.              *
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
	{ "Commands: Ping", test_commands_ping },
	{ "Commands: Dashboard Construction", test_commands_dashboard_construct_dump },
	{ "Commands: Dashboard Dump", test_commands_dashboard_dump }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "commands", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
