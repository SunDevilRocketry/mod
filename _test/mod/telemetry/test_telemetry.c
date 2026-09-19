/*******************************************************************************
*
* FILE:
*      test_telemetry.c
*
* DESCRIPTION:
*      Unit tests for error handling in the telemetry module.
*      Most of the telemetry module is covered by integration tests.
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
#include "main.h"
#include "telemetry.h"
#include "test_telemetry_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
PRESET_DATA preset_data;
SENSOR_DATA sensor_data;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_telemetry_invalid_message_type                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test that an unsupported message type reports the correct error.        *
*                                                                              *
*******************************************************************************/
void test_telemetry_invalid_message_type
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
TELEMETRY_MESSAGE message;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
telemetry_build_payload( &message, ( TELEMETRY_MESSAGE_TYPES )0xDEADBEEF );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Unsupported telemetry message does not fail fast in release builds.", get_error_fail_fast_calls(), 0 );

} /* test_telemetry_invalid_message_type */


void test_telemetry_next_message
	(
	void
	)
{
TELEMETRY_MESSAGE message;

stubs_reset();
set_fc_state( FC_STATE_LAUNCH_DETECT );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "First launch-detect message is vehicle identification.", message.header.mid, TELEMETRY_MSG_VEHICLE_ID );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "Second launch-detect message is calibration.", message.header.mid, TELEMETRY_MSG_CALIBRATION );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "Third launch-detect message is dashboard data.", message.header.mid, TELEMETRY_MSG_DASHBOARD_DATA );

} /* test_telemetry_next_message */


void test_telemetry_vehicle_id
	(
	void
	)
{
TELEMETRY_MESSAGE message;

telemetry_build_payload( &message, TELEMETRY_MSG_VEHICLE_ID );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload has the requested message type.", message.header.mid, TELEMETRY_MSG_VEHICLE_ID );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload uses the board identifier.", message.payload.vehicle_id.hw_opcode, PING_RESPONSE_CODE );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload uses the APPA firmware identifier.", message.payload.vehicle_id.fw_opcode, FIRMWARE_APPA );
TEST_ASSERT_EQ_MEMORY( "Vehicle ID payload contains the flight identifier.", message.payload.vehicle_id.flight_id, "AVIONICS_TEST", strlen( "AVIONICS_TEST" ) );

} /* test_telemetry_vehicle_id */


void test_telemetry_dashboard_data
	(
	void
	)
{
TELEMETRY_MESSAGE message;

set_fc_state( FC_STATE_ASCENT );
telemetry_build_payload( &message, TELEMETRY_MSG_DASHBOARD_DATA );
TEST_ASSERT_EQ_UINT( "Dashboard payload has the requested message type.", message.header.mid, TELEMETRY_MSG_DASHBOARD_DATA );
TEST_ASSERT_EQ_UINT( "Dashboard payload contains the flight state.", message.payload.dashboard_dump.fsm_state, FC_STATE_ASCENT );

} /* test_telemetry_dashboard_data */


void test_telemetry_calibration
	(
	void
	)
{
TELEMETRY_MESSAGE message;

preset_data.imu_offset.accel_x = 1.0f;
preset_data.baro_preset.baro_pres = 90000.0f;
preset_data.servo_preset.rp_servo1 = 12;
telemetry_build_payload( &message, TELEMETRY_MSG_CALIBRATION );
TEST_ASSERT_EQ_FLOAT( "Calibration payload copies IMU offsets.", message.payload.calibration.imu_offset.accel_x, 1.0f );
TEST_ASSERT_EQ_FLOAT( "Calibration payload copies barometer presets.", message.payload.calibration.baro_preset.baro_pres, 90000.0f );
TEST_ASSERT_EQ_UINT( "Calibration payload copies servo presets.", message.payload.calibration.servo_preset.rp_servo1, 12 );

} /* test_telemetry_calibration */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the telemetry error tests.      *
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
	{ "Telemetry: Next Message", test_telemetry_next_message, "RQ.MOD.00032" },
	{ "Telemetry: Vehicle ID", test_telemetry_vehicle_id, "RQ.MOD.00034" },
	{ "Telemetry: Dashboard Data", test_telemetry_dashboard_data, "RQ.MOD.00033" },
	{ "Telemetry: Calibration", test_telemetry_calibration, "RQ.MOD.00035" },
	{ "Telemetry: Invalid Message Error", test_telemetry_invalid_message_type }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "telemetry", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
