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
TEST_ASSERT_EQ_UINT( "Unsupported telemetry message reports one error.", get_error_fail_fast_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Unsupported telemetry message reports the record-flight-events error.", get_reported_error(), ERROR_RECORD_FLIGHT_EVENTS_ERROR );

} /* test_telemetry_invalid_message_type */


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
