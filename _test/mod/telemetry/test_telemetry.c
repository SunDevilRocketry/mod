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
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "telemetry.h"

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

void test_telemetry
	(
	void
	)
{
TEST_ASSERT_GT_UINT( "Verify that the telemetry message structure exists.", sizeof( TELEMETRY_MESSAGE ), 0 );
TEST_ASSERT_EQ_UINT( "Verify that the defined structure matches its defined size.", sizeof( TELEMETRY_MESSAGE ), TELEMETRY_MESSAGE_SIZE );

} /* test_telemetry */


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
	{ "Telemetry Contract", test_telemetry, "RQ.MOD.00051" }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "telemetry", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
