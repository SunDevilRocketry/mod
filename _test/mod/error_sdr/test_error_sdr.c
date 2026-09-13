/*******************************************************************************
*
* FILE:
*      test_error_sdr.c
*
* DESCRIPTION:
*      Unit tests for the error_sdr module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <string.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "error_sdr.h"
#include "test_error_sdr_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
volatile ERROR_CALLBACK error_callback_table[] =
	{
	{ ERROR_USB_UART_ERROR, test_error_callback },
	{ ERROR_SENSOR_CMD_ERROR, test_error_callback }
	};
uint16_t error_callback_table_size = sizeof( error_callback_table ) / sizeof( error_callback_table[0] );
extern volatile ERROR_CALLBACK default_error_handler;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

void test_error_sdr_callback_dispatch
	(
	void
	)
{
stubs_reset();
error_fail_fast( ERROR_SENSOR_CMD_ERROR );
TEST_begin_nested_case( "Callback table dispatch", "RQ.MOD.00014" );
TEST_ASSERT_EQ_UINT( "Table callback is called for a matching error.", get_callback_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Table callback receives the matching error.", get_callback_error(), ERROR_SENSOR_CMD_ERROR );
TEST_end_nested_case();

stubs_reset();
default_error_handler.error_callback = test_error_callback;
error_fail_fast( ERROR_BARO_CAL_ERROR );
TEST_begin_nested_case( "Overridable default callback", "RQ.MOD.00015" );
TEST_ASSERT_EQ_UINT( "Default callback handles an error missing from the table.", get_callback_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Default callback receives the missing error.", get_callback_error(), ERROR_BARO_CAL_ERROR );
TEST_end_nested_case();

} /* test_error_sdr_callback_dispatch */


int main
	(
	void
	)
{
unit_test tests[] =
	{
	{ "Error SDR: Callback Dispatch", test_error_sdr_callback_dispatch, "RQ.MOD.00013" }
	};

TEST_INITIALIZE_TEST( "error_sdr", tests );

} /* main */
