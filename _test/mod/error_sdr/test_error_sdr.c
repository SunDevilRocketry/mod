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
TEST_ASSERT_EQ_UINT( "Table callback is called for a matching error.", get_callback_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Table callback receives the matching error.", get_callback_error(), ERROR_SENSOR_CMD_ERROR );

stubs_reset();
default_error_handler.error_callback = test_error_callback;
error_fail_fast( ERROR_BARO_CAL_ERROR );
TEST_ASSERT_EQ_UINT( "Default callback handles an error missing from the table.", get_callback_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Default callback receives the missing error.", get_callback_error(), ERROR_BARO_CAL_ERROR );

} /* test_error_sdr_callback_dispatch */


void test_error_sdr_warning_queue
	(
	void
	)
{
TEXT_MESSAGE actual;
const char* message = "warning message";

is_pending_warning = false;
TEST_ASSERT_FALSE( "Warning queue starts empty.", error_is_pending_warning() );
TEST_ASSERT_FALSE( "Empty warning queue returns false.", error_get_warning( &actual ) );

__sdr_log_warning( message );
TEST_ASSERT_TRUE( "Warning queue becomes pending.", error_is_pending_warning() );
TEST_ASSERT_TRUE( "Pending warning is returned.", error_get_warning( &actual ) );
TEST_ASSERT_EQ_UINT( "Warning records the current tick.", actual.systick, 9876 );
TEST_ASSERT_EQ_MEMORY( "Warning message is copied.", actual.message, message, strlen( message ) );
TEST_ASSERT_FALSE( "Warning is cleared after retrieval.", error_is_pending_warning() );

} /* test_error_sdr_warning_queue */


void test_error_sdr_info_queue
	(
	void
	)
{
TEXT_MESSAGE actual;
const char* message = "info message";

is_pending_info = false;
TEST_ASSERT_FALSE( "Info queue starts empty.", error_is_pending_info() );
TEST_ASSERT_FALSE( "Empty info queue returns false.", error_get_info( &actual ) );

__sdr_log_info( message );
TEST_ASSERT_TRUE( "Info queue becomes pending.", error_is_pending_info() );
TEST_ASSERT_TRUE( "Pending info is returned.", error_get_info( &actual ) );
TEST_ASSERT_EQ_UINT( "Info records the current tick.", actual.systick, 9876 );
TEST_ASSERT_EQ_MEMORY( "Info message is copied.", actual.message, message, strlen( message ) );
TEST_ASSERT_FALSE( "Info is cleared after retrieval.", error_is_pending_info() );

} /* test_error_sdr_info_queue */


int main
	(
	void
	)
{
unit_test tests[] =
	{
	{ "Error SDR: Callback Dispatch", test_error_sdr_callback_dispatch },
	{ "Error SDR: Warning Queue", test_error_sdr_warning_queue },
	{ "Error SDR: Info Queue", test_error_sdr_info_queue }
	};

TEST_INITIALIZE_TEST( "error_sdr", tests );

} /* main */
