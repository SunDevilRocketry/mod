/*******************************************************************************
*
* FILE:
*      test_debug_sdr.c
*
* DESCRIPTION:
*      Unit tests for the debug_sdr module.
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
#include "debug_sdr.h"
#include "test_debug_sdr_stubs.h"

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

void test_debug_sdr_validation_and_levels
	(
	void
	)
{
const char* message = "debug";
char long_message[ DEBUG_MSG_USER_MAX_LEN + 1 ];
memset( long_message, 'x', sizeof( long_message ) );

stubs_reset();
TEST_ASSERT_EQ_UINT( "Debug init rejects a null write callback.", debug_init( NULL, test_overflow_callback ), DEBUG_FAIL );
TEST_ASSERT_EQ_UINT( "Debug init accepts a custom overflow callback.", debug_init( test_write_callback, test_overflow_callback ), DEBUG_OK );
TEST_ASSERT_EQ_UINT( "Debug log rejects a null message.", debug_log( NULL, 0, LOG_LVL_INFO ), DEBUG_FAIL );
TEST_ASSERT_EQ_UINT( "Debug log rejects an oversized message.", debug_log( long_message, sizeof( long_message ), LOG_LVL_INFO ), DEBUG_FAIL );
TEST_ASSERT_EQ_UINT( "Debug log rejects an unknown level.", debug_log( message, strlen( message ), ( DEBUG_LEVEL )99 ), DEBUG_FAIL );

TEST_ASSERT_EQ_UINT( "Info logging succeeds.", debug_log( message, strlen( message ), LOG_LVL_INFO ), DEBUG_OK );
debug_callback_handler();
TEST_ASSERT_EQ_UINT( "Warning logging succeeds.", debug_log( message, strlen( message ), LOG_LVL_WARN ), DEBUG_OK );
debug_callback_handler();
TEST_ASSERT_EQ_UINT( "Error logging succeeds.", debug_log( message, strlen( message ), LOG_LVL_ERROR ), DEBUG_OK );
debug_callback_handler();

} /* test_debug_sdr_validation_and_levels */


void test_debug_sdr_wrap_and_callback
	(
	void
	)
{
const char* message = "12345678901234567890";

stubs_reset();
debug_init( test_write_callback, test_overflow_callback );

TEST_ASSERT_EQ_UINT( "Initial logging starts a write.", debug_log( message, strlen( message ), LOG_LVL_INFO ), DEBUG_OK );
TEST_ASSERT_EQ_UINT( "A write callback was invoked.", get_write_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Logging while transmitting succeeds.", debug_log( message, strlen( message ), LOG_LVL_WARN ), DEBUG_OK );
debug_callback_handler();
TEST_ASSERT_EQ_UINT( "Callback handler transmits pending data.", get_write_calls(), 2 );
debug_callback_handler();
TEST_ASSERT_EQ_UINT( "Callback handler leaves an empty buffer idle.", get_write_calls(), 2 );

} /* test_debug_sdr_wrap_and_callback */


void test_debug_sdr_overflow
	(
	void
	)
{
char message[ 50 ];
memset( message, 'o', sizeof( message ) );

stubs_reset();
debug_init( test_write_callback, test_overflow_callback );
TEST_ASSERT_EQ_UINT( "Custom overflow returns DEBUG_OVERFLOW.", debug_log( message, sizeof( message ), LOG_LVL_ERROR ), DEBUG_OVERFLOW );
TEST_ASSERT_EQ_UINT( "Custom overflow callback is invoked.", get_overflow_calls(), 1 );

stubs_reset();
debug_init( test_write_callback, NULL );
TEST_ASSERT_EQ_UINT( "Default overflow returns DEBUG_OVERFLOW.", debug_log( message, sizeof( message ), LOG_LVL_ERROR ), DEBUG_OVERFLOW );
TEST_ASSERT_EQ_UINT( "Default overflow logs its replacement message.", get_write_calls(), 1 );

} /* test_debug_sdr_overflow */


int main
	(
	void
	)
{
unit_test tests[] =
	{
	{ "Debug SDR: Validation and Log Levels", test_debug_sdr_validation_and_levels },
	{ "Debug SDR: Wrap and Callback Handling", test_debug_sdr_wrap_and_callback },
	{ "Debug SDR: Overflow Handling", test_debug_sdr_overflow }
	};

TEST_INITIALIZE_TEST( "debug_sdr", tests );

} /* main */
