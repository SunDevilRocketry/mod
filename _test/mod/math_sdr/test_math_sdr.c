/*******************************************************************************
*
* FILE:
*      test_math_sdr.c
*
* DESCRIPTION:
*      Unit tests for functions in the math_sdr module.
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
#include "math_sdr.h"

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_crc32                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test CRC-32C checksums for empty and known input data.                  *
*                                                                              *
*******************************************************************************/
void test_math_sdr_crc32
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
const uint8_t test_data[] = "123456789";

/*------------------------------------------------------------------------------
Call FUT and verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "CRC-32C of an empty buffer is zero.", crc32( NULL, 0 ), 0 );
TEST_ASSERT_EQ_UINT( "CRC-32C matches the standard check value.", crc32( test_data, sizeof( test_data ) - 1 ), 0xE3069283u );

} /* test_math_sdr_crc32 */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_mult                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test quaternion multiplication.                                        *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_mult
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
QUAT input = { 2.0f, 3.0f, 4.0f, 5.0f };
QUAT actual;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
actual = quat_mult( identity, input );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_FLOAT( "Quaternion multiplication preserves the w component.", actual.w, input.w );
TEST_ASSERT_EQ_FLOAT( "Quaternion multiplication preserves the x component.", actual.x, input.x );
TEST_ASSERT_EQ_FLOAT( "Quaternion multiplication preserves the y component.", actual.y, input.y );
TEST_ASSERT_EQ_FLOAT( "Quaternion multiplication preserves the z component.", actual.z, input.z );

} /* test_math_sdr_quat_mult */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_dot                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the quaternion dot product.                                        *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_dot
	(
	void
	)
{
/*------------------------------------------------------------------------------
Call FUT and verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_FLOAT( "Quaternion dot product is calculated correctly.", quat_dot( ( QUAT ){ 1.0f, 2.0f, 3.0f, 4.0f }, ( QUAT ){ 5.0f, 6.0f, 7.0f, 8.0f } ), 70.0f );

} /* test_math_sdr_quat_dot */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_add                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test component-wise quaternion addition.                               *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_add
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
QUAT actual;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
actual = quat_add( ( QUAT ){ 1.0f, 2.0f, 3.0f, 4.0f }, ( QUAT ){ 5.0f, 6.0f, 7.0f, 8.0f } );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_FLOAT( "Quaternion addition calculates w.", actual.w, 6.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion addition calculates x.", actual.x, 8.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion addition calculates y.", actual.y, 10.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion addition calculates z.", actual.z, 12.0f );

} /* test_math_sdr_quat_add */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_scale                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test scalar quaternion multiplication.                                 *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_scale
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
QUAT actual;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
actual = quat_scale( ( QUAT ){ 1.0f, -2.0f, 3.0f, -4.0f }, 2.5f );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_FLOAT( "Quaternion scaling calculates w.", actual.w, 2.5f );
TEST_ASSERT_EQ_FLOAT( "Quaternion scaling calculates x.", actual.x, -5.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion scaling calculates y.", actual.y, 7.5f );
TEST_ASSERT_EQ_FLOAT( "Quaternion scaling calculates z.", actual.z, -10.0f );

} /* test_math_sdr_quat_scale */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_normalize                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test normal and zero-quaternion normalization behavior.                *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_normalize
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
QUAT actual;

/*------------------------------------------------------------------------------
Case 1: Non-zero quaternion
------------------------------------------------------------------------------*/
actual = quat_normalize( ( QUAT ){ 0.0f, 3.0f, 4.0f, 0.0f } );

TEST_ASSERT_EQ_FLOAT( "Quaternion normalization calculates w.", actual.w, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion normalization calculates x.", actual.x, 0.6f );
TEST_ASSERT_EQ_FLOAT( "Quaternion normalization calculates y.", actual.y, 0.8f );
TEST_ASSERT_EQ_FLOAT( "Quaternion normalization calculates z.", actual.z, 0.0f );

/*------------------------------------------------------------------------------
Case 2: Zero quaternion
------------------------------------------------------------------------------*/
actual = quat_normalize( ( QUAT ){ 0.0f, 0.0f, 0.0f, 0.0f } );

TEST_ASSERT_EQ_FLOAT( "Zero quaternion normalization returns identity w.", actual.w, 1.0f );
TEST_ASSERT_EQ_FLOAT( "Zero quaternion normalization returns identity x.", actual.x, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Zero quaternion normalization returns identity y.", actual.y, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Zero quaternion normalization returns identity z.", actual.z, 0.0f );

} /* test_math_sdr_quat_normalize */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_conj                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test quaternion conjugation.                                           *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_conj
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
QUAT actual;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
actual = quat_conj( ( QUAT ){ 1.0f, 2.0f, -3.0f, 4.0f } );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_FLOAT( "Quaternion conjugation preserves w.", actual.w, 1.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion conjugation negates x.", actual.x, -2.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion conjugation negates y.", actual.y, 3.0f );
TEST_ASSERT_EQ_FLOAT( "Quaternion conjugation negates z.", actual.z, -4.0f );

} /* test_math_sdr_quat_conj */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the math_sdr tests.              *
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
	{ "Math SDR: CRC-32C", test_math_sdr_crc32 },
	{ "Math SDR: Quaternion Multiplication", test_math_sdr_quat_mult },
	{ "Math SDR: Quaternion Dot Product", test_math_sdr_quat_dot },
	{ "Math SDR: Quaternion Addition", test_math_sdr_quat_add },
	{ "Math SDR: Quaternion Scaling", test_math_sdr_quat_scale },
	{ "Math SDR: Quaternion Normalization", test_math_sdr_quat_normalize },
	{ "Math SDR: Quaternion Conjugation", test_math_sdr_quat_conj }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "math_sdr", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
