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
#include <math.h>
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
*       test_math_sdr_clamp_float                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test float clamping below, within, and above its range.                *
*                                                                              *
*******************************************************************************/
void test_math_sdr_clamp_float
	(
	void
	)
{
TEST_ASSERT_EQ_FLOAT( "Values below the range clamp to its minimum.", clamp_float( -2.0f, -1.0f, 1.0f ), -1.0f );
TEST_ASSERT_EQ_FLOAT( "Values within the range are unchanged.", clamp_float( 0.5f, -1.0f, 1.0f ), 0.5f );
TEST_ASSERT_EQ_FLOAT( "Values above the range clamp to its maximum.", clamp_float( 2.0f, -1.0f, 1.0f ), 1.0f );

} /* test_math_sdr_clamp_float */


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
*       test_math_sdr_eul_to_quat                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test quaternion creation from ZYX Euler angles.                        *
*                                                                              *
*******************************************************************************/
void test_math_sdr_eul_to_quat
	(
	void
	)
{
QUAT actual = eul_to_quat( 1.57079632679f, 0.0f, 0.0f );

TEST_ASSERT_EQ_FLOAT( "A zero Euler rotation produces identity w.", eul_to_quat( 0.0f, 0.0f, 0.0f ).w, 1.0f );
TEST_ASSERT_EQ_FLOAT( "A zero Euler rotation produces identity x.", eul_to_quat( 0.0f, 0.0f, 0.0f ).x, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw calculates w.", actual.w, 0.70710678f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw calculates x.", actual.x, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw calculates y.", actual.y, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw calculates z.", actual.z, 0.70710678f );

} /* test_math_sdr_eul_to_quat */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_is_finite                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test detection of finite and non-finite quaternion components.         *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_is_finite
	(
	void
	)
{
TEST_ASSERT_TRUE( "A finite quaternion is reported as finite.", quat_is_finite( ( QUAT ){ 1.0f, -2.0f, 3.0f, -4.0f } ) );
TEST_ASSERT_FALSE( "A quaternion containing NaN is not finite.", quat_is_finite( ( QUAT ){ 1.0f, NAN, 0.0f, 0.0f } ) );
TEST_ASSERT_FALSE( "A quaternion containing infinity is not finite.", quat_is_finite( ( QUAT ){ 1.0f, 0.0f, 0.0f, INFINITY } ) );

} /* test_math_sdr_quat_is_finite */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_rotate_body_to_world                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test body-frame vector rotation into the world frame.                  *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_rotate_body_to_world
	(
	void
	)
{
QUAT attitude = eul_to_quat( 1.57079632679f, 0.0f, 0.0f );
QUAT actual = quat_rotate_body_to_world( attitude, ( QUAT ){ 0.0f, 1.0f, 0.0f, 0.0f } );

TEST_ASSERT_EQ_FLOAT( "Rotation preserves a pure vector scalar component.", actual.w, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw rotates body x out of world x.", actual.x, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw rotates body x into world y.", actual.y, 1.0f );
TEST_ASSERT_EQ_FLOAT( "A yaw rotation preserves the z vector component.", actual.z, 0.0f );

} /* test_math_sdr_quat_rotate_body_to_world */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_quat_rotate_world_to_body                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test world-frame vector rotation into the body frame.                  *
*                                                                              *
*******************************************************************************/
void test_math_sdr_quat_rotate_world_to_body
	(
	void
	)
{
QUAT attitude = eul_to_quat( 1.57079632679f, 0.0f, 0.0f );
QUAT actual = quat_rotate_world_to_body( attitude, ( QUAT ){ 0.0f, 0.0f, 1.0f, 0.0f } );

TEST_ASSERT_EQ_FLOAT( "Inverse rotation preserves a pure vector scalar component.", actual.w, 0.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw rotates world y into body x.", actual.x, 1.0f );
TEST_ASSERT_EQ_FLOAT( "A 90 degree yaw rotates world y out of body y.", actual.y, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Inverse yaw rotation preserves the z vector component.", actual.z, 0.0f );

} /* test_math_sdr_quat_rotate_world_to_body */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_math_sdr_vector_operations                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the required three-element vector operations.                     *
*                                                                              *
*******************************************************************************/
void test_math_sdr_vector_operations
	(
	void
	)
{
VECTOR_3F actual;
VECTOR_3F normalized = { 3.0f, 4.0f, 0.0f };
VECTOR_3F zero = { 0.0f, 0.0f, 0.0f };

actual = vector_add( ( VECTOR_3F ){ 1.0f, -2.0f, 3.0f }, ( VECTOR_3F ){ 4.0f, 5.0f, -6.0f } );
TEST_ASSERT_EQ_FLOAT( "Vector addition calculates x.", actual.x, 5.0f );
TEST_ASSERT_EQ_FLOAT( "Vector addition calculates y.", actual.y, 3.0f );
TEST_ASSERT_EQ_FLOAT( "Vector addition calculates z.", actual.z, -3.0f );

actual = vector_cross( ( VECTOR_3F ){ 1.0f, 0.0f, 0.0f }, ( VECTOR_3F ){ 0.0f, 1.0f, 0.0f } );
TEST_ASSERT_EQ_FLOAT( "Vector cross product calculates x.", actual.x, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Vector cross product calculates y.", actual.y, 0.0f );
TEST_ASSERT_EQ_FLOAT( "Vector cross product calculates z.", actual.z, 1.0f );

TEST_ASSERT_TRUE( "A finite vector is reported as finite.", vector_is_finite( ( VECTOR_3F ){ 1.0f, -2.0f, 3.0f } ) );
TEST_ASSERT_FALSE( "A vector containing NaN is not finite.", vector_is_finite( ( VECTOR_3F ){ 1.0f, NAN, 3.0f } ) );
TEST_ASSERT_EQ_FLOAT( "Vector magnitude is calculated correctly.", vector_magnitude( ( VECTOR_3F ){ 3.0f, 4.0f, 12.0f } ), 13.0f );

TEST_ASSERT_TRUE( "A nonzero vector is normalized.", vector_normalize( &normalized ) );
TEST_ASSERT_EQ_FLOAT( "Vector normalization calculates x.", normalized.x, 0.6f );
TEST_ASSERT_EQ_FLOAT( "Vector normalization calculates y.", normalized.y, 0.8f );
TEST_ASSERT_EQ_FLOAT( "Vector normalization calculates z.", normalized.z, 0.0f );
TEST_ASSERT_FALSE( "A zero vector cannot be normalized.", vector_normalize( &zero ) );

actual = vector_scale( ( VECTOR_3F ){ 1.0f, -2.0f, 3.0f }, -2.0f );
TEST_ASSERT_EQ_FLOAT( "Vector scaling calculates x.", actual.x, -2.0f );
TEST_ASSERT_EQ_FLOAT( "Vector scaling calculates y.", actual.y, 4.0f );
TEST_ASSERT_EQ_FLOAT( "Vector scaling calculates z.", actual.z, -6.0f );

} /* test_math_sdr_vector_operations */


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
	{ "Math SDR: CRC-32C", test_math_sdr_crc32, "RQ.MOD.00016" },
	{ "Math SDR: Clamp Float", test_math_sdr_clamp_float, "RQ.MOD.00040" },
	{ "Math SDR: Quaternion Multiplication", test_math_sdr_quat_mult, "RQ.MOD.00017" },
	{ "Math SDR: Quaternion Dot Product", test_math_sdr_quat_dot, "RQ.MOD.00018" },
	{ "Math SDR: Quaternion Addition", test_math_sdr_quat_add, "RQ.MOD.00019" },
	{ "Math SDR: Quaternion Scaling", test_math_sdr_quat_scale, "RQ.MOD.00020" },
	{ "Math SDR: Quaternion Normalization", test_math_sdr_quat_normalize, "RQ.MOD.00021" },
	{ "Math SDR: Quaternion Conjugation", test_math_sdr_quat_conj, "RQ.MOD.00022" },
	{ "Math SDR: Quaternion from Euler Angles", test_math_sdr_eul_to_quat, "RQ.MOD.00047" },
	{ "Math SDR: Quaternion Finiteness", test_math_sdr_quat_is_finite, "RQ.MOD.00048" },
	{ "Math SDR: Body to World Rotation", test_math_sdr_quat_rotate_body_to_world, "RQ.MOD.00049" },
	{ "Math SDR: World to Body Rotation", test_math_sdr_quat_rotate_world_to_body, "RQ.MOD.00050" },
	{ "Math SDR: Vector Operations", test_math_sdr_vector_operations, "RQ.MOD.00041 RQ.MOD.00042 RQ.MOD.00043 RQ.MOD.00044 RQ.MOD.00045 RQ.MOD.00046" }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "math_sdr", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
