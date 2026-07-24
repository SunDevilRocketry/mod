/*******************************************************************************
 *
 * FILE:
 *      test_mahony.c
 *
 * DESCRIPTION:
 *      Unit tests for the Mahony attitude filter.
 *
 ******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes
 ------------------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_PI        3.14159265358979323846f
#define TEST_TOLERANCE 0.001f

/*------------------------------------------------------------------------------
 Project Includes
 ------------------------------------------------------------------------------*/
#include "mahony.h"
#include "sdrtf_pub.h"

/*------------------------------------------------------------------------------
 Test Helpers
 ------------------------------------------------------------------------------*/

static void assert_quat_components
    (
    const char *description,
    QUAT actual,
    QUAT expected
    )
{
TEST_begin_nested_case(description);

TEST_ASSERT_EQ_FLOAT("Quaternion w component", actual.w, expected.w);
TEST_ASSERT_EQ_FLOAT("Quaternion x component", actual.x, expected.x);
TEST_ASSERT_EQ_FLOAT("Quaternion y component", actual.y, expected.y);
TEST_ASSERT_EQ_FLOAT("Quaternion z component", actual.z, expected.z);

TEST_end_nested_case();

} /* assert_quat_components */


/*------------------------------------------------------------------------------
 Initialization Tests
 ------------------------------------------------------------------------------*/

void test_mahony_init_identity_attitude
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        identity,
        1.0f,
        0.0f
        )
    );

assert_quat_components
    (
    "Identity attitude remains identity",
    filter.attitude,
    identity
    );

} /* test_mahony_init_identity_attitude */


void test_mahony_init_normalizes_attitude
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT initial_attitude =
    {
    .w = 2.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT expected =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        initial_attitude,
        1.0f,
        0.0f
        )
    );

assert_quat_components
    (
    "Initial attitude is normalized",
    filter.attitude,
    expected
    );

} /* test_mahony_init_normalizes_attitude */


void test_mahony_init_zero_quaternion_uses_identity
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT zero_quaternion =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT expected =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        zero_quaternion,
        1.0f,
        0.0f
        )
    );

assert_quat_components
    (
    "Zero quaternion falls back to identity",
    filter.attitude,
    expected
    );

} /* test_mahony_init_zero_quaternion_uses_identity */


void test_mahony_init_clears_integral_error
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        identity,
        1.0f,
        0.1f
        )
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Integral error x initializes to zero",
    filter.integral_error.x,
    0.0f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Integral error y initializes to zero",
    filter.integral_error.y,
    0.0f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Integral error z initializes to zero",
    filter.integral_error.z,
    0.0f
    );

} /* test_mahony_init_clears_integral_error */


void test_mahony_init_rejects_null_filter
    (
    void
    )
{
QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_FALSE
    (
    "Null filter is rejected",
    mahony_init
        (
        NULL,
        identity,
        1.0f,
        0.0f
        )
    );

} /* test_mahony_init_rejects_null_filter */


void test_mahony_init_rejects_negative_gain
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_FALSE
    (
    "Negative proportional gain is rejected",
    mahony_init
        (
        &filter,
        identity,
        -1.0f,
        0.0f
        )
    );

TEST_ASSERT_FALSE
    (
    "Negative integral gain is rejected",
    mahony_init
        (
        &filter,
        identity,
        1.0f,
        -0.1f
        )
    );

} /* test_mahony_init_rejects_negative_gain */


/*------------------------------------------------------------------------------
 Gyroscope Propagation Tests
 ------------------------------------------------------------------------------*/

void test_mahony_update_gyro_zero_rate
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F zero_rate =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        identity,
        0.0f,
        0.0f
        )
    );

TEST_ASSERT_TRUE
    (
    "Zero-rate update succeeds",
    mahony_update_gyro
        (
        &filter,
        zero_rate,
        0.01f
        )
    );

assert_quat_components
    (
    "Zero angular velocity preserves identity",
    filter.attitude,
    identity
    );

} /* test_mahony_update_gyro_zero_rate */


void test_mahony_update_gyro_positive_yaw
    (
    void
    )
{
int32_t index;

MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F gyro_body_rad_s =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.5f * TEST_PI
    };

QUAT body_x =
    {
    .w = 0.0f,
    .x = 1.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        identity,
        0.0f,
        0.0f
        )
    );

for ( index = 0; index < 1000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Positive-yaw propagation succeeds",
        mahony_update_gyro
            (
            &filter,
            gyro_body_rad_s,
            0.001f
            )
        );
    }

TEST_ASSERT_EQ_FLOAT
    (
    "Positive 90-degree yaw quaternion w component",
    filter.attitude.w,
    cosf(0.25f * TEST_PI)
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Positive 90-degree yaw quaternion z component",
    filter.attitude.z,
    sinf(0.25f * TEST_PI)
    );

QUAT world_vector = quat_rotate_body_to_world
    (
    filter.attitude,
    body_x
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Positive yaw rotates body positive X to world positive Y",
    world_vector.y,
    1.0f
    );

} /* test_mahony_update_gyro_positive_yaw */


void test_mahony_update_gyro_rejects_invalid_delta_time
    (
    void
    )
{
MAHONY_FILTER filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F zero_rate =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Mahony initialization succeeds",
    mahony_init
        (
        &filter,
        identity,
        0.0f,
        0.0f
        )
    );

TEST_ASSERT_FALSE
    (
    "Zero delta time is rejected",
    mahony_update_gyro
        (
        &filter,
        zero_rate,
        0.0f
        )
    );

TEST_ASSERT_FALSE
    (
    "Negative delta time is rejected",
    mahony_update_gyro
        (
        &filter,
        zero_rate,
        -0.01f
        )
    );

} /* test_mahony_update_gyro_rejects_invalid_delta_time */

/*------------------------------------------------------------------------------
 Main
 ------------------------------------------------------------------------------*/

int main
    (
    void
    )
{
unit_test tests[] =
    {
    {
    "mahony_init_identity_attitude",
    test_mahony_init_identity_attitude
    },
    {
    "mahony_init_normalizes_attitude",
    test_mahony_init_normalizes_attitude
    },
    {
    "mahony_init_zero_quaternion_uses_identity",
    test_mahony_init_zero_quaternion_uses_identity
    },
    {
    "mahony_init_clears_integral_error",
    test_mahony_init_clears_integral_error
    },
    {
    "mahony_init_rejects_null_filter",
    test_mahony_init_rejects_null_filter
    },
    {
    "mahony_init_rejects_negative_gain",
    test_mahony_init_rejects_negative_gain
    },
    {
    "mahony_update_gyro_zero_rate",
    test_mahony_update_gyro_zero_rate
    },
    {
    "mahony_update_gyro_positive_yaw",
    test_mahony_update_gyro_positive_yaw
    },
    {
    "mahony_update_gyro_rejects_invalid_delta_time",
    test_mahony_update_gyro_rejects_invalid_delta_time
    }
    };

TEST_INITIALIZE_TEST("mahony.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/