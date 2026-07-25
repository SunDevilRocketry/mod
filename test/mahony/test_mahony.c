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

 /**
 * @brief Verifies that the filter accepts an identity initial attitude.
 *
 * This test initializes the filter with a unit identity quaternion and checks
 * that initialization succeeds without changing the attitude.
 *
 * This is important because identity represents the simplest valid attitude
 * and is the expected starting state when the body and world frames are
 * aligned.
 */
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

/**
 * @brief Verifies that the initial attitude quaternion is normalized.
 *
 * This test initializes the filter with a valid but non-unit quaternion and
 * checks that the stored attitude is converted to unit length.
 *
 * This is important because only unit quaternions represent pure rotations.
 * Quaternion propagation and vector rotation can produce invalid results if
 * the attitude quaternion is not normalized.
 */
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

/**
 * @brief Verifies that a zero quaternion falls back to identity.
 *
 * This test initializes the filter with a quaternion whose components are all
 * zero and checks that quaternion normalization produces the identity
 * attitude.
 *
 * This is important because a zero quaternion does not represent a valid
 * rotation and cannot be normalized through ordinary division.
 */
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

/**
 * @brief Verifies that the integral correction state starts at zero.
 *
 * This test initializes the filter and checks that all components of the
 * accumulated integral error are cleared.
 *
 * This is important because stale integral error would introduce a false gyro
 * correction as soon as the filter begins operating.
 */
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

/**
 * @brief Verifies that initialization rejects a null filter pointer.
 *
 * This test calls mahony_init() without a valid filter instance and checks
 * that the function reports failure.
 *
 * This is important because dereferencing a null filter pointer would cause
 * undefined behavior or a runtime fault on the flight computer.
 */
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

/**
 * @brief Verifies that negative feedback gains are rejected.
 *
 * This test attempts to initialize the filter with negative proportional and
 * integral gains and checks that initialization fails.
 *
 * This is important because negative gains would reverse the intended
 * feedback direction and could cause attitude errors to grow rather than
 * converge.
 */
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

 /**
 * @brief Verifies that zero angular velocity preserves the current attitude.
 *
 * This test begins at the identity attitude, supplies a zero gyroscope vector,
 * and checks that gyro propagation does not rotate the attitude.
 *
 * This is important because a stationary body should not develop artificial
 * rotation when the measured angular rate is zero.
 */
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

/**
 * @brief Verifies positive yaw propagation from body-frame angular velocity.
 *
 * This test applies a positive 90-degree-per-second body Z-axis angular rate
 * for one second and checks that the resulting quaternion represents an
 * approximately positive 90-degree yaw.
 *
 * It also rotates the body positive X axis into the world frame to verify that
 * it points toward world positive Y.
 *
 * This is important because it confirms the quaternion multiplication order,
 * rotation sign, gyro units, and body-to-world attitude convention.
 */
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

/**
 * @brief Verifies that gyro propagation rejects invalid timesteps.
 *
 * This test supplies zero and negative elapsed times and checks that the
 * update reports failure.
 *
 * This is important because attitude propagation requires a positive elapsed
 * time. Invalid timing values could reverse propagation or hide timing errors
 * in the sensor update path.
 */
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

/**
 * @brief Verifies that aligned gravity produces no attitude correction.
 *
 * This test begins with an identity attitude, zero angular velocity, and an
 * accelerometer measurement aligned with the expected body-frame gravity
 * direction.
 *
 * The measured and estimated gravity vectors should have a zero cross product,
 * so the attitude should remain unchanged.
 *
 * This is important because a correctly aligned filter must not introduce
 * artificial rotation when there is no attitude error.
 */
void test_mahony_update_imu_aligned_gravity
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

VECTOR_3F zero_gyro =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F accel_body =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
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

for ( index = 0; index < 1000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Aligned IMU update succeeds",
        mahony_update_imu
            (
            &filter,
            zero_gyro,
            accel_body,
            0.001f,
            true
            )
        );
    }

assert_quat_components
    (
    "Aligned gravity preserves identity attitude",
    filter.attitude,
    identity
    );

} /* test_mahony_update_imu_aligned_gravity */

/**
 * @brief Verifies that accelerometer feedback reduces a roll error.
 *
 * This test initializes the attitude with a positive roll offset while
 * supplying zero angular velocity and a stationary gravity measurement.
 * Repeated Mahony updates should move the estimated body Z axis closer to the
 * world Z axis.
 *
 * This is important because it verifies that proportional accelerometer
 * feedback corrects roll drift and that the cross-product sign is consistent
 * with the body-to-world quaternion convention.
 */
void test_mahony_update_imu_roll_error_converges
    (
    void
    )
{
int32_t index;

MAHONY_FILTER filter;

const float initial_roll_rad = deg_to_rad(10.0f);

QUAT initial_attitude = eul_to_quat
    (
    0.0f,
    0.0f,
    initial_roll_rad
    );

VECTOR_3F zero_gyro =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F accel_body =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
    };

QUAT body_z =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 1.0f
    };

QUAT initial_world_z = quat_rotate_body_to_world
    (
    initial_attitude,
    body_z
    );

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

for ( index = 0; index < 2000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Roll-error correction update succeeds",
        mahony_update_imu
            (
            &filter,
            zero_gyro,
            accel_body,
            0.001f,
            true
            )
        );
    }

QUAT corrected_world_z = quat_rotate_body_to_world
    (
    filter.attitude,
    body_z
    );

TEST_ASSERT_TRUE
    (
    "Roll error decreases toward level",
    fabsf(corrected_world_z.y) < fabsf(initial_world_z.y)
    );

TEST_ASSERT_TRUE
    (
    "Corrected body Z approaches world Z",
    corrected_world_z.z > initial_world_z.z
    );

} /* test_mahony_update_imu_roll_error_converges */

/**
 * @brief Verifies that accelerometer feedback reduces a pitch error.
 *
 * This test initializes the attitude with a positive pitch offset while
 * supplying zero angular velocity and a stationary gravity measurement.
 * Repeated Mahony updates should move the estimated body Z axis closer to the
 * world Z axis.
 *
 * This is important because it confirms that accelerometer correction works
 * on both observable tilt axes rather than only for roll.
 */
void test_mahony_update_imu_pitch_error_converges
    (
    void
    )
{
int32_t index;

MAHONY_FILTER filter;

const float initial_pitch_rad = deg_to_rad(10.0f);

QUAT initial_attitude = eul_to_quat
    (
    0.0f,
    initial_pitch_rad,
    0.0f
    );

VECTOR_3F zero_gyro =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F accel_body =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
    };

QUAT body_z =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 1.0f
    };

QUAT initial_world_z = quat_rotate_body_to_world
    (
    initial_attitude,
    body_z
    );

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

for ( index = 0; index < 2000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Pitch-error correction update succeeds",
        mahony_update_imu
            (
            &filter,
            zero_gyro,
            accel_body,
            0.001f,
            true
            )
        );
    }

QUAT corrected_world_z = quat_rotate_body_to_world
    (
    filter.attitude,
    body_z
    );

TEST_ASSERT_TRUE
    (
    "Pitch error decreases toward level",
    fabsf(corrected_world_z.x) < fabsf(initial_world_z.x)
    );

TEST_ASSERT_TRUE
    (
    "Corrected body Z approaches world Z",
    corrected_world_z.z > initial_world_z.z
    );

} /* test_mahony_update_imu_pitch_error_converges */

/**
 * @brief Verifies that accelerometer feedback does not correct yaw.
 *
 * This test initializes the attitude with a yaw offset while roll and pitch
 * remain level. Because yaw rotation does not change the gravity direction,
 * repeated accelerometer corrections should leave the yaw attitude unchanged.
 *
 * This is important because gravity provides roll and pitch information but
 * contains no heading information. Yaw correction requires another reference,
 * such as a calibrated magnetometer.
 */
void test_mahony_update_imu_yaw_error_does_not_converge
    (
    void
    )
{
int32_t index;

MAHONY_FILTER filter;

const float initial_yaw_rad = deg_to_rad(20.0f);

QUAT initial_attitude = eul_to_quat
    (
    initial_yaw_rad,
    0.0f,
    0.0f
    );

VECTOR_3F zero_gyro =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F accel_body =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
    };

QUAT body_x =
    {
    .w = 0.0f,
    .x = 1.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT initial_world_x = quat_rotate_body_to_world
    (
    initial_attitude,
    body_x
    );

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

for ( index = 0; index < 2000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Yaw-only correction update succeeds",
        mahony_update_imu
            (
            &filter,
            zero_gyro,
            accel_body,
            0.001f,
            true
            )
        );
    }

QUAT final_world_x = quat_rotate_body_to_world
    (
    filter.attitude,
    body_x
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Accelerometer does not correct yaw X component",
    final_world_x.x,
    initial_world_x.x
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Accelerometer does not correct yaw Y component",
    final_world_x.y,
    initial_world_x.y
    );

} /* test_mahony_update_imu_yaw_error_does_not_converge */

/**
 * @brief Verifies that a zero accelerometer vector falls back to gyro-only
 *        propagation.
 *
 * This test runs two identical filters. One uses mahony_update_gyro(), while
 * the other uses mahony_update_imu() with accelerometer feedback requested but
 * a zero accelerometer vector.
 *
 * Both filters should produce the same attitude because the zero vector cannot
 * be normalized and must therefore be excluded from feedback.
 *
 * This is important because an invalid accelerometer sample should not stop
 * attitude propagation or introduce undefined normalization behavior.
 */
void test_mahony_update_imu_zero_accel_uses_gyro_only
    (
    void
    )
{
int32_t index;

MAHONY_FILTER gyro_filter;
MAHONY_FILTER imu_filter;

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
    .z = deg_to_rad(45.0f)
    };

VECTOR_3F zero_accel =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Gyro filter initialization succeeds",
    mahony_init
        (
        &gyro_filter,
        identity,
        1.0f,
        0.0f
        )
    );

TEST_ASSERT_TRUE
    (
    "IMU filter initialization succeeds",
    mahony_init
        (
        &imu_filter,
        identity,
        1.0f,
        0.0f
        )
    );

for ( index = 0; index < 1000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Gyro-only update succeeds",
        mahony_update_gyro
            (
            &gyro_filter,
            gyro_body_rad_s,
            0.001f
            )
        );

    TEST_ASSERT_TRUE
        (
        "Zero-accelerometer IMU update succeeds",
        mahony_update_imu
            (
            &imu_filter,
            gyro_body_rad_s,
            zero_accel,
            0.001f,
            true
            )
        );
    }

assert_quat_components
    (
    "Zero accelerometer matches gyro-only propagation",
    imu_filter.attitude,
    gyro_filter.attitude
    );

} /* test_mahony_update_imu_zero_accel_uses_gyro_only */

/**
 * @brief Verifies that disabling accelerometer feedback matches gyro-only
 *        propagation.
 *
 * This test runs two identical filters while providing a deliberately
 * misaligned accelerometer measurement. One filter uses gyro-only propagation,
 * and the other calls mahony_update_imu() with use_accel set to false.
 *
 * Both filters should produce the same attitude because the accelerometer
 * measurement must be completely ignored.
 *
 * This is important because flight-state logic must be able to disable
 * accelerometer correction during thrust, vibration, saturation, or other
 * high-dynamic conditions.
 */
void test_mahony_update_imu_disabled_accel_uses_gyro_only
    (
    void
    )
{
int32_t index;

MAHONY_FILTER gyro_filter;
MAHONY_FILTER imu_filter;

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F gyro_body_rad_s =
    {
    .x = deg_to_rad(30.0f),
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F misaligned_accel =
    {
    .x = GRAVITY,
    .y = 0.0f,
    .z = 0.0f
    };

TEST_ASSERT_TRUE
    (
    "Gyro filter initialization succeeds",
    mahony_init
        (
        &gyro_filter,
        identity,
        1.0f,
        0.0f
        )
    );

TEST_ASSERT_TRUE
    (
    "IMU filter initialization succeeds",
    mahony_init
        (
        &imu_filter,
        identity,
        1.0f,
        0.0f
        )
    );

for ( index = 0; index < 1000; index++ )
    {
    TEST_ASSERT_TRUE
        (
        "Gyro-only update succeeds",
        mahony_update_gyro
            (
            &gyro_filter,
            gyro_body_rad_s,
            0.001f
            )
        );

    TEST_ASSERT_TRUE
        (
        "Disabled-accelerometer IMU update succeeds",
        mahony_update_imu
            (
            &imu_filter,
            gyro_body_rad_s,
            misaligned_accel,
            0.001f,
            false
            )
        );
    }

assert_quat_components
    (
    "Disabled accelerometer matches gyro-only propagation",
    imu_filter.attitude,
    gyro_filter.attitude
    );

} /* test_mahony_update_imu_disabled_accel_uses_gyro_only */

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
    },
    {
    "mahony_update_imu_aligned_gravity",
    test_mahony_update_imu_aligned_gravity
    },
    {
    "mahony_update_imu_roll_error_converges",
    test_mahony_update_imu_roll_error_converges
    },
    {
    "mahony_update_imu_pitch_error_converges",
    test_mahony_update_imu_pitch_error_converges
    },
    {
    "mahony_update_imu_yaw_error_does_not_converge",
    test_mahony_update_imu_yaw_error_does_not_converge
    },
    {
    "mahony_update_imu_zero_accel_uses_gyro_only",
    test_mahony_update_imu_zero_accel_uses_gyro_only
    },
    {
    "mahony_update_imu_disabled_accel_uses_gyro_only",
    test_mahony_update_imu_disabled_accel_uses_gyro_only
    },
    };

TEST_INITIALIZE_TEST("mahony.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/