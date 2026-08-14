/*******************************************************************************
 *
 * FILE:
 *      test_mekf.c
 *
 * DESCRIPTION:
 *      Unit tests for the Multiplicative Extended Kalman Filter.
 *
 ******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes
 ------------------------------------------------------------------------------*/
#include <float.h>
#include <math.h>
#include <stddef.h>

/*------------------------------------------------------------------------------
 Project Includes
 ------------------------------------------------------------------------------*/
#include "mekf.h"
#include "sdrtf_pub.h"

/*------------------------------------------------------------------------------
 Test Helpers
 ------------------------------------------------------------------------------*/

/**
 * @brief Creates a valid configuration for MEKF unit tests.
 */
static MEKF_CONFIG make_valid_config
    (
    void
    )
{
MEKF_CONFIG config =
    {
    .initial_attitude_std_rad =
        {
        .x = 0.10f,
        .y = 0.20f,
        .z = 0.30f
        },
    .initial_gyro_bias_std_rad_s =
        {
        .x = 0.01f,
        .y = 0.02f,
        .z = 0.03f
        },
    .gyro_noise_density_rad_s_sqrt_hz = 0.005f,
    .gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0001f,
    .accelerometer_direction_std = 0.05f,
    .gravity_magnitude_m_s2 = 9.80665f,
    .accelerometer_magnitude_tolerance_m_s2 = 1.50f,
    .accelerometer_innovation_gate = 11.344867f,
    .maximum_delta_time_s = 0.10f
    };

return config;

} /* make_valid_config */

/**
 * @brief Checks all four quaternion components.
 */
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

/**
 * @brief Checks all three vector components.
 */
static void assert_vector_components
    (
    const char *description,
    VECTOR_3F actual,
    VECTOR_3F expected
    )
{
TEST_begin_nested_case(description);

TEST_ASSERT_EQ_FLOAT("Vector x component", actual.x, expected.x);
TEST_ASSERT_EQ_FLOAT("Vector y component", actual.y, expected.y);
TEST_ASSERT_EQ_FLOAT("Vector z component", actual.z, expected.z);

TEST_end_nested_case();

} /* assert_vector_components */

/*------------------------------------------------------------------------------
 Initialization Tests
 ------------------------------------------------------------------------------*/

/**
 * @brief Verifies identity-attitude and gyro-bias initialization.
 */
void test_mekf_init_identity_and_bias
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

VECTOR_3F initial_bias =
    {
    .x = 0.01f,
    .y = -0.02f,
    .z = 0.03f
    };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        initial_bias,
        &config
        )
    );

assert_quat_components
    (
    "Identity attitude is preserved",
    filter.attitude,
    identity
    );

assert_vector_components
    (
    "Initial gyro bias is preserved",
    filter.gyro_bias_rad_s,
    initial_bias
    );

} /* test_mekf_init_identity_and_bias */

/**
 * @brief Verifies that initialization normalizes the nominal quaternion.
 */
void test_mekf_init_normalizes_attitude
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

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

VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        initial_attitude,
        zero_bias,
        &config
        )
    );

assert_quat_components
    (
    "Initial attitude is normalized",
    filter.attitude,
    expected
    );

} /* test_mekf_init_normalizes_attitude */

/**
 * @brief Verifies the shared zero-quaternion identity fallback.
 */
void test_mekf_init_zero_quaternion_uses_identity
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT zero_quaternion = { 0.0f, 0.0f, 0.0f, 0.0f };
QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "Zero-quaternion initialization succeeds",
    mekf_init
        (
        &filter,
        zero_quaternion,
        zero_bias,
        &config
        )
    );

assert_quat_components
    (
    "Zero quaternion uses identity",
    filter.attitude,
    identity
    );

} /* test_mekf_init_zero_quaternion_uses_identity */

/**
 * @brief Verifies diagonal covariance initialization.
 */
void test_mekf_init_sets_diagonal_covariance
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude X variance",
    filter.covariance[MEKF_ATTITUDE_ERROR_X][MEKF_ATTITUDE_ERROR_X],
    0.10f * 0.10f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude Y variance",
    filter.covariance[MEKF_ATTITUDE_ERROR_Y][MEKF_ATTITUDE_ERROR_Y],
    0.20f * 0.20f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude Z variance",
    filter.covariance[MEKF_ATTITUDE_ERROR_Z][MEKF_ATTITUDE_ERROR_Z],
    0.30f * 0.30f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Bias X variance",
    filter.covariance[MEKF_GYRO_BIAS_ERROR_X][MEKF_GYRO_BIAS_ERROR_X],
    0.01f * 0.01f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Bias Y variance",
    filter.covariance[MEKF_GYRO_BIAS_ERROR_Y][MEKF_GYRO_BIAS_ERROR_Y],
    0.02f * 0.02f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Bias Z variance",
    filter.covariance[MEKF_GYRO_BIAS_ERROR_Z][MEKF_GYRO_BIAS_ERROR_Z],
    0.03f * 0.03f
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        if ( row != column )
            {
            TEST_ASSERT_EQ_FLOAT
                (
                "Initial cross covariance is zero",
                filter.covariance[row][column],
                0.0f
                );
            }
        }
    }

} /* test_mekf_init_sets_diagonal_covariance */

/**
 * @brief Verifies that MEKF configuration is copied into the filter.
 */
void test_mekf_init_copies_config
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Gyro noise density is copied",
    filter.config.gyro_noise_density_rad_s_sqrt_hz,
    config.gyro_noise_density_rad_s_sqrt_hz
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Bias random walk is copied",
    filter.config.gyro_bias_random_walk_rad_s2_sqrt_hz,
    config.gyro_bias_random_walk_rad_s2_sqrt_hz
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Accelerometer direction uncertainty is copied",
    filter.config.accelerometer_direction_std,
    config.accelerometer_direction_std
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Gravity magnitude is copied",
    filter.config.gravity_magnitude_m_s2,
    config.gravity_magnitude_m_s2
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Accelerometer magnitude tolerance is copied",
    filter.config.accelerometer_magnitude_tolerance_m_s2,
    config.accelerometer_magnitude_tolerance_m_s2
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Accelerometer innovation gate is copied",
    filter.config.accelerometer_innovation_gate,
    config.accelerometer_innovation_gate
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Maximum timestep is copied",
    filter.config.maximum_delta_time_s,
    config.maximum_delta_time_s
    );

} /* test_mekf_init_copies_config */

/**
 * @brief Verifies that null filter and configuration pointers are rejected.
 */
void test_mekf_init_rejects_null_pointers
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "Null filter is rejected",
    !mekf_init
        (
        NULL,
        identity,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Null configuration is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        NULL
        )
    );

} /* test_mekf_init_rejects_null_pointers */

/**
 * @brief Verifies rejection of nonfinite nominal-state inputs.
 */
void test_mekf_init_rejects_nonfinite_state
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT invalid_attitude = { NAN, 0.0f, 0.0f, 0.0f };
QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };
VECTOR_3F invalid_bias = { 0.0f, INFINITY, 0.0f };

TEST_ASSERT_TRUE
    (
    "Nonfinite attitude is rejected",
    !mekf_init
        (
        &filter,
        invalid_attitude,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Nonfinite gyro bias is rejected",
    !mekf_init
        (
        &filter,
        identity,
        invalid_bias,
        &config
        )
    );

} /* test_mekf_init_rejects_nonfinite_state */

/**
 * @brief Verifies rejection of invalid initial uncertainty.
 */
void test_mekf_init_rejects_invalid_uncertainty
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config;

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

config = make_valid_config();
config.initial_attitude_std_rad.x = -0.10f;

TEST_ASSERT_TRUE
    (
    "Negative attitude uncertainty is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.initial_gyro_bias_std_rad_s.y = NAN;

TEST_ASSERT_TRUE
    (
    "Nonfinite bias uncertainty is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.initial_attitude_std_rad.z = FLT_MAX;

TEST_ASSERT_TRUE
    (
    "Uncertainty variance overflow is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

} /* test_mekf_init_rejects_invalid_uncertainty */

/**
 * @brief Verifies rejection of invalid process-noise values.
 */
void test_mekf_init_rejects_invalid_process_noise
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config;

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

config = make_valid_config();
config.gyro_noise_density_rad_s_sqrt_hz = -0.005f;

TEST_ASSERT_TRUE
    (
    "Negative gyro noise density is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.gyro_bias_random_walk_rad_s2_sqrt_hz = NAN;

TEST_ASSERT_TRUE
    (
    "Nonfinite bias random walk is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.gyro_noise_density_rad_s_sqrt_hz = FLT_MAX;

TEST_ASSERT_TRUE
    (
    "Gyro noise variance overflow is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

} /* test_mekf_init_rejects_invalid_process_noise */

/**
 * @brief Verifies that invalid accelerometer-update configuration values are
 * rejected.
 */
void test_mekf_init_rejects_invalid_accelerometer_config
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config;

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

/*
 * Accelerometer direction uncertainty must be finite and greater than zero.
 */
config = make_valid_config();
config.accelerometer_direction_std = 0.0f;

TEST_ASSERT_FALSE
    (
    "Zero accelerometer direction uncertainty is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_direction_std = -0.01f;

TEST_ASSERT_FALSE
    (
    "Negative accelerometer direction uncertainty is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_direction_std = NAN;

TEST_ASSERT_FALSE
    (
    "Nonfinite accelerometer direction uncertainty is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

/*
 * Expected gravity magnitude must be finite and greater than zero.
 */
config = make_valid_config();
config.gravity_magnitude_m_s2 = 0.0f;

TEST_ASSERT_FALSE
    (
    "Zero gravity magnitude is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.gravity_magnitude_m_s2 = -9.80665f;

TEST_ASSERT_FALSE
    (
    "Negative gravity magnitude is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.gravity_magnitude_m_s2 = NAN;

TEST_ASSERT_FALSE
    (
    "Nonfinite gravity magnitude is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

/*
 * The accelerometer magnitude tolerance must also be finite and positive.
 */
config = make_valid_config();
config.accelerometer_magnitude_tolerance_m_s2 = 0.0f;

TEST_ASSERT_FALSE
    (
    "Zero accelerometer magnitude tolerance is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_magnitude_tolerance_m_s2 = -1.0f;

TEST_ASSERT_FALSE
    (
    "Negative accelerometer magnitude tolerance is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_magnitude_tolerance_m_s2 = NAN;

TEST_ASSERT_FALSE
    (
    "Nonfinite accelerometer magnitude tolerance is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

/*
 * The accelerometer innovation gate must be finite and greater than zero.
 */
config = make_valid_config();
config.accelerometer_innovation_gate = 0.0f;

TEST_ASSERT_FALSE
    (
    "Zero accelerometer innovation gate is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_innovation_gate = -1.0f;

TEST_ASSERT_FALSE
    (
    "Negative accelerometer innovation gate is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

config = make_valid_config();
config.accelerometer_innovation_gate = NAN;

TEST_ASSERT_FALSE
    (
    "Nonfinite accelerometer innovation gate is rejected",
    mekf_init(&filter, identity, zero_bias, &config)
    );

} /* test_mekf_init_rejects_invalid_accelerometer_config */

/**
 * @brief Verifies rejection of invalid maximum timestep values.
 */
void test_mekf_init_rejects_invalid_maximum_timestep
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config;

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

config = make_valid_config();
config.maximum_delta_time_s = 0.0f;

TEST_ASSERT_TRUE
    (
    "Zero maximum timestep is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.maximum_delta_time_s = -0.10f;

TEST_ASSERT_TRUE
    (
    "Negative maximum timestep is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

config = make_valid_config();
config.maximum_delta_time_s = NAN;

TEST_ASSERT_TRUE
    (
    "Nonfinite maximum timestep is rejected",
    !mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

} /* test_mekf_init_rejects_invalid_maximum_timestep */

/**
 * @brief Verifies that zero uncertainty and process noise are permitted.
 *
 * Zero values are mathematically valid and useful for deterministic unit tests,
 * even though real flight configuration should use measured nonzero values.
 */
void test_mekf_init_accepts_zero_uncertainty_and_noise
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

config.initial_attitude_std_rad = zero_bias;
config.initial_gyro_bias_std_rad_s = zero_bias;
config.gyro_noise_density_rad_s_sqrt_hz = 0.0f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0f;

TEST_ASSERT_TRUE
    (
    "Zero uncertainty and process noise are accepted",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Zero uncertainty produces zero covariance",
            filter.covariance[row][column],
            0.0f
            );
        }
    }

} /* test_mekf_init_accepts_zero_uncertainty_and_noise */

/**
 * @brief Verifies that failed initialization does not partially modify state.
 */
void test_mekf_init_failure_preserves_filter
    (
    void
    )
{
MEKF_FILTER filter = { 0 };
MEKF_CONFIG invalid_config = make_valid_config();

QUAT initial_attitude = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F initial_bias = { 0.0f, 0.0f, 0.0f };

QUAT sentinel_attitude = { 0.5f, 0.5f, 0.5f, 0.5f };
VECTOR_3F sentinel_bias = { 1.0f, 2.0f, 3.0f };

filter.attitude = sentinel_attitude;
filter.gyro_bias_rad_s = sentinel_bias;
filter.covariance[0][0] = 123.0f;
filter.config.maximum_delta_time_s = 0.25f;

invalid_config.maximum_delta_time_s = -0.10f;

TEST_ASSERT_TRUE
    (
    "Invalid configuration is rejected",
    !mekf_init
        (
        &filter,
        initial_attitude,
        initial_bias,
        &invalid_config
        )
    );

assert_quat_components
    (
    "Failed initialization preserves attitude",
    filter.attitude,
    sentinel_attitude
    );

assert_vector_components
    (
    "Failed initialization preserves gyro bias",
    filter.gyro_bias_rad_s,
    sentinel_bias
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Failed initialization preserves covariance",
    filter.covariance[0][0],
    123.0f
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Failed initialization preserves configuration",
    filter.config.maximum_delta_time_s,
    0.25f
    );

} /* test_mekf_init_failure_preserves_filter */

/*------------------------------------------------------------------------------
 Gyro Prediction Tests
 ------------------------------------------------------------------------------*/

/**
 * @brief Verifies that zero angular rate preserves nominal attitude.
 */
void test_mekf_predict_zero_rate
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Zero-rate prediction succeeds",
    mekf_predict
        (
        &filter,
        zero_vector,
        0.01f
        )
    );

assert_quat_components
    (
    "Zero angular rate preserves attitude",
    filter.attitude,
    identity
    );

assert_vector_components
    (
    "Prediction preserves gyro-bias estimate",
    filter.gyro_bias_rad_s,
    zero_vector
    );

} /* test_mekf_predict_zero_rate */

/**
 * @brief Verifies positive rotation about body Z for one second.
 *
 * A positive 90-degree-per-second body-Z rate should rotate body +X toward
 * world +Y after one second.
 */
void test_mekf_predict_positive_yaw
    (
    void
    )
{
unsigned int step;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

QUAT expected =
    {
    .w = 0.70710678f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.70710678f
    };

VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

VECTOR_3F gyro_body_rad_s =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = deg_to_rad(90.0f)
    };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

for ( step = 0U; step < 100U; step++ )
    {
    TEST_ASSERT_TRUE
        (
        "Positive-yaw prediction succeeds",
        mekf_predict
            (
            &filter,
            gyro_body_rad_s,
            0.01f
            )
        );
    }

assert_quat_components
    (
    "Positive body-Z rate produces positive yaw",
    filter.attitude,
    expected
    );

} /* test_mekf_predict_positive_yaw */

/**
 * @brief Verifies that the nominal gyro bias is subtracted.
 */
void test_mekf_predict_subtracts_gyro_bias
    (
    void
    )
{
unsigned int step;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

VECTOR_3F initial_bias =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = deg_to_rad(10.0f)
    };

/*
 * The measured rate exactly equals the estimated bias, so corrected angular
 * velocity should be zero.
 */
VECTOR_3F gyro_measurement = initial_bias;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        initial_bias,
        &config
        )
    );

for ( step = 0U; step < 100U; step++ )
    {
    TEST_ASSERT_TRUE
        (
        "Bias-corrected prediction succeeds",
        mekf_predict
            (
            &filter,
            gyro_measurement,
            0.01f
            )
        );
    }

assert_quat_components
    (
    "Measured rate equal to bias produces no rotation",
    filter.attitude,
    identity
    );

assert_vector_components
    (
    "Prediction does not alter nominal bias",
    filter.gyro_bias_rad_s,
    initial_bias
    );

} /* test_mekf_predict_subtracts_gyro_bias */

/*------------------------------------------------------------------------------
 Covariance Prediction Tests
 ------------------------------------------------------------------------------*/

/**
 * @brief Verifies that gyro-bias uncertainty propagates into attitude
 *        uncertainty.
 *
 * With zero angular rate and zero process noise:
 *
 *     Phi =
 *         [
 *         I   -I * dt
 *         0      I
 *         ]
 *
 * An initial bias variance of 4.0 and timestep of 0.1 seconds should produce:
 *
 *     attitude variance = 1.0 + 4.0 * 0.1^2 = 1.04
 *     attitude-bias covariance = -4.0 * 0.1 = -0.4
 *     bias variance = 4.0
 */
void test_mekf_predict_couples_bias_uncertainty
    (
    void
    )
{
unsigned int axis;
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

float expected[MEKF_ERROR_STATE_DIM][MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

config.initial_attitude_std_rad.x = 1.0f;
config.initial_attitude_std_rad.y = 1.0f;
config.initial_attitude_std_rad.z = 1.0f;

config.initial_gyro_bias_std_rad_s.x = 2.0f;
config.initial_gyro_bias_std_rad_s.y = 2.0f;
config.initial_gyro_bias_std_rad_s.z = 2.0f;

config.gyro_noise_density_rad_s_sqrt_hz = 0.0f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0f;
config.maximum_delta_time_s = 0.10f;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Zero-rate covariance prediction succeeds",
    mekf_predict
        (
        &filter,
        zero_vector,
        0.10f
        )
    );

for ( axis = 0U; axis < 3U; axis++ )
    {
    expected[axis][axis] = 1.04f;
    expected[axis + 3U][axis + 3U] = 4.0f;

    expected[axis][axis + 3U] = -0.40f;
    expected[axis + 3U][axis] = -0.40f;
    }

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Predicted covariance entry",
            filter.covariance[row][column],
            expected[row][column]
            );
        }
    }

} /* test_mekf_predict_couples_bias_uncertainty */

/**
 * @brief Verifies discrete gyro and gyro-bias process-noise propagation.
 *
 * This test starts with zero covariance so the predicted covariance consists
 * entirely of the discrete process-noise matrix Q_d.
 */
void test_mekf_predict_adds_process_noise
    (
    void
    )
{
unsigned int axis;
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

float expected[MEKF_ERROR_STATE_DIM][MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

float expected_attitude_variance;
float expected_attitude_bias_covariance;
float expected_bias_variance;

/*
 * Start with zero covariance so only Q_d contributes to the result.
 */
config.initial_attitude_std_rad = zero_vector;
config.initial_gyro_bias_std_rad_s = zero_vector;

/*
 * Use intentionally large synthetic noise values so every expected process
 * noise term is easily distinguishable from zero in the unit test.
 */
config.gyro_noise_density_rad_s_sqrt_hz = 2.0f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 1.0f;
config.maximum_delta_time_s = 0.10f;

/*
 * For dt = 0.1 seconds:
 *
 *     sigma_g^2 = 4
 *     sigma_b^2 = 1
 *
 *     Q_theta_theta =
 *         sigma_g^2 * dt + sigma_b^2 * dt^3 / 3
 *
 *     Q_theta_bias =
 *         -sigma_b^2 * dt^2 / 2
 *
 *     Q_bias_bias =
 *         sigma_b^2 * dt
 */
expected_attitude_variance =
    4.0f * 0.10f +
    1.0f * 0.001f / 3.0f;

expected_attitude_bias_covariance =
    -1.0f * 0.01f / 2.0f;

expected_bias_variance =
    1.0f * 0.10f;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Process-noise prediction succeeds",
    mekf_predict
        (
        &filter,
        zero_vector,
        0.10f
        )
    );

for ( axis = 0U; axis < 3U; axis++ )
    {
    expected[axis][axis] =
        expected_attitude_variance;

    expected[axis][axis + 3U] =
        expected_attitude_bias_covariance;

    expected[axis + 3U][axis] =
        expected_attitude_bias_covariance;

    expected[axis + 3U][axis + 3U] =
        expected_bias_variance;
    }

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Discrete process-noise entry",
            filter.covariance[row][column],
            expected[row][column]
            );
        }
    }

} /* test_mekf_predict_adds_process_noise */

/**
 * @brief Verifies that invalid prediction timesteps are rejected without
 * modifying the attitude or covariance.
 */
void test_mekf_predict_rejects_invalid_timestep
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_FILTER original_filter;

MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };
VECTOR_3F gyro_body_rad_s = { 0.1f, -0.2f, 0.3f };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

original_filter = filter;

TEST_ASSERT_FALSE
    (
    "Zero timestep is rejected",
    mekf_predict
        (
        &filter,
        gyro_body_rad_s,
        0.0f
        )
    );

TEST_ASSERT_FALSE
    (
    "Negative timestep is rejected",
    mekf_predict
        (
        &filter,
        gyro_body_rad_s,
        -0.01f
        )
    );

TEST_ASSERT_FALSE
    (
    "Timestep above configured maximum is rejected",
    mekf_predict
        (
        &filter,
        gyro_body_rad_s,
        config.maximum_delta_time_s + 0.01f
        )
    );

TEST_ASSERT_FALSE
    (
    "Nonfinite timestep is rejected",
    mekf_predict
        (
        &filter,
        gyro_body_rad_s,
        NAN
        )
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude W remains unchanged",
    filter.attitude.w,
    original_filter.attitude.w
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude X remains unchanged",
    filter.attitude.x,
    original_filter.attitude.x
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude Y remains unchanged",
    filter.attitude.y,
    original_filter.attitude.y
    );

TEST_ASSERT_EQ_FLOAT
    (
    "Attitude Z remains unchanged",
    filter.attitude.z,
    original_filter.attitude.z
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Covariance remains unchanged",
            filter.covariance[row][column],
            original_filter.covariance[row][column]
            );
        }
    }

} /* test_mekf_predict_rejects_invalid_timestep */

/**
 * @brief Verifies attitude-covariance propagation during nonzero rotation.
 *
 * For a corrected Z-axis rate of 1 rad/s and dt = 0.1 s, the attitude
 * transition block is:
 *
 *     Phi_theta =
 *         [
 *          1.0    0.1    0.0
 *         -0.1    1.0    0.0
 *          0.0    0.0    1.0
 *         ]
 *
 * Starting with attitude covariance diag(1, 4, 9), the propagated attitude
 * covariance should be:
 *
 *     [
 *      1.04    0.30    0.00
 *      0.30    4.01    0.00
 *      0.00    0.00    9.00
 *     ]
 */
void test_mekf_predict_rotates_attitude_covariance
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

float expected[MEKF_ERROR_STATE_DIM][MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

VECTOR_3F zero_vector =
    {
    0.0f,
    0.0f,
    0.0f
    };

VECTOR_3F gyro_body_rad_s =
    {
    0.0f,
    0.0f,
    1.0f
    };

/*
 * Use unequal initial attitude variances so an incorrect skew-matrix sign or
 * axis placement cannot accidentally produce the expected result.
 */
config.initial_attitude_std_rad.x = 1.0f;
config.initial_attitude_std_rad.y = 2.0f;
config.initial_attitude_std_rad.z = 3.0f;

config.initial_gyro_bias_std_rad_s = zero_vector;

config.gyro_noise_density_rad_s_sqrt_hz = 0.0f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0f;
config.maximum_delta_time_s = 0.10f;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Rotating covariance prediction succeeds",
    mekf_predict
        (
        &filter,
        gyro_body_rad_s,
        0.10f
        )
    );

expected[MEKF_ATTITUDE_ERROR_X][MEKF_ATTITUDE_ERROR_X] = 1.04f;
expected[MEKF_ATTITUDE_ERROR_X][MEKF_ATTITUDE_ERROR_Y] = 0.30f;

expected[MEKF_ATTITUDE_ERROR_Y][MEKF_ATTITUDE_ERROR_X] = 0.30f;
expected[MEKF_ATTITUDE_ERROR_Y][MEKF_ATTITUDE_ERROR_Y] = 4.01f;

expected[MEKF_ATTITUDE_ERROR_Z][MEKF_ATTITUDE_ERROR_Z] = 9.00f;

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Rotating attitude covariance entry",
            filter.covariance[row][column],
            expected[row][column]
            );
        }
    }

} /* test_mekf_predict_rotates_attitude_covariance */

/*------------------------------------------------------------------------------
 Accelerometer Update Tests
 ------------------------------------------------------------------------------*/

/**
 * @brief Verifies that a gravity measurement aligned with the predicted
 * direction does not change the nominal attitude or gyro-bias estimate.
 */
void test_mekf_update_accelerometer_aligned_measurement
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

VECTOR_3F aligned_gravity =
    {
    .x = 0.0f,
    .y = 0.0f,
    .z = 9.80665f
    };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Aligned accelerometer measurement is accepted",
    mekf_update_accelerometer
        (
        &filter,
        aligned_gravity
        )
    );

assert_quat_components
    (
    "Aligned measurement leaves attitude unchanged",
    filter.attitude,
    identity
    );

assert_vector_components
    (
    "Aligned measurement leaves gyro bias unchanged",
    filter.gyro_bias_rad_s,
    zero_bias
    );

} /* test_mekf_update_accelerometer_aligned_measurement */

/**
 * @brief Verifies that a tilted gravity measurement moves the attitude estimate
 * toward the measured gravity direction without introducing yaw correction.
 */
void test_mekf_update_accelerometer_corrects_tilt
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
QUAT gravity_world = { 0.0f, 0.0f, 0.0f, 1.0f };
QUAT predicted_gravity_after;

VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

VECTOR_3F tilted_gravity;

float tilt_angle_rad = 0.174532925f;
float direction_error_before;
float direction_error_after;
float quaternion_norm_squared;

/*
 * Give the filter meaningful tilt uncertainty so the accelerometer measurement
 * produces a visible correction.
 */
config.initial_attitude_std_rad.x = 0.20f;
config.initial_attitude_std_rad.y = 0.20f;
config.initial_attitude_std_rad.z = 0.20f;

/*
 * Construct a gravity measurement tilted ten degrees toward positive body Y.
 * Its magnitude remains exactly equal to the configured gravity magnitude.
 */
tilted_gravity.x = 0.0f;

tilted_gravity.y =
    config.gravity_magnitude_m_s2 *
    sinf(tilt_angle_rad);

tilted_gravity.z =
    config.gravity_magnitude_m_s2 *
    cosf(tilt_angle_rad);

direction_error_before = sinf(tilt_angle_rad);

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Tilted accelerometer measurement is accepted",
    mekf_update_accelerometer
        (
        &filter,
        tilted_gravity
        )
    );

predicted_gravity_after = quat_rotate_world_to_body
    (
    filter.attitude,
    gravity_world
    );

direction_error_after = fabsf
    (
    sinf(tilt_angle_rad) -
    predicted_gravity_after.y
    );

quaternion_norm_squared =
    filter.attitude.w * filter.attitude.w +
    filter.attitude.x * filter.attitude.x +
    filter.attitude.y * filter.attitude.y +
    filter.attitude.z * filter.attitude.z;

TEST_ASSERT_TRUE
    (
    "Tilt correction rotates about positive body X",
    filter.attitude.x > 0.0f
    );

TEST_ASSERT_TRUE
    (
    "Tilt correction reduces gravity-direction error",
    direction_error_after < direction_error_before
    );

TEST_ASSERT_TRUE
    (
    "Accelerometer does not introduce yaw correction",
    fabsf(filter.attitude.z) < 1.0e-6f
    );

TEST_ASSERT_TRUE
    (
    "Corrected quaternion remains normalized",
    fabsf(quaternion_norm_squared - 1.0f) < 1.0e-5f
    );

assert_vector_components
    (
    "Tilt correction leaves uncoupled gyro bias unchanged",
    filter.gyro_bias_rad_s,
    zero_bias
    );

} /* test_mekf_update_accelerometer_corrects_tilt */

/**
 * @brief Verifies that acceleration outside the configured gravity-magnitude
 * gate is rejected without modifying the filter.
 */
void test_mekf_update_accelerometer_rejects_dynamic_acceleration
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_FILTER original_filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_bias = { 0.0f, 0.0f, 0.0f };

VECTOR_3F dynamic_acceleration =
    {
    0.0f,
    0.0f,
    0.0f
    };

dynamic_acceleration.z =
    config.gravity_magnitude_m_s2 +
    config.accelerometer_magnitude_tolerance_m_s2 +
    0.10f;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_bias,
        &config
        )
    );

original_filter = filter;

TEST_ASSERT_FALSE
    (
    "Dynamic acceleration is rejected",
    mekf_update_accelerometer
        (
        &filter,
        dynamic_acceleration
        )
    );

assert_quat_components
    (
    "Rejected measurement leaves attitude unchanged",
    filter.attitude,
    original_filter.attitude
    );

assert_vector_components
    (
    "Rejected measurement leaves gyro bias unchanged",
    filter.gyro_bias_rad_s,
    original_filter.gyro_bias_rad_s
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Rejected measurement leaves covariance unchanged",
            filter.covariance[row][column],
            original_filter.covariance[row][column]
            );
        }
    }

} /* test_mekf_update_accelerometer_rejects_dynamic_acceleration */

/**
 * @brief Verifies that an aligned accelerometer update reduces observable tilt
 * uncertainty without reducing unobservable yaw uncertainty.
 */
void test_mekf_update_accelerometer_updates_covariance
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

float expected[MEKF_ERROR_STATE_DIM][MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

VECTOR_3F aligned_gravity =
    {
    0.0f,
    0.0f,
    9.80665f
    };

/*
 * Initial attitude variance:
 *
 *     p = 0.2^2 = 0.04
 *
 * Direction-measurement variance:
 *
 *     r = 0.1^2 = 0.01
 *
 * For each observable tilt axis:
 *
 *     p_new = p * r / (p + r) = 0.008
 */
config.initial_attitude_std_rad.x = 0.20f;
config.initial_attitude_std_rad.y = 0.20f;
config.initial_attitude_std_rad.z = 0.20f;

config.initial_gyro_bias_std_rad_s = zero_vector;
config.accelerometer_direction_std = 0.10f;

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Aligned covariance update succeeds",
    mekf_update_accelerometer
        (
        &filter,
        aligned_gravity
        )
    );

expected[MEKF_ATTITUDE_ERROR_X][MEKF_ATTITUDE_ERROR_X] = 0.008f;
expected[MEKF_ATTITUDE_ERROR_Y][MEKF_ATTITUDE_ERROR_Y] = 0.008f;

/*
 * Gravity cannot observe rotation about the gravity vector, so the Z-axis
 * attitude variance remains at its initial value.
 */
expected[MEKF_ATTITUDE_ERROR_Z][MEKF_ATTITUDE_ERROR_Z] = 0.040f;

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Accelerometer-updated covariance entry",
            filter.covariance[row][column],
            expected[row][column]
            );
        }
    }

} /* test_mekf_update_accelerometer_updates_covariance */

/**
 * @brief Verifies that accelerometer correction can update gyro bias through
 * attitude-to-bias cross-covariance created during prediction.
 */
void test_mekf_update_accelerometer_corrects_coupled_gyro_bias
    (
    void
    )
{
MEKF_FILTER filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };
VECTOR_3F tilted_gravity;

float tilt_angle_rad = 0.087266463f;

/*
 * Begin with both attitude and gyro-bias uncertainty. Prediction will create
 * negative attitude-to-bias cross-covariance.
 */
config.initial_attitude_std_rad.x = 0.20f;
config.initial_attitude_std_rad.y = 0.20f;
config.initial_attitude_std_rad.z = 0.20f;

config.initial_gyro_bias_std_rad_s.x = 0.10f;
config.initial_gyro_bias_std_rad_s.y = 0.10f;
config.initial_gyro_bias_std_rad_s.z = 0.10f;

config.gyro_noise_density_rad_s_sqrt_hz = 0.0f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0f;
config.accelerometer_direction_std = 0.10f;
config.maximum_delta_time_s = 0.10f;

tilted_gravity.x = 0.0f;

tilted_gravity.y =
    config.gravity_magnitude_m_s2 *
    sinf(tilt_angle_rad);

tilted_gravity.z =
    config.gravity_magnitude_m_s2 *
    cosf(tilt_angle_rad);

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

TEST_ASSERT_TRUE
    (
    "Zero-rate prediction creates bias coupling",
    mekf_predict
        (
        &filter,
        zero_vector,
        0.10f
        )
    );

TEST_ASSERT_TRUE
    (
    "Coupled accelerometer update succeeds",
    mekf_update_accelerometer
        (
        &filter,
        tilted_gravity
        )
    );

TEST_ASSERT_TRUE
    (
    "Accelerometer applies positive X tilt correction",
    filter.attitude.x > 0.0f
    );

TEST_ASSERT_TRUE
    (
    "Coupled X gyro-bias estimate is corrected",
    filter.gyro_bias_rad_s.x < 0.0f
    );

TEST_ASSERT_TRUE
    (
    "Uncoupled Y gyro-bias remains zero",
    fabsf(filter.gyro_bias_rad_s.y) < 1.0e-6f
    );

TEST_ASSERT_TRUE
    (
    "Unobservable Z gyro-bias remains zero",
    fabsf(filter.gyro_bias_rad_s.z) < 1.0e-6f
    );

} /* test_mekf_update_accelerometer_corrects_coupled_gyro_bias */

/**
 * @brief Verifies that null, zero-magnitude, and nonfinite accelerometer inputs
 * are rejected without modifying the filter.
 */
void test_mekf_update_accelerometer_rejects_invalid_input
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_FILTER original_filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };

VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

VECTOR_3F nonfinite_acceleration =
    {
    NAN,
    0.0f,
    9.80665f
    };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

original_filter = filter;

TEST_ASSERT_FALSE
    (
    "Null filter is rejected",
    mekf_update_accelerometer
        (
        NULL,
        zero_vector
        )
    );

TEST_ASSERT_FALSE
    (
    "Zero-magnitude acceleration is rejected",
    mekf_update_accelerometer
        (
        &filter,
        zero_vector
        )
    );

TEST_ASSERT_FALSE
    (
    "Nonfinite acceleration is rejected",
    mekf_update_accelerometer
        (
        &filter,
        nonfinite_acceleration
        )
    );

assert_quat_components
    (
    "Invalid input leaves attitude unchanged",
    filter.attitude,
    original_filter.attitude
    );

assert_vector_components
    (
    "Invalid input leaves gyro bias unchanged",
    filter.gyro_bias_rad_s,
    original_filter.gyro_bias_rad_s
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Invalid input leaves covariance unchanged",
            filter.covariance[row][column],
            original_filter.covariance[row][column]
            );
        }
    }

} /* test_mekf_update_accelerometer_rejects_invalid_input */

/**
 * @brief Verifies that a gravity measurement pointing opposite the predicted
 * direction is rejected without modifying the filter.
 */
void test_mekf_update_accelerometer_rejects_large_direction_error
    (
    void
    )
{
unsigned int row;
unsigned int column;

MEKF_FILTER filter;
MEKF_FILTER original_filter;
MEKF_CONFIG config = make_valid_config();

QUAT identity = { 1.0f, 0.0f, 0.0f, 0.0f };
VECTOR_3F zero_vector = { 0.0f, 0.0f, 0.0f };

VECTOR_3F opposite_gravity =
    {
    0.0f,
    0.0f,
    -9.80665f
    };

TEST_ASSERT_TRUE
    (
    "MEKF initialization succeeds",
    mekf_init
        (
        &filter,
        identity,
        zero_vector,
        &config
        )
    );

original_filter = filter;

TEST_ASSERT_FALSE
    (
    "Opposite gravity direction is rejected",
    mekf_update_accelerometer
        (
        &filter,
        opposite_gravity
        )
    );

assert_quat_components
    (
    "Rejected direction leaves attitude unchanged",
    filter.attitude,
    original_filter.attitude
    );

assert_vector_components
    (
    "Rejected direction leaves gyro bias unchanged",
    filter.gyro_bias_rad_s,
    original_filter.gyro_bias_rad_s
    );

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        TEST_ASSERT_EQ_FLOAT
            (
            "Rejected direction leaves covariance unchanged",
            filter.covariance[row][column],
            original_filter.covariance[row][column]
            );
        }
    }

} /* test_mekf_update_accelerometer_rejects_large_direction_error */

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
    "mekf_init_identity_and_bias",
    test_mekf_init_identity_and_bias
    },
    {
    "mekf_init_normalizes_attitude",
    test_mekf_init_normalizes_attitude
    },
    {
    "mekf_init_zero_quaternion_uses_identity",
    test_mekf_init_zero_quaternion_uses_identity
    },
    {
    "mekf_init_sets_diagonal_covariance",
    test_mekf_init_sets_diagonal_covariance
    },
    {
    "mekf_init_copies_config",
    test_mekf_init_copies_config
    },
    {
    "mekf_init_rejects_null_pointers",
    test_mekf_init_rejects_null_pointers
    },
    {
    "mekf_init_rejects_nonfinite_state",
    test_mekf_init_rejects_nonfinite_state
    },
    {
    "mekf_init_rejects_invalid_uncertainty",
    test_mekf_init_rejects_invalid_uncertainty
    },
    {
    "mekf_init_rejects_invalid_process_noise",
    test_mekf_init_rejects_invalid_process_noise
    },
    {
    "mekf_init_rejects_invalid_accelerometer_config",
    test_mekf_init_rejects_invalid_accelerometer_config
    },
    {
    "mekf_init_rejects_invalid_maximum_timestep",
    test_mekf_init_rejects_invalid_maximum_timestep
    },
    {
    "mekf_init_accepts_zero_uncertainty_and_noise",
    test_mekf_init_accepts_zero_uncertainty_and_noise
    },
    {
    "mekf_init_failure_preserves_filter",
    test_mekf_init_failure_preserves_filter
    },
    {
    "mekf_predict_zero_rate",
    test_mekf_predict_zero_rate
    },
    {
    "mekf_predict_positive_yaw",
    test_mekf_predict_positive_yaw
    },
    {
    "mekf_predict_subtracts_gyro_bias",
    test_mekf_predict_subtracts_gyro_bias
    },
    {
    "mekf_predict_couples_bias_uncertainty",
    test_mekf_predict_couples_bias_uncertainty
    },
    {
    "mekf_predict_adds_process_noise",
    test_mekf_predict_adds_process_noise
    },
    {
    "mekf_predict_rejects_invalid_timestep",
    test_mekf_predict_rejects_invalid_timestep
    },
    {
    "mekf_predict_rotates_attitude_covariance",
    test_mekf_predict_rotates_attitude_covariance
    },
    {
    "mekf_update_accelerometer_corrects_tilt",
    test_mekf_update_accelerometer_corrects_tilt
    },
    {
    "mekf_update_accelerometer_aligned_measurement",
    test_mekf_update_accelerometer_aligned_measurement
    },
    {
    "mekf_update_accelerometer_rejects_dynamic_acceleration",
    test_mekf_update_accelerometer_rejects_dynamic_acceleration
    },
    {
    "mekf_update_accelerometer_updates_covariance",
    test_mekf_update_accelerometer_updates_covariance
    },
    {
    "mekf_update_accelerometer_corrects_coupled_gyro_bias",
    test_mekf_update_accelerometer_corrects_coupled_gyro_bias
    },
    {
    "mekf_update_accelerometer_rejects_invalid_input",
    test_mekf_update_accelerometer_rejects_invalid_input
    },
    {
    "mekf_update_accelerometer_rejects_large_direction_error",
    test_mekf_update_accelerometer_rejects_large_direction_error
    },
    };

TEST_INITIALIZE_TEST("mekf.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/