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
 * @brief Verifies that prediction configuration is copied into the filter.
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
    }
    };

TEST_INITIALIZE_TEST("mekf.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/