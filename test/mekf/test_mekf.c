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
    };

TEST_INITIALIZE_TEST("mekf.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/