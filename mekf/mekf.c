/*******************************************************************************
 *
 * FILE:
 *      mekf.c
 *
 * DESCRIPTION:
 *      Multiplicative Extended Kalman Filter attitude estimator implementation.
 *
 ******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes
 ------------------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>

/*------------------------------------------------------------------------------
 Project Includes
 ------------------------------------------------------------------------------*/
#include "mekf.h"

/*------------------------------------------------------------------------------
 Private Functions
 ------------------------------------------------------------------------------*/

/**
 * @brief Determines whether every quaternion component is finite.
 */
static bool mekf_quat_is_finite
    (
    QUAT quaternion
    )
{
return
    (
    isfinite(quaternion.w) &&
    isfinite(quaternion.x) &&
    isfinite(quaternion.y) &&
    isfinite(quaternion.z)
    );

} /* mekf_quat_is_finite */

/**
 * @brief Determines whether every vector component is finite.
 */
static bool mekf_vector_is_finite
    (
    VECTOR_3F vector
    )
{
return
    (
    isfinite(vector.x) &&
    isfinite(vector.y) &&
    isfinite(vector.z)
    );

} /* mekf_vector_is_finite */

/**
 * @brief Determines whether a standard deviation and its variance are valid.
 */
static bool mekf_standard_deviation_is_valid
    (
    float standard_deviation
    )
{
float variance;

if ( !isfinite(standard_deviation) ||
     standard_deviation < 0.0f )
    {
    return false;
    }

variance = standard_deviation * standard_deviation;

return isfinite(variance);

} /* mekf_standard_deviation_is_valid */

/**
 * @brief Determines whether all components are valid standard deviations.
 */
static bool mekf_standard_deviation_vector_is_valid
    (
    VECTOR_3F standard_deviation
    )
{
return
    (
    mekf_standard_deviation_is_valid(standard_deviation.x) &&
    mekf_standard_deviation_is_valid(standard_deviation.y) &&
    mekf_standard_deviation_is_valid(standard_deviation.z)
    );

} /* mekf_standard_deviation_vector_is_valid */

/**
 * @brief Validates MEKF initialization and process-noise configuration.
 */
static bool mekf_config_is_valid
    (
    const MEKF_CONFIG *config
    )
{
if ( config == NULL )
    {
    return false;
    }

if ( !mekf_standard_deviation_vector_is_valid
        (
        config->initial_attitude_std_rad
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_vector_is_valid
        (
        config->initial_gyro_bias_std_rad_s
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_is_valid
        (
        config->gyro_noise_density_rad_s_sqrt_hz
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_is_valid
        (
        config->gyro_bias_random_walk_rad_s2_sqrt_hz
        ) )
    {
    return false;
    }

if ( !isfinite(config->maximum_delta_time_s) ||
     config->maximum_delta_time_s <= 0.0f )
    {
    return false;
    }

return true;

} /* mekf_config_is_valid */

/*------------------------------------------------------------------------------
 Public Functions
 ------------------------------------------------------------------------------*/

bool mekf_init
    (
    MEKF_FILTER *filter,
    QUAT initial_attitude,
    VECTOR_3F initial_gyro_bias_rad_s,
    const MEKF_CONFIG *config
    )
{
unsigned int row;
unsigned int column;

if ( filter == NULL )
    {
    return false;
    }

if ( !mekf_quat_is_finite(initial_attitude) )
    {
    return false;
    }

if ( !mekf_vector_is_finite(initial_gyro_bias_rad_s) )
    {
    return false;
    }

if ( !mekf_config_is_valid(config) )
    {
    return false;
    }

/*
 * Store the normalized nominal body-to-world attitude and the nominal
 * body-frame gyro-bias estimate.
 */
filter->attitude = quat_normalize(initial_attitude);
filter->gyro_bias_rad_s = initial_gyro_bias_rad_s;
filter->config = *config;

/*
 * The MEKF error state begins with zero mean. Initialize its covariance as a
 * diagonal matrix using the configured per-axis standard deviations.
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        filter->covariance[row][column] = 0.0f;
        }
    }

filter->covariance
    [MEKF_ATTITUDE_ERROR_X]
    [MEKF_ATTITUDE_ERROR_X] =
        config->initial_attitude_std_rad.x *
        config->initial_attitude_std_rad.x;

filter->covariance
    [MEKF_ATTITUDE_ERROR_Y]
    [MEKF_ATTITUDE_ERROR_Y] =
        config->initial_attitude_std_rad.y *
        config->initial_attitude_std_rad.y;

filter->covariance
    [MEKF_ATTITUDE_ERROR_Z]
    [MEKF_ATTITUDE_ERROR_Z] =
        config->initial_attitude_std_rad.z *
        config->initial_attitude_std_rad.z;

filter->covariance
    [MEKF_GYRO_BIAS_ERROR_X]
    [MEKF_GYRO_BIAS_ERROR_X] =
        config->initial_gyro_bias_std_rad_s.x *
        config->initial_gyro_bias_std_rad_s.x;

filter->covariance
    [MEKF_GYRO_BIAS_ERROR_Y]
    [MEKF_GYRO_BIAS_ERROR_Y] =
        config->initial_gyro_bias_std_rad_s.y *
        config->initial_gyro_bias_std_rad_s.y;

filter->covariance
    [MEKF_GYRO_BIAS_ERROR_Z]
    [MEKF_GYRO_BIAS_ERROR_Z] =
        config->initial_gyro_bias_std_rad_s.z *
        config->initial_gyro_bias_std_rad_s.z;

return true;

} /* mekf_init */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/