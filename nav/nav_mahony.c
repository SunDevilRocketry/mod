/**
  ******************************************************************************
  * @file           : nav_mahony.c
  * @brief          : Navigation & sensor fusion library's attitude filter 
  *                   interface.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                        ##### Integration Guide #####
  ==============================================================================
  [..]
  (+) Initialize the filter with a pointer to an existing or uninitialized filter, 
      an initial attiutude estimate, and the proportional and integral gains for
      accelerometer correction.
  (+) Call updates to the filter with gyroscope and accelerometer vectors and 
      time elapsed. Optionally disable accelerometer fusion during dynamic 
      flight.
  ******************************************************************************
  @endverbatim
  */

/* Standard Includes ---------------------------------------------------------*/
#include <math.h>
#include <stddef.h>

/* Project Includes ----------------------------------------------------------*/
#include "nav_prv.h"

/* Private Macros ------------------------------------------------------------*/

/*
 * Accelerometer feedback is only used when the measured magnitude is reasonably
 * close to one g. These are initial software validation thresholds and should
 * be tuned later using stationary, vibration, and flight data.
 */
#define MAHONY_ACCEL_MIN_MAGNITUDE    (0.85f * GRAVITY)
#define MAHONY_ACCEL_MAX_MAGNITUDE    (1.15f * GRAVITY)

/*
 * Limits each integral correction component to prevent windup. The value is
 * expressed as an angular-rate correction in radians per second and should be
 * tuned using sensor characterization and flight data.
 */
#define MAHONY_INTEGRAL_LIMIT_RAD_S    0.25f

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Initializes a Mahony attitude filter.
 * @note Assumes state reserved by the caller.
 *
 * @param filter Filter instance to initialize.
 * @param initial_attitude Initial body-to-world attitude quaternion.
 * @param proportional_gain Proportional correction gain.
 * @param integral_gain Integral correction gain.
 *
 * @return MAHONY_OK when initialization succeeds; otherwise a Mahony status
 *         code describing the failure.
 */
NAV_STATUS _nav_mahony_init
    (
    NAV_PRIVATE_STATE* prv_state
    )
{
nav_nullchk( prv_state );

/* Local Variables */
QUAT initial_attitude;

/* Initialize Locals */
initial_attitude = _nav_quat_grav_attitude
    (
    IDENTITY_QUAT,
    prv_state->cal_data.accel
    );


if ( !math_quat_is_finite(initial_attitude) )
    {
    _nav_debug_error( "Invalid initial attitude quaternion" );
    return NAV_INVALID_PARAMS;
    }

if ( !isfinite(prv_state->saved_configs.mahony_kp) 
  || !isfinite(prv_state->saved_configs.mahony_ki) )
    {
    _nav_debug_error( "Invalid mahony gains" );
    return NAV_INVALID_CONFIG;
    }

if ( prv_state->saved_configs.mahony_kp < 0.0f || prv_state->saved_configs.mahony_ki < 0.0f )
    {
    _nav_debug_error( "Invalid mahony gains" );
    return NAV_INVALID_CONFIG;
    }

/* Initialize mahony state */
prv_state->mahony_state.attitude = math_quat_normalize(initial_attitude);
prv_state->mahony_state.integral_error.x = 0.0f;
prv_state->mahony_state.integral_error.y = 0.0f;
prv_state->mahony_state.integral_error.z = 0.0f;
prv_state->mahony_state.proportional_gain = prv_state->saved_configs.mahony_kp;
prv_state->mahony_state.integral_gain = prv_state->saved_configs.mahony_ki;

return NAV_OK;

} /* _nav_mahony_init */


/**
 * @brief Propagates attitude using body-frame gyroscope measurements.
 * @note Assumes state reserved by the caller.
 *
 * @details
 * Verify the filter, gyro, and timestep are valid.
 * Convert the body-frame angular velocity into a pure quaternion.
 * Use the quaternion differential equation to calculate how quickly the
 * body-to-world attitude is changing. Multiply that derivative by the
 * timestep, add it to the current attitude, and normalize the result.
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame angular velocity in radians per second.
 * @param delta_time_s Elapsed time in seconds.
 *
 * @return NAV_OK when succeeds; otherwise a Mahony status
           code describing the failure.
 */
NAV_STATUS _nav_mahony_update_gyro
    (
    NAV_MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    )
{
QUAT angular_velocity;
QUAT attitude_derivative;
QUAT attitude_delta;

if ( filter == NULL )
    {
    _nav_debug_error( "Null mahony filter state provided" );
    return NAV_NULLPTR;
    }

if ( !math_quat_is_finite(filter->attitude) )
    {
    _nav_debug_error( "Bad mahony filter attitude provided" );
    return NAV_INVALID_PARAMS;
    }

if ( !math_vector_is_finite(gyro_body_rad_s) )
    {
    _nav_debug_error( "Bad mahony filter gyro state provided" );
    return NAV_INVALID_PARAMS;
    }

if ( !isfinite(delta_time_s) || delta_time_s <= 0.0f )
    {
    _nav_debug_error( "Bad mahony time delta provided" );
    return NAV_INVALID_PARAMS;
    }

/*
 * The attitude quaternion is a body-to-world rotation and angular velocity
 * is expressed in the body frame:
 */
angular_velocity.w = 0.0f;
angular_velocity.x = gyro_body_rad_s.x;
angular_velocity.y = gyro_body_rad_s.y;
angular_velocity.z = gyro_body_rad_s.z;

/* q_dot = 0.5 * q * omega_body */
attitude_derivative = math_quat_mult(filter->attitude, angular_velocity);
attitude_derivative = math_quat_scale(attitude_derivative, 0.5f);

/* Scale q_dot by delta_t, add to attitude, and normalize*/
attitude_delta = math_quat_scale(attitude_derivative, delta_time_s);
filter->attitude = math_quat_add(filter->attitude, attitude_delta);
filter->attitude = math_quat_normalize(filter->attitude);

return NAV_OK;

} /* _nav_mahony_update_gyro */


/**
 * @brief Updates attitude using gyroscope propagation and accelerometer
 *        proportional feedback.
 * @note Assumes state reserved by the caller.
 * 
 * @details
 * The gyroscope must be expressed in the body frame in radians per second.
 * The accelerometer must be expressed in the body frame. Its magnitude is
 * removed internally because the filter uses only its measured direction.
 *
 * Accelerometer feedback is applied only when the caller enables it and the
 * measured acceleration magnitude falls within the configured validity range.
 * Invalid accelerometer samples are ignored while gyro propagation continues.
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame angular velocity in radians per second.
 * @param accel_body Body-frame accelerometer measurement.
 * @param delta_time_s Elapsed time in seconds.
 * @param use_accel Whether accelerometer feedback should be applied.
 *
 * @return NAV_OK when the attitude was updated; otherwise a Mahony status
        code describing the failure.
 */
NAV_STATUS _nav_mahony_update_imu
    (
    NAV_MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    VECTOR_3F accel_body,
    float delta_time_s,
    bool use_accel
    )
{
QUAT gravity_world;
QUAT gravity_body_quat;

VECTOR_3F gravity_estimated_body;
VECTOR_3F attitude_error;
VECTOR_3F proportional_correction;
VECTOR_3F gyro_corrected;

float accel_magnitude;

bool accel_valid;
bool apply_accel;

if ( filter == NULL )
    {
    _nav_debug_error( "Null mahony filter state provided" );
    return NAV_NULLPTR;
    }

if ( !math_quat_is_finite(filter->attitude) )
    {
    _nav_debug_error( "Bad mahony filter attitude provided" );
    return NAV_INVALID_PARAMS;
    }

if ( !math_vector_is_finite(gyro_body_rad_s) )
    {
    _nav_debug_error( "Bad mahony filter gyro state provided" );
    return NAV_INVALID_PARAMS;
    }

if ( !math_vector_is_finite(accel_body) )
    {
    _nav_debug_error( "Bad mahony filter accel state provided. In release mode, this will continue." );
    }

if ( !isfinite(delta_time_s) || delta_time_s <= 0.0f )
    {
    _nav_debug_error( "Bad mahony time delta provided" );
    return NAV_INVALID_PARAMS;
    }

accel_magnitude = math_vector_magnitude(accel_body);

accel_valid = math_vector_is_finite(accel_body)
           && isfinite(accel_magnitude)
           && accel_magnitude >= MAHONY_ACCEL_MIN_MAGNITUDE
           && accel_magnitude <= MAHONY_ACCEL_MAX_MAGNITUDE;

apply_accel = use_accel && accel_valid;

gyro_corrected = gyro_body_rad_s;

/*
 * Accelerometer feedback is optional. If the measurement is invalid or has
 * zero magnitude, continue with gyroscope-only propagation.
 */
if ( apply_accel && math_vector_normalize(&accel_body) )
    {
    /*
    * The attitude quaternion represents the body-to-world rotation.
    * Rotate the fixed world gravity direction into the body frame to
    * predict where gravity should appear according to the estimate.
    */
    gravity_world.w = 0.0f;
    gravity_world.x = 0.0f;
    gravity_world.y = 0.0f;
    gravity_world.z = -1.0f;

    gravity_body_quat = math_quat_rotate_world_to_body(filter->attitude, gravity_world);

    gravity_estimated_body.x = gravity_body_quat.x;
    gravity_estimated_body.y = gravity_body_quat.y;
    gravity_estimated_body.z = gravity_body_quat.z;

    /*
    * measured x estimated produces a correction that drives the
    * estimated gravity direction toward the measured direction.
    */
    attitude_error = math_vector_cross(accel_body, gravity_estimated_body);

    /*
    * Accumulate persistent attitude error as a gyro-rate correction. Integral
    * feedback is updated only while accelerometer feedback is both permitted and
    * valid, preventing high-dynamic or corrupted measurements from winding up
    * the correction state.
    */
    if ( filter->integral_gain > 0.0f )
        {
        filter->integral_error.x += filter->integral_gain * attitude_error.x * delta_time_s;
        filter->integral_error.y += filter->integral_gain * attitude_error.y * delta_time_s;
        /* The filter has no yaw reference, so avoid accumulating drift on the z axis */
        /* filter->integral_error.z += filter->integral_gain * attitude_error.z * delta_time_s; */

        filter->integral_error.x = math_clamp_float(filter->integral_error.x, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S);
        filter->integral_error.y = math_clamp_float(filter->integral_error.y, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S);
        /* filter->integral_error.z = clamp_float(filter->integral_error.z, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S); */
        }

    proportional_correction = math_vector_scale(attitude_error, filter->proportional_gain);

    gyro_corrected = math_vector_add(gyro_corrected, proportional_correction);
    gyro_corrected = math_vector_add(gyro_corrected, filter->integral_error);
    }

return _nav_mahony_update_gyro(filter, gyro_corrected, delta_time_s);
    
} /* _nav_mahony_update_imu */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/