/**
  ******************************************************************************
  * @file           : mahony.h
  * @brief          : Mahony attitude filter interface.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
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
#include "mahony.h"

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
 *
 * @param filter Filter instance to initialize.
 * @param initial_attitude Initial body-to-world attitude quaternion.
 * @param proportional_gain Proportional correction gain.
 * @param integral_gain Integral correction gain.
 *
 * @return MAHONY_OK when initialization succeeds; otherwise a Mahony status
 *         code describing the failure.
 */
MAHONY_STATUS mahony_init
    (
    MAHONY_FILTER *filter,
    QUAT initial_attitude,
    float proportional_gain,
    float integral_gain
    )
{
if ( filter == NULL )
    {
    return MAHONY_NULL_POINTER;
    }

if ( !quat_is_finite(initial_attitude) )
    {
    return MAHONY_INVALID_QUATERNION;
    }

if ( !isfinite(proportional_gain)
    || !isfinite(integral_gain) )
    {
    return MAHONY_NONFINITE_GAIN;
    }

if ( proportional_gain < 0.0f
     || integral_gain < 0.0f )
    {
    return MAHONY_NEGATIVE_GAIN;
    }

filter->attitude = quat_normalize(initial_attitude);
filter->integral_error.x = 0.0f;
filter->integral_error.y = 0.0f;
filter->integral_error.z = 0.0f;
filter->proportional_gain = proportional_gain;
filter->integral_gain = integral_gain;

return MAHONY_OK;
} /* mahony_init */


/**
 * @brief Propagates attitude using body-frame gyroscope measurements.
 *
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
 * @return MAHONY_OK when initialization succeeds; otherwise a Mahony status
           code describing the failure.
 */
MAHONY_STATUS mahony_update_gyro
    (
    MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    )
{
QUAT angular_velocity;
QUAT attitude_derivative;
QUAT attitude_delta;

if ( filter == NULL )
    {
    return MAHONY_NULL_POINTER;
    }

if ( !quat_is_finite(filter->attitude) )
    {
    return MAHONY_INVALID_QUATERNION;
    }

if ( !vector_is_finite(gyro_body_rad_s) )
    {
    return MAHONY_INVALID_GYRO;
    }

if ( !isfinite(delta_time_s)
    || delta_time_s <= 0.0f )
    {
    return MAHONY_INVALID_DELTA_TIME;
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
attitude_derivative = quat_mult(filter->attitude, angular_velocity);
attitude_derivative = quat_scale(attitude_derivative, 0.5f);

/* Scale q_dot by delta_t, add to attitude, and normalize*/
attitude_delta = quat_scale(attitude_derivative, delta_time_s);
filter->attitude = quat_add(filter->attitude, attitude_delta);
filter->attitude = quat_normalize(filter->attitude);

return MAHONY_OK;

} /* mahony_update_gyro */


/**
 * @brief Updates attitude using gyroscope propagation and accelerometer
 *        proportional feedback.
 *
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
 * @return MAHONY_OK when the attitude was updated; otherwise a Mahony status
        code describing the failure.
 */
MAHONY_STATUS mahony_update_imu
    (
    MAHONY_FILTER *filter,
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
    return MAHONY_NULL_POINTER;
    }

if ( !vector_is_finite(gyro_body_rad_s) )
    {
    return MAHONY_INVALID_GYRO;
    }

if ( !isfinite(delta_time_s)
    || delta_time_s <= 0.0f )
    {
    return MAHONY_INVALID_DELTA_TIME;
    }

accel_magnitude = vector_magnitude(accel_body);

accel_valid = vector_is_finite(accel_body)
            && isfinite(accel_magnitude)
            && accel_magnitude >= MAHONY_ACCEL_MIN_MAGNITUDE
            && accel_magnitude <= MAHONY_ACCEL_MAX_MAGNITUDE;

apply_accel = use_accel && accel_valid;

gyro_corrected = gyro_body_rad_s;

/*
 * Accelerometer feedback is optional. If the measurement is invalid or has
 * zero magnitude, continue with gyroscope-only propagation.
 */
if ( apply_accel && vector_normalize(&accel_body) )
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

    gravity_body_quat = quat_rotate_world_to_body(filter->attitude, gravity_world);

    gravity_estimated_body.x = gravity_body_quat.x;
    gravity_estimated_body.y = gravity_body_quat.y;
    gravity_estimated_body.z = gravity_body_quat.z;

    /*
    * measured x estimated produces a correction that drives the
    * estimated gravity direction toward the measured direction.
    */
    attitude_error = vector_cross(accel_body, gravity_estimated_body);

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
        filter->integral_error.z += filter->integral_gain * attitude_error.z * delta_time_s;

        filter->integral_error.x = clamp_float(filter->integral_error.x, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S);
        filter->integral_error.y = clamp_float(filter->integral_error.y, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S);
        filter->integral_error.z = clamp_float(filter->integral_error.z, -MAHONY_INTEGRAL_LIMIT_RAD_S, MAHONY_INTEGRAL_LIMIT_RAD_S);
        }

    proportional_correction = vector_scale(attitude_error, filter->proportional_gain);

    gyro_corrected = vector_add(gyro_corrected, proportional_correction);
    gyro_corrected = vector_add(gyro_corrected, filter->integral_error);
    }

return mahony_update_gyro(filter, gyro_corrected, delta_time_s);

} /* mahony_update_imu */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/