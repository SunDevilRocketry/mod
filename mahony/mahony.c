/*******************************************************************************
 *
 * FILE:
 *      mahony.c
 *
 * DESCRIPTION:
 *      Mahony attitude filter implementation.
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
#include "mahony.h"

/*------------------------------------------------------------------------------
 Private Functions
 ------------------------------------------------------------------------------*/

/**
 * @brief Determines whether every quaternion component is finite.
 */
static bool mahony_quat_is_finite
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

} /* mahony_quat_is_finite */


/**
 * @brief Determines whether every vector component is finite.
 */
static bool mahony_vector_is_finite
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

} /* mahony_vector_is_finite */


/*------------------------------------------------------------------------------
 Public Functions
 ------------------------------------------------------------------------------*/

bool mahony_init
    (
    MAHONY_FILTER *filter,
    QUAT initial_attitude,
    float proportional_gain,
    float integral_gain
    )
{
if ( filter == NULL )
    {
    return false;
    }

if ( !mahony_quat_is_finite(initial_attitude) )
    {
    return false;
    }

if ( !isfinite(proportional_gain) ||
     !isfinite(integral_gain) )
    {
    return false;
    }

if ( proportional_gain < 0.0f ||
     integral_gain < 0.0f )
    {
    return false;
    }

filter->attitude = quat_normalize(initial_attitude);

filter->integral_error.x = 0.0f;
filter->integral_error.y = 0.0f;
filter->integral_error.z = 0.0f;

filter->proportional_gain = proportional_gain;
filter->integral_gain = integral_gain;

return true;

} /* mahony_init */


bool mahony_update_gyro
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
    return false;
    }

if ( !mahony_quat_is_finite(filter->attitude) )
    {
    return false;
    }

if ( !mahony_vector_is_finite(gyro_body_rad_s) )
    {
    return false;
    }

if ( !isfinite(delta_time_s) ||
     delta_time_s <= 0.0f )
    {
    return false;
    }

/*
 * The attitude quaternion is a body-to-world rotation and angular velocity is
 * expressed in the body frame:
 *
 *     q_dot = 0.5 * q * omega_body
 */
angular_velocity.w = 0.0f;
angular_velocity.x = gyro_body_rad_s.x;
angular_velocity.y = gyro_body_rad_s.y;
angular_velocity.z = gyro_body_rad_s.z;

attitude_derivative = quat_mult
    (
    filter->attitude,
    angular_velocity
    );

attitude_derivative = quat_scale
    (
    attitude_derivative,
    0.5f
    );

attitude_delta = quat_scale
    (
    attitude_derivative,
    delta_time_s
    );

filter->attitude = quat_add
    (
    filter->attitude,
    attitude_delta
    );

filter->attitude = quat_normalize(filter->attitude);

return true;

} /* mahony_update_gyro */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/