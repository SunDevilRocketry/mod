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
 Private Macros
 ------------------------------------------------------------------------------*/

/*
 * Accelerometer feedback is only used when the measured magnitude is reasonably
 * close to one g. These are initial software validation thresholds and should
 * be tuned later using stationary, vibration, and flight data.
 */
#define MAHONY_ACCEL_MIN_MAGNITUDE    (0.85f * GRAVITY)
#define MAHONY_ACCEL_MAX_MAGNITUDE    (1.15f * GRAVITY)

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

static float vector_magnitude
    (
    VECTOR_3F vector
    )
{
return sqrtf
    (
    vector.x * vector.x +
    vector.y * vector.y +
    vector.z * vector.z
    );

} /* vector_magnitude */


static bool vector_normalize
    (
    VECTOR_3F *vector
    )
{
float magnitude;

if ( vector == NULL )
    {
    return false;
    }

if ( !mahony_vector_is_finite(*vector) )
    {
    return false;
    }

magnitude = vector_magnitude(*vector);

if ( magnitude <= 0.0f || !isfinite(magnitude) )
    {
    return false;
    }

vector->x /= magnitude;
vector->y /= magnitude;
vector->z /= magnitude;

return true;

} /* vector_normalize */


static VECTOR_3F vector_cross
    (
    VECTOR_3F a,
    VECTOR_3F b
    )
{
VECTOR_3F result;

result.x = a.y * b.z - a.z * b.y;
result.y = a.z * b.x - a.x * b.z;
result.z = a.x * b.y - a.y * b.x;

return result;

} /* vector_cross */


static VECTOR_3F vector_add
    (
    VECTOR_3F a,
    VECTOR_3F b
    )
{
VECTOR_3F result;

result.x = a.x + b.x;
result.y = a.y + b.y;
result.z = a.z + b.z;

return result;

} /* vector_add */


static VECTOR_3F vector_scale
    (
    VECTOR_3F vector,
    float scalar
    )
{
VECTOR_3F result;

result.x = vector.x * scalar;
result.y = vector.y * scalar;
result.z = vector.z * scalar;

return result;

} /* vector_scale */


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

/*
 * Verify the filter, gyro, and timestep are valid.
 * Convert the body-frame angular velocity into a pure quaternion.
 * Use the quaternion differential equation to calculate how quickly the
 * body-to-world attitude is changing. Multiply that derivative by the
 * timestep, add it to the current attitude, and normalize the result.
 */

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

bool mahony_update_imu
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

accel_magnitude = vector_magnitude(accel_body);

accel_valid =
    mahony_vector_is_finite(accel_body) &&
    isfinite(accel_magnitude) &&
    accel_magnitude >= MAHONY_ACCEL_MIN_MAGNITUDE &&
    accel_magnitude <= MAHONY_ACCEL_MAX_MAGNITUDE;

apply_accel =
    use_accel &&
    accel_valid;

gyro_corrected = gyro_body_rad_s;

/*
 * Accelerometer feedback is optional. If the measurement is invalid or has
 * zero magnitude, continue with gyroscope-only propagation.
 */
if ( apply_accel )
    {
    if ( vector_normalize(&accel_body) )
        {
        /*
         * The attitude quaternion represents the body-to-world rotation.
         * Rotate the fixed world gravity direction into the body frame to
         * predict where gravity should appear according to the estimate.
         */
        gravity_world.w = 0.0f;
        gravity_world.x = 0.0f;
        gravity_world.y = 0.0f;
        gravity_world.z = 1.0f;

        gravity_body_quat = quat_rotate_world_to_body
            (
            filter->attitude,
            gravity_world
            );

        gravity_estimated_body.x = gravity_body_quat.x;
        gravity_estimated_body.y = gravity_body_quat.y;
        gravity_estimated_body.z = gravity_body_quat.z;

        /*
         * measured x estimated produces a correction that drives the
         * estimated gravity direction toward the measured direction.
         */
        attitude_error = vector_cross
            (
            accel_body,
            gravity_estimated_body
            );

        proportional_correction = vector_scale
            (
            attitude_error,
            filter->proportional_gain
            );

        gyro_corrected = vector_add
            (
            gyro_corrected,
            proportional_correction
            );
        }
    }

return mahony_update_gyro
    (
    filter,
    gyro_corrected,
    delta_time_s
    );

} /* mahony_update_imu */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/