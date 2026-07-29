/*******************************************************************************
 *
 * FILE:
 *      mekf.c
 *
 * DESCRIPTION:
 *      Multiplicative Extended Kalman Filter attitude estimator implementation.
 *
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
 Private Macros
 ------------------------------------------------------------------------------*/

/**
 * @brief Rotation magnitude below which the small-angle approximation is used.
 */
#define MEKF_SMALL_ANGLE_RAD    1.0e-6f

/**
 * @brief Smallest usable absolute determinant for a three-by-three matrix.
 */
#define MEKF_MATRIX_INVERSE_MIN_DETERMINANT    1.0e-20f

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

if ( !isfinite(config->accelerometer_direction_std) ||
     config->accelerometer_direction_std <= 0.0f )
    {
    return false;
    }

if ( !isfinite(config->gravity_magnitude_m_s2) ||
     config->gravity_magnitude_m_s2 <= 0.0f )
    {
    return false;
    }

if ( !isfinite(config->accelerometer_magnitude_tolerance_m_s2) ||
     config->accelerometer_magnitude_tolerance_m_s2 <= 0.0f )
    {
    return false;
    }

if ( !isfinite(config->accelerometer_innovation_gate) ||
     config->accelerometer_innovation_gate <= 0.0f )
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

/**
 * @brief Determines whether every covariance entry is finite.
 */
static bool mekf_covariance_is_finite
    (
    const float covariance
        [MEKF_ERROR_STATE_DIM]
        [MEKF_ERROR_STATE_DIM]
    )
{
unsigned int row;
unsigned int column;

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        if ( !isfinite(covariance[row][column]) )
            {
            return false;
            }
        }
    }

return true;

} /* mekf_covariance_is_finite */

/**
 * @brief Inverts a finite nonsingular three-by-three matrix.
 */
static bool mekf_matrix_3x3_inverse
    (
    const float matrix[3][3],
    float inverse[3][3]
    )
{
unsigned int row;
unsigned int column;

float determinant;

determinant =
    matrix[0][0] *
        (
        matrix[1][1] * matrix[2][2] -
        matrix[1][2] * matrix[2][1]
        ) -
    matrix[0][1] *
        (
        matrix[1][0] * matrix[2][2] -
        matrix[1][2] * matrix[2][0]
        ) +
    matrix[0][2] *
        (
        matrix[1][0] * matrix[2][1] -
        matrix[1][1] * matrix[2][0]
        );

if ( !isfinite(determinant) ||
     fabsf(determinant) <= MEKF_MATRIX_INVERSE_MIN_DETERMINANT )
    {
    return false;
    }

inverse[0][0] =
    (
    matrix[1][1] * matrix[2][2] -
    matrix[1][2] * matrix[2][1]
    ) / determinant;

inverse[0][1] =
    (
    matrix[0][2] * matrix[2][1] -
    matrix[0][1] * matrix[2][2]
    ) / determinant;

inverse[0][2] =
    (
    matrix[0][1] * matrix[1][2] -
    matrix[0][2] * matrix[1][1]
    ) / determinant;

inverse[1][0] =
    (
    matrix[1][2] * matrix[2][0] -
    matrix[1][0] * matrix[2][2]
    ) / determinant;

inverse[1][1] =
    (
    matrix[0][0] * matrix[2][2] -
    matrix[0][2] * matrix[2][0]
    ) / determinant;

inverse[1][2] =
    (
    matrix[0][2] * matrix[1][0] -
    matrix[0][0] * matrix[1][2]
    ) / determinant;

inverse[2][0] =
    (
    matrix[1][0] * matrix[2][1] -
    matrix[1][1] * matrix[2][0]
    ) / determinant;

inverse[2][1] =
    (
    matrix[0][1] * matrix[2][0] -
    matrix[0][0] * matrix[2][1]
    ) / determinant;

inverse[2][2] =
    (
    matrix[0][0] * matrix[1][1] -
    matrix[0][1] * matrix[1][0]
    ) / determinant;

for ( row = 0U; row < 3U; row++ )
    {
    for ( column = 0U; column < 3U; column++ )
        {
        if ( !isfinite(inverse[row][column]) )
            {
            return false;
            }
        }
    }

return true;

} /* mekf_matrix_3x3_inverse */

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

/*
 * mekf_predict
 * Predicts the next nominal attitude and error-state covariance using the
 * current gyroscope measurement.
 *
 * Execution steps:
 *
 * 1. Take the measured angular velocity and correct it by subtracting the estimated bias:
 *
 *        omega_corrected = omega_measured - bias_estimated
 *
 * 2. Integrate the corrected angular rate over the timestep to obtain the
 *    incremental body-frame rotation vector:
 *
 *        delta_theta = omega_corrected * delta_time
 *
 * 3. Convert the rotation vector into an incremental rotation quaternion:
 *
 *        delta_q =
 *            [
 *            cos(norm(delta_theta) / 2),
 *            delta_theta / norm(delta_theta) *
 *                sin(norm(delta_theta) / 2)
 *            ]
 *
 * 4. Apply the incremental quaternion through right multiplication:
 *
 *        q_new = normalize(q_old * delta_q)
 *
 * 5. Construct the error-state transition matrix and propagate covariance:
 *
 *        P_new = Phi * P_old * transpose(Phi) + Q_d
 *
 * 6. Commit the predicted attitude and covariance only after both calculations
 *    complete successfully.
 */
bool mekf_predict
    (
    MEKF_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    )
{
unsigned int row;
unsigned int column;
unsigned int inner;
unsigned int axis;

VECTOR_3F corrected_gyro_rad_s;
VECTOR_3F rotation_vector_rad;

QUAT delta_attitude;
QUAT predicted_attitude;

float state_transition
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

float transition_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float predicted_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float rotation_magnitude_rad;
float half_rotation_rad;
float quaternion_vector_scale;

float gyro_noise_variance;
float bias_random_walk_variance;

float delta_time_squared;
float delta_time_cubed;

float attitude_process_variance;
float attitude_bias_process_covariance;
float bias_process_variance;

float matrix_sum;
float symmetric_value;

if ( filter == NULL )
    {
    return false;
    }

if ( !mekf_quat_is_finite(filter->attitude) )
    {
    return false;
    }

if ( !mekf_vector_is_finite(filter->gyro_bias_rad_s) ||
     !mekf_vector_is_finite(gyro_body_rad_s) )
    {
    return false;
    }

if ( !mekf_config_is_valid(&filter->config) )
    {
    return false;
    }

if ( !mekf_covariance_is_finite(filter->covariance) )
    {
    return false;
    }

if ( !isfinite(delta_time_s) ||
     delta_time_s <= 0.0f ||
     delta_time_s > filter->config.maximum_delta_time_s )
    {
    return false;
    }

/*
 * The raw gyro measurement (gyro_body_rad_s) consists of the true angular rate
 * plus sensor bias and measurement noise:
 *
 *     omega_m = omega_true + b_g + n_g
 *
 * Where:
 *     omega_m    is what the gyroscope reports.
 *     omega_true is how fast the rocket is actually rotating.
 *     b_g        is the true gyro bias, a slowly changing offset.
 *     n_g        is random measurement noise.
 *
 * The true bias is unknown, so subtract the current nominal bias estimate,
 * filter->gyro_bias_rad_s, to obtain the corrected angular rate:
 *
 *     omega_corrected = omega_m - b_hat_g
 *
 * In the code:
 *
 *     corrected_gyro_rad_s =
 *         gyro_body_rad_s - filter->gyro_bias_rad_s
 */
corrected_gyro_rad_s.x =
    gyro_body_rad_s.x - filter->gyro_bias_rad_s.x;

corrected_gyro_rad_s.y =
    gyro_body_rad_s.y - filter->gyro_bias_rad_s.y;

corrected_gyro_rad_s.z =
    gyro_body_rad_s.z - filter->gyro_bias_rad_s.z;

/*
 * Integrate angular velocity over the timestep:
 *
 *     delta_theta = omega_corrected * delta_time
 *
 * This produces the incremental body-frame rotation vector. Its magnitude is
 * the total rotation angle:
 *
 *     theta =
 *         norm(delta_theta) =
 *         sqrt
 *             (
 *             delta_theta_x^2 +
 *             delta_theta_y^2 +
 *             delta_theta_z^2
 *             )
 */
rotation_vector_rad.x = corrected_gyro_rad_s.x * delta_time_s;
rotation_vector_rad.y = corrected_gyro_rad_s.y * delta_time_s;
rotation_vector_rad.z = corrected_gyro_rad_s.z * delta_time_s;

rotation_magnitude_rad = sqrtf
    (
    rotation_vector_rad.x * rotation_vector_rad.x +
    rotation_vector_rad.y * rotation_vector_rad.y +
    rotation_vector_rad.z * rotation_vector_rad.z
    );

if ( !isfinite(rotation_magnitude_rad) )
    {
    return false;
    }

/*
 * Construct the incremental rotation quaternion:
 *
 *     delta_q =
 *         [
 *         cos(theta / 2),
 *         rotation_vector * sin(theta / 2) / theta
 *         ]
 *
 * For a very small rotation:
 *
 *     sin(theta / 2) / theta approaches 0.5
 *
 * The small-angle branch avoids division by a value close to zero.
 */
if ( rotation_magnitude_rad <= MEKF_SMALL_ANGLE_RAD )
    {
    delta_attitude.w = 1.0f;
    quaternion_vector_scale = 0.5f;
    }
else
    {
    half_rotation_rad = 0.5f * rotation_magnitude_rad;

    delta_attitude.w = cosf(half_rotation_rad);

    quaternion_vector_scale =
        sinf(half_rotation_rad) /
        rotation_magnitude_rad;
    }

delta_attitude.x =
    rotation_vector_rad.x * quaternion_vector_scale;

delta_attitude.y =
    rotation_vector_rad.y * quaternion_vector_scale;

delta_attitude.z =
    rotation_vector_rad.z * quaternion_vector_scale;

/*
 * The stored quaternion maps body coordinates into world coordinates, and the
 * angular velocity is expressed in the body frame. Therefore, apply the
 * incremental rotation through right multiplication:
 *
 *     q_new = q_old * delta_q
 */
predicted_attitude = quat_mult
    (
    filter->attitude,
    delta_attitude
    );

predicted_attitude = quat_normalize(predicted_attitude);

/*
 * This whole next section propagates the filter’s six-element error state
 *
 * Construct the first-order discrete error-state transition matrix:
 *
 *     Phi =
 *         [
 *         I - skew(omega) * dt    -I * dt
 *                   0                I
 *         ]
 *
 * The upper-right block expresses that an error in the gyro-bias estimate
 * produces an error in propagated attitude.
 * An error that was entirely about one body axis can appear
 * partly along another axis after rotation.
 *
 * Constructs:
 *   -[ω]× * Δt = [     0     -ω_z*Δt   ω_y*Δt ]
 *                [  ω_z*Δt      0     -ω_x*Δt ]
 *                [ -ω_y*Δt   ω_x*Δt      0    ]
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    state_transition[row][row] = 1.0f;
    }

state_transition
    [MEKF_ATTITUDE_ERROR_X]
    [MEKF_ATTITUDE_ERROR_Y] =
        corrected_gyro_rad_s.z * delta_time_s;

state_transition
    [MEKF_ATTITUDE_ERROR_X]
    [MEKF_ATTITUDE_ERROR_Z] =
        -corrected_gyro_rad_s.y * delta_time_s;

state_transition
    [MEKF_ATTITUDE_ERROR_Y]
    [MEKF_ATTITUDE_ERROR_X] =
        -corrected_gyro_rad_s.z * delta_time_s;

state_transition
    [MEKF_ATTITUDE_ERROR_Y]
    [MEKF_ATTITUDE_ERROR_Z] =
        corrected_gyro_rad_s.x * delta_time_s;

state_transition
    [MEKF_ATTITUDE_ERROR_Z]
    [MEKF_ATTITUDE_ERROR_X] =
        corrected_gyro_rad_s.y * delta_time_s;

state_transition
    [MEKF_ATTITUDE_ERROR_Z]
    [MEKF_ATTITUDE_ERROR_Y] =
        -corrected_gyro_rad_s.x * delta_time_s;

for ( axis = 0U; axis < 3U; axis++ )
    {
    state_transition[axis][axis + 3U] = -delta_time_s;
    }

/*
 * First multiplication:
 *
 *     transition_covariance = Phi * P
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U; inner < MEKF_ERROR_STATE_DIM; inner++ )
            {
            matrix_sum +=
                state_transition[row][inner] *
                filter->covariance[inner][column];
            }

        transition_covariance[row][column] = matrix_sum;
        }
    }

/*
 * Second multiplication:
 *
 *     predicted_covariance =
 *         transition_covariance * transpose(Phi)
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U; inner < MEKF_ERROR_STATE_DIM; inner++ )
            {
            matrix_sum +=
                transition_covariance[row][inner] *
                state_transition[column][inner];
            }

        predicted_covariance[row][column] = matrix_sum;
        }
    }

/*
 * Construct the discrete process-noise contribution. Gyro white noise directly
 * increases attitude uncertainty. Bias random walk increases bias uncertainty
 * and also contributes to attitude and attitude-bias uncertainty during the
 * timestep.
 */
gyro_noise_variance =
    filter->config.gyro_noise_density_rad_s_sqrt_hz *
    filter->config.gyro_noise_density_rad_s_sqrt_hz;

bias_random_walk_variance =
    filter->config.gyro_bias_random_walk_rad_s2_sqrt_hz *
    filter->config.gyro_bias_random_walk_rad_s2_sqrt_hz;

delta_time_squared = delta_time_s * delta_time_s;
delta_time_cubed = delta_time_squared * delta_time_s;

attitude_process_variance =
    gyro_noise_variance * delta_time_s +
    bias_random_walk_variance *
    delta_time_cubed /
    3.0f;

attitude_bias_process_covariance =
    -bias_random_walk_variance *
    delta_time_squared /
    2.0f;

bias_process_variance =
    bias_random_walk_variance * delta_time_s;

for ( axis = 0U; axis < 3U; axis++ )
    {
    predicted_covariance[axis][axis] +=
        attitude_process_variance;

    predicted_covariance[axis][axis + 3U] +=
        attitude_bias_process_covariance;

    predicted_covariance[axis + 3U][axis] +=
        attitude_bias_process_covariance;

    predicted_covariance[axis + 3U][axis + 3U] +=
        bias_process_variance;
    }

/*
 * Floating-point matrix multiplication can introduce very small asymmetric
 * roundoff. Explicitly restore covariance symmetry.
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = row + 1U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        symmetric_value =
            0.5f *
            (
            predicted_covariance[row][column] +
            predicted_covariance[column][row]
            );

        predicted_covariance[row][column] = symmetric_value;
        predicted_covariance[column][row] = symmetric_value;
        }
    }

if ( !mekf_covariance_is_finite(predicted_covariance) )
    {
    return false;
    }

/*
 * Commit the nominal attitude and covariance only after both predictions have
 * completed successfully.
 */
filter->attitude = predicted_attitude;

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        filter->covariance[row][column] =
            predicted_covariance[row][column];
        }
    }

return true;

} /* mekf_predict */

bool mekf_update_accelerometer
    (
    MEKF_FILTER *filter,
    VECTOR_3F acceleration_body_m_s2
    )
{
unsigned int row;
unsigned int column;
unsigned int inner;
unsigned int measurement;

VECTOR_3F measured_direction;
VECTOR_3F predicted_direction;
VECTOR_3F corrected_bias;

QUAT gravity_world = { 0.0f, 0.0f, 0.0f, 1.0f };
QUAT gravity_body;
QUAT correction_quaternion;
QUAT corrected_attitude;

float measurement_jacobian[3][MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

float covariance_measurement_cross[MEKF_ERROR_STATE_DIM][3];
float innovation_covariance[3][3];
float inverse_innovation_covariance[3][3];
float kalman_gain[MEKF_ERROR_STATE_DIM][3];

float identity_minus_gain_jacobian
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

float intermediate_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float joseph_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float reset_jacobian
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM] =
    {
    { 0.0f }
    };

float reset_intermediate_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float corrected_covariance
    [MEKF_ERROR_STATE_DIM]
    [MEKF_ERROR_STATE_DIM];

float residual[3];
float error_state[MEKF_ERROR_STATE_DIM] = { 0.0f };

float acceleration_magnitude;
float predicted_direction_magnitude;
float measurement_variance;
float normalized_innovation_squared;
float correction_magnitude;
float half_correction;
float quaternion_vector_scale;
float matrix_sum;
float symmetric_value;

if ( filter == NULL )
    {
    return false;
    }

if ( !mekf_quat_is_finite(filter->attitude) ||
     !mekf_vector_is_finite(filter->gyro_bias_rad_s) ||
     !mekf_vector_is_finite(acceleration_body_m_s2) )
    {
    return false;
    }

if ( !mekf_config_is_valid(&filter->config) ||
     !mekf_covariance_is_finite(filter->covariance) )
    {
    return false;
    }

/*
 * The accelerometer can be treated as a gravity reference only when its
 * magnitude is sufficiently close to the expected gravity magnitude.
 */
acceleration_magnitude = sqrtf
    (
    acceleration_body_m_s2.x * acceleration_body_m_s2.x +
    acceleration_body_m_s2.y * acceleration_body_m_s2.y +
    acceleration_body_m_s2.z * acceleration_body_m_s2.z
    );

if ( !isfinite(acceleration_magnitude) ||
     acceleration_magnitude <= 0.0f )
    {
    return false;
    }

if ( fabsf
        (
        acceleration_magnitude -
        filter->config.gravity_magnitude_m_s2
        ) >
     filter->config.accelerometer_magnitude_tolerance_m_s2 )
    {
    return false;
    }

measured_direction.x =
    acceleration_body_m_s2.x / acceleration_magnitude;

measured_direction.y =
    acceleration_body_m_s2.y / acceleration_magnitude;

measured_direction.z =
    acceleration_body_m_s2.z / acceleration_magnitude;

/*
 * The stored attitude maps body coordinates into world coordinates. Rotate the
 * world-frame unit gravity vector into the body frame to predict what the
 * accelerometer direction should be.
 */
gravity_body = quat_rotate_world_to_body
    (
    filter->attitude,
    gravity_world
    );

predicted_direction_magnitude = sqrtf
    (
    gravity_body.x * gravity_body.x +
    gravity_body.y * gravity_body.y +
    gravity_body.z * gravity_body.z
    );

if ( !isfinite(predicted_direction_magnitude) ||
     predicted_direction_magnitude <= 0.0f )
    {
    return false;
    }

predicted_direction.x =
    gravity_body.x / predicted_direction_magnitude;

predicted_direction.y =
    gravity_body.y / predicted_direction_magnitude;

predicted_direction.z =
    gravity_body.z / predicted_direction_magnitude;

/*
 * Measurement residual:
 *
 *     residual =
 *         measured_gravity_direction -
 *         predicted_gravity_direction
 */
residual[0] = measured_direction.x - predicted_direction.x;
residual[1] = measured_direction.y - predicted_direction.y;
residual[2] = measured_direction.z - predicted_direction.z;

/*
 * For a right-multiplicative local attitude error, the linearized gravity
 * measurement Jacobian is:
 *
 *     H = [ skew(predicted_gravity_direction)   0 ]
 */
measurement_jacobian[0][MEKF_ATTITUDE_ERROR_Y] =
    -predicted_direction.z;

measurement_jacobian[0][MEKF_ATTITUDE_ERROR_Z] =
    predicted_direction.y;

measurement_jacobian[1][MEKF_ATTITUDE_ERROR_X] =
    predicted_direction.z;

measurement_jacobian[1][MEKF_ATTITUDE_ERROR_Z] =
    -predicted_direction.x;

measurement_jacobian[2][MEKF_ATTITUDE_ERROR_X] =
    -predicted_direction.y;

measurement_jacobian[2][MEKF_ATTITUDE_ERROR_Y] =
    predicted_direction.x;

/*
 * Compute the state-to-measurement cross covariance:
 *
 *     PHT = P * transpose(H)
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( measurement = 0U; measurement < 3U; measurement++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U; inner < MEKF_ERROR_STATE_DIM; inner++ )
            {
            matrix_sum +=
                filter->covariance[row][inner] *
                measurement_jacobian[measurement][inner];
            }

        covariance_measurement_cross[row][measurement] =
            matrix_sum;
        }
    }

/*
 * Innovation covariance:
 *
 *     S = H * P * transpose(H) + R
 */
measurement_variance =
    filter->config.accelerometer_direction_std *
    filter->config.accelerometer_direction_std;

for ( row = 0U; row < 3U; row++ )
    {
    for ( column = 0U; column < 3U; column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U; inner < MEKF_ERROR_STATE_DIM; inner++ )
            {
            matrix_sum +=
                measurement_jacobian[row][inner] *
                covariance_measurement_cross[inner][column];
            }

        innovation_covariance[row][column] = matrix_sum;

        if ( row == column )
            {
            innovation_covariance[row][column] +=
                measurement_variance;
            }
        }
    }

if ( !mekf_matrix_3x3_inverse
        (
        innovation_covariance,
        inverse_innovation_covariance
        ) )
    {
    return false;
    }

/*
 * Normalized innovation squared:
 *
 *     NIS = transpose(residual) * inverse(S) * residual
 *
 * Reject a direction measurement that is inconsistent with the estimator's
 * predicted uncertainty. This also protects the small-error MEKF linearization
 * from very large attitude disagreements.
 */
normalized_innovation_squared = 0.0f;

for ( row = 0U; row < 3U; row++ )
    {
    for ( column = 0U; column < 3U; column++ )
        {
        normalized_innovation_squared +=
            residual[row] *
            inverse_innovation_covariance[row][column] *
            residual[column];
        }
    }

if ( !isfinite(normalized_innovation_squared) ||
     normalized_innovation_squared >
        filter->config.accelerometer_innovation_gate )
    {
    return false;
    }

/*
 * Kalman gain:
 *
 *     K = P * transpose(H) * inverse(S)
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < 3U; column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U; inner < 3U; inner++ )
            {
            matrix_sum +=
                covariance_measurement_cross[row][inner] *
                inverse_innovation_covariance[inner][column];
            }

        kalman_gain[row][column] = matrix_sum;
        }
    }

/*
 * Estimate the six-state correction:
 *
 *     delta_x = K * residual
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( measurement = 0U; measurement < 3U; measurement++ )
        {
        error_state[row] +=
            kalman_gain[row][measurement] *
            residual[measurement];
        }

    if ( !isfinite(error_state[row]) )
        {
        return false;
        }
    }

/*
 * Convert the local attitude-error correction into a quaternion and inject it
 * through right multiplication.
 */
correction_magnitude = sqrtf
    (
    error_state[MEKF_ATTITUDE_ERROR_X] *
        error_state[MEKF_ATTITUDE_ERROR_X] +
    error_state[MEKF_ATTITUDE_ERROR_Y] *
        error_state[MEKF_ATTITUDE_ERROR_Y] +
    error_state[MEKF_ATTITUDE_ERROR_Z] *
        error_state[MEKF_ATTITUDE_ERROR_Z]
    );

if ( !isfinite(correction_magnitude) )
    {
    return false;
    }

if ( correction_magnitude <= MEKF_SMALL_ANGLE_RAD )
    {
    correction_quaternion.w = 1.0f;
    quaternion_vector_scale = 0.5f;
    }
else
    {
    half_correction = 0.5f * correction_magnitude;

    correction_quaternion.w = cosf(half_correction);

    quaternion_vector_scale =
        sinf(half_correction) /
        correction_magnitude;
    }

correction_quaternion.x =
    error_state[MEKF_ATTITUDE_ERROR_X] *
    quaternion_vector_scale;

correction_quaternion.y =
    error_state[MEKF_ATTITUDE_ERROR_Y] *
    quaternion_vector_scale;

correction_quaternion.z =
    error_state[MEKF_ATTITUDE_ERROR_Z] *
    quaternion_vector_scale;

corrected_attitude = quat_mult
    (
    filter->attitude,
    correction_quaternion
    );

corrected_attitude = quat_normalize(corrected_attitude);

corrected_bias.x =
    filter->gyro_bias_rad_s.x +
    error_state[MEKF_GYRO_BIAS_ERROR_X];

corrected_bias.y =
    filter->gyro_bias_rad_s.y +
    error_state[MEKF_GYRO_BIAS_ERROR_Y];

corrected_bias.z =
    filter->gyro_bias_rad_s.z +
    error_state[MEKF_GYRO_BIAS_ERROR_Z];

if ( !mekf_quat_is_finite(corrected_attitude) ||
     !mekf_vector_is_finite(corrected_bias) )
    {
    return false;
    }

/*
 * Use the Joseph covariance update:
 *
 *     A = I - K * H
 *
 *     P_corrected =
 *         A * P * transpose(A) +
 *         K * R * transpose(K)
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < MEKF_ERROR_STATE_DIM; column++ )
        {
        if ( row == column )
            {
            identity_minus_gain_jacobian[row][column] = 1.0f;
            }

        for ( measurement = 0U; measurement < 3U; measurement++ )
            {
            identity_minus_gain_jacobian[row][column] -=
                kalman_gain[row][measurement] *
                measurement_jacobian[measurement][column];
            }
        }
    }

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U;
              inner < MEKF_ERROR_STATE_DIM;
              inner++ )
            {
            matrix_sum +=
                identity_minus_gain_jacobian[row][inner] *
                filter->covariance[inner][column];
            }

        intermediate_covariance[row][column] = matrix_sum;
        }
    }

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U;
              inner < MEKF_ERROR_STATE_DIM;
              inner++ )
            {
            matrix_sum +=
                intermediate_covariance[row][inner] *
                identity_minus_gain_jacobian[column][inner];
            }

        joseph_covariance[row][column] = matrix_sum;

        for ( measurement = 0U; measurement < 3U; measurement++ )
            {
            joseph_covariance[row][column] +=
                measurement_variance *
                kalman_gain[row][measurement] *
                kalman_gain[column][measurement];
            }
        }
    }

/*
 * Reset the attitude-error state after injecting its correction into the
 * nominal quaternion:
 *
 *     G_theta = I - 0.5 * skew(delta_theta)
 *
 *     P_reset = G * P_corrected * transpose(G)
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    reset_jacobian[row][row] = 1.0f;
    }

reset_jacobian
    [MEKF_ATTITUDE_ERROR_X]
    [MEKF_ATTITUDE_ERROR_Y] =
        0.5f * error_state[MEKF_ATTITUDE_ERROR_Z];

reset_jacobian
    [MEKF_ATTITUDE_ERROR_X]
    [MEKF_ATTITUDE_ERROR_Z] =
        -0.5f * error_state[MEKF_ATTITUDE_ERROR_Y];

reset_jacobian
    [MEKF_ATTITUDE_ERROR_Y]
    [MEKF_ATTITUDE_ERROR_X] =
        -0.5f * error_state[MEKF_ATTITUDE_ERROR_Z];

reset_jacobian
    [MEKF_ATTITUDE_ERROR_Y]
    [MEKF_ATTITUDE_ERROR_Z] =
        0.5f * error_state[MEKF_ATTITUDE_ERROR_X];

reset_jacobian
    [MEKF_ATTITUDE_ERROR_Z]
    [MEKF_ATTITUDE_ERROR_X] =
        0.5f * error_state[MEKF_ATTITUDE_ERROR_Y];

reset_jacobian
    [MEKF_ATTITUDE_ERROR_Z]
    [MEKF_ATTITUDE_ERROR_Y] =
        -0.5f * error_state[MEKF_ATTITUDE_ERROR_X];

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U;
              inner < MEKF_ERROR_STATE_DIM;
              inner++ )
            {
            matrix_sum +=
                reset_jacobian[row][inner] *
                joseph_covariance[inner][column];
            }

        reset_intermediate_covariance[row][column] =
            matrix_sum;
        }
    }

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        matrix_sum = 0.0f;

        for ( inner = 0U;
              inner < MEKF_ERROR_STATE_DIM;
              inner++ )
            {
            matrix_sum +=
                reset_intermediate_covariance[row][inner] *
                reset_jacobian[column][inner];
            }

        corrected_covariance[row][column] = matrix_sum;
        }
    }

/*
 * Remove small floating-point asymmetry and verify the complete result before
 * committing any part of the correction.
 */
for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = row + 1U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        symmetric_value =
            0.5f *
            (
            corrected_covariance[row][column] +
            corrected_covariance[column][row]
            );

        corrected_covariance[row][column] = symmetric_value;
        corrected_covariance[column][row] = symmetric_value;
        }
    }

if ( !mekf_covariance_is_finite(corrected_covariance) )
    {
    return false;
    }

filter->attitude = corrected_attitude;
filter->gyro_bias_rad_s = corrected_bias;

for ( row = 0U; row < MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < MEKF_ERROR_STATE_DIM;
          column++ )
        {
        filter->covariance[row][column] =
            corrected_covariance[row][column];
        }
    }

return true;

} /* mekf_update_accelerometer */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/