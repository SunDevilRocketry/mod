/*******************************************************************************
 *
 * FILE:
 *      mekf.h
 *
 * DESCRIPTION:
 *      Multiplicative Extended Kalman Filter attitude estimator interface.
 *
 ******************************************************************************/

#ifndef MEKF_H
#define MEKF_H

#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------------------------------------
 Standard Includes
 ------------------------------------------------------------------------------*/
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes
 ------------------------------------------------------------------------------*/
#include "math_sdr.h"

/*------------------------------------------------------------------------------
 Macros
 ------------------------------------------------------------------------------*/

/**
 * @brief Number of elements in the MEKF error state.
 */
#define MEKF_ERROR_STATE_DIM    6U

/*------------------------------------------------------------------------------
 Typedefs
 ------------------------------------------------------------------------------*/

/**
 * @brief Indices into the six-state MEKF error vector.
 *
 * The multiplicative error state is:
 *
 *     delta_x =
 *         [
 *         delta_theta_x,
 *         delta_theta_y,
 *         delta_theta_z,
 *         delta_bias_x,
 *         delta_bias_y,
 *         delta_bias_z
 *         ]
 *
 * The attitude error is a local body-frame rotation in radians. The gyro-bias
 * error is expressed in body-frame radians per second.
 */
typedef enum _MEKF_ERROR_STATE_INDEX
    {
    MEKF_ATTITUDE_ERROR_X = 0,
    MEKF_ATTITUDE_ERROR_Y,
    MEKF_ATTITUDE_ERROR_Z,
    MEKF_GYRO_BIAS_ERROR_X,
    MEKF_GYRO_BIAS_ERROR_Y,
    MEKF_GYRO_BIAS_ERROR_Z
    } MEKF_ERROR_STATE_INDEX;

/**
 * @brief MEKF initialization, prediction, and measurement-update configuration.
 */
typedef struct _MEKF_CONFIG
    {
    /**
    * Initial one-sigma uncertainty of the local attitude-error estimate, in radians.
    *
    * Each component specifies the standard deviation of the unknown small-angle
    * rotation between the nominal attitude estimate and the true attitude about
    * the body X, Y, and Z axes.
    *
    * These values describe uncertainty in the attitude-error estimate; they are
    * not estimates of the attitude error itself. The expected initial attitude
    * error is zero. mekf_init() squares these standard deviations to initialize
    * the corresponding attitude-error covariance diagonal entries.
    *
    * A larger value tells the filter that the initial attitude estimate is less
    * trustworthy, while a smaller value indicates greater confidence in it.
    */
    VECTOR_3F initial_attitude_std_rad;

    /**
    * Initial one-sigma uncertainty of the gyro-bias estimate, in radians per second.
    *
    * Each component specifies the standard deviation of the unknown difference
    * between the estimated gyro bias and the true gyro bias about the body X, Y, and Z axes.
    *
    * These values describe uncertainty in the bias estimate; they are not the
    * estimated gyro-bias values themselves. The expected initial gyro-bias error
    * is zero. mekf_init() squares these standard deviations to initialize the
    * corresponding gyro-bias covariance diagonal entries.
    *
    * A larger value tells the filter that the initial gyro-bias estimate is less
    * trustworthy, while a smaller value indicates greater confidence in it.
     */
    VECTOR_3F initial_gyro_bias_std_rad_s;

    /**
     * Continuous gyroscope white-noise density in radians per second per
     * square-root hertz. This represents short term random noise in the gyro measurement.
     * A noisier gyro will produce a larger attitude covariance growth during prediction.
     */
    float gyro_noise_density_rad_s_sqrt_hz;

    /**
     * Continuous gyro-bias random-walk density in radians per second squared
     * per square-root hertz.
     * This represents how quickly the gyro bias is expected to drift over time.
     * A larger random-walk density will produce a larger gyro-bias covariance growth during prediction.
     * Bias can change due to temperature, sensor warmup, mechanical stress, etc..
     */
    float gyro_bias_random_walk_rad_s2_sqrt_hz;

        /**
     * One-sigma uncertainty of each component of the normalized accelerometer
     * direction measurement.
     *
     * The accelerometer update compares normalized measured and predicted gravity
     * directions, so this value is dimensionless. A larger value makes the filter
     * trust accelerometer direction less strongly.
     */
    float accelerometer_direction_std;

    /**
     * Expected local gravitational acceleration magnitude in meters per second
     * squared.
     *
     * This value is used to determine whether the measured acceleration magnitude
     * is sufficiently close to gravity for attitude correction.
     */
    float gravity_magnitude_m_s2;

    /**
     * Maximum permitted absolute difference between measured acceleration
     * magnitude and gravity magnitude, in meters per second squared.
     *
     * Measurements outside this range are rejected because vehicle acceleration,
     * vibration, or free fall makes the accelerometer unreliable as a gravity
     * reference.
     */
    float accelerometer_magnitude_tolerance_m_s2;

    /**
     * Maximum permitted normalized innovation squared for an accelerometer
     * measurement.
     *
     * This rejects gravity-direction residuals that are inconsistent with the
     * predicted covariance and configured accelerometer uncertainty. A value of
     * approximately 11.345 corresponds to a 99 percent chi-square threshold for
     * a three-component residual.
     */
    float accelerometer_innovation_gate;

    /**
     * Maximum valid gyro prediction timestep in seconds.
     * This represents the largest allowable time interval for one prediction.
     * This protects the estimator from propagating across unreasonable timing gaps
     * caused by missed data, timestamp corruption, or task delays.
     */
    float maximum_delta_time_s;

    } MEKF_CONFIG;

/**
 * @brief Nominal state, covariance, and configuration for a six-state MEKF.
 *
 * The nominal attitude is a body-to-world quaternion:
 *
 *     vector_world =
 *         attitude * vector_body * conjugate(attitude)
 *
 * A right-multiplicative local attitude error is used:
 *
 *     attitude_true =
 *         attitude_nominal * delta_attitude
 *
 * The covariance represents uncertainty in the local six-state error vector,
 * not uncertainty in the four quaternion components:
 *
 *     P = E[delta_x * transpose(delta_x)]
 */
typedef struct _MEKF_FILTER
    {
    /**
     * Nominal body-to-world attitude quaternion.
     */
    QUAT attitude;

    /**
     * Nominal body-frame gyro-bias estimate in radians per second.
     */
    VECTOR_3F gyro_bias_rad_s;

    /**
     * Six-by-six error-state covariance matrix.
     */
    float covariance[MEKF_ERROR_STATE_DIM][MEKF_ERROR_STATE_DIM];

    /**
     * Initial uncertainty, process-noise, and timestep configuration.
     */
    MEKF_CONFIG config;

    } MEKF_FILTER;

/*------------------------------------------------------------------------------
 Function Prototypes
 ------------------------------------------------------------------------------*/

/**
 * @brief Initializes a six-state attitude and gyro-bias MEKF.
 *
 * The initial attitude is normalized. The initial covariance is diagonal and
 * is constructed from the squared per-axis standard deviations in the
 * configuration.
 *
 * @param filter Filter instance to initialize.
 * @param initial_attitude Initial body-to-world attitude quaternion.
 * @param initial_gyro_bias_rad_s Initial body-frame gyro-bias estimate in
 *        radians per second.
 * @param config Initial uncertainty, process-noise, and timestep configuration.
 *
 * @return true when initialization succeeds; otherwise false.
 */
bool mekf_init
    (
    MEKF_FILTER *filter,
    QUAT initial_attitude,
    VECTOR_3F initial_gyro_bias_rad_s,
    const MEKF_CONFIG *config
    );

/**
 * @brief Predicts nominal attitude and covariance using a gyro measurement.
 *
 * The gyro measurement must be expressed in body-frame radians per second.
 * The stored gyro-bias estimate is subtracted before attitude propagation.
 *
 * The attitude quaternion is propagated using right multiplication:
 *
 *     attitude_new =
 *         normalize(attitude_old * delta_attitude)
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame gyroscope measurement in radians per
 *        second.
 * @param delta_time_s Elapsed time in seconds.
 *
 * @return true when prediction succeeds; otherwise false.
 */
bool mekf_predict
    (
    MEKF_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    );

/**
 * @brief Corrects attitude and gyro bias using a body-frame accelerometer
 * measurement.
 *
 * The acceleration measurement is normalized and compared with the predicted
 * body-frame gravity direction. Correction is applied only when its magnitude
 * is sufficiently close to the configured gravity magnitude.
 *
 * Accelerometer correction constrains tilt relative to gravity but cannot
 * independently observe rotation about the gravity vector.
 *
 * @param filter Initialized filter instance.
 * @param acceleration_body_m_s2 Body-frame accelerometer measurement in meters
 *        per second squared.
 *
 * @return true when the measurement is accepted and the update succeeds;
 *         otherwise false.
 */
bool mekf_update_accelerometer
    (
    MEKF_FILTER *filter,
    VECTOR_3F acceleration_body_m_s2
    );

#ifdef __cplusplus
}
#endif

#endif /* MEKF_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/