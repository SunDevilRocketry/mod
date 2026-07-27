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
 * @brief Initial uncertainty and gyro prediction configuration.
 */
typedef struct _MEKF_CONFIG
    {
    /**
     * Initial one-sigma local attitude uncertainty in radians.
     */
    VECTOR_3F initial_attitude_std_rad;

    /**
     * Initial one-sigma gyro-bias uncertainty in radians per second.
     */
    VECTOR_3F initial_gyro_bias_std_rad_s;

    /**
     * Continuous gyroscope white-noise density in radians per second per
     * square-root hertz.
     */
    float gyro_noise_density_rad_s_sqrt_hz;

    /**
     * Continuous gyro-bias random-walk density in radians per second squared
     * per square-root hertz.
     */
    float gyro_bias_random_walk_rad_s2_sqrt_hz;

    /**
     * Maximum valid gyro prediction timestep in seconds.
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

#ifdef __cplusplus
}
#endif

#endif /* MEKF_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/