/*******************************************************************************
 *
 * FILE:
 *      mahony.h
 *
 * DESCRIPTION:
 *      Mahony attitude filter interface.
 *
 ******************************************************************************/

#ifndef MAHONY_H
#define MAHONY_H

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
 Typedefs
 ------------------------------------------------------------------------------*/

/**
 * @brief Three-dimensional floating-point vector.
 */
typedef struct _VECTOR_3F
    {
    float x;
    float y;
    float z;
    } VECTOR_3F;

/**
 * @brief State and gains for a Mahony attitude filter.
 */
typedef struct _MAHONY_FILTER
    {
    /**
     * Body-to-world attitude quaternion.
     */
    QUAT attitude;

    /**
     * Accumulated attitude error used for integral gyro-bias correction.
     */
    VECTOR_3F integral_error;

    float proportional_gain;
    float integral_gain;

    } MAHONY_FILTER;

/*------------------------------------------------------------------------------
 Function Prototypes
 ------------------------------------------------------------------------------*/

/**
 * @brief Initializes a Mahony attitude filter.
 *
 * @param filter Filter instance to initialize.
 * @param initial_attitude Initial body-to-world attitude quaternion.
 * @param proportional_gain Proportional correction gain.
 * @param integral_gain Integral correction gain.
 *
 * @return true when initialization succeeds; otherwise false.
 */
bool mahony_init
    (
    MAHONY_FILTER *filter,
    QUAT initial_attitude,
    float proportional_gain,
    float integral_gain
    );

/**
 * @brief Propagates attitude using body-frame gyroscope measurements.
 *
 * The attitude quaternion represents the body-to-world rotation. The angular
 * velocity vector must be expressed in the body frame in radians per second.
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame angular velocity in radians per second.
 * @param delta_time_s Elapsed time in seconds.
 *
 * @return true when the attitude was updated; otherwise false.
 */
bool mahony_update_gyro
    (
    MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    );

/**
 * @brief Updates attitude using gyroscope propagation and accelerometer
 *        proportional feedback.
 *
 * The gyroscope must be expressed in the body frame in radians per second.
 * The accelerometer must be expressed in the body frame. Its magnitude is
 * removed internally because the filter uses only its measured direction.
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame angular velocity in radians per second.
 * @param accel_body Body-frame accelerometer measurement.
 * @param delta_time_s Elapsed time in seconds.
 * @param use_accel Whether accelerometer feedback should be applied.
 *
 * @return true when the attitude was updated; otherwise false.
 */
bool mahony_update_imu
    (
    MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    VECTOR_3F accel_body,
    float delta_time_s,
    bool use_accel
    );

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/