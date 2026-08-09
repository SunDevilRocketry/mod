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
 * @brief Mahony attitude filter status codes.
 */
typedef enum
    {
    MAHONY_OK = 0,              //success (SDR convention)
    MAHONY_NULL_POINTER,        //filter == NULL
    MAHONY_INVALID_QUATERNION,  //attitude contains NaN/Inf
    MAHONY_NONFINITE_GAIN,      //Kp or Ki is NaN/Inf
    MAHONY_NEGATIVE_GAIN,       //Kp or Ki is below zero
    MAHONY_INVALID_GYRO,        //gyro contains NaN/Inf
    MAHONY_INVALID_DELTA_TIME   //dt is NaN/Inf, zero, or negative
    // no MAHONY_INVALID_ACCEL because Invalid acceleration is..
    // deliberately treated as "don't use accel correction; continue gyro-only,"..
    // not as a failed Mahony update.
    } MAHONY_STATUS;

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
 * @return MAHONY_OK when initialization succeeds; otherwise a Mahony status
 *         code describing the failure.
 */
MAHONY_STATUS mahony_init
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
 * @return MAHONY_OK when initialization succeeds; otherwise a Mahony status
        code describing the failure.
 */
MAHONY_STATUS mahony_update_gyro
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
    );

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/