/**
  ******************************************************************************
  * @file   ahrs.h
  * @brief  Attitude and Heading Reference System 
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
                            ##### AHRS features #####
  ==============================================================================
  [..]
  (+) @todo
  ******************************************************************************
  @endverbatim
  @dir      ahrs
  @brief    Provides a uhhhhhhh ahrs
  @section
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/

/* Project Includes ----------------------------------------------------------*/
#include "math_sdr.h"

/* Private Constants ---------------------------------------------------------*/
#define _AHRS_PRIVATE_STATE_SIZE ( 224u )


/* Typedefs ------------------------------------------------------------------*/

typedef enum _AHRS_STATUS
    {
    AHRS_OK = 0,
    AHRS_FAIL,
    AHRS_NULLPTR,
    AHRS_INVALID_CONFIG
    } AHRS_STATUS;

/**
 * @brief AHRS config struct.
 * 
 * Description of each model parameter:
 * - @ref gyro_noise is the spread of measurements around the actual value
 * - @ref gyro_bas is the strength of the white noise random walk of the gyro 
 *        bias
 * - @ref mag_noise is the variance of the normalized magnetometer measurements
 * - @ref accel_noise is the variance of the normalized accelerometer 
 *        measurements
 * - @ref @todo
 */
typedef struct _AHRS_CONFIG
    {
    /* Tuning parameters */
    float gyro_noise;                   /* u: rad^2/s */
    float gyro_bias;                    /* u: rad^2/s^3 */
    VECTOR_3F mag_noise;                /* u: dimensionless */
    VECTOR_3F accel_noise;              /* u: dimensionless */

    /* Expected magnetic field vector when aligned to NED, q = (1, 0, 0, 0)  */
    VECTOR_3F global_mag_ref;           /* u: uT */

    /* Maximum angular rate to use the small angle approximation algorithms  */
    float small_angle_threshold;        /* u: rad/s*/
    } AHRS_CONFIG;

typedef struct _AHRS_PUBLIC_STATE
    {
    const uint8_t __opaque_private_contents[_AHRS_PRIVATE_STATE_SIZE]; 
    } AHRS_PUBLIC_STATE;

/* Function Prototypes -------------------------------------------------------*/

AHRS_STATUS ahrs_init
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    AHRS_CONFIG config,
    float time
    );

AHRS_STATUS ahrs_get_attitude
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    QUAT* attitude
    );

AHRS_STATUS ahrs_gyro_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F angular_rate,         /* u: deg/s */
    float current_time              /* u: s */
    );

/**
 * 
 * @warning Only use this when the measured acceleration is only from gravity. 
 * Do not use this during dynamic flight.
 */
AHRS_STATUS ahrs_accel_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F accel  /* u: m/s */
    );

AHRS_STATUS ahrs_mag_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F mag  /* u: uT */
    );

#undef _AHRS_PRIVATE_STATE_SIZE

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/