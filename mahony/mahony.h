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
                       ##### Mahony filter features #####
  ==============================================================================
  [..]
  (+) Quaternion-based gyroscope attitude propogation
  (+) Accelerometer proportional and integral feedback filtering
  ******************************************************************************
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/
#include <stdbool.h>

/* Project Includes ----------------------------------------------------------*/
#include "math_sdr.h"

/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief Mahony attitude filter status codes.
 */
typedef enum
    {
    MAHONY_OK = 0,
    MAHONY_NULL_POINTER,        
    MAHONY_INVALID_QUATERNION,
    MAHONY_NONFINITE_GAIN,
    MAHONY_NEGATIVE_GAIN,
    MAHONY_INVALID_GYRO,
    MAHONY_INVALID_DELTA_TIME
    } MAHONY_STATUS;

/**
 * @brief State and gains for a Mahony attitude filter.
 */
typedef struct _MAHONY_FILTER
    {
    /** Body-to-world attitude quaternion. */
    QUAT attitude;

    /** Accumulated attitude error used for integral gyro-bias correction. */
    VECTOR_3F integral_error;

    float proportional_gain;
    float integral_gain;

    } MAHONY_FILTER;

/* Function Prototypes -------------------------------------------------------*/

MAHONY_STATUS mahony_init
    (
    MAHONY_FILTER *filter,
    QUAT initial_attitude,
    float proportional_gain,
    float integral_gain
    );


MAHONY_STATUS mahony_update_gyro
    (
    MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    );

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