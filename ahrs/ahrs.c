/**
  ******************************************************************************
  * @file   ahrs.c
  * @brief  todo.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "ahrs.h"
#include "ahrs_prv.h"


/*------------------------------------------------------------------------------
 Constants                                                                   
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

AHRS_STATUS ahrs_init
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    AHRS_CONFIG config,
    float time
    )
{
ahrs_nullchk(ahrs_state);

AHRS_STATUS status = AHRS_OK;
AHRS_PRIVATE_STATE* state = ahrs_state_to_prv( ahrs_state );

/* Type conversions */
float mag_noise[3] = { config.mag_noise.x, config.mag_noise.y, config.mag_noise.z };
float accel_noise[3] = { config.accel_noise.x, config.accel_noise.y, config.accel_noise.z };
float global_mag_ref[3] = { config.global_mag_ref.x, config.global_mag_ref.y, config.global_mag_ref.z };

/* Check config validity */
if ( config.gyro_bias < 0 || config.gyro_noise < 0 )
    {
    return AHRS_INVALID_CONFIG;
    }

float to_check[] = { 
                   config.gyro_bias, config.gyro_noise, config.small_angle_threshold,
                   accel_noise[0], accel_noise[1], accel_noise[2],
                   mag_noise[0], mag_noise[1], mag_noise[2],
                   };

for ( size_t i = 0; i < array_size(to_check); i++ )
    {
    if ( to_check[i] < 0 || !isfinite(to_check[i]) )
        {
        return AHRS_INVALID_CONFIG;
        }
    }

AHRS_IKF_AHRS_IKF
    (
    state,
    config.gyro_noise,
    config.gyro_bias,
    mag_noise,
    accel_noise,
    global_mag_ref,
    config.small_angle_threshold,
    time
    );

return status;
}


AHRS_STATUS ahrs_get_attitude
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    QUAT* attitude
    )
{
ahrs_nullchk(ahrs_state);
ahrs_nullchk(attitude);

AHRS_STATUS status = AHRS_OK;
AHRS_PRIVATE_STATE* state = ahrs_state_to_prv( ahrs_state );

quaternion q = AHRS_IKF_get_attitude(state);

/* MATLAB uses abcd instead of wxyz, but otherwise it's the same */
*attitude = (QUAT){ q.a, q.b, q.c, q.d }; 

return status;

}


AHRS_STATUS ahrs_gyro_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F angular_rate,
    float current_time
    )
{
ahrs_nullchk(ahrs_state);

AHRS_STATUS status = AHRS_OK;
AHRS_PRIVATE_STATE* state = ahrs_state_to_prv( ahrs_state );

float gyro[3] = { angular_rate.x, angular_rate.y, angular_rate.z };

AHRS_IKF_update_gyro(state, gyro, current_time);

return status;

}

/**
 * 
 * @warning Only use this when measured acceleration is only from gravity. Do
 * not use this during dynamic flight.
 */
AHRS_STATUS ahrs_accel_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F accel                 /* u: m/s */
    )
{
ahrs_nullchk(ahrs_state);

AHRS_STATUS status = AHRS_OK;
AHRS_PRIVATE_STATE* state = ahrs_state_to_prv( ahrs_state );

float accel_arr[3] = { accel.x, accel.y, accel.z };

AHRS_IKF_update_accel(state, accel_arr);

return status;

}

AHRS_STATUS ahrs_mag_update
    (
    AHRS_PUBLIC_STATE* ahrs_state,
    VECTOR_3F mag  /* u: uT */
    )
{
ahrs_nullchk(ahrs_state);

AHRS_STATUS status = AHRS_OK;
AHRS_PRIVATE_STATE* state = ahrs_state_to_prv( ahrs_state );

float mag_arr[3] = { mag.x, mag.y, mag.z };

AHRS_IKF_update_accel(state, mag_arr);

return status;

}


/**
  * END OF FILE
  */