/**
  ******************************************************************************
  * @file           : nav_prv.h
  * @brief          : Navigation component private header.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
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
              ##### DO NOT INCLUDE THIS HEADER OUTSIDE OF NAV #####
  ==============================================================================
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NAV_PRV_H
#define NAV_PRV_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>

/* Project Includes ----------------------------------------------------------*/
#include "nav.h"
#include "nav_contract.h"

/* Public Constants ----------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief State and gains for a Mahony attitude filter.
 */
typedef struct _NAV_MAHONY_FILTER
    {
    /** Body-to-world attitude quaternion. */
    QUAT attitude;

    /** Accumulated attitude error used for integral gyro-bias correction. */
    VECTOR_3F integral_error;

    float proportional_gain;
    float integral_gain;

    } NAV_MAHONY_FILTER;
_Static_assert( sizeof( NAV_MAHONY_FILTER ) == 9 * 4u );

/* Top-Level Defines */

/**
 * @brief The private state of the navigation library.
 */
typedef struct _NAV_PRIVATE_STATE
    {
    NAV_PUBLIC_FIELDS public_state;
    NAV_CONFIG saved_configs;       /* 16B: Internal configs    */
    NAV_MAHONY_FILTER mahony_state; /* 36B: Mahony filter state */
    NAV_SENSORS cal_data;           /* 60B: Calibration data    */
    VECTOR_3F velocity;             /* 12B: Velocity data       */
    float last_update_time;         /* 4B: Float                */
    float curr_update_time;         /* 4B: Float                */
    } NAV_PRIVATE_STATE;
_Static_assert( sizeof( NAV_PRIVATE_STATE ) == sizeof( NAV_PUBLIC_STATE ),
                "There is a mismatch between the public and private NAV state types." );

/* Internal Macros -----------------------------------------------------------*/
#define _nav_debug_error( msg ) debug_assert( false, "[NAV] at " __FILE__":"__LINE__" - " msg )
#define nav_state_to_prv( state ) ( (NAV_PRIVATE_STATE*)state )
#define nav_nullchk( state ) do { \
        if ( (state) == NULL ) \
            { \
            _nav_debug_error( "Null NAV state" );\
            return NAV_NULLPTR; \
            } \
        } while(0)

/* Function Prototypes -------------------------------------------------------*/

/* nav_isa.c */

/**
  * @brief Calculates altitude (relative to sea level).
  * @param baro_pres Barometric pressure in Pa.
  * @param baro_temp Barometric temperature in C.
  * @details
  * \latexonly
  * \begin{equation}
  *   h = \frac{T_{0}}{\lambda}[1-\frac{P}{P_{0}}^{\frac{R\lambda}{g_{0}}}]
  * \end{equation}
  * \endlatexonly
  * 
  * @return Calculated ISA altitude above sea level.
  */
float _nav_isa_get_altitude_qnh
    (
    float baro_pres, 
    float baro_temp
    );

/**
  * @brief Calculates altitude (relative to ground).
  * @param curr_pres Barometric pressure in Pa.
  * @param curr_temp Barometric temperature in C.
  * @param reference_pres Calibrated reference barometric pressure in Pa.
  * @param reference_temp Calibrated reference barometric temperature in C.
  * 
  * @return Calculated ISA altitude above ground reference.
  */
float _nav_isa_get_altitude_qfe
    (
    float curr_pres, 
    float curr_temp,
    float reference_pres,
    float reference_temp
    );

/**
  * @brief Calculate the ISA temperature at the given barometric pressure.
  * 
  * @param baro_pres The barometric pressure
  * @return The ISA temperature
  */
float _nav_isa_get_temp
    (
    float baro_pres
    );

/* nav_mahony.c */

NAV_STATUS _nav_mahony_init
    (
    NAV_PRIVATE_STATE* state
    );

/**
 * @brief Propagates attitude using body-frame gyroscope measurements.
 * @note Assumes state reserved by the caller.
 *
 * @details
 * Verify the filter, gyro, and timestep are valid.
 * Convert the body-frame angular velocity into a pure quaternion.
 * Use the quaternion differential equation to calculate how quickly the
 * body-to-world attitude is changing. Multiply that derivative by the
 * timestep, add it to the current attitude, and normalize the result.
 *
 * @param filter Initialized filter instance.
 * @param gyro_body_rad_s Body-frame angular velocity in radians per second.
 * @param delta_time_s Elapsed time in seconds.
 *
 * @return NAV_OK when succeeds; otherwise a Mahony status
           code describing the failure.
 */
NAV_STATUS _nav_mahony_update_gyro
    (
    NAV_MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    float delta_time_s
    );

/**
 * @brief Updates attitude using gyroscope propagation and accelerometer
 *        proportional feedback.
 * @note Assumes state reserved by the caller.
 * 
 * @details
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
 * @return NAV_OK when the attitude was updated; otherwise a Mahony status
        code describing the failure.
 */
NAV_STATUS _nav_mahony_update_imu
    (
    NAV_MAHONY_FILTER *filter,
    VECTOR_3F gyro_body_rad_s,
    VECTOR_3F accel_body,
    float delta_time_s,
    bool use_accel
    );

/* nav_utl.c */

/**
  * @brief Resets accumulated velocity values to prevent drift.
  * 
  * @param prv_state The nav library private state
  * 
  * @return The status of this operation.
  */
NAV_STATUS _nav_reset_velo
	(
	NAV_PRIVATE_STATE* prv_state
	);

QUAT _nav_quat_grav_attitude
	(
    QUAT attitude,
    VECTOR_3F accel
	);

float _nav_quat_to_yaw
	(
	QUAT q
	);

/**
  * @brief Calculates velocity from converted accelerometer measurements.
  * @note Assumes state reserved by the caller.
  * @note Requires attitude estimate to already be updated.
  * 
  * @param prv_state The private nav library state.
  * @param tdelta The delta between measurements
  * 
  * @return The status of the operation
  */
NAV_STATUS _nav_imu_velo
	(
	NAV_PRIVATE_STATE* prv_state,
    float tdelta
	);

#ifdef __cplusplus
}
#endif

#endif /* NAV_PRV_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/