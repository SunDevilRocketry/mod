/**
  ******************************************************************************
  * @file           : nav.h
  * @brief          : Navigation component public header.
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
                            ##### Nav Features #####
  ==============================================================================
  [..]
  (+) Platform-independent high power rocketrysensor data processing and 
      filtering library
      (+) Dependencies: debug_sdr, math_sdr.
  (+) Stateless (library state owned by integrating project)
  ******************************************************************************
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NAV_H
#define NAV_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/
#include <stdbool.h>

/* Project Includes ----------------------------------------------------------*/
#include "debug_sdr.h"
#include "math_sdr.h"

/* Public Constants ----------------------------------------------------------*/


/* Private Constants ---------------------------------------------------------*/
#define _PRIVATE_STATE_SIZE ( 132u )

/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief Navigation library status codes.
 */
typedef enum
    {
    NAV_OK,
    NAV_FAIL,
    NAV_BUSY,
    NAV_TIMEOUT,
    NAV_NULLPTR,
    NAV_INVALID_PARAMS,
    NAV_INVALID_CONFIG
    } NAV_STATUS;

/**
 * @brief Navigation library configuration parameters.
 * 
 * Currently, the top three parameters are required to be
 * TRUE. Eventually, we should support every permutation.
 */
typedef struct _NAV_CONFIG
    {
    float mahony_kp;
    float mahony_ki;
    bool accel_enabled;
    bool gyro_enabled;
    bool mag_enabled;
    bool baro_enabled;
    bool gps_enabled;
    } NAV_CONFIG;

/**
 * @brief Navigation library position type.
 */
typedef struct _NAV_POSN
    {
    float latitude;  /* u: degrees */
    float longitude; /* u: degrees */
    } NAV_POSN;

/**
 * @brief Navigation library sensor inputs.
 * 
 * @details If the device has multiple sensors of
 * a type, the project must fuse that data together
 * before passing to NAV.
 */
typedef struct _NAV_SENSORS
    {
    VECTOR_3F accel; /* u: m/s^2   */
    VECTOR_3F gyro;  /* u: deg/s   */
    VECTOR_3F mag;   /* u: uT (?)  */
    float baro_pres; /* u: Pa      */
    float baro_temp; /* u: C       */
    NAV_POSN gps_posn;
                     /* u: deg     */
    float gps_alt;   /* u: m       */
    float gps_brg;   /* u: deg     */
    } NAV_SENSORS;

/**
 * @brief Navigation library fused outputs.
 */
typedef struct _NAV_OUTPUTS
    {
    QUAT orientation;
    VECTOR_3F velocity;
    float velo_mag;
    NAV_POSN fused_posn;
    float ind_alt_agl; /* u: m   */
    float ind_alt_asl; /* u: m   */
    } NAV_OUTPUTS;

/**
 * @brief The public fields in the public nav state.
 */
typedef struct _NAV_PUBLIC_FIELDS
    {
    bool in_air;                    /* t: rocket is flying      */
    NAV_SENSORS nav_sensors;        /* Input sensor data        */
    NAV_OUTPUTS nav_outputs;        /* Output fused data        */
    } NAV_PUBLIC_FIELDS;
/**
 * @brief The public state of the navigation library.
 */
typedef struct _NAV_PUBLIC_STATE
    {
    NAV_PUBLIC_FIELDS pub;          /* Public data to NAV       */
    const uint8_t __opaque_private_contents 
        [ _PRIVATE_STATE_SIZE ];    /* Private data to NAV      */
    } NAV_PUBLIC_STATE;


/* Function Prototypes -------------------------------------------------------*/

/* nav.c */
/**
 * @brief Initializes the navigation library.
 * 
 * @param nav_state Public state of the nav library.
 * @param config Configures the nav library
 * @param calibration Calibrated sensor readings at launch site
 * @param current_time The time the readings were gathered.
 * 
 * @return The state of the nav library
 */
NAV_STATUS nav_init
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_CONFIG config,
    NAV_SENSORS calibration
    );

/**
 * @brief Updates the navigation library state.
 * 
 * @param nav_state Public state of the nav library.
 * @param current_time The time these readings were gathered.
 * 
 * @return The state of the nav library.
 */
NAV_STATUS nav_update
    (
    NAV_PUBLIC_STATE* nav_state,
    float current_time
    );

/* nav_utl.c */

/**
  * @brief Resets accumulated velocity values to prevent drift.
  * 
  * @param nav_state The nav library state
  * 
  * @return The status of this operation.
  */
NAV_STATUS nav_reset_velo
	(
	NAV_PUBLIC_STATE* nav_state
	);

/* Undefine Private Constants ------------------------------------------------*/
#undef _PRIVATE_STATE_SIZE

#ifdef __cplusplus
}
#endif

#endif /* NAV_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/