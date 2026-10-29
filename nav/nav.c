/**
  ******************************************************************************
  * @file           : nav.c
  * @brief          : Navigation & sensor fusion library's main module.
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
#include "nav.h"
#include "nav_prv.h"

/*------------------------------------------------------------------------------
 Constants                                                                   
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
 * @brief Initializes the navigation library.
 * 
 * @param[in,out] nav_state Public state of the nav library.
 * @param[in] config Configures the nav library
 * @param[in] calibration Calibrated sensor readings at launch site
 * @param time The time of initialization for use in deltas on update
 * 
 * @return The state of the nav library
 */
NAV_STATUS nav_init
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_CONFIG config,
    NAV_SENSORS calibration,
    float time
    )
{
nav_nullchk( nav_state );

/* Local Variables -------------------------------------------------------*/
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_STATUS status = NAV_OK;

if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    /* Check Config Validity ---------------------------------------------*/

    /* This configuration MUST be enabled for now */
    if( !( config.accel_enabled
        && config.gyro_enabled
        && config.baro_enabled ) )
        {
        _nav_debug_error( "Accelerometer, gyroscope, and barometer data are all required" );
        return NAV_INVALID_CONFIG;
        }
    
    /* check baro EMA. alpha of 1 == no filtering */
    if ( !( config.baro_ema_pres_alpha > 0.0f && config.baro_ema_pres_alpha <= 1.0f )
      || !( config.baro_ema_temp_alpha > 0.0f && config.baro_ema_temp_alpha <= 1.0f ) )
        {
        _nav_debug_error( "Baro EMA must be tuned with valid alpha" );
        }

    /* Initialize State --------------------------------------------------*/

    /* Clear struct */
    memset( prv_state, 0, sizeof( NAV_PRIVATE_STATE ) );

    /* Copy configuration data */
    prv_state->saved_configs = config;

    /* Copy calibration data */
    prv_state->cal_data = calibration;

    /* Initialize timing */
    prv_state->last_update_time = time;

    /* Initialize subsystems */
    status |= _nav_mahony_init( prv_state ); /* start mahony               */
    prv_state->baro_state = (NAV_BARO_FILTER_STATE){ INFINITY, INFINITY };
                                             /* invalidate EMA             */
    status |= _nav_reset_velo( prv_state );  /* reset velocity integration */
    status |= _nav_pof_init( prv_state );    /* start phase of flight      */

    status |= nav_release_state( nav_state );
    }
else
    {
    status = NAV_BUSY;
    }

return( status );

} /* nav_init */


/**
 * @brief Updates the navigation library state.
 * 
 * @param[in,out] nav_state Public state of the nav library.
 * @param[in] sensors The sensor readings to fuse
 * @param[out] outputs The outputs from fusion
 * @param current_time The time these readings were gathered.
 * 
 * @return The state of the nav library.
 */
NAV_STATUS nav_update
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs,
    float current_time
    )
{
nav_nullchk( nav_state );

/* Local Variables -------------------------------------------------------*/
NAV_STATUS status = NAV_OK;
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );

if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    float tdelta   = current_time - prv_state->last_update_time;
    prv_state->curr_update_time = current_time;

    /* Calculate barometer outputs */
    status |= _nav_baro_ema( prv_state, sensors );
    outputs->ind_alt_asl  = _nav_isa_get_altitude_qnh( sensors->baro_pres, 
                                                _nav_isa_get_temp( sensors->baro_pres ) );
    outputs->ind_alt_agl  = outputs->ind_alt_asl;
    outputs->ind_alt_agl -= _nav_isa_get_altitude_qnh( prv_state->cal_data.baro_pres, 
                                                _nav_isa_get_temp( prv_state->cal_data.baro_pres ) );

    /* Update orientation estimate */
    /* TODO: MAGNETOMETER FUSION */
    status |= _nav_mahony_update_imu( &(prv_state->mahony_state), 
                                      sensors->gyro, 
                                      sensors->accel, 
                                      tdelta, 
                                      _nav_in_air( prv_state ) );
    outputs->orientation = prv_state->mahony_state.attitude;

    /* Update velocity estimate */
    status |= _nav_imu_velo( prv_state, sensors, outputs, tdelta );

    /* Update GPS fusion */
    if( prv_state->saved_configs.gps_enabled )
        {
        /* TODO: Fuse in other data */
        outputs->fused_posn = sensors->gps_posn;
        }
    else
        {
        outputs->fused_posn.latitude = INFINITY;
        outputs->fused_posn.longitude = INFINITY;
        }

    /* Update phase-of-flight with fused data */
    status |= _nav_pof_update( prv_state, sensors, outputs );
    
    prv_state->last_update_time = current_time;
    status |= nav_release_state( nav_state );

    if( status != NAV_OK )
        {
        status = NAV_FAIL;
        }
    }
else
    {
    status = NAV_BUSY;
    }

return( status );

} /* nav_update */

/**
  * END OF FILE
  */