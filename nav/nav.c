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

    /* Initialize State --------------------------------------------------*/

    /* Clear struct */
    memset( prv_state, 0, sizeof( NAV_PRIVATE_STATE ) );

    /* Show inputs as invalid */
    memset( &(nav_state->pub.nav_sensors), 0xFF, sizeof( NAV_OUTPUTS ) );

    /* Show outputs as invalid */
    memset( &(nav_state->pub.nav_outputs), 0xFF, sizeof( NAV_OUTPUTS ) );

    /* Copy configuration data */
    prv_state->saved_configs = config;

    /* Copy calibration data */
    prv_state->cal_data = calibration;

    /* Initialize timing */
    prv_state->last_update_time = INFINITY;

    /* Initialize subsystems */
    status |= _nav_mahony_init( prv_state ); /* start mahony */
    status |= _nav_reset_velo( prv_state );  /* reset velocity integration */

    nav_release_state( nav_state );
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
 * @param nav_state Public state of the nav library.
 * @param current_time The time these readings were gathered.
 * 
 * @return The state of the nav library.
 */
NAV_STATUS nav_update
    (
    NAV_PUBLIC_STATE* nav_state,
    float current_time
    )
{
nav_nullchk( nav_state );

/* Local Variables -------------------------------------------------------*/
NAV_STATUS status = NAV_OK;
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_OUTPUTS* o = &(nav_state->pub.nav_outputs);
NAV_SENSORS* s = &(nav_state->pub.nav_sensors);

if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    float tdelta   = prv_state->curr_update_time - prv_state->last_update_time;

    /* Invalidate Outputs ----------------------------------------------------*/

    /* Calculate barometer outputs */
    o->ind_alt_asl  = _nav_isa_get_altitude_qnh( s->baro_pres, 
                                                _nav_isa_get_temp( s->baro_pres ) );
    o->ind_alt_agl  = o->ind_alt_asl;
    o->ind_alt_agl -= _nav_isa_get_altitude_qnh( prv_state->cal_data.baro_pres, 
                                                _nav_isa_get_temp( s->baro_pres ) );

    /* Update orientation estimate */
    /* TODO: MAGNETOMETER FUSION */
    status |= _nav_mahony_update_imu( &(prv_state->mahony_state), 
                                      s->gyro, 
                                      s->accel, 
                                      tdelta, 
                                      nav_state->pub.in_air );

    /* Update velocity estimate */
    status |= _nav_imu_velo( prv_state, tdelta );

    /* Update GPS fusion */
    if( prv_state->saved_configs.gps_enabled )
        {
        /* TODO: Fuse in other data */
        o->fused_posn = s->gps_posn;
        }
    else
        {
        o->fused_posn.latitude = INFINITY;
        o->fused_posn.longitude = INFINITY;
        }
    }
else
    {
    
    }

return( status != NAV_OK );

} /* nav_update */

/**
  * END OF FILE
  */