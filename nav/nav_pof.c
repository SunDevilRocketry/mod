/**
  ******************************************************************************
  * @file           : nav_pof.c
  * @brief          : Navigation & sensor fusion library phase of flight.
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
#include "math_sdr.h"

#include "nav.h"
#include "nav_prv.h"

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Overwrite the navigation phase of flight.
  * 
  * @param[in,out] nav_state The nav library state
  * @param[in] pof The phase of flight
  * 
  * @return The status of this operation.
  */
NAV_STATUS nav_pof_set
	(
	NAV_PUBLIC_STATE* nav_state,
    NAV_POF* pof
	)
{
nav_nullchk( nav_state );

/* Local Variables */
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_STATUS status = NAV_OK;

/* Overwrite nav POF */
if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    prv_state->pof = *pof;

    if( nav_release_state( nav_state ) != NAV_OK )
        {
        status = NAV_FAIL;
        }
    }
else
    {
    status = NAV_BUSY;
    }

return( status );

} /* nav_set_pof */


/**
  * @brief Get the navigation phase of flight.
  * 
  * @param[in,out] nav_state The nav library state
  * @param[out] pof The nav phase of flight 
  *
  * @return The status of this operation.
  */
NAV_STATUS nav_pof_get
	(
	NAV_PUBLIC_STATE* nav_state,
    NAV_POF* pof
	)
{
nav_nullchk( nav_state );

/* Local Variables */
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_STATUS status = NAV_OK;

/* Get the nav POF */
if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    *pof = prv_state->pof;

    if( nav_release_state( nav_state ) != NAV_OK )
        {
        status = NAV_FAIL;
        }
    }
else
    {
    status = NAV_BUSY;
    }

return( status );

} /* nav_get_pof */

/*------------------------------------------------------------------------------
 Private Functions 
------------------------------------------------------------------------------*/

NAV_STATUS _nav_pof_init
    (
    NAV_PRIVATE_STATE* prv_state
    )
{
NAV_CONFIG* configs = &(prv_state->saved_configs);

/* Check the launch detect configuration */
if( ( !configs->launch_detect_accel_enabled && !configs->launch_detect_baro_enabled ) )
    {
    _nav_debug_error( "At least one launch detect option must be configured." );
    return NAV_INVALID_CONFIG;
    }
else if ( configs->launch_detect_accel_enabled
       && ( configs->launch_detect_accel_counts == 0
         || configs->launch_detect_accel_threshold <= 0.0f ) )
    {
    _nav_debug_error( "Accel-based launch detect misconfigured." );
    return NAV_INVALID_CONFIG;    
    }
else if ( configs->launch_detect_baro_enabled
       && ( configs->launch_detect_baro_counts == 0
         || configs->launch_detect_baro_threshold <= 0.0f ) )
    {
    _nav_debug_error( "Baro-based launch detect misconfigured." );
    return NAV_INVALID_CONFIG;    
    }

/* Check the coast detect configuration */
if( configs->coast_detect_counts == 0 || !isfinite( configs->coast_detect_threshold ) )
    {
    _nav_debug_error( "Coast detect must specify a number of samples below the threshold." );
    return NAV_INVALID_CONFIG;
    }

/* Check the apogee detect configuration */
if( configs->apogee_decreasing_counts == 0 )
    {
    _nav_debug_error( "Apogee detect must specify a number of consecutive decreasing samples." );
    return NAV_INVALID_CONFIG;
    }

/* Set up the POF state */
memset( &(prv_state->pof_state), 0, sizeof( NAV_POF_STATE ) );

return( NAV_OK );

} /* _nav_pof_init */


bool _nav_pof_detect_launch
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    )
{
float accX = sensors->accel.x;
float pressure = sensors->baro_pres;
float acc_scalar = fabsf(accX);

if ( prv_state->saved_configs.launch_detect_accel_enabled )
    {
    if ( acc_scalar > (float)prv_state->saved_configs.launch_detect_accel_threshold * 9.81f )
        {
        prv_state->pof_state.ld_accel_counts++;
        } 
    else 
        {
        prv_state->pof_state.ld_accel_counts = 0;
        }
    }
if ( prv_state->saved_configs.launch_detect_baro_enabled )
    {
    if ( pressure < ( prv_state->cal_data.baro_pres - prv_state->saved_configs.launch_detect_baro_threshold  ) )
        {
        prv_state->pof_state.ld_baro_counts++;
        } 
    else 
        {
        prv_state->pof_state.ld_baro_counts = 0;
        }
    }

// Trigger the flag once pass the threshold for number of times
if ( prv_state->pof_state.ld_accel_counts > prv_state->saved_configs.launch_detect_accel_counts 
    || prv_state->pof_state.ld_baro_counts > prv_state->saved_configs.launch_detect_baro_counts  )
    {
    return true;
    }
else
    {
    return false;
    }

} /* _nav_detect_launch */

/**
  * END OF FILE
  */