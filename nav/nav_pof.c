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
 Static Procedure Definitions
------------------------------------------------------------------------------*/

static bool detect_launch
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    );

static bool detect_coast
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    );

static bool detect_apogee
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    );

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
    NAV_POF pof
	)
{
nav_nullchk( nav_state );

/* Local Variables */
NAV_PRIVATE_STATE* prv_state = nav_state_to_prv( nav_state );
NAV_STATUS status = NAV_OK;

/* Overwrite nav POF */
if( nav_reserve_state( nav_state ) == NAV_OK )
    {
    prv_state->pof = pof;

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

/**
 * @brief Initialize the phase of flight module
 * 
 * @param prv_state The nav library state
 * @return NAV_STATUS 
 */
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


/**
 * @brief Update the POF state.
 * 
 * @param prv_state The navigation library state
 * @param sensors The inputs to the library
 * @param outputs The library's computed outputs so far
 * @return The status of the navigation library
 */
NAV_STATUS _nav_pof_update
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    )
{
NAV_POF starting_state = prv_state->pof;
switch( starting_state )
    {
    case NAV_ON_GROUND:
        if( detect_launch( prv_state, sensors, outputs ) )
            {
            prv_state->pof = NAV_POWERED_ASCENT;
            nav_pof_transition_event( (NAV_PUBLIC_STATE*)prv_state, starting_state, prv_state->pof );
            }
        break;
    case NAV_POWERED_ASCENT:
        if( detect_coast( prv_state, sensors, outputs ) )
            {
            prv_state->pof = NAV_UNPOWERED_ASCENT;
            nav_pof_transition_event( (NAV_PUBLIC_STATE*)prv_state, starting_state, prv_state->pof );
            }
        else if( detect_apogee( prv_state, sensors, outputs ) )
            {
            prv_state->pof = NAV_DESCENT;
            nav_pof_transition_event( (NAV_PUBLIC_STATE*)prv_state, starting_state, prv_state->pof );
            }
        break;
    case NAV_UNPOWERED_ASCENT:
        if( detect_apogee( prv_state, sensors, outputs ) )
            {
            prv_state->pof = NAV_DESCENT;
            nav_pof_transition_event( (NAV_PUBLIC_STATE*)prv_state, starting_state, prv_state->pof );
            }
        break;

    case NAV_DESCENT:
    case NAV_LANDED:
        /* no applicable transitions; do nothing */
        break;
    /**
      * GCOVR_EXCL_START
      *
      * This branch is tested, but gcovr doesn't see this statement in
      * release mode so it reports a coverage hole on SDR's simulator builds.
      */
    default:
        _nav_debug_error( "Invalid state transition detected" );
        return( NAV_FAIL );
     /**
      * GCOVR_EXCL_STOP
      */
    }

return( NAV_OK );

} /* _nav_pof_update */

/*------------------------------------------------------------------------------
 Static Functions 
------------------------------------------------------------------------------*/

/**
 * @brief Detect if launch has occurred.
 * 
 * @param prv_state The navigation library state
 * @param sensors The inputs to the library
 * @param outputs The library's computed outputs so far
 * @return t: launch has occurred
 */
static bool detect_launch
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    )
{
/* Only check launch detect if:
 * 1. Launch detect is accel based AND there is accel data to fuse
 *    OR
 * 2. Launch detect is baro based AND there is baro data to fuse
 */
if( !( prv_state->saved_configs.launch_detect_accel_enabled && math_vector_is_finite( sensors->accel ) )
 && !( prv_state->saved_configs.launch_detect_baro_enabled && isfinite( sensors->baro_pres ) ) )
    {
    /* do not update the internal state, we will try again on the next cycle */
    return false;
    }

float accX = sensors->accel.x;
float pressure = sensors->baro_pres;
float acc_scalar = fabsf(accX);

if ( prv_state->saved_configs.launch_detect_accel_enabled )
    {
    if ( acc_scalar > (float)prv_state->saved_configs.launch_detect_accel_threshold * GRAVITY )
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

} /* detect_launch */


/**
 * @brief Detect if coast has occurred.
 * 
 * @param prv_state The navigation library state
 * @param sensors The inputs to the library
 * @param outputs The library's computed outputs so far
 * @return t: coast has occurred
 */
static bool detect_coast
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    )
{
/* Only check coast if there is valid accel data */
if( !math_vector_is_finite( sensors->accel ) )
    {
    /* do not update the internal state, we will try again on the next cycle */
    return false;
    }

/* check if accel is sufficiently low */
if ( sensors->accel.x < prv_state->saved_configs.coast_detect_threshold * GRAVITY )
    {
    prv_state->pof_state.coast_detect_counts++;
    }
else
    {
    prv_state->pof_state.coast_detect_counts = 0;
    }

/* check if we've hit our sample threshold */
if( prv_state->pof_state.coast_detect_counts >= prv_state->saved_configs.coast_detect_counts )
    {
    return true;
    }
else
    {
    return false;
    }

} /* detect_coast */


/**
 * @brief Detect if apogee has occurred.
 * 
 * @param prv_state The navigation library state
 * @param sensors The inputs to the library
 * @param outputs The library's computed outputs so far
 * @return t: apogee has occurred
 */
static bool detect_apogee
    (
    NAV_PRIVATE_STATE* prv_state,
    const NAV_SENSORS* sensors,
    NAV_OUTPUTS* outputs
    )
{
/* Only check coast if there is a valid altitude estimate */
if( !isfinite( outputs->ind_alt_asl ) )
    {
    /* do not update the internal state, we will try again on the next cycle */
    return false;
    }

float curr_alt = outputs->ind_alt_asl;

if( prv_state->pof_state.apogee_last_sample != 0.0f )
    {
    if( curr_alt < prv_state->pof_state.apogee_last_sample )
        {
        prv_state->pof_state.apogee_decreasing_counts++;
        }
    else
        {
        prv_state->pof_state.apogee_decreasing_counts = 0;
        }
    }

prv_state->pof_state.apogee_last_sample = curr_alt;

if( prv_state->pof_state.apogee_decreasing_counts >= prv_state->saved_configs.apogee_decreasing_counts )
    {
    return true; /* Apogee detected */
    }
else
    {
    return false; /* Apogee not detected */
    }
    
} /* detect_apogee */

/**
  * END OF FILE
  */