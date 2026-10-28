/**
  ******************************************************************************
  * @file           : isa.c
  * @brief          : Contains functions to compute vehicle dynamics
  *                   information at different altitudes and speeds.
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
#include "isa.h"
#include "math_sdr.h"

/*------------------------------------------------------------------------------
 Constants                                                                   
------------------------------------------------------------------------------*/
#define ISA_PRESSURE_SEA_LEVEL  ( (float)101325.0f )                /* Pa */
#define ISA_EXP                 ( (float)0.190294958f )             /* Unitless */
#define ISA_TEMP_LAPSE_RATE     ( (float)0.0065 )                   /* K/m */
#define ISA_TROPOPAUSE_PRES     ( (float)22632.06f )                /* Pa, pressure at 11,000 m */
#define ISA_TROPOPAUSE_ALT      ( (float)11000.0f )                 /* m */
#define ISA_STRATO_SCALE_H      ( (float)6341.62f )                 /* m, R*216.65/g0 */
#define KELVIN_OFFSET           ( (float)273.15f)                   /* K */
#define ISA_SEA_LEVEL_TEMP_K    ( (float)(15.0f + KELVIN_OFFSET) )  /* 288.15 K */
#define ISA_TROPOPAUSE_TEMP_C   ( (float)-56.5f )                   /* Constant temp in lower stratosphere */

/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Calculates altitude (relative to sea level).
  * @param baro_pres Barometric pressure in Pa.
  * @param baro_temp Barometric temperature in C. Use INFINITY to use ISA temp.
  * 
  * @details
  * \latexonly
  * \begin{equation}
  *   h = \frac{T_{0}}{\lambda}[1-\frac{P}{P_{0}}^{\frac{R\lambda}{g_{0}}}]
  * \end{equation}
  * \endlatexonly
  * 
  * @return Calculated ISA altitude above sea level.
  */
float isa_get_altitude_qnh
    (
    float baro_pres, 
    float baro_temp
    )
{
float altitude;
if ( baro_pres <= 0.0f ) 
    {
    /* allow error handling */
    return INFINITY;
    }
if ( baro_temp == INFINITY )
    {
    baro_temp = isa_get_temp( baro_pres );
    }

if( baro_pres < ISA_TROPOPAUSE_PRES ) 
    {
    altitude = ISA_TROPOPAUSE_ALT + ISA_STRATO_SCALE_H * logf(ISA_TROPOPAUSE_PRES / baro_pres);
    }
else
    {
    altitude = ( (powf(ISA_PRESSURE_SEA_LEVEL / baro_pres, ISA_EXP) - 1.0f)
               * (baro_temp + KELVIN_OFFSET) / ISA_TEMP_LAPSE_RATE );
    }

return( altitude );

} /* isa_get_altitude_qnh */


/**
  * @brief Calculates altitude (relative to ground).
  * @param curr_pres Barometric pressure in Pa.
  * @param curr_temp Barometric temperature in C.
  * @param reference_pres Calibrated reference barometric pressure in Pa.
  * @param reference_temp Calibrated reference barometric temperature in C.
  * 
  * @return Calculated ISA altitude above ground reference.
  */
float isa_get_altitude_qfe
    (
    float curr_pres, 
    float curr_temp,
    float reference_pres,
    float reference_temp
    )
{
float current_alt = isa_get_altitude_qnh(curr_pres, curr_temp);
float reference_alt = isa_get_altitude_qnh(reference_pres, reference_temp);

return( current_alt - reference_alt );

} /* isa_get_altitude_qfe */


/**
  * @brief Calculate the ISA temperature at the given barometric pressure.
  * 
  * @param baro_pres The barometric pressure
  * @return The ISA temperature
  */
float isa_get_temp
    (
    float baro_pres
    )
{
/* Local Variables */
float temp_k;

/* Above the tropopause, return the constant */
if ( baro_pres < ISA_TROPOPAUSE_PRES ) 
    {
    return ISA_TROPOPAUSE_TEMP_C;
    }
/* Below the tropopause, reverse the ISA equation to find the altitude 
   at the given pressure at ISA temp, then calculate the ISA temp from there */
else
    {
    temp_k = ISA_SEA_LEVEL_TEMP_K 
           * powf( (baro_pres / ISA_PRESSURE_SEA_LEVEL), ISA_EXP );
    return temp_k - KELVIN_OFFSET;
    }

} /* isa_get_temp */

/**
  * END OF FILE
  */