/**
  ******************************************************************************
  * @file           : ahrs.h
  * @brief          : AHRS private header. 
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
              ##### DO NOT INCLUDE THIS HEADER OUTSIDE OF AHRS #####
  ==============================================================================
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AHRS_PRV_H
#define AHRS_PRV_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/

/* Project Includes ----------------------------------------------------------*/
#include "AHRS_IKF.h"
#include "AHRS_IKF_types.h"
#include "ahrs.h"

/* Typedefs ------------------------------------------------------------------*/

typedef AHRS_IKF AHRS_PRIVATE_STATE;
_Static_assert( sizeof(AHRS_PRIVATE_STATE) == sizeof(AHRS_PUBLIC_STATE), 
                "Mismatch between AHRS private and public states." );

/* Internal Macros -----------------------------------------------------------*/
#define ahrs_state_to_prv( state ) ( (AHRS_PRIVATE_STATE*)state )
#define ahrs_nullchk( state ) do { \
        if ( (state) == NULL ) \
            { \
            return AHRS_NULLPTR; \
            } \
        } while(0) // TODO

/* Function Prototypes -------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* AHRS_PRV_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/