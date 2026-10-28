/**
  ******************************************************************************
  * @file           : nav_contract.h
  * @brief          : Navigation component contract header.
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
  # THE FUNCTIONS IN THIS FILE MUST BE IMPLEMENTED BY THE INTEGRATING PROJECT  #
  ==============================================================================
  @endverbatim
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NAV_CONTRACT_H
#define NAV_CONTRACT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Standard Includes ---------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>

/* Project Includes ----------------------------------------------------------*/
#include "nav.h"

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Reserve this nav state instance.
 * 
 * @param nav_state The state to reserve.
 * 
 * @retval NAV_BUSY Could not reserve state.
 * @retval NAV_OK State reservation succeeded.
 */
NAV_STATUS nav_reserve_state
    (
    NAV_PUBLIC_STATE* nav_state
    );

/**
 * @brief Release this nav state instance.
 * 
 * @param nav_state The state to release.
 * 
 * @retval NAV_BUSY Could not release state.
 * @retval NAV_OK State release succeeded.
 */
NAV_STATUS nav_release_state
    (
    NAV_PUBLIC_STATE* nav_state
    );

#ifdef __cplusplus
}
#endif

#endif /* NAV_CONTRACT_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/