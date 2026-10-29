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

/**
 * @brief Notify the integrating project of a change in NAV's phase of flight.
 *
 * @param[in,out] nav_state The state of the NAV library
 * @param[in] original_state The starting state during this update
 * @param[in] new_state The new NAV state.
 */
void nav_pof_transition_event
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_POF original_state,
    NAV_POF new_state
    );

#ifdef __cplusplus
}
#endif

#endif /* NAV_CONTRACT_H */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/