/**
  ******************************************************************************
  * @file           : isa.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ISA_H
#define ISA_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

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
float isa_get_altitude_qnh
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
float isa_get_altitude_qfe
    (
    float curr_pres, 
    float curr_temp,
    float reference_pres,
    float reference_temp
    );

#ifdef __cplusplus
}
#endif

#endif /* ISA_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
