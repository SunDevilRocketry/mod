/**
  ******************************************************************************
  * @file   math_linalg.c
  * @brief  Contains linear algebra functions for SDR code.
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
  */

/* Standard Includes ---------------------------------------------------------*/
#include <math.h>

/* Project Includes ----------------------------------------------------------*/
#include "math_sdr.h"

/* API Functions -------------------------------------------------------------*/

VECTOR_3F math_mat_vec_mult
    (
    MAT33 mat,
    VECTOR_3F vec
    )
{
VECTOR_3F out = { 0 };

/* Loop over rows */
for (uint8_t i = 0; i < 3; i++ )
    {
    out.x += mat[i][0];
    out.y += mat[i][1];
    out.z += mat[i][2];
    }
}


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/