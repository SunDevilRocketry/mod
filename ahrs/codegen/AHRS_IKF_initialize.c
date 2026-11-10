/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRS_IKF_initialize.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "AHRS_IKF_initialize.h"
#include "AHRS_IKF_data.h"
#include "BuiltinReg.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void AHRS_IKF_initialize(void)
{
  BuiltinReg_init();
  isInitialized_AHRS_IKF = true;
}

/*
 * File trailer for AHRS_IKF_initialize.c
 *
 * [EOF]
 */
