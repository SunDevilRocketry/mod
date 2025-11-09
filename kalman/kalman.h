/*******************************************************************************
*
* FILE: 
* 		kalman.h
*
* DESCRIPTION: 
* 		Contains functions related to sensor data filtration.
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

typedef struct _GYRO_STATE
    {
    //Euler rate definitions
    float phi_rad;
    float theta_rad;
    float psi_rad;

    //Error covariance matrix
    float P[9];
    //Model noise (roll)
    float Q[3];
    //Model noise (accel)
    float R[3];
    //Sample time
    float T;
    } GYRO_STATE;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

void gyro_kalman_filter_Init (float Pinit, float *Qinit, float *Rinit);
void gyro_kalman_filter_predict (float T);
void gyro_kalman_filter_update ();

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/