/*******************************************************************************
*
* FILE: 
* 		servo.h
*
* DESCRIPTION: 
* 		Contains API functions to get access to the servo motors driver
*
*******************************************************************************/


// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SERVO_H
#define SERVO_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Defines 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Defines subcommand codes
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
/* Structure containing PWM duty cycle (0-100) for each servo*/
typedef struct _SERVOS_DATA 
	{
    uint8_t motor1_duty;
    uint8_t motor2_duty;
    uint8_t motor3_duty;
    uint8_t motor4_duty;
	} SERVOS_DATA;

typedef enum SERVO_STATUS
    {
    SERVO_OK = 0,
    SERVO_FAIL
    } SERVO_STATUS;

typedef struct _SERVO_PRESET
    {
    uint8_t rp_servo1;
    uint8_t rp_servo2;
    } SERVO_PRESET;

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Initialize/Reset servo rotate to default */
SERVO_STATUS servo_init();
void servo_reset();

void motor1_pwm_drive(uint8_t pulse);
void motor2_pwm_drive(uint8_t pulse);
void motor3_pwm_drive(uint8_t pulse);
void motor4_pwm_drive(uint8_t pulse);

/* Drive the first servo motor with a desired value (0-100) */
void motor1_drive(uint8_t angle);

/* Drive the second servo motor with a desired value (0-100) */
void motor2_drive(uint8_t angle);

/* Drive the third servo motor with a desired value (0-100) */
void motor3_drive(uint8_t angle);

/* Drive the forth servo motor with a desired value (0-100) */
void motor4_drive(uint8_t angle);

/* A complete function that drives each servo in this board  */
// void motors_drive(SERVOS_DATA servos_data);

/* Execute servo subcommand */
// void servo_cmd_execute(uint8_t subcommand);

uint8_t angle_to_pulse(uint8_t angle);


/* A function that handles a error driven from controlling servo */
void error_handler();

#endif /* SERVO_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
