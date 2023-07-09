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

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
/* Initialize PWM Timers */
SERVO_STATUS pwm_timer_init();

/* Initialize/Reset servo rotate to default */
void servo_init();

/* Drive the first servo motor with a desired value (0-100) */
void motor1_drive(uint8_t duty_cycle);

/* Drive the second servo motor with a desired value (0-100) */
void motor2_drive(uint8_t duty_cycle);

/* Drive the third servo motor with a desired value (0-100) */
void motor3_drive(uint8_t duty_cycle);

/* Drive the forth servo motor with a desired value (0-100) */
void motor4_drive(uint8_t duty_cycle);

/* A complete function that drives each servo in this board  */
void motors_drive(SERVOS_DATA servos_data);

/* Execute servo subcommand */
void servo_cmd_execute(uint8_t subcommand);

/* A function that handles a error driven from controlling servo */
void error_handler();

#endif /* SERVO_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
