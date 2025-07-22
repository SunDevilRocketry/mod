/*******************************************************************************
*
* FILE: 
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

typedef enum SERVO_OPCODE
    {
    SERVO_SWEEP = 0x00,
    SERVO_RESET = 0x01   
    } SERVO_OPCODE;

typedef struct _SERVO_PRESET
    {
    uint8_t rp_servo1;
    uint8_t rp_servo2;
    uint8_t rp_servo3;
    uint8_t rp_servo4;
    } SERVO_PRESET;

/*** NEW: ServoID enum ***/
/*** replaces hard-coded motor 1/2/3/4 functions ***/
typedef enum {
    SERVO_1,
    SERVO_2,
    SERVO_3,
    SERVO_4
} ServoID;

/*** NEW: consolidated function prototype ***/
void motor_drive(ServoID servo, uint8_t angle);

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

/* A complete function that drives all servos in this board  */
void motors_drive(uint8_t angle);

/* Execute servo subcommand */
SERVO_STATUS servo_cmd_execute(uint8_t subcommand);

uint8_t angle_to_pulse(uint8_t angle);

uint8_t motor_snap_to_bound(uint8_t angle, uint8_t upper, uint8_t lower);

/* A function that handles a error driven from controlling servo */
void error_handler();

#endif /* SERVO_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
