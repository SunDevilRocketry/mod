/*******************************************************************************
*
* FILE: 
*       servo.h
*
* DESCRIPTION: 
* 		Contains API functions to get access to the servo motors driver
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SERVO_H
#define SERVO_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Structure containing PWM duty cycle (0-100) for each servo */
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

typedef enum 
    {
    SERVO_1 = 1u,
    SERVO_2 = 2u,
    SERVO_3 = 3u,
    SERVO_4 = 4u
    } SERVO_ID;

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

SERVO_STATUS servo_init
    (
    void
    );
void servo_reset
    (
    void
    );
void motor_drive(SERVO_ID servo, uint8_t angle);
void motors_drive(uint8_t angle);
SERVO_STATUS servo_cmd_execute(uint8_t subcommand);
uint8_t angle_to_pulse(uint8_t angle);
uint8_t motor_snap_to_bound(uint8_t angle, uint8_t upper, uint8_t lower);

#endif /* SERVO_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
