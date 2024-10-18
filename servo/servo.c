/*******************************************************************************
*
* FILE: 
* 		servo.c
*
* DESCRIPTION: 
* 		Contains API functions to get access to the servo motors driver
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "servo.h"
#include "led.h"
#include "init.h"
/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
#define SER_PER 0.55555

extern uint8_t rp_servo1;
extern uint8_t rp_servo2;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_init                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initialize/Reset servo                             *
*                                                                              *
*******************************************************************************/
SERVO_STATUS servo_init()
{
    // GPIO Initialization
    HAL_GPIO_WritePin(MOTOR1_EN_PORT, MOTOR1_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_EN_PORT, MOTOR2_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR3_EN_PORT, MOTOR3_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR4_EN_PORT, MOTOR4_EN, GPIO_PIN_SET);

    // Timer intialization
    HAL_StatusTypeDef hal_status1, hal_status2, hal_status3, hal_status4;

    hal_status1 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    hal_status2 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    hal_status3 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    hal_status4 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    if ( hal_status1 == HAL_OK &&
        hal_status2 == HAL_OK &&
        hal_status3 == HAL_OK &&
        hal_status4 == HAL_OK )
        {
        return SERVO_OK;
        }
        else
        {
        return SERVO_FAIL;
        }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_reset                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initialize/Reset servo rotate to default                               *
*                                                                              *
*******************************************************************************/
void servo_reset()
{
    motor1_drive(rp_servo1);
    motor2_drive(rp_servo2);
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor1_pwm_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the first servo motor with a desired value (250-1250)            *
*                                                                              *
*******************************************************************************/
void motor1_pwm_drive(uint8_t pulse)
{
    htim3.Instance->CCR4 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor2_pwm_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the second servo motor with a desired value (250-1250)           *
*                                                                              *
*******************************************************************************/
void motor2_pwm_drive(uint8_t pulse)
{
    htim3.Instance->CCR3 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor3_pwm_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the third servo motor with a desired value (250-1250)            *
*                                                                              *
*******************************************************************************/
void motor3_pwm_drive(uint8_t pulse)
{
    htim3.Instance->CCR1 = pulse;

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor4_pwm_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the forth servo motor with a desired value (250-1250)            *
*                                                                              *
*******************************************************************************/
void motor4_pwm_drive(uint8_t pulse)
{
    htim2.Instance->CCR1 = pulse;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor1_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the first servo motor with a desired angle (0-180)            *
*                                                                              *
*******************************************************************************/
void motor1_drive(uint8_t angle)
{
    uint8_t pulse = angle_to_pulse(angle);
    htim3.Instance->CCR4 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor2_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the second servo motor with a desired angle (0-180)              *
*                                                                              *
*******************************************************************************/
void motor2_drive(uint8_t angle)
{
    uint8_t pulse = angle_to_pulse(angle);
    htim3.Instance->CCR3 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor3_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the third servo motor with a desired angle (0-180)               *
*                                                                              *
*******************************************************************************/
void motor3_drive(uint8_t angle)
{
    uint8_t pulse = angle_to_pulse(angle);
    htim3.Instance->CCR1 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor4_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the forth servo motor with a desired angle (0-180)               *
*                                                                              *
*******************************************************************************/
void motor4_drive(uint8_t angle)
{
    uint8_t pulse = angle_to_pulse(angle);
    htim2.Instance->CCR1 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motors_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		A complete function that drives each servo in this board               *
*                                                                              *
*******************************************************************************/
void motors_drive(SERVOS_DATA servos_data)
{
    uint8_t default_angle = angle_to_pulse(0);
    motor1_drive(default_angle);
    motor2_drive(default_angle);
    motor3_drive(default_angle);
    motor4_drive(default_angle);
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_cmd_execute  
*
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Execute servo subcommand                                               *
*                                                                              *
*******************************************************************************/

// void motor_drive(uint8_t subcommand_code)
// {
//     uint8_t motor_number = usb_receive( &subcommand_code, sizeof( subcommand_code ), HAL_DEFAULT_TIMEOUT );
//     uint8_t deg = usb_receive( &subcommand_code, sizeof( subcommand_code ), HAL_DEFAULT_TIMEOUT );

//     switch (motor_number){
//         case 1: {
//             htim3.Instance->CCR4 = deg;
//         }

//         case 2: {
//             htim3.Instance->CCR3 = deg;
//         }
//         case 3: {
//             htim3.Instance->CCR2 = deg;
//         }
//         case 4: {
//             htim3.Instance->CCR1 = deg;
//         }
//     }
    
// }


// void servo_cmd_execute(uint8_t servo_cmd_opcode){
//     //TODO: Implement cases for testing servo controlling loop and testing individual servo
//     switch(servo_cmd_opcode){
        
//         case SERVO_INIT:
//         {
//             servo_init();
//             break;
//         }

//         case SERVO_TURN:
//         {
//             motor_drive(servo_cmd_opcode);
//             break;
//         }
        

//     }}

uint8_t angle_to_pulse(uint8_t angle)
{
    return 25 + (angle*SER_PER);
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_handler                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		A function that handles a error driven from controlling servo          *
*                                                                              *
*******************************************************************************/
void error_handler()
{

}

