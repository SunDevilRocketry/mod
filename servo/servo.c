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
#include "usb.h"
/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
#define SER_PER 0.55555

extern SERVO_PRESET servo_preset;

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
    motor1_drive(servo_preset.rp_servo1);
    motor2_drive(servo_preset.rp_servo2);
    motor3_drive(servo_preset.rp_servo3);
    motor4_drive(servo_preset.rp_servo4);
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
    uint8_t pulse = angle_to_pulse(motor_snap_to_bound(angle, 180, 0));
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
    uint8_t pulse = angle_to_pulse(motor_snap_to_bound(angle, 180, 0));
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
    uint8_t pulse = angle_to_pulse(motor_snap_to_bound(angle, 180, 0));
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
    uint8_t pulse = angle_to_pulse(motor_snap_to_bound(angle, 180, 0));
    htim2.Instance->CCR1 = pulse;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motors_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		A complete function that drives all servos in this board               *
*                                                                              *
*******************************************************************************/
void motors_drive(uint8_t angle)
{
    motor1_drive(angle);
    motor2_drive(angle);
    motor3_drive(angle);
    motor4_drive(angle);
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a servo command given from the terminal.                      *
*                                                                              *
*******************************************************************************/
SERVO_STATUS servo_cmd_execute(uint8_t subcommand){
    USB_STATUS usb_status;

    switch (subcommand){
        case SERVO_SWEEP:
        {
            uint8_t degree;
            usb_status = usb_receive(&degree, sizeof(uint8_t), 1000);
            motors_drive(degree);
            if (usb_status == USB_OK) 
                {
                return SERVO_OK;
                }
            else
                {
                led_set_color( LED_YELLOW );
                HAL_Delay( 5000 );
                return SERVO_FAIL;
                }
        }
        case SERVO_RESET:
        {
            servo_reset();
            return SERVO_OK;
        }
        default:
            return SERVO_FAIL;
    }
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		angle_to_pulse                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Convert turn angle degree into PWM pulse                               *
*                                                                              *
*******************************************************************************/
uint8_t angle_to_pulse(uint8_t angle)
{
    return 25 + (angle*SER_PER);
}

uint8_t motor_snap_to_bound(uint8_t angle, uint8_t upper, uint8_t lower)
{
    if (angle >= lower && angle <= upper) {
        return angle;
    } else if (angle > upper && angle <= (upper + ((255 - upper) / 2))) {
        return upper;
    } else {
        return lower;
    }
}