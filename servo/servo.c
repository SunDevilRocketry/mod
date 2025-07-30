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
 Local Constants (macros, #defines)
------------------------------------------------------------------------------*/
#define SER_PER 0.55555
/*------------------------------------------------------------------------------
 Global Variables (memory objects shared across files)
------------------------------------------------------------------------------*/
extern SERVO_PRESET servo_preset;
/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_init                                                             *                                                                      *
* DESCRIPTION:                                                                 * 
* 		Initialize/Reset servo                                                 *                                                                 *
*******************************************************************************/
SERVO_STATUS servo_init
    (
    void
    )
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

} /* servo_init */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor_drive                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drives the specified servo motor to the desired angle (0-180).         *
*                                                                              *
*******************************************************************************/
void motor_drive
    (
    SERVO_ID servo, 
    uint8_t angle
    )
{
uint8_t pulse = angle_to_pulse(motor_snap_to_bound(angle, 180, 0));

switch (servo) 
    {
    case SERVO_1:
        htim3.Instance->CCR4 = pulse;
        break;
    case SERVO_2:
        htim3.Instance->CCR3 = pulse;
        break;
    case SERVO_3:
        htim3.Instance->CCR1 = pulse;
        break;
    case SERVO_4:
        htim2.Instance->CCR1 = pulse;
        break;
    default:
        // Optionally: handle error
        break;
    }

} /* motor_drive */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_reset                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Resets all servos to their preset positions                            *
*                                                                              *
*******************************************************************************/
void servo_reset
    (
    void
    )
{
motor_drive(SERVO_1, servo_preset.rp_servo1);
motor_drive(SERVO_2, servo_preset.rp_servo2);
motor_drive(SERVO_3, servo_preset.rp_servo3);
motor_drive(SERVO_4, servo_preset.rp_servo4);

} /* servo_reset */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motors_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		A complete function that drives all servos in this board               *
*       Uses motor_drive for each servo.                                       *
*                                                                              *
*******************************************************************************/

void motors_drive
    (
    uint8_t angle
    )
{
motor_drive(SERVO_1, angle);
motor_drive(SERVO_2, angle);
motor_drive(SERVO_3, angle);
motor_drive(SERVO_4, angle);

} /* motors_drive */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a servo command given from the terminal.                      *
*                                                                              *
*******************************************************************************/
SERVO_STATUS servo_cmd_execute
    (
    uint8_t subcommand
    )
{
USB_STATUS usb_status;

switch ( subcommand )
    {
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
        {
        return SERVO_FAIL;
        }
    }

} /* servo_cmd_execute */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		angle_to_pulse                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Convert turn angle degree into PWM pulse                               *
*                                                                              *
*******************************************************************************/
uint8_t angle_to_pulse
    (
    uint8_t angle
    )
{
return 25 + (angle*SER_PER);

} /* angle_to_pulse */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor_snap_to_bound                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Given an angle, upper bound, and lower bound, snap the angle to upper  *
*       or lower if the angle is outside of the bounds.                        *
*                                                                              *
*******************************************************************************/
uint8_t motor_snap_to_bound
    (
    uint8_t angle, 
    uint8_t upper, 
    uint8_t lower
    )
{
if (angle >= lower && angle <= upper) 
    {
    return angle;
    } 
    else if ( angle > upper && angle <= ( upper + ( ( 255 - upper ) / 2 ) ) ) 
    {
    return upper;
    } 
    else 
    {
    return lower;
    }

} /* motor_snap_to_bound */