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


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor1_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the first servo motor with a desired value (0-100)               *
*                                                                              *
*******************************************************************************/
void motor1_drive(uint8 duty_cycle)
{

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor2_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the second servo motor with a desired value (0-100)              *
*                                                                              *
*******************************************************************************/
void motor2_drive(uint8 duty_cycle)
{

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor3_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the third servo motor with a desired value (0-100)               *
*                                                                              *
*******************************************************************************/
void motor3_drive(uint8 duty_cycle)
{

}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		motor4_drive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Drive the forth servo motor with a desired value (0-100)               *
*                                                                              *
*******************************************************************************/
void motor4_drive(uint8 duty_cycle)
{

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
    motor1_drive();
    motor2_drive();
    motor3_drive();
    motor4_drive();
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		servo_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Execute servo subcommand                                               *
*                                                                              *
*******************************************************************************/
void servo_cmd_execute(uint8 subcommand){
    //TODO: Implement cases for testing servo controlling loop and testing individual servo
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
