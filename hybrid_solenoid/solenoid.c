/*******************************************************************************
*
* FILE:
* 		solenoid.c
*
* DESCRIPTION:
* 		Basic solenoid actuation API for hyprid engine
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "solenoid.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_cmd_execute                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a solenoid function based on a subcommand code from a PC      *
*                                                                              *
*******************************************************************************/
void solenoid_cmd_execute
    (
	uint8_t solenoid_cmd_opcode  /* Solenoid actuation code */
    )
{

    
    //TODO: Implement solenoid execution based on subcommands code from the PC terminal software application
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_on                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Applies power to a specified solenoid                                 *
*                                                                              *
*******************************************************************************/
void solenoid_on(){
    //TODO: Implement code to turn on the solenoid
    
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

}
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_off                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Removes power from a specified solenoid                               *
*                                                                              *
*******************************************************************************/
void solenoid_off(){
    //TODO: Implement code to turn off the solenoid

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

}
/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/