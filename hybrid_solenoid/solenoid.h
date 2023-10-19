/*******************************************************************************
*
* FILE:
* 		solenoid.h
*
* DESCRIPTION:
* 		Basic solenoid actuation API for hyprid engine
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SOLENOID_H
#define SOLENOID_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/




/*------------------------------------------------------------------------------
 Types                                                                     
------------------------------------------------------------------------------*/

/* Execute a solenoid command */
void solenoid_cmd_execute
	(
	uint8_t solenoid_cmd_opcode  /* Solenoid actuation code */
	); /* solenoid_cmd_execute */

/* Turn a solenoid on */
void solenoid_on
	(
	  /* Solenoid number to actuate */
	); /* solenoid_on */

/* Turn a solenoid off */
void solenoid_off
	(
	  /* Solenoid number to actuate */
	); /* solenoid_off */



#ifdef __cplusplus
}
#endif

#endif /* SOLENOID_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/