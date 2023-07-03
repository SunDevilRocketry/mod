/*******************************************************************************
*
* FILE: 
* 		solenoid.h
*
* DESCRIPTION: 
* 		Basic solenoid actuation API
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
#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Solenoid Subcommand Codes */
#define SOL_ON_BASE_CODE	    0x00
#define SOL_OFF_BASE_CODE	    0x08
#define SOL_TOGGLE_BASE_CODE	0x10
#define SOL_RESET_CODE	        0x18
#define SOL_GETSTATE_CODE       0x20
#define SOL_OPEN_CODE           0x28
#define SOL_CLOSE_CODE          0x30

/* Number of solenoids */
#define NUM_SOLENOIDS           6


/*------------------------------------------------------------------------------
 Types                                                                     
------------------------------------------------------------------------------*/

/* Solenoid GPIO port and pin */
struct sol_GPIO_handle {
	GPIO_TypeDef* GPIOx;    /* GPIO Port */
	uint16_t      GPIO_pin; /* Solenoid GPIO Pin */
};

/* State of solenoids on/off 
   bit 7-6: Not used 
   bit   5: Fuel Purge Line Solenoid 
   bit   4: LOX Purge Line Solenoid 
   bit   3: Fuel Venting Solenoid 
   bit   2: LOX Venting Solenoid 
   bit   1: Fuel Pressurization Solenoid 
   bit   0: LOX Pressurization Solenoid */
typedef uint8_t SOL_STATE;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Map solenoid numbers to GPIO ports */
void solenoid_map
	(
	struct sol_GPIO_handle* sol_GPIO_config, /* Pointer to GPIO port and pin 
                                                configuration for target 
                                                solenoid                      */
	uint8_t solenoid_num /* Solenoid number to actuate */ 
	); /* solenoid_map */

/* Execute a solenoid command */
void solenoid_cmd_execute
	(
	uint8_t solenoid_cmd_opcode  /* Solenoid actuation code */
	); /* solenoid_cmd_execute */

/* Turn a solenoid on */
void solenoid_on
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_on */

/* Turn a solenoid off */
void solenoid_off
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_off */

/* Toggle the state of a solenoid */
void solenoid_toggle
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_toggle */

/* Reset the solenoids */
void solenoid_reset
	(
	void
	); /* solenoid_reset */

/* Get the state of the solenoids */
SOL_STATE solenoid_get_state
	(
	void
	);

/* Open Solenoids */
void solenoid_open 
	(
	uint8_t solenoid_num
	);

/* Close solenoids */
void solenoid_close
	(
	uint8_t solenoid_num
	);


#ifdef __cplusplus
}
#endif

#endif /* SOLENOID_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/