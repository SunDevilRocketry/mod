/*******************************************************************************
*
* FILE: 
* 		ignition.h
*
* DESCRIPTION: 
* 		Contains API functions to the flight computer parachute deployment 
*       system and contintuity readings
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IGNITION_H
#define IGNITION_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 MCU Peripheral Configuration 
------------------------------------------------------------------------------*/

#if !( defined( A0002_REV1 ) || \
       defined( L0002_REV4 ) || \
	   defined( A0002_REV2 ) || \
	   defined( L0002_REV5 ) || \
	   defined( A0007_REV1 ) ) 
	#error No IGNITION compatible device specified
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Ignition Continuity Status */
/* IGN_CONT_STAT = bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 

   bits6-7: unused
   bit5:   Nozzle continuity of liquid engine, 1 indicates continuity between 
           between screw terminals
   bit4:   Solid propellant continuity for liquid engine, 1 indiciates 
           continuity between screw terminals
   bit3:   Ematch continuity for liquid engine, 1 indicates continuity between
           srew terminals 
   bit2:   Drogue parachute deployment continuity, 1 indicates continuity between 
           screw terminals
   bit1:   Main Parachute Deployment continuity, 1 indicates continuity between 
		   screw terminals
   bit0:   Switch continuity */

/* Ignition Status Response Code */
typedef enum IGN_STATUS
	{
	IGN_OK               = 0x40, /* Cont status takes up first 64 values */
	IGN_SUCCESS          = 0x41,
	IGN_EMATCH_CONT_FAIL = 0x42, 
	IGN_POWER_FAIL       = 0x43, 
	IGN_FAIL             = 0x44,
	IGN_SWITCH_FAIL      = 0x45,
	IGN_MAIN_CONT_FAIL   = 0x46,
	IGN_MAIN_FAIL        = 0x47,
    IGN_DROGUE_CONT_FAIL = 0x48, 
	IGN_DROGUE_FAIL      = 0x49,
	IGN_UNRECOGNIZED_CMD = 0x4A
	} IGN_STATUS;

/* SDEC Subcommand Codes */
typedef enum IGN_SUBCOMMAND
	{
	IGN_FIRE_CODE = 0x01,
	IGN_CONT_CODE       ,
	IGN_MAIN_DEPLOY_CODE,
	IGN_DROGUE_DEPLOY_CODE     
	} IGN_SUBCOMMAND;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Ignition continuity code bitmasks */
#define IGN_SWITCH_MASK   	    0b00000001
#define IGN_MAIN_CONT_MASK  	0b00000010
#define IGN_DROGUE_CONT_MASK 	0b00000100
#define IGN_E_CONT_MASK         0b00001000
#define IGN_SP_CONT_MASK        0b00010000
#define IGN_NOZ_CONT_MASK       0b00100000

/* Ignition burn time */
#define IGN_BURN_DELAY          1250 


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Executes an ignition subcommand based on user input from the sdec terminal */
IGN_STATUS ign_cmd_execute
	(
    IGN_SUBCOMMAND ign_subcommand
    );

/* Polls each continuity pin and sets the continuity bits in the response 
   code */
IGN_STATUS ign_get_cont_info
	(
    void
    );

#if defined( ENGINE_CONTROLLER )
/* Asserts the ignition signal to ignite the engine ematch. Returns a response 
code indicating if the ignition occured succesfully */
IGN_STATUS ign_ignite
    (
	void
    );

/* Check for continuity across solid propellant wire screw terminals */
bool ign_solid_prop_cont
	(
    void
    );

/* Check for continuity across nozzle wire screw terminals           */
bool ign_nozzle_cont
	(
    void
    );

/* Check for continuity across ematch and switch screw terminals     */
bool ign_ematch_cont
	(
    void
    );

#endif /* #if defined( ENGINE_CONTROLLER ) */


#if ( defined( FLIGHT_COMPUTER ) || defined( FLIGHT_COMPUTER_LITE ) )
/* Asserts the ignition signal to ignite the main parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_main 
    (
	void
    );


/* Asserts the ignition signal to ignite the drogue parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_drogue 
    (
	void
    );


/* Returns TRUE if there is continuity across the main parachute deployment 
   ematch */
bool ign_main_cont
	(
	void
	);


/* Returns TRUE if there is continuity across the drogue parachute deployment 
   ematch */
bool ign_drogue_cont
	(
	void
	);

/* Returns TRUE if there is continuity across the switch screw terminals */
bool ign_switch_cont
	(
	void
	);

#endif /* #if defined( FLIGHT_COMPUTER )*/

#ifdef __cplusplus
}
#endif

#endif /* IGNITION_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/