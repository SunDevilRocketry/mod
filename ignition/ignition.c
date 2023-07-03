/*******************************************************************************
*
* FILE: 
* 		ignition.c
*
* DESCRIPTION: 
* 		Contains API function to the engine controller ignition system and 
*       contintuity readings
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "ignition.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_cmd_execute                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes an ignition subcommand based on user input from the sdec      *
*       terminal                                                               *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_cmd_execute
	(
    IGN_SUBCOMMAND ign_subcommand
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STATUS ign_status = 0; /* Status code returned by ignite API function */

/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
switch( ign_subcommand )
	{
	#if defined( FLIGHT_COMPUTER )
    /* Deploy main */
	case IGN_MAIN_DEPLOY_CODE:
		{
		ign_status = ign_deploy_main();
		break;
		}

    /* Deploy drogue */
	case IGN_DROGUE_DEPLOY_CODE:
		{
		ign_status = ign_deploy_drogue();
		break;
		}

	#elif defined( ENGINE_CONTROLLER )
    /* Light ematch */
	case IGN_FIRE_CODE:
		{
		ign_status = ign_ignite();
		break;
		}

	#endif /* #elif defined( ENGINE_CONTROLLER ) */

	/* Return continuity information */
	case IGN_CONT_CODE:
		{
		ign_status = ign_get_cont_info();
		break;
		}

    /* Unrecognized subcommand code: call error handler */
	default:
		{
		return IGN_UNRECOGNIZED_CMD;
		break;
		}
    }

/* Return the response code */
return ign_status;

} /* ign_cmd_execute */


#if defined( ENGINE_CONTROLLER )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_ignite                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the engine ematch. Returns a     *
*       response code indicating if the ignition occured succesfully           *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_ignite
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check for e-match/switch continuity */
if ( !ign_ematch_cont() )
	{
    /* No continuity across ematch and/or switch */
    return IGN_EMATCH_CONT_FAIL; 
    }

/* Check that power supply is not USB */

/* Assert ignition signal for 10 ms */
HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_PIN, GPIO_PIN_SET  );
HAL_Delay( IGN_BURN_DELAY );
HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_PIN, GPIO_PIN_RESET);

/* Check ematch continuity to check that ematch was lit */
if ( !ign_ematch_cont() )
	{
    return IGN_SUCCESS;
    }
else /* Ignition unsuccessful */
	{
    return IGN_FAIL;
    }

} /* ignite */
#endif /* #if defined( ENGINE_CONTROLLER ) */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_get_cont_info                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Polls each continuity pin and sets the continuity bits in the          *
*       response code                                                          *   
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_get_cont_info
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STATUS ign_status = 0; /* Status code to be returned */


/*------------------------------------------------------------------------------
 Call API functions 
------------------------------------------------------------------------------*/

#if defined( ENGINE_CONTROLLER )
/* Poll the ematch continuity pin */
if ( ign_ematch_cont() )
	{
    ign_status |= IGN_E_CONT_MASK;
    }

/* Poll the solid propellant continuity pin */
if ( ign_solid_prop_cont() )
	{
    ign_status |= IGN_SP_CONT_MASK;
    }

/* Poll the nozzle continuity pin */
if ( ign_nozzle_cont() )
	{
    ign_status |= IGN_NOZ_CONT_MASK;
    }
#elif ( defined( FLIGHT_COMPUTER ) || defined( FLIGHT_COMPUTER_LITE ) )
/* Poll the switch continuity pin */
if ( ign_switch_cont() )
	{
    ign_status |= IGN_SWITCH_MASK;
    }

/* Poll the main parachute deployment continuity pin */
if ( ign_main_cont() )
	{
    ign_status |= IGN_MAIN_CONT_MASK;
    }

/* Poll the drogue parachute deployment continuity pin */
if ( ign_drogue_cont() )
	{
    ign_status |= IGN_DROGUE_CONT_MASK;
    }
#endif /* elif defined( FLIGHT_COMPUTER ) */

/* Return the status code */
return ign_status;

} /* ign_get_cont_info */


#if defined( ENGINE_CONTROLLER )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		solid_prop_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the solid propellant wire   *
*       screw terminals                                                        *
*                                                                              *
*******************************************************************************/
bool ign_solid_prop_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t solid_prop_cont_pinstate = HAL_GPIO_ReadPin(SP_CONT_GPIO_PORT, SP_CONT_PIN);

/* Return true if GPIO state is high*/
if ( solid_prop_cont_pinstate == 0 )
	{
    return true;
	}
else
	{
    return false;
    }

} /* ign_solid_prop_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_nozzle_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the nozzle wire screw       * 
*       terminals                                                              *
*                                                                              *
*******************************************************************************/
bool ign_nozzle_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t nozzle_cont_pinstate = HAL_GPIO_ReadPin(NOZ_CONT_GPIO_PORT, NOZ_CONT_PIN);

/* Return true if GPIO state is high*/
if ( nozzle_cont_pinstate == 0 )
	{
    return true;
	}
else
	{
    return false;
    }

} /* ign_nozzle_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_ematch_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the ematch and switch screw * 
*       terminals                                                              *
*                                                                              *
*******************************************************************************/
bool ign_ematch_cont
	(
	void
	)
{
/* Check MCU GPIO State */
uint8_t ematch_cont_pinstate = HAL_GPIO_ReadPin(E_CONT_GPIO_PORT, E_CONT_PIN);

/* Return true if GPIO state is low */
if ( ematch_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* ign_ematch_cont */

#endif /* #if defined( ENGINE_CONTROLLER ) */


#if ( defined( FLIGHT_COMPUTER ) || defined( FLIGHT_COMPUTER_LITE ) )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_main                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the main parachute deployment    *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_main 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_main_cont()   )
	{
    return IGN_MAIN_CONT_FAIL; 
    }
else /* Continuity is good for main */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_SET );
	HAL_Delay( IGN_BURN_DELAY );
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_main_cont() )
	{
	return IGN_SUCCESS;
	}
else /* Ignition unsuccessful */
	{
	return IGN_MAIN_FAIL;
	}

} /* ign_deploy_main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_drogue                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the drogue parachute deployment  *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_drogue 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_drogue_cont()   )
	{
    return IGN_DROGUE_CONT_FAIL; 
    }
else /* Continuity is good for drogue */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_SET   );
	HAL_Delay( IGN_BURN_DELAY );
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_drogue_cont() )
	{
	return IGN_SUCCESS;
	}
else /* Ignition unsuccessful */
	{
	return IGN_DROGUE_FAIL;
	}

} /* ign_deploy_drogue */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_main_cont                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the main parachute          *
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_main_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t main_cont_pinstate = HAL_GPIO_ReadPin( MAIN_CONT_GPIO_PORT, MAIN_CONT_PIN );

/* Return true if GPIO state is low */
if ( main_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* ign_main_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_drogue_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the drogue parachute        * 
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_drogue_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t drogue_cont_pinstate = HAL_GPIO_ReadPin( DROGUE_CONT_GPIO_PORT, 
                                                 DROGUE_CONT_PIN );

/* Return true if GPIO state is low */
if ( drogue_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* drogue_cont */
#endif /* #if defined( FLIGHT_COMPUTER ) */


#if ( defined( FLIGHT_COMPUTER ) || defined( FLIGHT_COMPUTER_LITE ) )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_switch_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the switch screw terminals  * 
*                                                                              *
*******************************************************************************/
bool ign_switch_cont
	(
	void
	)
{
/* Check MCU GPIO State */
uint8_t switch_cont_pinstate = HAL_GPIO_ReadPin(SWITCH_GPIO_PORT, SWITCH_PIN);

/* Return true if GPIO state is low */
if ( switch_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* switch_cont */
#endif /* #if defined( FLIGHT_COMPUTER ) */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/