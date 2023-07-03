/*******************************************************************************
*
* FILE:
* 		solenoid.c
*
* DESCRIPTION:
* 		Basic solenoid actuation API
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_L0005.h"
#include "solenoid.h"
#include "stm32h7xx_hal.h"
#include "usb.h"
#include "valve.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Solenoid Open States */
GPIO_PinState sol_open_states[]   = { GPIO_PIN_SET  , 
                                      GPIO_PIN_SET  , 
					                  GPIO_PIN_RESET, 
					                  GPIO_PIN_RESET,
					                  GPIO_PIN_RESET, 
									  GPIO_PIN_RESET };

/* Solenoid Closed States */
GPIO_PinState sol_closed_states[] = { GPIO_PIN_RESET, 
                                      GPIO_PIN_RESET, 
					                  GPIO_PIN_SET  ,
					                  GPIO_PIN_SET  ,
					                  GPIO_PIN_SET  ,
					                  GPIO_PIN_SET };


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_map                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Maps a solenoid number to a microcontroller port and pin number        *
*                                                                              *
*******************************************************************************/
void solenoid_map
	(
	struct sol_GPIO_handle* psol_GPIO_handle, /* Pointer to GPIO port and pin
                                                 configuration for target
                                                 solenoid                     */
	uint8_t solenoid_num /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t solenoid_pin_map[] =       {2, 3, 4, 0, 1, 2}; /* Solenoid bit shifts */
GPIO_TypeDef* solenoid_port_map[] = {SOL1_GPIO_PORT,   /* Solenoid GPIO Ports */
                                     SOL2_GPIO_PORT,
                                     SOL3_GPIO_PORT, 
									 SOL4_GPIO_PORT,
                                     SOL5_GPIO_PORT, 
									 SOL6_GPIO_PORT};


/*------------------------------------------------------------------------------
 Mapping
------------------------------------------------------------------------------*/
/* Port Setting Mapping */
psol_GPIO_handle -> GPIOx    = solenoid_port_map[solenoid_num-1];

/* Pin Setting Mapping */
psol_GPIO_handle -> GPIO_pin = 1 << solenoid_pin_map[solenoid_num-1];

} /* solenoid_map */


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
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t   solenoid_bitmask;    /* Solenoid bits: 0000 0111   */
uint8_t   solenoid_base_code;  /* Subcommand base code       */
uint8_t   solenoid_number;     /* Solenoid number to actuate */
SOL_STATE sol_state;           /* State of solenoid valves   */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
solenoid_bitmask = 0x07;
sol_state        = 0;


/*------------------------------------------------------------------------------
 Command Input Processing
------------------------------------------------------------------------------*/

/* Extract Solenoid base code */
solenoid_base_code = (~solenoid_bitmask) & solenoid_cmd_opcode;

/* Extract Solenoid number */
solenoid_number = solenoid_bitmask & solenoid_cmd_opcode;


/*------------------------------------------------------------------------------
 Call Solenoid API Function
------------------------------------------------------------------------------*/
switch(solenoid_base_code)
	{
	/* Solenoid On */
	case SOL_ON_BASE_CODE:
		{
		solenoid_on(solenoid_number);
		break;
		}

	/* Solenoid Off */
	case SOL_OFF_BASE_CODE:
		{
		solenoid_off(solenoid_number);
		break;
		}

	/* Solenoid Toggle */
	case SOL_TOGGLE_BASE_CODE:
		{
		solenoid_toggle(solenoid_number);
		break;
		}

	/* Solenoid Reset */
	case SOL_RESET_CODE:
		{
		solenoid_reset();
		break;
		}
	
	/* Solenoid GetState */
	case SOL_GETSTATE_CODE:
		{
		sol_state = solenoid_get_state();
		#if   defined( TERMINAL )
			usb_transmit( &sol_state, sizeof( sol_state ), HAL_DEFAULT_TIMEOUT );
		#elif defined( HOTFIRE  ) 
			valve_transmit( &sol_state, sizeof( sol_state ), HAL_DEFAULT_TIMEOUT );
		#endif
		break;
		}

	/* Solenoid open */
	case SOL_OPEN_CODE:
		{
		solenoid_open( solenoid_number );
		break;
		}

	/* Solenoid closed */
	case SOL_CLOSE_CODE:
		{
		solenoid_close( solenoid_number );
		break;
		}

	/* Unrecognized command, do nothing */
	default:
		{
		break;
		}
	}
} /* solenoid_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_on                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Applies power to a specified solenoid                                 *
*                                                                              *
*******************************************************************************/
void solenoid_on
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;


/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin,
                       GPIO_PIN_SET);
} /* solenoid_on */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_off                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Removes power from a specified solenoid                               *
*                                                                              *
*******************************************************************************/
void solenoid_off
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;

/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin,
                       GPIO_PIN_RESET);
} /* solenoid_off */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_toggle                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Toggles the actuation state of a specified solenoid                   *
*                                                                              *
*******************************************************************************/
void solenoid_toggle
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;

/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_TogglePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin);
} /* solenoid_toggle */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_reset                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Resets all solenoids to their default state                           *
*                                                                              *
*******************************************************************************/
void solenoid_reset
	(
	void
	)
{
// Call HAL_GPIO_WritePin for all pins
HAL_GPIO_WritePin(SOL1_GPIO_PORT, SOL1_PIN|SOL2_PIN|SOL3_PIN, GPIO_PIN_RESET);
HAL_GPIO_WritePin(SOL4_GPIO_PORT, SOL4_PIN|SOL5_PIN|SOL6_PIN, GPIO_PIN_RESET);
} /* solenoid_reset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_get_state                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*        Get the state of the solenoids                                        *
*                                                                              *
*******************************************************************************/
SOL_STATE solenoid_get_state
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_GPIO_handle; /* GPIO handles             */
SOL_STATE              sol_state;            /* Return value             */
GPIO_PinState          pinstate;             /* On/off state of GPIO pin */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
sol_state = 0;
pinstate  = GPIO_PIN_RESET;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
for ( uint8_t i = 0; i < NUM_SOLENOIDS; i++ )
	{
	solenoid_map( &solenoid_GPIO_handle, i+1 );
	pinstate = HAL_GPIO_ReadPin( solenoid_GPIO_handle.GPIOx, 
	                             solenoid_GPIO_handle.GPIO_pin );
	if ( pinstate == GPIO_PIN_SET )
		{
		sol_state |= ( 1 << i );
		}
	}
return sol_state;
} /* solenoid_get_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_open                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*        Open Solenoids                                                        *
*                                                                              *
*******************************************************************************/
void solenoid_open 
	(
	uint8_t solenoid_num
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;
GPIO_PinState          pin_state;


/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/

/* Solenoid number to GPIO port/pin mapping */
solenoid_map( &solenoid_on_GPIO_handle, solenoid_num );

/* Determine Solenoid Pinstate */
pin_state = sol_open_states[solenoid_num-1];

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin( solenoid_on_GPIO_handle.GPIOx,
                   solenoid_on_GPIO_handle.GPIO_pin,
                   pin_state );

} /* solenoid_get_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_close                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*        Close solenoids                                                       *
*                                                                              *
*******************************************************************************/
void solenoid_close
	(
	uint8_t solenoid_num
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;
GPIO_PinState          pin_state;


/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/

/* Solenoid number to GPIO port/pin mapping */
solenoid_map( &solenoid_on_GPIO_handle, solenoid_num );

/* Determine Solenoid Pinstate */
pin_state = sol_closed_states[solenoid_num-1];

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin( solenoid_on_GPIO_handle.GPIOx,
                   solenoid_on_GPIO_handle.GPIO_pin,
                   pin_state );

} /* solenoid_close */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/