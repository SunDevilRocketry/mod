/*******************************************************************************
*
* FILE:
* 		valve.c
*
* DESCRIPTION:
* 		Servo valve actuation API	
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#if   defined( VALVE_CONTROLLER  )
	#include "sdr_pin_defines_L0005.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#endif
#include "stm32h7xx_hal.h"
#include "main.h"
#include "valve.h"
#include "usb.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

#ifdef VALVE_CONTROLLER
/* Encoder variables */
volatile static int32_t  lox_valve_pos       = 0;  /* LOX Valve Encoder count  */
volatile static bool     lox_channelA_state  = ENCODER_LOW; /* Voltage on channel 
                                                             A pin */
volatile static bool     lox_channelB_state  = ENCODER_LOW; /* Voltage on Channel 
                                                             B pin */
volatile static int32_t  fuel_valve_pos      = 0;  /* Fuel Valve Encoder count */
volatile static bool     fuel_channelA_state = ENCODER_LOW; /* Voltage on channel 
                                                             A pin */
volatile static bool     fuel_channelB_state = ENCODER_LOW; /* Voltage on Channel 
                                                             B pin */

/* Stepper Driver States */
static STEPPER_DRIVER_STATE lox_driver_state;
static STEPPER_DRIVER_STATE fuel_driver_state;

/* Valve openining/closing states */
volatile static bool ox_valve_closing    = false; /* LOX valve is closing   */
volatile static bool ox_valve_opening    = false; /* LOX valve is opening   */
volatile static bool fuel_valve_closing  = false; /* Fuel valve is closing  */
volatile static bool fuel_valve_opening  = false; /* Fuel valve is opening  */
volatile static bool ox_valve_cracking   = false; /* LOX valve is cracking  */
volatile static bool fuel_valve_cracking = false; /* Fuel valve is cracking */
#endif /* #ifdef VALVE_CONTROLLER */


/*------------------------------------------------------------------------------
 Internal Function Prototypes 
------------------------------------------------------------------------------*/

#ifdef VALVE_CONTROLLER
/* Enable the lox stepper motor driver */
static void lox_driver_enable
	(
	void
	);

/* Enable the fuel stepper motor driver */
static void fuel_driver_enable
	(
	void
	);

/* Disable the lox stepper motor driver */
static void lox_driver_disable
	(
	void
	);

/* Disable the fuel stepper motor driver */
static void fuel_driver_disable
	(
	void
	);

/* Set the lox stepper motor direction */
static VALVE_STATUS lox_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	);

/* Set the fuel stepper motor direction */
static VALVE_STATUS fuel_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	);

/* Increase the lox encoder count by one */
static void inc_lox_encoder
	(
	void
	);


/* Increase the fuel encoder count by one */
static void inc_fuel_encoder
	(
	void
	);


/* Decrease the lox encoder count by one */
static void dec_lox_encoder
	(
	void
	);


/* Decrease the fuel encoder count by one */
static void dec_fuel_encoder
	(
	void
	);

#endif


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

#ifdef VALVE_CONTROLLER
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Execute a valve subcommand                                             *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_cmd_execute
	(
	uint8_t subcommand
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t           valve_num;         /* Valve number, 0 -> ox, 1 -> fuel   */
VALVE_STATUS      valve_status[2];   /* Valve return codes                 */
MAIN_VALVE_STATES main_valve_states; /* Main valve open/close states       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_num         = subcommand & 0x01;
subcommand       -= valve_num;
main_valve_states = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch( subcommand )
	{
	/*--------------------------------------------------------------------------
	 VALVE ENABLE	
	--------------------------------------------------------------------------*/
	case VALVE_ENABLE_CODE:
		{
		valve_enable_valves();
		return VALVE_OK;
		} /* VALVE_ENABLE_CODE*/

	/*--------------------------------------------------------------------------
	 VALVE DISABLE 
	--------------------------------------------------------------------------*/
	case VALVE_DISABLE_CODE:
		{
		valve_disable_valves();
		return VALVE_OK;
		} /* VALVE_DISABLE_CODE */

	/*--------------------------------------------------------------------------
	 VALVE OPEN 
	--------------------------------------------------------------------------*/
	case VALVE_OPEN_CODE:
		{
		if ( valve_num )
			{
			return valve_open_fuel_valve();
			}
		else
			{
			return valve_open_ox_valve();
			}
		} /* VALVE_OPEN_CODE */

	/*--------------------------------------------------------------------------
	 VALVE CLOSE 
	--------------------------------------------------------------------------*/
	case VALVE_CLOSE_CODE:
		{
		if ( valve_num )
			{
			return valve_close_fuel_valve();
			}
		else
			{
			return valve_close_ox_valve();
			}
		} /* VALVE_CLOSE_CODE */

	/*--------------------------------------------------------------------------
	 VALVE CALIBRATE 
	--------------------------------------------------------------------------*/
	case VALVE_CALIBRATE_CODE:
		{
		return valve_calibrate_valves();
		} /* VALVE_CALIBRATE_CODE */

	/*--------------------------------------------------------------------------
	 VALVE CRACK 
	--------------------------------------------------------------------------*/
	case VALVE_CRACK_CODE:
		{
		if ( valve_num )
			{
			return valve_crack_fuel_valve();
			}
		else
			{
			return valve_crack_ox_valve();
			}
		} /* VALVE_CRACK_CODE */

	/*--------------------------------------------------------------------------
	 VALVE RESET 
	--------------------------------------------------------------------------*/
	case VALVE_RESET_CODE:
		{
		valve_status[0] = valve_close_ox_valve  ();
		valve_status[1] = valve_close_fuel_valve();
		if ( ( valve_status[0] != VALVE_OK ) || ( valve_status[1] != VALVE_OK ) )
			{
			return VALVE_ERROR;
			}
		else
			{
			return VALVE_OK;
			}
		} /* VALVE_RESET_CODE */

	/*--------------------------------------------------------------------------
	 VALVE OPENALL 
	--------------------------------------------------------------------------*/
	case VALVE_OPENALL_CODE:
		{
		valve_status[0] = valve_open_ox_valve  ();
		valve_status[1] = valve_open_fuel_valve();
		if ( ( valve_status[0] != VALVE_OK ) || ( valve_status[1] != VALVE_OK ) )
			{
			return VALVE_ERROR;
			}
		else
			{
			return VALVE_OK;
			}
		} /* VALVE_OPENALL_CODE */
	
	/*--------------------------------------------------------------------------
	 VALVE GETSTATE 
	--------------------------------------------------------------------------*/
	case VALVE_GETSTATE_CODE:
		{
		main_valve_states = valve_get_valve_states();
		#if defined( HOTFIRE )
			valve_transmit( &main_valve_states         , 
			                sizeof( main_valve_states ), 
							HAL_DEFAULT_TIMEOUT );
		#elif defined( TERMINAL )
			usb_transmit( &main_valve_states, 
			              sizeof( main_valve_states ),
						  HAL_DEFAULT_TIMEOUT );
		#endif
		}

	/*--------------------------------------------------------------------------
	 UNRECOGNIZED SUBCOMMAND 
	--------------------------------------------------------------------------*/
	default:
		{
		return VALVE_UNRECOGNIZED_SUBCOMMAND;
		}
	
	} /* switch( subcommand ) */

} /* valve_cmd_execute */
#endif


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_transmit                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Transmits a specified number of bytes over the valve control           *
*       serial port                                                            *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_transmit
	(
	void*    tx_data_ptr , /* Data to send          */
	size_t   tx_data_size, /* Size of transmit data */
	uint32_t timeout
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return code from HAL CMSIS API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Transmit buffer */
hal_status = HAL_UART_Transmit( &( VALVE_HUART ), 
                                tx_data_ptr     , 
								tx_data_size    , 
								timeout );

/* Return status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return VALVE_UART_TIMEOUT;
		}
	case HAL_OK:
		{
		return VALVE_OK;
		}
	default:
		{
		return VALVE_UART_ERROR;
		}
	} /* switch ( hal_status ) */

} /* valve_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_receive                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Receive bytes from the valve control serial port                       *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_receive
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout                    */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return code from HAL CMSIS API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Transmit buffer */
hal_status = HAL_UART_Receive( &( VALVE_HUART ), 
                               rx_data_ptr     , 
							   rx_data_size    , 
							   timeout );

/* Return status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return VALVE_UART_TIMEOUT;
		}
	case HAL_OK:
		{
		return VALVE_OK;
		}
	default:
		{
		return VALVE_UART_ERROR;
		}
	} /* switch ( hal_status ) */

} /* valve_receive */

#ifdef VALVE_CONTROLLER
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_enable_valves                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable valve drivers to actuate valves                                 *
*                                                                              *
*******************************************************************************/
void valve_enable_valves
	(
	void
	)
{
/* Enable the oxidizer valve */
lox_driver_enable();

/* Enable the fuel valve     */
fuel_driver_enable();

} /* valve_enable_valves */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_disable_valves                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable valve drivers to actuate valves                                *
*                                                                              *
*******************************************************************************/
void valve_disable_valves
	(
	void
	)
{
/* Disable the oxidizer valve */
lox_driver_disable();

/* Disable the fuel valve     */
fuel_driver_disable();

} /* valve_disable_valves */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_open_ox_valve                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Open the main oxidizer valve                                           *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_open_ox_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check if valve is already open */
if ( lox_valve_pos == VALVE_OPEN_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = lox_driver_set_direction( STEPPER_DRIVER_CW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve   */
ox_valve_opening = true;
HAL_TIM_PWM_Start( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
return VALVE_OK;
} /* valve_open_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_open_fuel_valve                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Open the main fuel valve                                               *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_open_fuel_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check if valve is already open */
if ( fuel_valve_pos == VALVE_OPEN_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = fuel_driver_set_direction( STEPPER_DRIVER_CW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve   */
fuel_valve_opening = true;
HAL_TIM_PWM_Start( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
return VALVE_OK;
} /* valve_open_fuel_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_close_ox_valve                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Close the main oxidizer valve                                          *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_close_ox_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check if valve is already closed */
if ( lox_valve_pos == VALVE_CLOSED_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = lox_driver_set_direction( STEPPER_DRIVER_CCW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve   */
ox_valve_closing = true;
HAL_TIM_PWM_Start( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
return VALVE_OK;
} /* valve_close_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_close_fuel_valve                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Close the main fuel valve                                              *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_close_fuel_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* check if valve is already closed */
if ( fuel_valve_pos == VALVE_CLOSED_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = fuel_driver_set_direction( STEPPER_DRIVER_CCW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve */
fuel_valve_closing = true;
HAL_TIM_PWM_Start( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
return VALVE_OK;
} /* valve_close_fuel_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_crack_ox_valve                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Slightly open the main oxidizer valve                                  *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_crack_ox_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check if valve is already open */
if ( lox_valve_pos == VALVE_OPEN_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = lox_driver_set_direction( STEPPER_DRIVER_CW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve   */
ox_valve_cracking = true;
HAL_TIM_PWM_Start( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
return VALVE_OK;
} /* valve_crack_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_crack_fuel_valve                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Slightly open the main fuel valve                                      *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_crack_fuel_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check if valve is already open */
if ( fuel_valve_pos == VALVE_OPEN_POS )
	{
	return VALVE_OK;
	}

/* Set the direction   */
valve_status = fuel_driver_set_direction( STEPPER_DRIVER_CW );
if ( valve_status != VALVE_OK )
	{
	return valve_status;
	}

/* Actuate the valve   */
fuel_valve_cracking = true;
HAL_TIM_PWM_Start( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
return VALVE_OK;
} /* valve_crack_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_ox_valve_pos                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the position of the main oxidizer valve                            *
*                                                                              *
*******************************************************************************/
int32_t valve_get_ox_valve_pos
	(
	void
	)
{
return lox_valve_pos*360/1000;
} /* valve_get_ox_valve_pos */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_fuel_valve_pos                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the position of the main fuel valve                                *
*                                                                              *
*******************************************************************************/
int32_t valve_get_fuel_valve_pos
	(
	void
	)
{
return fuel_valve_pos*360/1000;
} /* valve_get_fuel_valve_pos */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_ox_valve_state                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the state of the main oxidizer valve                               *
*                                                                              *
*******************************************************************************/
VALVE_STATE valve_get_ox_valve_state
	(
	void
	)
{
if ( HAL_GPIO_ReadPin( PHOTOGATE_GPIO_PORT, LOX_PHOTOGATE_PIN ) == PHOTOGATE_STATE_LOW )
	{
	return VALVE_OPEN;
	}
else
	{
	return VALVE_CLOSED;
	}
} /* valve_get_ox_valve_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_fuel_valve_state                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the status of the main fuel valve                                  *
*                                                                              *
*******************************************************************************/
VALVE_STATE valve_get_fuel_valve_state
	(
	void
	)
{
if ( HAL_GPIO_ReadPin( PHOTOGATE_GPIO_PORT, FUEL_PHOTOGATE_PIN ) == PHOTOGATE_STATE_LOW )
	{
	return VALVE_OPEN;
	}
else
	{
	return VALVE_CLOSED;
	}
} /* valve_get_fuel_valve_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_calibrate_valves                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calibrate initial valve positions                                      *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_calibrate_valves
	(
	void
	)
{
/* Calibrate the oxidizer valve  */
lox_driver_enable();
lox_driver_set_direction( STEPPER_DRIVER_CW );
while ( valve_get_ox_valve_state() == VALVE_OPEN )
	{
	HAL_TIM_PWM_Start( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
	}

HAL_Delay( 5 );
HAL_TIM_PWM_Stop( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
lox_valve_pos = 0;

/* Calibrate the fuel valve      */
fuel_driver_enable();
fuel_driver_set_direction( STEPPER_DRIVER_CCW );
while ( valve_get_fuel_valve_state() == VALVE_OPEN )
	{
	HAL_TIM_PWM_Start( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
	}
HAL_Delay( 5 );
HAL_TIM_PWM_Stop( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
fuel_valve_pos = 0;

return VALVE_OK;
} /* valve_calibrate_valves */
#endif /* #ifdef VALVE_CONTROLLER */

/*------------------------------------------------------------------------------
 Interrupt Service Routines 
------------------------------------------------------------------------------*/

#ifdef VALVE_CONTROLLER
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_channelA_ISR                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       LOX Main Valve Encoder Channel A Interrupt                             *
*                                                                              *
*******************************************************************************/
void lox_channelA_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( LOX_ENC_GPIO_PORT, LOX_ENC_A_PIN ) )
	{
	lox_channelA_state = ENCODER_HIGH;
	if ( !lox_channelB_state )
		{
		//lox_valve_pos -= 1;
		dec_lox_encoder();

		/* Detect valve closed position */
		if ( ox_valve_closing && ( lox_valve_pos == VALVE_CLOSED_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
			ox_valve_closing = false;
			}
		/*
		if ( ox_valve_closing && ( valve_get_ox_valve_state() == VALVE_CLOSED ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
			ox_valve_closing = false;
			lox_valve_pos = 0;
			}
		*/
		}
	}
/* High to Low Transition */
else
	{
	lox_channelA_state = ENCODER_LOW;
	}
} /* lox_channelA_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_channelB_ISR                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       LOX Main Valve Encoder Channel B Interrupt                             *
*                                                                              *
*******************************************************************************/
void lox_channelB_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( LOX_ENC_GPIO_PORT, LOX_ENC_B_PIN ) )
	{
	lox_channelB_state = ENCODER_HIGH;
	if ( !lox_channelA_state )
		{
		//lox_valve_pos += 1;
		inc_lox_encoder();

		/* Detect valve open */
		if ( ox_valve_opening && ( lox_valve_pos == VALVE_OPEN_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL );
			ox_valve_opening = false;
			}
		/* Detect valve cracked */
		else if ( ox_valve_cracking && ( lox_valve_pos == VALVE_CRACKED_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_LOX_TIM ), VALVE_LOX_TIM_CHANNEL  );
			ox_valve_cracking = false;
			}
		}
	}
/* High to Low Transition */
else
	{
	lox_channelB_state = ENCODER_LOW;
	}

} /* lox_channelB_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_channelA_ISR                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fuel Main Valve Encoder Channel A Interrupt                            *
*                                                                              *
*******************************************************************************/
void fuel_channelA_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( KER_ENC_GPIO_PORT, KER_ENC_A_PIN ) )
	{
	fuel_channelA_state = ENCODER_HIGH;
	if ( !fuel_channelB_state )
		{
		//fuel_valve_pos -= 1;
		dec_fuel_encoder();

		/* Detect valve closed */
		if ( fuel_valve_closing && ( fuel_valve_pos == VALVE_CLOSED_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
			fuel_valve_closing = false;
			}
		}
	}
/* High to Low Transition */
else
	{
	fuel_channelA_state = ENCODER_LOW;
	}
} /* fuel_channelA_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_channelB_ISR                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fuel Main Valve Encoder Channel B Interrupt                            *
*                                                                              *
*******************************************************************************/
void fuel_channelB_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( KER_ENC_GPIO_PORT, KER_ENC_B_PIN ) )
	{
	fuel_channelB_state = ENCODER_HIGH;
	if ( !fuel_channelA_state )
		{
		//fuel_valve_pos += 1;
		inc_fuel_encoder();

		/* Detect valve open */
		if      ( fuel_valve_opening && ( fuel_valve_pos == VALVE_OPEN_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
			fuel_valve_opening = false;
			}
		/* Detect valve crakced */
		else if ( fuel_valve_cracking && ( fuel_valve_pos == VALVE_CRACKED_POS ) )
			{
			HAL_TIM_PWM_Stop( &( VALVE_FUEL_TIM ), VALVE_FUEL_TIM_CHANNEL );
			fuel_valve_cracking = false;
			}
		}
	}
/* High to Low Transition */
else
	{
	fuel_channelB_state = ENCODER_LOW;
	}

} /* fuel_channelB_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_enable                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable the lox stepper motor driver                                    *
*                                                                              *
*******************************************************************************/
static void lox_driver_enable
	(
	void
	)
{
/* Set GPIO State      */
HAL_GPIO_WritePin( LOX_EN_GPIO_PORT, LOX_EN_PIN, GPIO_PIN_SET );

/* Update global state */
lox_driver_state.enable = STEPPER_DRIVER_ENABLED;

} /* lox_driver_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable the fuel stepper motor driver                                   *
*                                                                              *
*******************************************************************************/
static void fuel_driver_enable
	(
	void
	)
{
/* Set GPIO level           */
HAL_GPIO_WritePin( KER_EN_GPIO_PORT, KER_EN_PIN, GPIO_PIN_RESET );

/* Update the globale state */
fuel_driver_state.enable = STEPPER_DRIVER_ENABLED;

} /* fuel_driver_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_disable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable the lox stepper motor driver                                   *
*                                                                              *
*******************************************************************************/
static void lox_driver_disable
	(
	void
	)
{
/* Set GPIO Level          */
HAL_GPIO_WritePin( LOX_EN_GPIO_PORT, LOX_EN_PIN, GPIO_PIN_SET );

/* Update the global state */
lox_driver_state.enable = STEPPER_DRIVER_DISABLED;

} /* lox_driver_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable the fuel stepper motor driver                                  *
*                                                                              *
*******************************************************************************/
static void fuel_driver_disable
	(
	void
	)
{
/* Set GPIO level          */
HAL_GPIO_WritePin( KER_EN_GPIO_PORT, KER_EN_PIN, GPIO_PIN_SET );

/* Update the global state */
fuel_driver_state.enable = STEPPER_DRIVER_DISABLED;

} /* fuel_driver_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_set_direction                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the lox stepper motor direction                                    *
*                                                                              *
*******************************************************************************/
static VALVE_STATUS lox_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	)
{
/* Set the GPIO level      */
if      ( direction == STEPPER_DRIVER_CW  )
	{
	HAL_GPIO_WritePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN, GPIO_PIN_RESET );
	}
else if ( direction == STEPPER_DRIVER_CCW )
	{
	HAL_GPIO_WritePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN, GPIO_PIN_SET );
	}
else
	{
	/* Invalid direction */
	return VALVE_INVALID_DIR;
	}

/* Update the global state */
lox_driver_state.direction = direction;
return VALVE_OK;
} /* lox_driver_set_direction */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_set_direction                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the fuel stepper motor direction                                   *
*                                                                              *
*******************************************************************************/
static VALVE_STATUS fuel_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	)
{
/* Set the GPIO level      */
if      ( direction == STEPPER_DRIVER_CW  )
	{
	HAL_GPIO_WritePin( KER_DIR_GPIO_PORT, KER_DIR_PIN, GPIO_PIN_RESET );
	}
else if ( direction == STEPPER_DRIVER_CCW )
	{
	HAL_GPIO_WritePin( KER_DIR_GPIO_PORT, KER_DIR_PIN, GPIO_PIN_SET );
	}
else
	{
	return VALVE_INVALID_DIR;
	}

/* Update the global state */
fuel_driver_state.direction = direction;
return VALVE_OK;
} /* fuel_driver_set_direction */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_valve_states                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the state of both main valves                                      *
*                                                                              *
*******************************************************************************/
MAIN_VALVE_STATES valve_get_valve_states
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
MAIN_VALVE_STATES main_valve_states; /* Return valve        */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
main_valve_states = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check the LOX valve  */
if ( valve_get_ox_valve_state() == VALVE_OPEN )
	{
	main_valve_states |= ( 1 << 7 );
	}

/* Check the fuel valve */
if ( valve_get_fuel_valve_state() == VALVE_OPEN )
	{
	main_valve_states |= ( 1 << 6 );
	}

return main_valve_states;
} /* valve_get_valve_states */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		inc_lox_encoder                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Increase the lox encoder count by one                                  *
*                                                                              *
*******************************************************************************/
static void inc_lox_encoder
	(
	void
	)
{
if ( lox_valve_pos == 999 )
	{
	lox_valve_pos = 0;
	}
else
	{
	lox_valve_pos += 1;
	}
} /* inc_lox_encoder */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		inc_fuel_encoder                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Increase the fuel encoder count by one                                 *
*                                                                              *
*******************************************************************************/
static void inc_fuel_encoder
	(
	void
	)
{
if ( fuel_valve_pos == 999 )
	{
	fuel_valve_pos = 0;
	}
else
	{
	fuel_valve_pos += 1;
	}

} /* inc_fuel_encoder */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		dec_lox_encoder                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Decrease the lox encoder count by one                                  *
*                                                                              *
*******************************************************************************/
static void dec_lox_encoder
	(
	void
	)
{
if ( lox_valve_pos == 0 )
	{
	lox_valve_pos = 999;
	}
else
	{
	lox_valve_pos -= 1;
	}

} /* dec_lox_encoder */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		dec_fuel_encoder                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Decrease the fuel encoder count by one                                 *
*                                                                              *
*******************************************************************************/
static void dec_fuel_encoder
	(
	void
	)
{
if ( fuel_valve_pos == 0 )
	{
	fuel_valve_pos = 999;
	}
else
	{
	fuel_valve_pos -= 1;
	}

} /* dec_fuel_encoder */
#endif /* #ifdef VALVE_CONTROLLER */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/