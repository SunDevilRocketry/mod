/*******************************************************************************
*
* FILE: 
* 		valve.h
*
* DESCRIPTION: 
* 		Servo valve actuation API	
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VALVE_H 
#define VALVE_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Encoder states */
#define ENCODER_HIGH              true
#define ENCODER_LOW               false

/* Photogate states */
#define PHOTOGATE_STATE_LOW       GPIO_PIN_RESET
#define PHOTOGATE_STATE_HIGH      GPIO_PIN_SET

/* Valve open/close positions */
#define VALVE_CLOSED_POS          0
#define VALVE_OPEN_POS            250
#define VALVE_CRACKED_POS         71 /* 29% */

/* Subcommand codes */
#define VALVE_ENABLE_CODE         0x00
#define VALVE_DISABLE_CODE        0x02
#define VALVE_OPEN_CODE           0x04
#define VALVE_CLOSE_CODE          0x06
#define VALVE_CALIBRATE_CODE      0x08
#define VALVE_CRACK_CODE          0x0A
#define VALVE_RESET_CODE          0x10
#define VALVE_OPENALL_CODE        0x12
#define VALVE_GETSTATE_CODE       0x14


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return Codes */
typedef enum _VALVE_STATUS
	{
	VALVE_OK                     ,
	VALVE_INVALID_DIR            ,    /* Invalid motor direction    */
	VALVE_UNRECOGNIZED_SUBCOMMAND,    /* Unrecognized subcommand    */
	VALVE_UART_ERROR             ,    /* Unknown UART error         */
	VALVE_UART_TIMEOUT           ,    /* Valve control UART timeout */
	VALVE_ERROR
	} VALVE_STATUS;

/* Stepper driver enable states */
typedef enum _STEPPER_DRIVER_EN_STATE
	{
	STEPPER_DRIVER_ENABLED,
	STEPPER_DRIVER_DISABLED
	} STEPPER_DRIVER_EN_STATE;

/* Stepper driver rotation directions */
typedef enum _STEPPER_DRIVER_DIR_STATE
	{
	STEPPER_DRIVER_CCW, /* Counter clockwise */
	STEPPER_DRIVER_CW   /* Clockwise         */
	} STEPPER_DRIVER_DIR_STATE;

/* Stepper Motor driver state */
typedef struct _STEPPER_DRIVER_STATE
	{
	STEPPER_DRIVER_EN_STATE  enable;    /* Motor enabled      */
	STEPPER_DRIVER_DIR_STATE direction; /* Rotation Direction */
	} STEPPER_DRIVER_STATE; 

/* Valve States */
typedef enum _VALVE_STATE
	{
	VALVE_OPEN,
	VALVE_CLOSED
	} VALVE_STATE;

/* Encoding for communicating the state of both valves */
/* bit    7: Main fuel valve (1 open/0 closed)
   bit    6: Main lox valve  (1 open/0 closed) 
   bits 5-0: Unused */
typedef uint8_t MAIN_VALVE_STATES;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

#ifdef VALVE_CONTROLLER
/* Execute a valve subcommand */
VALVE_STATUS valve_cmd_execute
	(
	uint8_t    subcommand    /* sdec subcommand  */
	);
#endif

/* Transmits a specified number of bytes over the valve control serial port */
VALVE_STATUS valve_transmit
	(
	void*    tx_data_ptr , /* Data to send          */
	size_t   tx_data_size, /* Size of transmit data */
	uint32_t timeout
	);

/* Receive bytes from the valve control serial port */
VALVE_STATUS valve_receive
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout                    */
	);

#ifdef VALVE_CONTROLLER
/* Enable valve drivers to actuate valves */
void valve_enable_valves
	(
	void
	);

/* Disable valve drivers to actuate valves */
void valve_disable_valves
	(
	void
	);

/* Open the main oxidizer valve */
VALVE_STATUS valve_open_ox_valve
	(
	void
	);

/* Open the main fuel valve */
VALVE_STATUS valve_open_fuel_valve
	(
	void
	);

/* Close the main oxidizer valve */
VALVE_STATUS valve_close_ox_valve
	(
	void
	);

/* Close the main fuel valve */
VALVE_STATUS valve_close_fuel_valve
	(
	void
	);

/* Crack the oxidizer valve */
VALVE_STATUS valve_crack_ox_valve
	(
	void
	);

/* Crack the fuel valve */ 
VALVE_STATUS valve_crack_fuel_valve
	(
	void
	);

/* LOX Main Valve Encoder Channel A Interrupt */
void lox_channelA_ISR
	(
	void
	);

/* LOX Main Valve Encoder Channel B Interrupt */
void lox_channelB_ISR
	(
	void
	);

/* Fuel Main Valve Encoder Channel A Interrupt */
void fuel_channelA_ISR
	(
	void
	);

/* Fuel Main Valve Encoder Channel B Interrupt */
void fuel_channelB_ISR
	(
	void
	);

/* Get the position of the main oxidizer valve */
int32_t valve_get_ox_valve_pos
	(
	void
	);

/* Get the position of the main fuel valve */
int32_t valve_get_fuel_valve_pos
	(
	void
	);

/* Get the position of the main fuel valve */
int32_t valve_get_fuel_valve_pos
	(
	void
	);

/* Get the state of the main oxidizer valve */
VALVE_STATE valve_get_ox_valve_state
	(
	void
	);

/* Get the status of the main fuel valve */
VALVE_STATE valve_get_fuel_valve_state
	(
	void
	);

/* Calibrate initial valve positions */
VALVE_STATUS valve_calibrate_valves
	(
	void
	);

/* Get the state of both main valves */
MAIN_VALVE_STATES valve_get_valve_states
	(
	void
	);

#endif /* #ifdef VALVE_CONTROLLER */

#ifdef __cplusplus
}
#endif

#endif /* VALVE_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/