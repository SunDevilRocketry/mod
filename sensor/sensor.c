/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#elif defined( VALVE_CONTROLLER     )
	#include "sdr_pin_defines_L0005.h"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
	#include "baro.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "baro.h"
#endif
#include "usb.h"
#include "sensor.h"
#if defined( ENGINE_CONTROLLER )
	#include "pressure.h"
	#include "loadcell.h"
	#include "temp.h"
#endif
#if defined( VALVE_CONTROLLER  )
	#include "valve.h"
#endif


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Hash table of sensor readout sizes and offsets */
static SENSOR_DATA_SIZE_OFFSETS sensor_size_offsets_table[ NUM_SENSORS ];

extern GPS_DATA gps_data;

#ifdef FLIGHT_COMPUTER
extern uint32_t tdelta, previous_time;
extern IMU_OFFSET imu_offset;
#endif


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Sensor ID to size and pointer mapping */
void static sensor_map
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID    sensor_id      ,
	uint8_t**    sensorid_pptr  ,
	size_t*      sensor_size
	);

/* Extract bytes for export from SENSOR_ID struct */
void static extract_sensor_bytes 
	(
	SENSOR_DATA* sensor_data_ptr      ,
	SENSOR_ID*   sensor_ids_ptr       ,
	uint8_t      num_sensors          ,
	uint8_t*     sensor_data_bytes_ptr,
	uint8_t*     num_sensor_bytes
	);

/* reads from all sensors using the MCUs ADCs */
#ifdef L0002_REV5
SENSOR_STATUS sensor_adc_burst_read
	(
	SENSOR_DATA* sensor_data_ptr /* Pointer to sensor data structure */
	);

/* Mapping from pressure transducer number to mutliplexor GPIO pin */
static inline uint16_t mux_map
	(
    PRESSURE_PT_NUM    pt_num    
    );
#endif

#ifdef L0002_REV5
/* Select the adc channel for PT1 */
static void pt1_adc_channel_select
	(
	void
	);

/* Select the adc channel for PT7 */
static void pt7_adc_channel_select
	(
	void
	);

/* Select the adc channel for the loadcell */
static void loadcell_adc_channel_select
	(
	void
	);

/* Select the adc channel for the pt8 */
static void pt8_adc_channel_select
	(
	void
	);

/* Select the adc channel for the pt5 */
static void pt5_adc_channel_select
	(
	void
	);

/* Select the adc channel for the pt6 */
static void pt6_adc_channel_select
	(
	void
	);
#endif /* #ifdef L0002_REV5 */


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_init                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the sensor module                                           *
*                                                                              *
*******************************************************************************/
void sensor_init 
	(
	void
	)
{
/* Setup the sensor id hash table */
#if defined( FLIGHT_COMPUTER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].offset = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].offset = 4;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].offset = 6;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].offset = 8;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].offset = 10; /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].offset = 12; /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].offset = 14; /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].offset = 16; /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].offset = 18; /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].offset = 20; /* SENSOR_ACCX_CONV  */
	sensor_size_offsets_table[ 11 ].offset = 24; /* SENSOR_ACCY_CONV  */
	sensor_size_offsets_table[ 12 ].offset = 28; /* SENSOR_ACCZ_CONV  */
	sensor_size_offsets_table[ 13 ].offset = 32; /* SENSOR_GYROX_CONV  */
	sensor_size_offsets_table[ 14 ].offset = 36; /* SENSOR_GYROY_CONV  */
	sensor_size_offsets_table[ 15 ].offset = 40; /* SENSOR_GYROZ_CONV  */
	sensor_size_offsets_table[ 16 ].offset = 44; /* SENSOR_ROLL_DEG  */
	sensor_size_offsets_table[ 17 ].offset = 48; /* SENSOR_PITCH_DEG  */
	sensor_size_offsets_table[ 18 ].offset = 52; /* SENSOR_ROLL_RATE  */
	sensor_size_offsets_table[ 19 ].offset = 56; /* SENSOR_PITCH_RATE  */
	sensor_size_offsets_table[ 20 ].offset = 60; /* SENSOR_VELOCITY  */
	sensor_size_offsets_table[ 21 ].offset = 64; /* SENSOR_VELO_X  */
	sensor_size_offsets_table[ 22 ].offset = 68; /* SENSOR_VELO_Y  */
	sensor_size_offsets_table[ 23 ].offset = 72; /* SENSOR_VELO_Z  */
	sensor_size_offsets_table[ 24 ].offset = 76; /* SENSOR_POSITION  */
	sensor_size_offsets_table[ 25 ].offset = 80; /* SENSOR_BARO_PRES */
	sensor_size_offsets_table[ 26 ].offset = 84; /* SENSOR_BARO_TEMP  */
	sensor_size_offsets_table[ 27 ].offset = 88; /* SENSOR_BARO_ALT  */
	sensor_size_offsets_table[ 28 ].offset = 92; /* SENSOR_BARO_VELO  */
	sensor_size_offsets_table[ 29 ].offset = 96; /* SENSOR_GPS_ALT  */
	sensor_size_offsets_table[ 30 ].offset = 100; /* SENSOR_GPS_SPEED  */
	sensor_size_offsets_table[ 31 ].offset = 104; /* SENSOR_GPS_TIME  */
	sensor_size_offsets_table[ 32 ].offset = 108; /* SENSOR_GPS_LONG  */
	sensor_size_offsets_table[ 33 ].offset = 112; /* SENSOR_GPS_LAT  */
	sensor_size_offsets_table[ 34 ].offset = 116; /* SENSOR_GPS_NS  */
	sensor_size_offsets_table[ 35 ].offset = 117; /* SENSOR_GPS_EW  */
	sensor_size_offsets_table[ 36 ].offset = 118; /* SENSOR_GPS_GLL  */
	sensor_size_offsets_table[ 37 ].offset = 119; /* SENSOR_GPS_RMC  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 2;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].size   = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].size   = 2;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].size   = 2;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].size   = 2;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].size   = 2;  /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].size   = 2;  /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].size   = 2;  /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].size   = 2;  /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].size   = 2;  /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].size   = 4; /* SENSOR_ACCX_CONV  */
	sensor_size_offsets_table[ 11 ].size 	= 4; /* SENSOR_ACCY_CONV  */
	sensor_size_offsets_table[ 12 ].size 	= 4; /* SENSOR_ACCZ_CONV  */
	sensor_size_offsets_table[ 13 ].size	= 4; /* SENSOR_GYROX_CONV  */
	sensor_size_offsets_table[ 14 ].size 	= 4; /* SENSOR_GYROY_CONV  */
	sensor_size_offsets_table[ 15 ].size	= 4; /* SENSOR_GYROZ_CONV  */
	sensor_size_offsets_table[ 16 ].size	= 4; /* SENSOR_ROLL_DEG  */
	sensor_size_offsets_table[ 17 ].size	= 4; /* SENSOR_PITCH_DEG  */
	sensor_size_offsets_table[ 18 ].size	= 4; /* SENSOR_ROLL_RATE  */
	sensor_size_offsets_table[ 19 ].size	= 4; /* SENSOR_PITCH_RATE  */
	sensor_size_offsets_table[ 20 ].size	= 4; /* VELOCITY  */
	sensor_size_offsets_table[ 21 ].size	= 4; /* VELO_X  */
	sensor_size_offsets_table[ 22 ].size	= 4; /* VELO_Y  */
	sensor_size_offsets_table[ 23 ].size	= 4; /* VELO_Z  */
	sensor_size_offsets_table[ 24 ].size	= 4; /* POSITION  */
	sensor_size_offsets_table[ 25 ].size	= 4; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 26 ].size	= 4; /* SENSOR_TEMP  */
	sensor_size_offsets_table[ 27 ].size	= 4; /* SENSOR_BARO_ALT  */
	sensor_size_offsets_table[ 28 ].size	= 4; /* SENSOR_BARO_VELO  */
	sensor_size_offsets_table[ 29 ].size	= 4; /* SENSOR_GPS_ALT  */
	sensor_size_offsets_table[ 30 ].size	= 4; /* SENSOR_GPS_SPEED  */
	sensor_size_offsets_table[ 31 ].size	= 4; /* SENSOR_GPS_TIME  */
	sensor_size_offsets_table[ 32 ].size	= 4; /* SENSOR_GPS_LONG  */
	sensor_size_offsets_table[ 33 ].size	= 4; /* SENSOR_GPS_LAT  */
	sensor_size_offsets_table[ 34 ].size	= 1; /* SENSOR_GPS_NS  */
	sensor_size_offsets_table[ 35 ].size	= 1; /* SENSOR_GPS_EW  */
	sensor_size_offsets_table[ 36 ].size	= 1; /* SENSOR_GPS_GLL  */
	sensor_size_offsets_table[ 37 ].size	= 1; /* SENSOR_GPS_RMC  */


	
#elif defined( ENGINE_CONTROLLER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_PT0  */
	sensor_size_offsets_table[ 1  ].offset = 4;  /* SENSOR_PT1  */
	sensor_size_offsets_table[ 2  ].offset = 8;  /* SENSOR_PT2  */
	sensor_size_offsets_table[ 3  ].offset = 12; /* SENSOR_PT3  */
	sensor_size_offsets_table[ 4  ].offset = 16; /* SENSOR_PT4  */
	sensor_size_offsets_table[ 5  ].offset = 20; /* SENSOR_PT5  */
	sensor_size_offsets_table[ 6  ].offset = 24; /* SENSOR_PT6  */
	sensor_size_offsets_table[ 7  ].offset = 28; /* SENSOR_PT7  */
	sensor_size_offsets_table[ 8  ].offset = 36; /* SENSOR_TC   */
	sensor_size_offsets_table[ 9  ].offset = 32; /* SENSOR_LC   */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 4;  /* SENSOR_PT0  */
	sensor_size_offsets_table[ 1  ].size   = 4;  /* SENSOR_PT1  */
	sensor_size_offsets_table[ 2  ].size   = 4;  /* SENSOR_PT2  */
	sensor_size_offsets_table[ 3  ].size   = 4;  /* SENSOR_PT3  */
	sensor_size_offsets_table[ 4  ].size   = 4;  /* SENSOR_PT4  */
	sensor_size_offsets_table[ 5  ].size   = 4;  /* SENSOR_PT5  */
	sensor_size_offsets_table[ 6  ].size   = 4;  /* SENSOR_PT6  */
	sensor_size_offsets_table[ 7  ].size   = 4;  /* SENSOR_PT7  */
	sensor_size_offsets_table[ 8  ].size   = 4;  /* SENSOR_TC   */
	sensor_size_offsets_table[ 9  ].size   = 4;  /* SENSOR_LC   */
#elif defined( FLIGHT_COMPUTER_LITE )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0 ].offset = 0; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].offset = 4; /* SENSOR_TEMP  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].size   = 4;  /* SENSOR_TEMP  */
#elif defined( VALVE_CONTROLLER     )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0 ].offset = 0;  /* SENSOR_ENCO  */
	sensor_size_offsets_table[ 1 ].offset = 4;  /* SENSOR_ENCF  */

	/* Sensor sizes */
	sensor_size_offsets_table[ 0 ].size   = 4;  /* SENSOR_ENCO */
	sensor_size_offsets_table[ 1 ].size   = 4;  /* SENSOR_ENCF */
#endif

} /* sensor_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
	#ifndef VALVE_CONTROLLER
		uint8_t subcommand 
	#else
		uint8_t    subcommand,   /* SDEC subcommand         */
		CMD_SOURCE cmd_source    /* Serial interface source */
	#endif
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_status;                         /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
USB_STATUS    usb_status;                            /* USB return codes      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
uint8_t       num_sensor_bytes = SENSOR_DATA_SIZE;   /* Size of data in bytes */
uint8_t       num_sensors;                           /* Number of sensors to 
                                                        use for polling       */
uint8_t       poll_sensors[ SENSOR_MAX_NUM_POLL ];   /* Codes for sensors to
                                                        be polled             */
uint8_t       sensor_poll_cmd;                       /* Command codes used by 
                                                        sensor poll           */
#ifdef VALVE_CONTROLLER
	VALVE_STATUS valve_status; /* status codes from valve API */
#endif

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
usb_status      = USB_OK;
sensor_status   = SENSOR_OK;
#ifdef VALVE_CONTROLLER
	valve_status = VALVE_OK;
#endif
num_sensors     = 0;
sensor_poll_cmd = 0;
memset( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset( &sensor_data         , 0, sizeof( sensor_data       ) );
memset( &poll_sensors[0]     , 0, sizeof( poll_sensors      ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{
	/*--------------------------------------------------------------------------
	 SENSOR POLL 
	--------------------------------------------------------------------------*/
    case SENSOR_POLL_CODE:
		{
		/* Determine the number of sensors to poll */
		#ifndef VALVE_CONTROLLER 
			usb_status = usb_receive( &num_sensors, 
									sizeof( num_sensors ), 
									HAL_DEFAULT_TIMEOUT );
			if ( usb_status != USB_OK )
				{
				return SENSOR_USB_FAIL;
				}
		#else
			if ( cmd_source == CMD_SOURCE_USB )
				{
				usb_status = usb_receive( &num_sensors, 
										sizeof( num_sensors ), 
										HAL_DEFAULT_TIMEOUT );
				if ( usb_status != USB_OK )
					{
					return SENSOR_USB_FAIL;
					}
				}
			else
				{
				valve_status = valve_receive( &num_sensors, 
				                              sizeof( num_sensors ), 
											  HAL_DEFAULT_TIMEOUT );
				if ( valve_status != VALVE_OK )
					{
					return SENSOR_VALVE_UART_ERROR;
					}
				}
		#endif /* #ifdef VALVE_CONTROLLER */

		/* Determine which sensors to poll */
		#ifndef VALVE_CONTROLLER 
			usb_status = usb_receive( &poll_sensors[0],
									num_sensors     , 
									HAL_SENSOR_TIMEOUT );
			if ( usb_status != USB_OK )
				{
				return SENSOR_USB_FAIL;
				}
		#else
			if ( cmd_source == CMD_SOURCE_USB )
				{
				usb_status = usb_receive( &poll_sensors[0],
										num_sensors     , 
										HAL_SENSOR_TIMEOUT );
				if ( usb_status != USB_OK )
					{
					return SENSOR_USB_FAIL;
					}
				}
			else
				{
				valve_status = valve_receive( &poll_sensors[0],
											num_sensors     ,
											HAL_SENSOR_TIMEOUT );
				if ( valve_status != VALVE_OK )
					{
					return SENSOR_VALVE_UART_ERROR;
					}
				}
		#endif /* #ifndef VALVE_CONTROLLER */

		/* Receive initiating command code  */
		#ifndef VALVE_CONTROLLER
			usb_status = usb_receive( &sensor_poll_cmd,
									sizeof( sensor_poll_cmd ),
									HAL_DEFAULT_TIMEOUT );
			if      ( usb_status      != USB_OK            )
				{
				return SENSOR_USB_FAIL; /* USB error */
				}
			else if ( sensor_poll_cmd != SENSOR_POLL_START )
				{
				/* SDEC fails to initiate sensor poll */
				return SENSOR_POLL_FAIL_TO_START;
				}
		#else
			if ( cmd_source == CMD_SOURCE_USB )
				{
				usb_status = usb_receive( &sensor_poll_cmd,
										sizeof( sensor_poll_cmd ),
										HAL_DEFAULT_TIMEOUT );
				if      ( usb_status      != USB_OK            )
					{
					return SENSOR_USB_FAIL; /* USB error */
					}
				else if ( sensor_poll_cmd != SENSOR_POLL_START )
					{
					/* SDEC fails to initiate sensor poll */
					return SENSOR_POLL_FAIL_TO_START;
					}
				}
			else
				{
				valve_status = valve_receive( &sensor_poll_cmd, 
											sizeof( sensor_poll_cmd ), 
											HAL_DEFAULT_TIMEOUT );
				if ( valve_status != VALVE_OK )
					{
					return SENSOR_VALVE_UART_ERROR;
					}
				else if ( sensor_poll_cmd != SENSOR_POLL_START )
					{
					return SENSOR_POLL_FAIL_TO_START;
					}
				}
		#endif

		// Reset start time
		previous_time = HAL_GetTick();

		/* Start polling sensors */
		while ( sensor_poll_cmd != SENSOR_POLL_STOP )
			{
			/* Get command code */
			#ifndef VALVE_CONTROLLER 
				usb_status = usb_receive( &sensor_poll_cmd         ,
										sizeof( sensor_poll_cmd ),
										HAL_DEFAULT_TIMEOUT );
				if ( usb_status != USB_OK ) 
					{
					return SENSOR_USB_FAIL;
					}
			#else
				if ( cmd_source == CMD_SOURCE_USB )
					{
					usb_status = usb_receive( &sensor_poll_cmd         ,
											sizeof( sensor_poll_cmd ),
											HAL_DEFAULT_TIMEOUT );
					if ( usb_status != USB_OK ) 
						{
						return SENSOR_USB_FAIL;
						}
					}
				else
					{
					valve_status = valve_receive( &sensor_poll_cmd         , 
					                              sizeof( sensor_poll_cmd ), 
												  HAL_DEFAULT_TIMEOUT );
					if ( valve_status != VALVE_OK )
						{
						return SENSOR_VALVE_UART_ERROR;
						}
					}
			#endif /* #ifndef VALVE_CONTROLLER */
			
			/* Execute command */
			switch ( sensor_poll_cmd )
				{
				
				/* Poll Sensors */
				case SENSOR_POLL_REQUEST:
					{
					tdelta = HAL_GetTick() - previous_time;
					previous_time = HAL_GetTick();
					sensor_status = sensor_poll( &sensor_data    , 
												 &poll_sensors[0],
												 num_sensors );
					if ( sensor_status != SENSOR_OK )
						{
						return SENSOR_POLL_FAIL;
						}
					else
						{
						/* Copy over sensor data into buffer */
						extract_sensor_bytes( &sensor_data, 
						                      &poll_sensors[0],
											  num_sensors     ,
											  &sensor_data_bytes[0],
											  &num_sensor_bytes );

						/* Transmit sensor bytes back to SDEC */
						usb_transmit( &sensor_data_bytes[0],
						              num_sensor_bytes     ,
									  HAL_SENSOR_TIMEOUT );
								
						break;
						}
					} /* case SENSOR_POLL_REQUEST */

				/* STOP Executtion */
				case SENSOR_POLL_STOP:
					{
					// Reset timing
					previous_time = 0;
					tdelta = 0;
					break;
					} /* case SENSOR_POLL_STOP */

				/* WAIT, Pause execution */
				case SENSOR_POLL_WAIT:
					{
					/* Poll USB port until resume signal arrives */
					while( sensor_poll_cmd != SENSOR_POLL_RESUME )
						{
						#ifndef VALVE_CONTROLLER
							usb_receive( &sensor_poll_cmd, 
										sizeof( sensor_poll_cmd ),
										HAL_DEFAULT_TIMEOUT );
						#else
							if ( cmd_source == CMD_SOURCE_USB )
								{
								usb_receive( &sensor_poll_cmd, 
											sizeof( sensor_poll_cmd ),
											HAL_DEFAULT_TIMEOUT );
								}
							else
								{
								valve_receive( &sensor_poll_cmd         , 
								               sizeof( sensor_poll_cmd ), 
											   HAL_DEFAULT_TIMEOUT );
								}
						#endif
						}
					break;
					} /* case SENSOR_POLL_WAIT */

				/* Erroneous Command*/
				default:
					{
					return SENSOR_POLL_UNRECOGNIZED_CMD;
					}
				} /* switch( sensor_poll_cmd ) */

			} /* while( sensor_poll_cmd != SENSOR_POLL_STOP ) */
		
		return sensor_status ;
        } /* SENSOR_POLL_CODE */ 

	/*--------------------------------------------------------------------------
	 SENSOR DUMP 
	--------------------------------------------------------------------------*/
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		#ifndef VALVE_CONTROLLER 
			usb_transmit( &num_sensor_bytes,
						sizeof( num_sensor_bytes ), 
						HAL_DEFAULT_TIMEOUT );
		#else
			if ( cmd_source == CMD_SOURCE_USB )
				{
				usb_transmit( &num_sensor_bytes,
							sizeof( num_sensor_bytes ), 
							HAL_DEFAULT_TIMEOUT );
				}
			else
				{
				valve_transmit( &num_sensor_bytes, 
				                sizeof( num_sensor_bytes ), 
								HAL_DEFAULT_TIMEOUT );
				}
		#endif /* #ifndef VALVE_CONTROLLER */

		/* Get the sensor readings */
	    sensor_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_status == SENSOR_OK )
			{
			#ifndef VALVE_CONTROLLER
				usb_transmit( &sensor_data_bytes[0]      , 
							sizeof( sensor_data_bytes ), 
							HAL_SENSOR_TIMEOUT );
			#else
				if ( cmd_source == CMD_SOURCE_USB )
					{
					usb_transmit( &sensor_data_bytes[0]      , 
								sizeof( sensor_data_bytes ), 
								HAL_SENSOR_TIMEOUT );
					}
				else
					{
					valve_transmit( &sensor_data_bytes[0],
					                sizeof( sensor_data_bytes ), 
									HAL_SENSOR_TIMEOUT );
					}
			#endif /* #ifndef VALVE_CONTROLLER */
			return ( sensor_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/*--------------------------------------------------------------------------
	 UNRECOGNIZED SUBCOMMAND 
	--------------------------------------------------------------------------*/
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
	IMU_STATUS      gyro_status;
	IMU_STATUS      mag_status; 
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;
#elif defined( ENGINE_CONTROLLER    )
	#ifdef L0002_REV4
		PRESSURE_STATUS pt_status;          /* Pressure status codes       */
		THERMO_STATUS   tc_status;          /* Thermocouple status codes   */
		LOADCELL_STATUS lc_status;          /* Loadcell status codes       */
	#elif defined( L0002_REV5 )
		SENSOR_STATUS   sensor_status;      /* Sensor module return codes  */
		THERMO_STATUS   tc_status;          /* Thermocouple status codes   */
	#endif
#elif defined( FLIGHT_COMPUTER_LITE )
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;
#endif

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	accel_status = IMU_OK;         
	gyro_status  = IMU_OK;
	mag_status   = IMU_OK; 
	press_status = BARO_OK;           
	temp_status  = BARO_OK;
#elif defined( ENGINE_CONTROLLER    )
	#ifdef L0002_REV4
		pt_status    = PRESSURE_OK;          
		tc_status    = THERMO_OK;        
	#elif defined( L0002_REV5 )
		sensor_status = SENSOR_OK;
		tc_status     = THERMO_OK;
	#endif
#elif defined( FLIGHT_COMPUTER_LITE )
	press_status = BARO_OK;           
	temp_status  = BARO_OK;
#endif

/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

/* Poll Sensors  */
#if defined( FLIGHT_COMPUTER )
	#if defined( A0002_REV2 )
	memset( &(sensor_data_ptr->imu_data), 0, sizeof( IMU_DATA ) );
	accel_status = imu_get_accel_and_gyro( &(sensor_data_ptr->imu_data) );
	#else
	/* IMU sensors */
	accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
	gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );

	mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );
	#endif
	sensor_data_ptr -> imu_data.temp = 0;     // Figure out what to do with this 
											  // readout, temporarily being used 
											  // as struct padding

	/* GPS sensor */
	sensor_data_ptr->gps_altitude_ft	= gps_data.altitude_ft;
	sensor_data_ptr->gps_speed_kmh		= gps_data.speed_km;
	sensor_data_ptr->gps_utc_time 		= gps_data.utc_time;
	sensor_data_ptr->gps_dec_longitude 	= gps_data.dec_longitude;
	sensor_data_ptr->gps_dec_latitude 	= gps_data.dec_latitude;
	sensor_data_ptr->gps_ns				= gps_data.ns;
	sensor_data_ptr->gps_ew				= gps_data.ew;
	sensor_data_ptr->gps_gll_status		= gps_data.gll_status;
	sensor_data_ptr->gps_rmc_status		= gps_data.rmc_status;

	/* Baro sensors */
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

	/* Calculated and retrieve converted IMU data */
	sensor_conv_imu( &(sensor_data_ptr->imu_data) );

	/* Calculated to get body state */
	sensor_body_state( &(sensor_data_ptr->imu_data) );

	/* Calculated velocity and position */
	sensor_imu_velo( &(sensor_data_ptr->imu_data) );

	/* Calculated velocity from barometer */
	sensor_baro_velo( sensor_data_ptr );


#elif defined( ENGINE_CONTROLLER )
	#ifndef L0002_REV5
	/* Pressure Transducers */
	pt_status    = pressure_poll_pts( &( sensor_data_ptr -> pt_pressures[0] ) );

	/* Load cell */
	lc_status    = loadcell_get_reading( &( sensor_data_ptr -> load_cell_force ) );
	#else
	/* PTs and Load Cell */
	sensor_status = sensor_adc_burst_read( sensor_data_ptr );
	#endif /* #ifndef L0002_REV5 */

	/* Thermocouple */
//	tc_status    = temp_get_temp( &( sensor_data_ptr -> tc_temp ), 
	//                              THERMO_HOT_JUNCTION );
#elif defined( FLIGHT_COMPUTER_LITE )
	/* Baro sensors */
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#elif defined( VALVE_CONTROLLER     )
	/* Main Valve encoders */
	sensor_data_ptr -> lox_valve_pos  = valve_get_ox_valve_pos();
	sensor_data_ptr -> fuel_valve_pos = valve_get_fuel_valve_pos();
#endif


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
#if defined( FLIGHT_COMPUTER )
	if      ( accel_status != IMU_OK )
		{
		return SENSOR_ACCEL_ERROR;
		}
	else if ( gyro_status  != IMU_OK )
		{
		return SENSOR_GYRO_ERROR;
		}
	else if ( mag_status   != IMU_OK )
		{
		return SENSOR_MAG_ERROR;	
		}
	else if ( press_status != BARO_OK ||
			temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}
	else
		{
		return SENSOR_OK;
		}
#elif defined( ENGINE_CONTROLLER )
	#ifdef L0002_REV4
		if      ( pt_status != PRESSURE_OK )
			{
			return SENSOR_PT_ERROR;
			}
		else if ( tc_status != THERMO_OK   )
			{
			return SENSOR_TC_ERROR;
			}
		else if ( lc_status != LOADCELL_OK )
			{
			return SENSOR_LC_ERROR;
			}
		else
			{
			return SENSOR_OK;
			}
	#elif defined( L0002_REV5 )
		if ( sensor_status != SENSOR_OK )
			{
			return sensor_status;
			}
		else if ( tc_status != THERMO_OK )
			{
			return SENSOR_TC_ERROR;
			}
		else
			{
			return SENSOR_OK;
			}
	#endif
#elif defined( FLIGHT_COMPUTER_LITE )
	if ( press_status != BARO_OK ||
		 temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}
	else
		{
		return SENSOR_OK;
		}
#elif defined( VALVE_CONTROLLER     )
	return SENSOR_OK;
#endif /* #elif defined( ENGINE_CONTROLLER )*/

} /* sensor_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_poll                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Poll specific sensors on the board                                     *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr, /* Data Export target               */
	SENSOR_ID*   sensor_ids_ptr , /* Array containing sensor IDS      */
	uint8_t      num_sensors      /* Number of sensors to poll        */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_ID  sensor_id;        /* ID of sensor currently being polled */
SENSOR_ID* sensor_id_ptr;    /* Pointer to sensor id                */

/* Module return codes */
#if   defined( FLIGHT_COMPUTER   )
	IMU_STATUS      imu_status;      /* IMU Module return codes   */ 
	BARO_STATUS     baro_status;     /* Baro module return codes  */
#elif defined( ENGINE_CONTROLLER )
	THERMO_STATUS   thermo_status;   /* Thermocouple return codes */
	LOADCELL_STATUS lc_status;       /* Loadcell return codes     */
	PRESSURE_STATUS pt_status;       /* PT return codes           */
	#ifdef L0002_REV5
	SENSOR_STATUS   sensor_status;   /* Sensor return codes       */
	#endif
#elif defined( FLIGHT_COMPUTER_LITE )
	BARO_STATUS     baro_status;     /* Baro module return codes  */
#endif

/* Sensor poll memory to prevent multiple calls to same API function */
#if defined( FLIGHT_COMPUTER )
	bool imu_accel_read;
	bool imu_gyro_read;
	bool imu_mag_read;
	bool body_state_converted;
	bool velo_pos_calculated;
#endif

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
sensor_id_ptr     = sensor_ids_ptr;
sensor_id         = *(sensor_id_ptr   );

/* Module return codes */
#if   defined( FLIGHT_COMPUTER   )
	imu_status    = IMU_OK;
	baro_status   = BARO_OK;
#elif defined( ENGINE_CONTROLLER )
	thermo_status = THERMO_OK;
	lc_status     = LOADCELL_OK;
	pt_status     = PRESSURE_OK;
	#ifdef L0002_REV5
	sensor_status = SENSOR_OK;
	#endif
#elif defined( FLIGHT_COMPUTER_LITE )
	baro_status   = BARO_OK;
#endif

/* Sensor poll memory */
#if defined( FLIGHT_COMPUTER )
	imu_accel_read = false;
	imu_gyro_read  = false;
	imu_mag_read   = false;
	body_state_converted = false;
	velo_pos_calculated = false;
#endif

/* Burst read ADC sensors on Engine controller Rev 5 */
#ifdef L0002_REV5
	sensor_status = sensor_adc_burst_read( sensor_data_ptr );
	if ( sensor_status != SENSOR_OK )
		{
		return sensor_status;
		}
	thermo_status = temp_get_temp( &( sensor_data_ptr -> tc_temp ),
				                   THERMO_HOT_JUNCTION );
	if ( thermo_status != THERMO_OK )
		{
		return SENSOR_TC_ERROR;
		}
	return SENSOR_OK;
#endif


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Iterate over each sensor readout */
for ( int i = 0; i < num_sensors; ++i )
	{
	
	/* Poll sensor */
	switch ( sensor_id )
		{
		#if defined( FLIGHT_COMPUTER )
			case SENSOR_ACCX:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCY:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCZ:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_GYROX:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROY:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROZ:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_MAGX:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGY:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGZ:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_IMUT:
				{
				sensor_data_ptr -> imu_data.temp = 0;
				break;
				}
			case SENSOR_ACCX_CONV:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
				break;
				}
			case SENSOR_ACCY_CONV:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
				break;
				}
			case SENSOR_ACCZ_CONV:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
				break;
				}
			case SENSOR_GYROX_CONV:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}	
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );			
				break;
				}
			case SENSOR_GYROY_CONV:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}	
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
				break;
				}
			case SENSOR_GYROZ_CONV:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}		
				sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
				break;
				}
			case SENSOR_ROLL_DEG:
				{
				if (!body_state_converted)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					if (!imu_gyro_read)
						{
							imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_GYRO_ERROR;
								}
							imu_gyro_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						}
					sensor_body_state( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_PITCH_DEG:
				{
				if (!body_state_converted)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					if (!imu_gyro_read)
						{
							imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_GYRO_ERROR;
								}
							imu_gyro_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						}
					sensor_body_state( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_ROLL_RATE:
				{
				if (!body_state_converted)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					if (!imu_gyro_read)
						{
							imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_GYRO_ERROR;
								}
							imu_gyro_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						}
					sensor_body_state( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_PITCH_RATE:
				{
				if (!body_state_converted)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					if (!imu_gyro_read)
						{
							imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_GYRO_ERROR;
								}
							imu_gyro_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						}
					sensor_body_state( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_VELOCITY:
				{
				if (!velo_pos_calculated)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					sensor_imu_velo( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_POSITION:
				{
				if (!velo_pos_calculated)
					{
					if (!imu_accel_read)
						{
							imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
							if ( imu_status != IMU_OK )
								{
								return SENSOR_ACCEL_ERROR;
								}
							imu_accel_read = true;
							sensor_conv_imu( &( sensor_data_ptr -> imu_data ) );
						} 	
					sensor_imu_velo( &( sensor_data_ptr -> imu_data ) );
					}
				break;
				}
			case SENSOR_GPS_ALT:
				{
				sensor_data_ptr->gps_altitude_ft = gps_data.altitude_ft;
				break;
				}
			case SENSOR_GPS_SPEED:
				{
				sensor_data_ptr->gps_speed_kmh	= gps_data.speed_km;
				break;
				}
			case SENSOR_GPS_TIME:
				{
				sensor_data_ptr->gps_utc_time	= gps_data.utc_time;
				break;
				}
			case SENSOR_GPS_DEC_LONG:
				{
				sensor_data_ptr->gps_dec_longitude = gps_data.dec_longitude;
				break;
				}
			case SENSOR_GPS_DEC_LAT:
				{
				sensor_data_ptr->gps_dec_latitude = gps_data.dec_latitude;
				break;
				}
			case SENSOR_GPS_NS:
				{
				sensor_data_ptr->gps_ns = gps_data.ns;
				break;
				}
			case SENSOR_GPS_EW:
				{
				sensor_data_ptr->gps_ew = gps_data.ew;
				break;
				}
			case SENSOR_GPS_GLL:
				{
				sensor_data_ptr->gps_gll_status = gps_data.gll_status;
				break;
				}
			case SENSOR_GPS_RMC:
				{
				sensor_data_ptr->gps_rmc_status = gps_data.rmc_status;
				break;
				}
		#endif /* #if defined( FLIGHT_COMPUTER ) */

		#if ( defined( FLIGHT_COMPUTER )  || defined( FLIGHT_COMPUTER_LITE ) )
			case SENSOR_PRES:
				{
				baro_status = baro_get_temp(     &( sensor_data_ptr -> baro_temp     ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				baro_status = baro_get_pressure( &( sensor_data_ptr -> baro_pressure ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				break;
				}

			case SENSOR_TEMP:
				{
				baro_status = baro_get_temp( &( sensor_data_ptr -> baro_temp ) );
				if ( baro_status != BARO_OK )
					{
					return SENSOR_BARO_ERROR;
					}
				break;
				}
		#endif /* if defined( FLIGHT_COMPUTER ) || defined( FLIGHT_COMPUTER_LITE ) */

		#if defined( ENGINE_CONTROLLER )
			case SENSOR_PT0:
				{
				pt_status = pressure_get_pt_reading( PT_NUM0, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT1:
				{
				pt_status = pressure_get_pt_reading( PT_NUM1, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT2:
				{
				pt_status = pressure_get_pt_reading( PT_NUM2, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT3:
				{
				pt_status = pressure_get_pt_reading( PT_NUM3, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT4:
				{
				pt_status = pressure_get_pt_reading( PT_NUM4, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT5:
				{
				pt_status = pressure_get_pt_reading( PT_NUM5, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT6:
				{
				pt_status = pressure_get_pt_reading( PT_NUM6, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_PT7:
				{
				pt_status = pressure_get_pt_reading( PT_NUM7, 
				                                    &( sensor_data_ptr -> pt_pressures[0]) );
				if ( pt_status != PRESSURE_OK )
					{
					return SENSOR_PT_ERROR;
					}
				break;
				}

			case SENSOR_TC:
				{
				thermo_status = temp_get_temp( &( sensor_data_ptr -> tc_temp ),
				                               THERMO_HOT_JUNCTION );
				if ( thermo_status != THERMO_OK )
					{
					return SENSOR_TC_ERROR;
					}
				break;
				}

			case SENSOR_LC:
				{
				lc_status = loadcell_get_reading( &( sensor_data_ptr -> load_cell_force ) );
				if ( lc_status != LOADCELL_OK )
					{
					return SENSOR_LC_ERROR;
					}
				break;
				}
		
		#elif defined( VALVE_CONTROLLER )

			case SENSOR_ENCO:
				{
				sensor_data_ptr -> lox_valve_pos = valve_get_ox_valve_pos();
				break;
				}

			case SENSOR_ENCF:
				{
				sensor_data_ptr -> fuel_valve_pos = valve_get_fuel_valve_pos();
				break;
				}

		#endif /* #if defined( ENGINE_CONTROLLER ) */

		default:
			{
			/* Unrecognized sensor id */
			return SENSOR_UNRECOGNIZED_SENSOR_ID; 
			}
		} /* switch( sensor_id ) */

		/* Go to next sensor */
		sensor_id_ptr++;
		sensor_id        = *(sensor_id_ptr   );

	} /*  while( i < num_sensors ) */

return SENSOR_OK;
} /* sensor_poll */


#ifdef FLIGHT_COMPUTER
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_conv_imu                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Conversion of IMU raw chip readouts into 9-axis Acceralometer and Gyro                                                     *
*                                                                              *
*******************************************************************************/
void sensor_conv_imu(IMU_DATA* imu_data){
	imu_data->imu_converted.accel_x = sensor_acc_conv(imu_data->accel_x);
	imu_data->imu_converted.accel_y = sensor_acc_conv(imu_data->accel_y);
	imu_data->imu_converted.accel_z = sensor_acc_conv(imu_data->accel_z);

	imu_data->imu_converted.accel_x = imu_data->imu_converted.accel_x - imu_offset.accel_x;
	imu_data->imu_converted.accel_y = imu_data->imu_converted.accel_y - imu_offset.accel_y;
	imu_data->imu_converted.accel_z = imu_data->imu_converted.accel_z - imu_offset.accel_z;

	imu_data->imu_converted.gyro_x = sensor_gyro_conv(imu_data->gyro_x);
	imu_data->imu_converted.gyro_y = sensor_gyro_conv(imu_data->gyro_y);
	imu_data->imu_converted.gyro_z = sensor_gyro_conv(imu_data->gyro_z);

	imu_data->imu_converted.gyro_x = imu_data->imu_converted.gyro_x - imu_offset.gyro_x;
	imu_data->imu_converted.gyro_y = imu_data->imu_converted.gyro_y - imu_offset.gyro_y;
	imu_data->imu_converted.gyro_z = imu_data->imu_converted.gyro_z - imu_offset.gyro_z;

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_body_state                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Perform sensor fusion on imu converted data to get body rate           *
*                                                                              *
*******************************************************************************/
void sensor_body_state(IMU_DATA* imu_data){
	// Calculate body state angles (pitch, roll)
	float pitch, roll;
	float g = 9.8;
	roll = atanf( imu_data->imu_converted.accel_z / imu_data->imu_converted.accel_y );
	pitch = atanf( imu_data->imu_converted.accel_x / g );

	// Calculate body state anglular rate
	float pitch_rate, roll_rate;
	roll_rate = imu_data->imu_converted.gyro_x + 									\
			imu_data->imu_converted.gyro_y * ( sinf(roll)*tanf(pitch) ) +	 		\
			imu_data->imu_converted.gyro_z * ( cosf(roll)*tanf(pitch) );

	pitch_rate = imu_data->imu_converted.gyro_y * ( cosf(roll) ) - imu_data->imu_converted.gyro_z * ( sinf(roll) );

	// Store calculated data
	imu_data->state_estimate.roll_angle 	= roll;
	imu_data->state_estimate.pitch_angle 	= pitch;
	imu_data->state_estimate.roll_rate 		= roll_rate;
	imu_data->state_estimate.pitch_rate 	= pitch_rate;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_acc_conv                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert Acc readouts to m/s^2                                          *
*                                                                              *
*******************************************************************************/
float sensor_acc_conv(uint16_t readout){
	// sign check
	uint16_t num_sign = (readout & (0x8000)) >> 0xF;
	int8_t sign_bit = 1;

	if (num_sign == 1){
		readout = ((uint16_t) ( ( ~readout + 1 ) & (0xFFFF) ));
		sign_bit = -1;
	}

	// Convert to accel
	uint8_t g_setting = 16;
	float g = 9.8;
	float accel_step = 2*g_setting*g/65535.0;

	return sign_bit*(accel_step*readout);
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_gyro_conv                                                       *   
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert gyro readouts to deg/s                                         *
*                                                                              *
*******************************************************************************/
float sensor_gyro_conv(uint16_t readout){
	// sign check
	uint16_t num_sign = (readout & (0x8000)) >> 0xF;
	int8_t sign_bit = 1;
	if (num_sign == 1){
		readout = ((uint16_t) ( ( ~readout + 1 ) & (0xFFFF) ));
		sign_bit = -1;
	}

	// Convert to accel
	float gyro_setting = 2000.0;
	float gyro_sens = 65535.0 / (2*gyro_setting);
	
	return sign_bit * (readout / gyro_sens);
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_imu_velo                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate the velocity depending on accel 								*
*                                                                              *
*******************************************************************************/
float velo_x_prev, velo_y_prev, velo_z_prev = 0.0;
void sensor_imu_velo(IMU_DATA* imu_data){
	float velo_x, velo_y, velo_z, velocity;

	float accel_x = imu_data->imu_converted.accel_x;
	float accel_y = imu_data->imu_converted.accel_y;
	float accel_z = imu_data->imu_converted.accel_z;

	float ts_delta = tdelta / 1000.0;

	// Calculate 3 velocity vectors using motion equations
	velo_x = velo_x_prev + accel_x*ts_delta;
	velo_y = velo_y_prev + accel_y*ts_delta;
	velo_z = velo_z_prev + accel_z*ts_delta;
	
	// Calculate the velocity scalar
	velocity = sqrtf(powf(velo_x, 2.0) + powf(velo_y, 2.0) + powf(velo_z, 2.0));

	imu_data->state_estimate.velocity = velocity;

	// Save current velocity for next computation
	velo_x_prev = velo_x;
	velo_y_prev = velo_y;
	velo_z_prev = velo_z;

	imu_data->state_estimate.position = 0; //TODO: Implement position
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_baro_velo                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate the velocity from pressure readings 								*
*                                                                              *
*******************************************************************************/
float velo_prev, alt_prev = 0.0;
void sensor_baro_velo(SENSOR_DATA* sen_data)
{
	float velocity;

	float pressure = sen_data->baro_pressure;
	float temp = sen_data->baro_temp;
	// conv pressure to pascal for equation
	// pressure *= 6894.76;
	float ts_delta = tdelta / 1000.0;

	// calc altitude
	float PRESSURE_SEA_LEVEL = 101325;
    float EXP = 0.190294958;
    float TEMP_LAPSE_RATE = 0.0065;

    float alt = (pow(PRESSURE_SEA_LEVEL / pressure, EXP) - 1) * (temp + 273.15) / TEMP_LAPSE_RATE;


	// Calculate the velocity scalar
	velocity = (alt-alt_prev)/ts_delta;
	alt_prev = alt;
	velo_prev = velocity;

	sen_data->baro_alt = alt;
	sen_data->baro_velo = velocity;

}

#endif


#ifdef ENGINE_CONTROLLER 
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_conv_pressure                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Converts a pressure transducer ADC readout to a floating point         *
*       pressure in psi                                                        *
*                                                                              *
*******************************************************************************/
float sensor_conv_pressure
	( 
	uint32_t adc_readout, /* Pressure readout from ADC */
	PT_INDEX pt_num       /* PT used for readout       */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
float voltage; /* ADC voltage    */
float gain;    /* Amplifier gain */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
voltage = 0;
gain    = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Convert readout to voltage */
voltage = ( 3.3/( pow( 2, 16 ) ) )*( (float) adc_readout );

/* Convert voltage to pressure in psi */
if ( pt_num > PT_NONE_INDEX )
	{
	return ( voltage*( 2000.0/5.0 ) );
	}
else
	{
	gain = 1 + ( 100.0/3.3 );
	return ( voltage*( 1000.0/(gain*0.1) ) );
	}
} /* sensor_conv_pressure */
#endif


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_map                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Sensor ID to size and pointer mapping                                  *
*                                                                              *
*******************************************************************************/
void static sensor_map
	(
	SENSOR_DATA* sensor_data_ptr, /* In:  Base pointer to sensor data   */
	SENSOR_ID    sensor_id      , /* In:  Sensor id                    */
	uint8_t**    sensorid_pptr  , /* Out: Pointer to sensor readout in 
	                                      sensor_data_ptr              */
	size_t*      sensor_size_ptr  /* Out: Size of readout in bytes     */
	)
{
/* Lookup sensor offset and size from table */
*sensor_size_ptr = sensor_size_offsets_table[ sensor_id ].size;
*sensorid_pptr   = ( (uint8_t*) sensor_data_ptr ) + 
                   sensor_size_offsets_table[ sensor_id ].offset;

} /*  sensor_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		extract_sensor_bytes                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Extract bytes for export from SENSOR_ID struct                         *
*                                                                              *
*******************************************************************************/
void static extract_sensor_bytes 
	(
	SENSOR_DATA* sensor_data_ptr      , /* In:  Sensor data in struct         */
	SENSOR_ID*   sensor_ids_ptr       , /* In:  Sensor ids                    */
	uint8_t      num_sensors          , /* In:  Number of sensors polled      */
	uint8_t*     sensor_data_bytes_ptr, /* Out: Sensor data in bytes          */
	uint8_t*     num_sensor_bytes       /* Out: Size of output data           */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
uint8_t*   output_ptr;    /* Pointer to data export output                    */
uint8_t*   input_ptr;     /* Pointer to data within SENSOR_ID struct          */
size_t     sensor_size;   /* Size in bytes of current sensor readout          */
SENSOR_ID  sensor_id;     /* Current Sensor ID                                */
SENSOR_ID* sensor_id_ptr; /* Pointer to current sensor ID                     */


/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
output_ptr        = sensor_data_bytes_ptr;
sensor_id_ptr     = sensor_ids_ptr;
sensor_id         = *(sensor_id_ptr);
*num_sensor_bytes = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
for ( uint8_t i = 0; i < num_sensors; ++i )
	{
	/* Get position info of sensor readout */
	sensor_map( sensor_data_ptr, 
	            sensor_id      ,
				&input_ptr      ,
				&sensor_size );

	/* Copy data into output buffer */
	memcpy( output_ptr, input_ptr, sensor_size );

	/* Update size of output */
	*num_sensor_bytes += (uint8_t) sensor_size;

	/* Go to next sensor */ 
	if ( i != ( num_sensors-1) )
		{
		sensor_id_ptr++;
		sensor_id = *(sensor_id_ptr);
		output_ptr += sensor_size;
		}
	}

} /* extract_sensor_bytes */

#ifdef L0002_REV5
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_adc_burst_read                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       reads from all sensors using the MCUs ADCs                             *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_adc_burst_read
	(
	SENSOR_DATA* sensor_data_ptr /* Pointer to sensor data structure */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef      adc_status[2];    /* Return codes from ADC API         */
uint16_t               mux_pins_bitmask; /* GPIO Pins for PT1-3 MUX           */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
memset( &adc_status[0], HAL_OK, sizeof( adc_status ) );
mux_pins_bitmask = 0;
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, PRESSURE_MUX_ALL_PINS, GPIO_PIN_RESET );


/*------------------------------------------------------------------------------
 Poll ADCs 
------------------------------------------------------------------------------*/

/* PT1/PT7 readout */
pt1_adc_channel_select();
HAL_ADC_Start( &hadc1 );
adc_status[0] = HAL_ADC_PollForConversion( &hadc1, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[0] = HAL_ADC_GetValue( &hadc1 );
HAL_ADC_Stop( &hadc1 );
pt7_adc_channel_select();
HAL_ADC_Start( &hadc1 );
adc_status[1] = HAL_ADC_PollForConversion( &hadc1, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[6] = HAL_ADC_GetValue( &hadc1 );
HAL_ADC_Stop( &hadc1 );
mux_pins_bitmask = mux_map( 1 );
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, PRESSURE_MUX_ALL_PINS, GPIO_PIN_RESET );
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, mux_pins_bitmask     , GPIO_PIN_SET   );
if ( adc_status[0] != HAL_OK || adc_status[1] != HAL_OK )
	{
	return SENSOR_ADC_POLL_ERROR;
	}

/* Load Cell/PT8 readout */
loadcell_adc_channel_select();
HAL_ADC_Start( &hadc2 );
adc_status[0] = HAL_ADC_PollForConversion( &hadc2, ADC_TIMEOUT );
sensor_data_ptr -> load_cell_force = HAL_ADC_GetValue( &hadc2 );
HAL_ADC_Stop( &hadc2 );
pt8_adc_channel_select();
HAL_ADC_Start( &hadc2 );
adc_status[1] = HAL_ADC_PollForConversion( &hadc2, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[7] = HAL_ADC_GetValue( &hadc2 );
HAL_ADC_Stop( &hadc2 );
if ( adc_status[0] != HAL_OK || adc_status[1] != HAL_OK )
	{
	return SENSOR_ADC_POLL_ERROR;
	}

/* PT2 readout           */
pt1_adc_channel_select(); /* wtf why patrick? */
HAL_ADC_Start( &hadc1 );
adc_status[0] = HAL_ADC_PollForConversion( &hadc1, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[1] = HAL_ADC_GetValue( &hadc1 );
HAL_ADC_Stop( &hadc1 );
mux_pins_bitmask = mux_map( 2 );
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, PRESSURE_MUX_ALL_PINS, GPIO_PIN_RESET );
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, mux_pins_bitmask     , GPIO_PIN_SET   );
if ( adc_status[0] != HAL_OK )
	{
	return SENSOR_ADC_POLL_ERROR;
	}

/* PT5/PT6 readout */
pt5_adc_channel_select();
HAL_ADC_Start( &hadc3 );
adc_status[0] = HAL_ADC_PollForConversion( &hadc3, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[4] = HAL_ADC_GetValue( &hadc3 );
HAL_ADC_Stop( &hadc3 );
pt6_adc_channel_select();
HAL_ADC_Start( &hadc3 );
adc_status[1] = HAL_ADC_PollForConversion( &hadc3, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[5] = HAL_ADC_GetValue( &hadc3 );
HAL_ADC_Stop( &hadc3 );
if ( adc_status[0] != HAL_OK || adc_status[1] != HAL_OK )
	{
	return SENSOR_ADC_POLL_ERROR;
	}

/* PT3 readout */
pt1_adc_channel_select();
HAL_ADC_Start( &hadc1 );
adc_status[0] = HAL_ADC_PollForConversion( &hadc1, ADC_TIMEOUT );
sensor_data_ptr -> pt_pressures[2] = HAL_ADC_GetValue( &hadc1 );
sensor_data_ptr -> pt_pressures[3] = 0;
HAL_ADC_Stop( &hadc1 );
HAL_GPIO_WritePin( PRESSURE_GPIO_PORT, PRESSURE_MUX_ALL_PINS, GPIO_PIN_RESET );
if ( adc_status[0] != HAL_OK )
	{
	return SENSOR_ADC_POLL_ERROR;
	}
else
	{
	return SENSOR_OK;
	}

} /* sensor_adc_burst_read */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		mux_map                                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Mapping from pressure transducer number to mutliplexor GPIO pin        *
*       bitmask. ex. PTNUM5 -> 101 -> GPIO_PIN_C | GPIO_PIN_A                  *
*                                                                              *
*******************************************************************************/
static inline uint16_t mux_map
	(
    PRESSURE_PT_NUM    pt_num    
    )
{
/* Mux pins are adjacent and from the same port. Just shift the ptnum bits up
   to create the bitmask */
#ifdef L0002_REV4
	return ( (uint16_t) pt_num) << PT_MUX_BITMASK_SHIFT; 
#elif defined( L0002_REV5 )
	if ( pt_num <= 3 )
		{
		return ( (uint16_t) pt_num) << PT_MUX_BITMASK_SHIFT; 
		}
	else
		{
		return 0;
		}
#endif
} /* mux_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pt1_adc_channel_select                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for PT1                                         *
*                                                                              *
*******************************************************************************/
static void pt1_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig   = {0};
sConfig.Channel                  = ADC_CHANNEL_10;
sConfig.Rank                     = ADC_REGULAR_RANK_1;
sConfig.SamplingTime             = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff               = ADC_SINGLE_ENDED;
sConfig.OffsetNumber             = ADC_OFFSET_NONE;
sConfig.Offset                   = 0;
sConfig.OffsetSignedSaturation   = DISABLE;
HAL_ADC_ConfigChannel( &hadc1, &sConfig );

} /* pt1_adc_channel_select */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pt7_adc_channel_select                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for PT7                                         *
*                                                                              *
*******************************************************************************/
static void pt7_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig   = {0};
sConfig.Channel                  = ADC_CHANNEL_4;
sConfig.Rank                     = ADC_REGULAR_RANK_1;
sConfig.SamplingTime             = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff               = ADC_SINGLE_ENDED;
sConfig.OffsetNumber             = ADC_OFFSET_NONE;
sConfig.Offset                   = 0;
sConfig.OffsetSignedSaturation   = DISABLE;
HAL_ADC_ConfigChannel( &hadc1, &sConfig );

} /* pt7_adc_channel_select */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		loadcell_adc_channel_select                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for the loadcell                                *
*                                                                              *
*******************************************************************************/
static void loadcell_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig      = {0};
sConfig.Channel                     = ADC_CHANNEL_11;
sConfig.Rank                        = ADC_REGULAR_RANK_1;
sConfig.SamplingTime                = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff                  = ADC_SINGLE_ENDED;
sConfig.OffsetNumber                = ADC_OFFSET_NONE;
sConfig.Offset                      = 0;
sConfig.OffsetSignedSaturation      = DISABLE;
HAL_ADC_ConfigChannel( &hadc2, &sConfig );
} /* loadcell_adc_channel_select */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pt8_adc_channel_select                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for the pt8                                     *
*                                                                              *
*******************************************************************************/
static void pt8_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig      = {0};
sConfig.Channel                     = ADC_CHANNEL_8;
sConfig.Rank                        = ADC_REGULAR_RANK_1;
sConfig.SamplingTime                = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff                  = ADC_SINGLE_ENDED;
sConfig.OffsetNumber                = ADC_OFFSET_NONE;
sConfig.Offset                      = 0;
sConfig.OffsetSignedSaturation      = DISABLE;
HAL_ADC_ConfigChannel( &hadc2, &sConfig );
} /* pt8_adc_channel_select */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pt5_adc_channel_select                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for the pt5                                     *
*                                                                              *
*******************************************************************************/
static void pt5_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel                = ADC_CHANNEL_0;
sConfig.Rank                   = ADC_REGULAR_RANK_1;
sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff             = ADC_SINGLE_ENDED;
sConfig.OffsetNumber           = ADC_OFFSET_NONE;
sConfig.Offset                 = 0;
sConfig.OffsetSignedSaturation = DISABLE;
HAL_ADC_ConfigChannel( &hadc3, &sConfig );
} /* pt5_adc_channel_select */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pt6_adc_channel_select                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Select the adc channel for the pt6                                     *
*                                                                              *
*******************************************************************************/
static void pt6_adc_channel_select
	(
	void
	)
{
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel                = ADC_CHANNEL_1;
sConfig.Rank                   = ADC_REGULAR_RANK_1;
sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
sConfig.SingleDiff             = ADC_SINGLE_ENDED;
sConfig.OffsetNumber           = ADC_OFFSET_NONE;
sConfig.Offset                 = 0;
sConfig.OffsetSignedSaturation = DISABLE;
HAL_ADC_ConfigChannel( &hadc3, &sConfig );
} /* pt6_adc_channel_select */


static void imu_get_state_estimate(IMU_DATA* imu_data){
    float roll_angle = atan(imu_data->accel_y/imu_data->accel_z);
	float pitch_angle = atan(imu_data->accel_x/9.81);

	float roll_rate = imu_data->gyro_x + imu_data->gyro_y*sin(roll_angle)*tan(pitch_angle) + imu_data->gyro_z*cos(roll_angle)*tan(pitch_angle);
	float pitch_rate = imu_data->gyro_y*cos(roll_angle) - imu_data->gyro_z*sin(roll_angle);	

	imu_data->state_estimate->roll_angle = roll_angle;
	imu_data->state_estimate->pitch_angle = pitch_angle;
	imu_data->state_estimate->roll_rate = roll_rate;
	imu_data->state_estimate->pitch_rate = pitch_rate;
}


#endif /* #ifdef L0002_REV5 */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/