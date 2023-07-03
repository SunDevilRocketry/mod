/*******************************************************************************
*
* FILE: 
* 		sensor.h
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
#endif

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif

/* Project includes */
#if defined( ENGINE_CONTROLLER )
	#include "pressure.h"
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE        ( 0x01 )
#define SENSOR_POLL_CODE        ( 0x02 )

/* Max allowed number of sensors for polling */
#define SENSOR_MAX_NUM_POLL     ( 5    )

#if   defined( FLIGHT_COMPUTER   )
	/* General */
	#define NUM_SENSORS         ( 12   )
	#define IMU_DATA_SIZE       ( 20   )
	#define SENSOR_DATA_SIZE	( 28   )
#elif defined( ENGINE_CONTROLLER )
	/* General */
	#define NUM_SENSORS         ( 10   )
	#define SENSOR_DATA_SIZE    ( 40   )
#elif defined( FLIGHT_COMPUTER_LITE )
	/* General */
	#define NUM_SENSORS         ( 2    )
	#define SENSOR_DATA_SIZE    ( 8    )
#elif defined( VALVE_CONTROLLER     )
	/* General */
	#define NUM_SENSORS         ( 2   )
	#define SENSOR_DATA_SIZE    ( 8   )

	/* Timeouts */
	#ifndef SDR_DEBUG
		#define HAL_SENSOR_TIMEOUT ( 40 )
	#else
		/* Disable timeouts when debugging */
		#define HAL_SENSOR_TIMEOUT ( 0xFFFFFFFF )
	#endif
#elif defined( GROUND_STATION )
	/* General */
	#define NUM_SENSORS         ( 10   )
	#define SENSOR_DATA_SIZE    ( 40   )
#else
	#error Board is not compatible with SENSOR module
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Sensor status return codes */
typedef enum 
	{
    SENSOR_OK = 0                ,
	SENSOR_UNRECOGNIZED_OP       ,
	SENSOR_UNSUPPORTED_OP        ,
	SENSOR_IMU_FAIL              ,
	SENSOR_PT_ERROR              ,
	SENSOR_TC_ERROR              ,
	SENSOR_LC_ERROR              ,
	SENSOR_ACCEL_ERROR           ,
    SENSOR_GYRO_ERROR            ,
	SENSOR_MAG_ERROR             ,
	SENSOR_BARO_ERROR            ,
	SENSOR_USB_FAIL              ,
	SENSOR_UNRECOGNIZED_SENSOR_ID,
	SENSOR_POLL_FAIL_TO_START    ,
	SENSOR_POLL_FAIL             ,
	SENSOR_POLL_UNRECOGNIZED_CMD ,
	SENSOR_VALVE_UART_ERROR      ,
	SENSOR_ADC_POLL_ERROR        ,
    SENSOR_FAIL   
    } SENSOR_STATUS;

/* Sensor poll command codes */
typedef enum
	{
	SENSOR_POLL_START   = 0xF3,
	SENSOR_POLL_REQUEST = 0x51,
	SENSOR_POLL_WAIT    = 0x44,
	SENSOR_POLL_RESUME  = 0xEF,
	SENSOR_POLL_STOP    = 0x74
	} SENSOR_POLL_CMD;

/* Sensor idenification code instance*/
typedef uint8_t SENSOR_ID;

/* Sensor Names/codes */
typedef enum
	{
	#if defined( FLIGHT_COMPUTER )
		SENSOR_ACCX  = 0x00,
		SENSOR_ACCY  = 0x01,
		SENSOR_ACCZ  = 0x02,
		SENSOR_GYROX = 0x03,
		SENSOR_GYROY = 0x04,
		SENSOR_GYROZ = 0x05,
		SENSOR_MAGX  = 0x06,
		SENSOR_MAGY  = 0x07,
		SENSOR_MAGZ  = 0x08,
		SENSOR_IMUT  = 0x09,
		SENSOR_PRES  = 0x0A,
		SENSOR_TEMP  = 0x0B
	#elif ( defined( ENGINE_CONTROLLER ) || defined( GROUND_STATION ) )
		SENSOR_PT0   = 0x00,
		SENSOR_PT1   = 0x01,
		SENSOR_PT2   = 0x02,
		SENSOR_PT3   = 0x03,
		SENSOR_PT4   = 0x04,
		SENSOR_PT5   = 0x05,
		SENSOR_PT6   = 0x06,
		SENSOR_PT7   = 0x07,
		SENSOR_LC    = 0x09,
		SENSOR_TC    = 0x08
	#elif defined( FLIGHT_COMPUTER_LITE )
		SENSOR_PRES  = 0x00,
		SENSOR_TEMP  = 0x01
	#elif defined( VALVE_CONTROLLER     )
		SENSOR_ENCO  = 0x00,
		SENSOR_ENCF  = 0x01
	#endif
	} SENSOR_IDS;

/* Sensor Data */
typedef struct SENSOR_DATA 
	{
	#if   defined( FLIGHT_COMPUTER      )
		IMU_DATA imu_data;
		float    baro_pressure;
		float    baro_temp;	
	#elif ( defined( ENGINE_CONTROLLER ) || defined( GROUND_STATION ) )
		uint32_t pt_pressures[ NUM_PTS ];
		uint32_t load_cell_force;
		uint32_t tc_temp;
	#elif defined( FLIGHT_COMPUTER_LITE )
		float baro_pressure;
		float baro_temp;
	#elif defined( VALVE_CONTROLLER     )
		int32_t lox_valve_pos;
		int32_t fuel_valve_pos;
	#endif /* #elif defined( ENGINE_CONTROLLER ) */
	} SENSOR_DATA;

/* Sensor Data sizes and offsets */
typedef struct SENSOR_DATA_SIZE_OFFSETS
	{
	uint8_t offset;  /* Offset of sensor readout in SENSOR_DATA struct  */
	size_t  size;    /* Size of readout in bytes                        */
	} SENSOR_DATA_SIZE_OFFSETS;

/* Pressure Transducer Indices */
#ifdef ENGINE_CONTROLLER 
	typedef enum 
		{
		PT_LOX_PRESS_INDEX = 0 ,
		PT_LOX_FLOW_UP_INDEX   ,
		PT_LOX_FLOW_DOWN_INDEX ,
		PT_NONE_INDEX          ,
		PT_ENGINE_PRESS_INDEX  ,
		PT_FUEL_FLOW_DOWN_INDEX,
		PT_FUEL_PRESS_INDEX
		} PT_INDEX;
#endif


/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/* Initialize the sensor module */
void sensor_init 
	(
	void
	);

/* Execute a sensor subcommand */
SENSOR_STATUS sensor_cmd_execute
	(
	#ifndef VALVE_CONTROLLER
		uint8_t subcommand
	#else
		uint8_t    subcommand,   /* SDEC subcommand         */
		CMD_SOURCE cmd_source    /* serial interface source */
	#endif
    );

/* Poll specific sensors on the board */
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID* sensor_ids_ptr  ,
	uint8_t    num_sensors
	);

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );

#ifdef ENGINE_CONTROLLER
/* Converts a pressure transducer ADC readout to a floating point pressure in 
   psi */
float sensor_conv_pressure
	( 
	uint32_t adc_readout, /* Pressure readout from ADC */
	PT_INDEX pt_num       /* PT used for readout       */
	);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/