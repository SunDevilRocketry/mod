/*******************************************************************************
*
* FILE: 
* 		baro.c
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "baro.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/

/* Current Baro Sensor configuration */
static BARO_CONFIG   baro_configuration;

/* Baro calibration coefficients for measurement compensation */
static BARO_CAL_DATA baro_cal_data;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Converts two bytes to uint16_t format */
static uint16_t bytes_to_uint16_t
	(
	uint8_t lsb_byte, /* In: Least significant byte */
	uint8_t msb_byte  /* In: Most significant byte  */
	);

/* Converts two bytes to int16_t format */
static int16_t bytes_to_int16_t
	(
	uint8_t lsb_byte, /* In: Least significant byte */
	uint8_t msb_byte  /* In: Most significant byte  */
	);

/* Read from the baro's registers at a specified address  */
static BARO_STATUS read_regs
	(
	uint8_t  reg_addr, /* In:  Register address            */
	uint8_t  num_regs, /* In:  Number of registers to read */
	uint8_t* pData     /* Out: Register contents           */
	);


/* Write to one of the baro's registers at a specified address */
static BARO_STATUS write_reg
	(
	uint8_t  reg_addr, /* In: Register address            */
	uint8_t  data      /* In: Register contents           */
	);

/* Load the compensation data from the baro sensor */
static BARO_STATUS load_cal_data
	(
	void
	);

/* Apply the compensation formula to raw temperature readouts */
static float temp_compensate
	(
	uint32_t raw_readout
	);

/* Apply the compensation formula to raw pressure readouts */
static float press_compensate 
	(
	uint32_t raw_readout
	);

/* Reset the baro sensor */
static BARO_STATUS baro_reset
	(
	void
	);

/* Flush the FIFO buffer */
static BARO_STATUS baro_flush_fifo
	(
	void
	);


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_init                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Intialize the barometric pressure sensor                               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
BARO_STATUS       baro_status;    /* Status code from Baro API calls   */
uint8_t           baro_device_id; /* Baro device id                    */
uint8_t           baro_err_reg;   /* Contents of error register        */
float             init_temp;      /* Temperature read after init       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_status    = BARO_OK;
baro_device_id = 0;
baro_err_reg   = 0xFF;
init_temp      = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Verify functional I2C connection to sensor */
baro_status = baro_get_device_id( &baro_device_id );
if      ( baro_status   != BARO_OK         )
	{
	return BARO_I2C_ERROR;
	}
else if ( baro_device_id != BMP390_DEVICE_ID &&
          baro_device_id != BMP388_DEVICE_ID )
	{
	return BARO_UNRECOGNIZED_CHIP_ID;
	}

/* Reset the sensor */
baro_status = baro_reset();
if ( baro_status != BARO_OK )
	{
	return BARO_CANNOT_RESET;
	}
HAL_Delay( 10 );

/* Check the Baro error register */
baro_status = read_regs( BARO_REG_ERR_REG      , 
                         sizeof( baro_err_reg ), 
						 &baro_err_reg );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}
else if ( baro_err_reg )
	{
	return BARO_ERROR;
	}

/* Load the calibration coefficients */
baro_status = load_cal_data();
if ( baro_status != BARO_OK )
	{
	return BARO_CAL_ERROR;
	}

/* Flush the sensor FIFO buffer */
baro_status = baro_flush_fifo();
if ( baro_status != BARO_OK )
	{
	return BARO_FIFO_ERROR;
	}

/* Initialize the compensation temperature */
baro_status = baro_get_temp( &init_temp );
if ( baro_status != BARO_OK )
	{ 
	return baro_status;
	}

/* Configure the sensor */
baro_status = baro_config( config_ptr );
return baro_status;

} /* baro_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_config                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Configure the barometric pressure sensor                               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_config
	(
	BARO_CONFIG* config_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
BARO_STATUS   baro_status; /* Baro API call return codes                  */
uint8_t       pwr_ctrl;    /* Contents of PWR_CTRL register               */
uint8_t       osr;         /* Contents of OSR register                    */
uint8_t       odr;         /* Contents of ODR register                    */
uint8_t       iir;         /* Contents of CONFIG register (IIR Filter)    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_status = BARO_OK;
pwr_ctrl    = 0;
osr         = 0;
odr         = 0;
iir         = 0;


/*------------------------------------------------------------------------------
 Extract Baro settings from config struct  
------------------------------------------------------------------------------*/

/* Set register contents */
pwr_ctrl |= config_ptr -> enable;                /* Enable sensors           */
pwr_ctrl |= config_ptr -> mode << 4;             /* Set operating mode       */
osr      |= config_ptr -> press_OSR_setting;     /* Pressure oversampling    */
osr      |= config_ptr -> temp_OSR_setting << 3; /* Temperature oversampling */
odr      |= config_ptr -> ODR_setting;           /* Sampling Frequency       */
iir      |= config_ptr -> IIR_setting;           /* IIR Filter Selection     */

/* Set global baro configuration */
baro_configuration.enable            = config_ptr -> enable;
baro_configuration.mode              = config_ptr -> mode;
baro_configuration.press_OSR_setting = config_ptr -> press_OSR_setting;
baro_configuration.temp_OSR_setting  = config_ptr -> temp_OSR_setting;
baro_configuration.ODR_setting       = config_ptr -> ODR_setting;
baro_configuration.IIR_setting       = config_ptr -> IIR_setting;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Write to the PWR_CTRL register -> Operating mode and enable sensors */
baro_status = write_reg( BARO_REG_PWR_CTRL, pwr_ctrl );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the OSR register -> set the oversampling rate */
baro_status = write_reg( BARO_REG_OSR, osr );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the ODR register -> set the sampling frequency */
baro_status = write_reg( BARO_REG_ODR, odr );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the CONFIG register -> Configure the IIR filter */
baro_status = write_reg( BARO_REG_CONFIG, iir );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Configuration complete */
return BARO_OK;

} /* baro_config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Gets the device ID of the barometric pressure sensor, primarily used   *
*       to verify that the sensor can be accessed by the MCU                   *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id_ptr /* Out: Baro device id */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
BARO_STATUS baro_status; /* Status codes returned from baro API calls */


/*------------------------------------------------------------------------------
 API Function implementation 
------------------------------------------------------------------------------*/

/* Read baro register with I2C */
baro_status = read_regs( BARO_REG_CHIP_ID, 
                         sizeof( uint8_t ), 
						 baro_id_ptr );
return baro_status;
} /* baro_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_pressure                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a pressure reading from the sensor                           *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_pressure
	(
    float* pressure_ptr  /* Out: Baro pressure */
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t     pressure_bytes[3]; /* Pressure raw readout bytes, LSB first   */
uint32_t    raw_pressure;      /* Pressure raw readout in uint32_t format */
float       comp_temp;         /* Compensation temperature                */
BARO_STATUS baro_status;       /* Return codes for baro API calls         */


/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
raw_pressure = 0;
comp_temp    = 0;
baro_status  = BARO_OK;
memset( &pressure_bytes[0], 0, sizeof( pressure_bytes ) );


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive pressure data registers */
baro_status = read_regs( BARO_REG_PRESS_DATA, 
                         sizeof( pressure_bytes ), 
						 &pressure_bytes[0] );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Combine all bytes value to 24 bit value */
raw_pressure = ( ( (uint32_t) pressure_bytes[2] << 16 ) |
                 ( (uint32_t) pressure_bytes[1] <<  8 ) |
				 ( (uint32_t) pressure_bytes[0]       ) );

/* Get compensation temperature for pressure compensation calculation */
baro_status   = baro_get_temp( &comp_temp );

/* Compensate using calibration data */
*pressure_ptr = press_compensate( raw_pressure );

return BARO_OK;
} /* baro_get_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_temp                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a temperature reading from the sensor                        *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_temp
	(
    float* temp_ptr 
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t     temp_bytes[3]; /* Raw temperature readout bytes, LSB first */
uint32_t    raw_temp;      /* Raw temperature readout in uint32_t format */
BARO_STATUS baro_status;   /* Status codes returned from baro API calls  */


/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
raw_temp     = 0;
baro_status  = BARO_OK;
memset( &temp_bytes[0], 0, sizeof( temp_bytes ) );


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive temperature data registers */
baro_status = read_regs( BARO_REG_TEMP_DATA  , 
                         sizeof( temp_bytes ), 
						 &temp_bytes[0] );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Combine all bytes value to 24 bit value */
raw_temp = ( ( (uint32_t) temp_bytes[2] << 16 ) |
             ( (uint32_t) temp_bytes[1] <<  8 ) |
			 ( (uint32_t) temp_bytes[0]       ) ); 

/* Adjust using calibration data */
*temp_ptr = temp_compensate( raw_temp );

return BARO_OK;

} /* baro_get_temp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_altitude                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		gets the altitude of the rocket from the sensor readouts               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_altitude
	(
    void
	)
{
return BARO_OK;
} /* baro_get_altitude */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       bytes_to_uin16_t                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Converts two bytes to uint16_t format                                  *
*                                                                              *
*******************************************************************************/
static uint16_t bytes_to_uint16_t
	(
	uint8_t lsb_byte, /* In: Least significant byte */
	uint8_t msb_byte  /* In: Most significant byte  */
	)
{
return ( ( (uint16_t) lsb_byte      ) |
         ( (uint16_t) msb_byte << 8 ) );
} /* bytes_to_uint16_t */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       bytes_to_in16_t                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Converts two bytes to int16_t format                                   *
*                                                                              *
*******************************************************************************/
static int16_t bytes_to_int16_t
	(
	uint8_t lsb_byte, /* In: Least significant byte */
	uint8_t msb_byte  /* In: Most significant byte  */
	)
{
uint16_t bytes_comb = ( ( (uint16_t) lsb_byte      ) |
                        ( (uint16_t) msb_byte << 8 ) );
return ( (int16_t) bytes_comb );
} /* bytes_to_uint16_t */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       read_regs                                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read from the baro's registers at a specified address                  *
*                                                                              *
*******************************************************************************/
static BARO_STATUS read_regs
	(
	uint8_t  reg_addr, /* In:  Register address            */
	uint8_t  num_regs, /* In:  Number of registers to read */
	uint8_t* pData     /* Out: Register contents           */
	)
{   
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;  /* HAL API Return codes */
uint32_t          i2c_timeout; /* I2C read timeout     */


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/
hal_status  = HAL_OK;
i2c_timeout = BARO_DEFAULT_TIMEOUT*num_regs;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read I2C register*/
hal_status = HAL_I2C_Mem_Read( &( BARO_I2C )       ,
				               BARO_I2C_ADDR       ,
				               reg_addr            ,
				               I2C_MEMADD_SIZE_8BIT,
				               pData               ,
				               num_regs            , 
				               i2c_timeout );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}
else
	{
	return BARO_OK;
	}

} /* read_reg */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       write_reg                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Write to one of the baro's registers at a specified address            *
*                                                                              *
*******************************************************************************/
static BARO_STATUS write_reg
	(
	uint8_t  reg_addr, /* In: Register address            */
	uint8_t  data      /* In: Register contents           */
	)
{   
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* HAL API Return codes */


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Write to register with I2C */
hal_status = HAL_I2C_Mem_Write( &( BARO_I2C )       ,
				                BARO_I2C_ADDR       ,
				                reg_addr            ,
				                I2C_MEMADD_SIZE_8BIT,
				                &data               ,
				                sizeof( uint8_t )   , 
				                BARO_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}
else
	{
	return BARO_OK;
	}

} /* write_reg */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       load_cal_data                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Load the compensation data from the baro sensor                        *
*                                                                              *
*******************************************************************************/
static BARO_STATUS load_cal_data
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
BARO_CAL_DATA_INT cal_data_int;       /* Raw calibration data from baro       */
uint8_t           buffer[BARO_CAL_BUFFER_SIZE]; /* Buffer for cal data        */
BARO_STATUS       baro_status;        /* Baro API return codes                */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
memset( &cal_data_int, 0, sizeof( cal_data_int ) );
memset( &buffer[0]   , 0, sizeof( buffer       ) );


/*------------------------------------------------------------------------------
 Read Baro Registers 
------------------------------------------------------------------------------*/

/* Get Data */
baro_status = read_regs( BARO_REG_NVM_PAR_T1, 
                         BARO_CAL_BUFFER_SIZE, 
						 &buffer[0] );
if ( baro_status != BARO_OK )
	{
	return baro_status;
	}

/* Convert to proper integer format */
cal_data_int.par_t1  = bytes_to_uint16_t( buffer[0], buffer[1] );
cal_data_int.par_t2  = bytes_to_uint16_t( buffer[2], buffer[3] );
cal_data_int.par_t3  = (int8_t) buffer[4];
cal_data_int.par_p1  = bytes_to_int16_t( buffer[5], buffer[6] );
cal_data_int.par_p2  = bytes_to_int16_t( buffer[7], buffer[8] );
cal_data_int.par_p3  = (int8_t)  buffer[9];
cal_data_int.par_p4  = (int8_t)  buffer[10];
cal_data_int.par_p5  = bytes_to_uint16_t( buffer[11], buffer[12] );
cal_data_int.par_p6  = bytes_to_uint16_t( buffer[13], buffer[14] );
cal_data_int.par_p7  = (int8_t)  buffer[15];
cal_data_int.par_p8  = (int8_t)  buffer[16];
cal_data_int.par_p9  = bytes_to_int16_t( buffer[17], buffer[18] );
cal_data_int.par_p10 = (int8_t) buffer[19];
cal_data_int.par_p11 = (int8_t) buffer[20];


/*------------------------------------------------------------------------------
 Convert to floating point format ( BMP390 Datasheet pg. 55 ) 
------------------------------------------------------------------------------*/

/* Temp Compensation */
baro_cal_data.par_t1   = ( ( (float) cal_data_int.par_t1  )/( 0.00390625f        ) );
baro_cal_data.par_t2   = ( ( (float) cal_data_int.par_t2  )/( 1073741824.0f      ) );
baro_cal_data.par_t3   = ( ( (float) cal_data_int.par_t3  )/( 281474976710656.0f ) );

/* Pressure Compensation */
baro_cal_data.par_p1   = ( (float) ( cal_data_int.par_p1 - 16384 ) );
baro_cal_data.par_p1  /= ( 1048576.0f );
baro_cal_data.par_p2   = ( (float) ( cal_data_int.par_p2 - 16384 ) );
baro_cal_data.par_p2  /= ( 536870912.0f );
baro_cal_data.par_p3   = ( ( (float) cal_data_int.par_p3  )/( 4294967296.0f           ) );
baro_cal_data.par_p4   = ( ( (float) cal_data_int.par_p4  )/( 137438953472.0f         ) );
baro_cal_data.par_p5   = ( ( (float) cal_data_int.par_p5  )/( 0.125f                  ) );
baro_cal_data.par_p6   = ( ( (float) cal_data_int.par_p6  )/( 64.0f                   ) );
baro_cal_data.par_p7   = ( ( (float) cal_data_int.par_p7  )/( 256.0f                  ) );
baro_cal_data.par_p8   = ( ( (float) cal_data_int.par_p8  )/( 32768.0f                ) );
baro_cal_data.par_p9   = ( ( (float) cal_data_int.par_p9  )/( 281474976710656.0f      ) );
baro_cal_data.par_p10  = ( ( (float) cal_data_int.par_p10 )/( 281474976710656.0f      ) );
baro_cal_data.par_p11  = ( ( (float) cal_data_int.par_p11 )/( 36893488147419103232.0f ) );

/* Load Successful */
return BARO_OK;

} /* load_cal_data */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       temp_compensate                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Apply the compensation formula to raw temperature readout              *
*                                                                              *
*******************************************************************************/
static float temp_compensate
	(
	uint32_t raw_readout
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
float partial_data1; /* Intermediate compensation results */
float partial_data2;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
partial_data1 = 0;
partial_data2 = 0;


/*------------------------------------------------------------------------------
 Calculations 
------------------------------------------------------------------------------*/
partial_data1 = (float)( raw_readout - baro_cal_data.par_t1 );
partial_data2 = (float)( partial_data1*baro_cal_data.par_t2 );
baro_cal_data.comp_temp = (float)( partial_data2 + 
                            powf( partial_data1, 2)*baro_cal_data.par_t3 );
return baro_cal_data.comp_temp;

} /* temp_compensate */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_compensate                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Apply the compensation formula to raw pressure readouts                *
*                                                                              *
*******************************************************************************/
static float press_compensate 
	(
	uint32_t raw_readout
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
float partial_data1; /* Intermediate compensation results */
float partial_data2;
float partial_data3;
float partial_data4;
float partial_out1;
float partial_out2;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
partial_data1 = 0;
partial_data2 = 0;
partial_data3 = 0;
partial_data3 = 0;
partial_out1  = 0;
partial_out2  = 0;


/*------------------------------------------------------------------------------
 Calculations 
------------------------------------------------------------------------------*/
partial_data1 = baro_cal_data.par_p6*baro_cal_data.comp_temp;
partial_data2 = baro_cal_data.par_p7*powf( baro_cal_data.comp_temp, 2 );
partial_data3 = baro_cal_data.par_p8*powf( baro_cal_data.comp_temp, 3 );
partial_out1  =  ( baro_cal_data.par_p5 + partial_data1 + 
                   partial_data2        + partial_data3 );

partial_data1 = baro_cal_data.par_p2*baro_cal_data.comp_temp;
partial_data2 = baro_cal_data.par_p3*powf( baro_cal_data.comp_temp, 2 );
partial_data3 = baro_cal_data.par_p4*powf( baro_cal_data.comp_temp, 3 );
partial_out2  = (float) raw_readout*( baro_cal_data.par_p1 + 
                                      partial_data1        +
									  partial_data2        +
									  partial_data3 );

partial_data1 = powf( ( (float) raw_readout ), 2 );
partial_data2 = ( baro_cal_data.par_p9 + 
                  baro_cal_data.par_p10*baro_cal_data.comp_temp );
partial_data3 = partial_data1*partial_data2;
partial_data4 = ( partial_data3 + 
                  powf( ( (float) raw_readout ), 3 )*baro_cal_data.par_p11 );

return partial_out1 + partial_out2 + partial_data4;

} /* press_compensate */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       baro_reset                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Performs a soft reset of the barometric pressure sensor                *
*                                                                              *
*******************************************************************************/
static BARO_STATUS baro_reset
	(
	void
	)
{
return write_reg( BARO_REG_CMD, BARO_CMD_RESET );
} /* baro_reset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       baro_flush_fifo                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Flushes the barometric pressure sensor FIFO buffer                     *
*                                                                              *
*******************************************************************************/
static BARO_STATUS baro_flush_fifo
	(
	void
	)
{
return write_reg( BARO_REG_CMD, BARO_CMD_FIFO_FLUSH );
} /* baro_flush_fifo */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/