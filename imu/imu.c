/*******************************************************************************
*
* FILE: 
* 		imu.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <string.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "imu.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* BMI270 Initialization File */
#ifdef A0002_REV2
    uint8_t bmi270_init_file[] = {
        #include "bmi270_init_file.tbin"
    };
#endif

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Initialize the magnetometer */
#if defined( A0002_REV2 )
static IMU_STATUS mag_init
    (
    IMU_CONFIG* imu_config_ptr
    );
#endif

/* Read IMU registers */
static IMU_STATUS read_imu_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/* Write to a specified IMU register */
#if defined( A0002_REV2 )
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    );

/* Write IMU registers */
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint32_t num_regs  /* Number of registers */
    ); 
#endif /* #if defined( A0002_REV2 )*/

/* Read Magnetometer registers */
static IMU_STATUS read_mag_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/* Write to a specified magnetometer register */
#if defined( A0002_REV2 )
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ); 
#endif 


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

#if defined( A0002_REV2 )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		imu_init                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the IMU                                                     *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_init 
	(
    IMU_CONFIG* imu_config_ptr /* IMU Configuration */ 
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS imu_status;          /* IMU API call return codes       */
uint8_t    imu_dev_id;          /* IMU identification code         */
uint8_t    imu_status_reg;      /* Contents of IMU status register */
uint8_t    imu_acc_conf;        /* IMU ACC_CONF Register contents  */
uint8_t    imu_gyr_conf;        /* IMU GYR_CONF Register contents  */
uint8_t    imu_sensor_data[12]; /* IMU Sensor Data                 */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
imu_status     = IMU_OK;
imu_dev_id     = 0;
imu_status_reg = 0;
imu_acc_conf   = ( imu_config_ptr -> acc_odr         ) |
                 ( imu_config_ptr -> acc_filter      ) |
                 ( imu_config_ptr -> acc_filter_mode );
imu_gyr_conf   = ( imu_config_ptr -> gyro_odr        ) |
                 ( imu_config_ptr -> gyro_filter     ) |
                 ( 1 << 7 );
memset( &imu_sensor_data[0], 0, sizeof( imu_sensor_data ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read IMU ID and verify correct ID */
imu_status = imu_get_device_id( &imu_dev_id );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* BMI270 Initialization Sequence */
#if defined( A0002_REV2 )
    /* Disable advanced power save */
    imu_status = write_imu_reg( IMU_REG_PWR_CONF, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    HAL_Delay( 1 );

    /* Prepare Config Load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    
    /* Load the initialization data */
    imu_status = write_imu_regs( IMU_REG_INIT_DATA   , 
                                 &bmi270_init_file[0], 
                                 sizeof( bmi270_init_file ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Complete config load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x01 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Check if Initialization was Successful */
    HAL_Delay( 150 );
    imu_status = read_imu_regs( IMU_REG_INTERNAL_STATUS, 
                                &imu_status_reg        ,
                                sizeof( imu_status_reg ) );
    if ( imu_status != IMU_OK || !( imu_status_reg & 0b00000001 ) )
        {
        return IMU_INIT_FAIL;
        }
#endif /* #if defined( A0002_REV2 ) */

/* Initial IMU Configuration */
#if defined( A0002_REV2 )
    /* Enable Sensors */
    imu_status = write_imu_reg( IMU_REG_PWR_CTRL, 
                                imu_config_ptr -> sensor_enable );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Configure the Accelerometer */
    imu_status = write_imu_reg( IMU_REG_ACC_CONF, imu_acc_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = write_imu_reg( IMU_REG_ACC_RANGE,
                                imu_config_ptr -> acc_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    
    /* Configure the Gyroscope */
    imu_status = write_imu_reg( IMU_REG_GYR_CONF, imu_gyr_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = write_imu_reg( IMU_REG_GYR_RANGE, 
                                imu_config_ptr -> gyro_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Disable Advanced Power Save */
    imu_status = write_imu_reg( IMU_REG_PWR_CONF, 0x02 );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Readout sensor registers */
    imu_status = read_imu_regs( IMU_REG_DATA_8     , 
                                &imu_sensor_data[0], 
                                sizeof( imu_sensor_data ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL; 
        }

    /* Initialize the magnetometer */
    imu_status = mag_init( imu_config_ptr );
    if ( imu_status != IMU_OK )
        {
        return IMU_MAG_INIT_FAIL;
        }
#endif /* #if defined( A0002_REV2 ) */

/* IMU Inititialization Successful */
return IMU_OK;
} /* imu_init */
#endif /* #if defined( A0002_REV2 ) */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_accel_xyz                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the                       * 
*       x,y,z acceleration values from the IMU                                 *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_accel_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t       regAccel[6];    /* Bytes from accelerometer registers */
uint16_t      accel_x_raw ;   /* Raw sensor readouts                */    
uint16_t      accel_y_raw ;  
uint16_t      accel_z_raw ; 
IMU_STATUS    imu_status;     /* IMU status codes                   */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read ACCEL_X, ACCEL_Y, ACCEL_Z high byte and low byte registers */
#if   defined( A0002_REV1 )
    imu_status = read_imu_regs( IMU_REG_ACCEL_XOUT_H, 
                                &regAccel[0]        , 
                                sizeof( regAccel ) );
#elif defined( A0002_REV2 )
    imu_status = read_imu_regs( IMU_REG_DATA_8, 
                                &regAccel[0]  ,
                                sizeof( regAccel ) );
#endif

/* Check for HAL IMU error */
if ( imu_status != IMU_OK )
	{
	return imu_status;
	}

/* Combine high byte and low byte to 16 bit data */ 
#if   defined( A0002_REV1 )
    accel_x_raw    = ( (uint16_t) regAccel[0] ) << 8  | regAccel[1];
    accel_y_raw    = ( (uint16_t) regAccel[2] ) << 8  | regAccel[3];
    accel_z_raw    = ( (uint16_t) regAccel[4] ) << 8  | regAccel[5]; 
#elif defined( A0002_REV2 )
    accel_x_raw    = ( (uint16_t) regAccel[1] ) << 8  | regAccel[0];
    accel_y_raw    = ( (uint16_t) regAccel[3] ) << 8  | regAccel[2];
    accel_z_raw    = ( (uint16_t) regAccel[5] ) << 8  | regAccel[4]; 
#endif

/* Export data to IMU sstruct */
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw;

return IMU_OK;
} /* imu_get_accel_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_gryo_xyz                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the x,y,z gyro values     * 
*       from the IMU                                                           *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regGyro[6];   /* Bytes from gyro registers */
uint16_t    gyro_x_raw;   /* Raw gyro sensor readouts  */
uint16_t    gyro_y_raw; 
uint16_t    gyro_z_raw; 
IMU_STATUS  imu_status;   /* IMU status return codes   */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read GYRO_X, GYRO_Y, GYRO_Z high byte and low byte registers */
#if   defined( A0002_REV1 )
    imu_status = read_imu_regs( IMU_REG_GYRO_XOUT_H, 
                                &regGyro[0]        , 
                                sizeof( regGyro ) );
#elif defined( A0002_REV2 )
    imu_status = read_imu_regs( IMU_REG_DATA_14, 
                                &regGyro[0]    , 
                                sizeof( regGyro ) );
#endif
 
/* Check for HAL IMU error */
if ( imu_status != IMU_OK )
	{
	return imu_status;
	}

/* Combine high byte and low byte to 16 bit data  */
#if   defined( A0002_REV1 )
    gyro_x_raw = ( (uint16_t) regGyro[0] ) << 8 | regGyro[1];
    gyro_y_raw = ( (uint16_t) regGyro[0] ) << 8 | regGyro[1];
    gyro_z_raw = ( (uint16_t) regGyro[0] ) << 8 | regGyro[1];
#elif defined( A0002_REV2 )
    gyro_x_raw = ( (uint16_t) regGyro[1] ) << 8 | regGyro[0];
    gyro_y_raw = ( (uint16_t) regGyro[3] ) << 8 | regGyro[2];
    gyro_z_raw = ( (uint16_t) regGyro[5] ) << 8 | regGyro[4];
#endif

/* Export Sensor Readouts */
pIMU->gyro_x = gyro_x_raw;
pIMU->gyro_y = gyro_y_raw;
pIMU->gyro_z = gyro_z_raw; 

return IMU_OK;
} /* imu_get_gyro_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_mag_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the x,y,z magnetometer    *
*       values from the IMU                                                    *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_mag_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regMag[6];    /* Magnetometer register bytes      */
uint16_t    mag_x_raw;    /* Raw magnetometer sensor readouts */ 
uint16_t    mag_y_raw; 
uint16_t    mag_z_raw; 
IMU_STATUS  imu_status;   /* IMU status return codes          */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers */
#if   defined( A0002_REV1 )
    imu_status = read_mag_regs( IMU_REG_MAG_XOUT_H, 
                                &regMag[0]        , 
                                sizeof( regMag ) );
#elif defined( A0002_REV2 )
    imu_status = read_mag_regs( MAG_REG_DATAX_L, 
                                &regMag[0]     , 
                                sizeof( regMag ) );
#endif

/* Check for HAL IMU error */
if ( imu_status == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

/* Combine high byte and low byte to 16 bit data */
#if   defined( A0002_REV1 )
    mag_x_raw  = ( (uint16_t) regMag[1] ) << 8 | regMag[0];
    mag_y_raw  = ( (uint16_t) regMag[3] ) << 8 | regMag[2];
    mag_z_raw  = ( (uint16_t) regMag[5] ) << 8 | regMag[4];
#elif defined( A0002_REV2 )
    mag_x_raw  = (   (uint16_t) regMag[1]                         << MAG_XY_MSB_BITSHIFT ) | 
                 ( ( (uint16_t) regMag[0] && MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
    mag_y_raw  = (   (uint16_t) regMag[3]                         << MAG_XY_MSB_BITSHIFT ) | 
                 ( ( (uint16_t) regMag[2] && MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
    mag_z_raw  = (   (uint16_t) regMag[5]                         << MAG_Z_MSB_BITSHIFT  ) | 
                 ( ( (uint16_t) regMag[4] && MAG_Z_LSB_BITMASK )  >> MAG_Z_LSB_BITSHIFT  );
#endif

/* Export sensor data */
pIMU->mag_x = mag_x_raw;
pIMU->mag_y = mag_y_raw;
pIMU->mag_z = mag_z_raw;

return IMU_OK;
} /* imu_get_mag_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		imu_get_device_id                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		return the device ID of the IMU to verify that the                     *
*       IMU registers are accessible                                           *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS  imu_status;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read Device ID register */
#if defined( A0002_REV1 )
    imu_status = read_imu_regs( IMU_REG_WHO_AM_I, pdevice_id, sizeof( uint8_t ) );
#elif defined( A0002_REV2 )
    imu_status = read_imu_regs( IMU_REG_CHIP_ID, pdevice_id, sizeof( uint8_t ) );
#endif

if ( *pdevice_id != IMU_ID )
    {
    imu_status = IMU_UNRECOGNIZED_OP;
    }

return imu_status;
} /* imu_get_device_id */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

#if defined( A0002_REV2 )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		mag_init                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the magnetometer                                            *
*                                                                              *
*******************************************************************************/
static IMU_STATUS mag_init
    (
    IMU_CONFIG* imu_config_ptr
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
IMU_STATUS imu_status;      /* Status return codes from IMU API     */
uint8_t    device_id;       /* Magnetometer Device ID               */
uint8_t    num_reps_xy_reg; /* Content of XY repetititions register */
uint8_t    num_reps_z_reg;  /* Content of Z repititions register    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
imu_status      = IMU_OK;
device_id       = 0;
num_reps_xy_reg = ( ( imu_config_ptr -> mag_xy_repititions ) - 1 ) >> 2;
num_reps_z_reg  = ( ( imu_config_ptr -> mag_z_repititions  ) - 1 );


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Put the Magnetometer into sleep mode from suspend mode */
imu_status = write_mag_reg( MAG_REG_PWR_CTRL, 0x01 );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Check Device ID */
HAL_Delay( 5 );
imu_status = read_mag_regs( MAG_REG_CHIP_ID, &device_id, sizeof( device_id ) );
if      ( imu_status != IMU_OK )
    {
    return imu_status;
    }
else if ( device_id != MAG_ID )
    {
    return IMU_MAG_UNRECOGNIZED_ID; 
    }

/* Set the magnetometer operating mode and output data rate */
imu_status = write_mag_reg( MAG_REG_CTRL1, 
                            ( imu_config_ptr -> mag_op_mode ) |
                            ( imu_config_ptr -> mag_odr     ) ); 
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Set the magnetometer measurement repetitions */
imu_status = write_mag_reg( MAG_REG_REP_CTRL_XY, num_reps_xy_reg );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }
imu_status = write_mag_reg( MAG_REG_REP_CTRL_Z, num_reps_z_reg );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Successful magnetometer Initialization */
return IMU_OK;
} /* mag_init */
#endif /* #if defined( A0002_REV2 ) */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		read_mag_regs                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Read the specific numbers of registers at one time from magnetometer   *
*       module in the IMU                                                      *
*                                                                              *
*******************************************************************************/
static IMU_STATUS read_mag_regs 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status return code of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C registers */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_MAG_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_IMU_TIMEOUT );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
	{
	return IMU_MAG_ERROR;
	}
else 
	{
	return IMU_OK;
	}

} /* read_mag_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		read_imu_regs                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Read the specific numbers of registers at one time from acceleration   *
*       and gyroscope module in the IMU                                        *
*                                                                              *
*******************************************************************************/
static IMU_STATUS read_imu_regs 
    (
    uint8_t  reg_addr, /* Register address            */
    uint8_t* data_ptr, /* Register data               */ 
    uint8_t  num_regs  /* Number of registers to read */
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read I2C register */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_ADDR            , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_MAX_DELAY );

if ( hal_status != HAL_OK )
	{
	return IMU_ERROR;
	}
else
	{
	return IMU_OK;
	}
} /* read_imu_regs */


#if defined( A0002_REV2 )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_imu_reg                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to a specified IMU register                                      *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_IMU_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_reg */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_imu_regs                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to specified IMU registers                                       *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint32_t num_regs  /* Number of registers */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C ), 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                data_ptr            , 
                                num_regs            , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_mag_reg                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to a specified magnetometer register                             *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_MAG_ADDR        , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_IMU_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_mag_regs */
#endif /* #if defined( A0002_REV2 ) */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/