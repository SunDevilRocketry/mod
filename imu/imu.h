/*******************************************************************************
*
* FILE: 
* 		imu.h
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* I2C Addresses */
#define IMU_ADDR                0x68<<1
#if   defined( A0002_REV1 )
    #define IMU_MAG_ADDR        0x0C<<1
#elif defined( A0002_REV2 )
    #define IMU_MAG_ADDR        0x10<<1
#endif

/* Device IDs */
#if   defined( A0002_REV1 )
    #define IMU_ID                  0x71
#elif defined( A0002_REV2 )
    #define IMU_ID                  0x24
    #define MAG_ID                  0x32
#endif

/* SDEC Subcommand Codes */
#define IMU_DUMP_CODE               0x01
#define IMU_POLL_CODE               0x02
#define IMU_LIST_CODE               0x03

/* Timeouts */
#define HAL_IMU_TIMEOUT             10

/* Register Bitmasks/Bitshifts */
#define MAG_XY_LSB_BITMASK          0b11111000
#define MAG_XY_LSB_BITSHIFT         3 /* Bit 3 to position 0 */
#define MAG_XY_MSB_BITSHIFT         5 /* Bit 0 to position 5 */
#define MAG_Z_LSB_BITMASK           0b11111110
#define MAG_Z_LSB_BITSHIFT          1 /* Bit 1 to position 0 */
#define MAG_Z_MSB_BITSHIFT          7 /* Bit 0 to position 7 */


/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
#if defined( A0002_REV1 )
    /* MPU 9250 Registers */
    #define IMU_REG_GYRO_CONFIG         0x1B
    #define IMU_REG_ACCEL_CONFIG        0x1C
    #define IMU_REG_ACCEL_XOUT_H        0x3B
    #define IMU_REG_ACCEL_XOUT_L        0x3C
    #define IMU_REG_ACCEL_YOUT_H        0X3D
    #define IMU_REG_ACCEL_YOUT_L        0x3E
    #define IMU_REG_ACCEL_ZOUT_H        0x3F
    #define IMU_REG_ACCEL_ZOUT_L        0x40
    #define IMU_REG_TEMP_OUT_H          0x41
    #define IMU_REG_TEMP_OUT_L          0x42
    #define IMU_REG_GYRO_XOUT_H         0x43
    #define IMU_REG_GYRO_XOUT_L         0x44
    #define IMU_REG_GYRO_YOUT_H         0X45
    #define IMU_REG_GYRO_YOUT_L         0x46
    #define IMU_REG_GYRO_ZOUT_H         0x47
    #define IMU_REG_GYRO_ZOUT_L         0x48
    #define IMU_REG_MAG_XOUT_H          0x04
    #define IMU_REG_MAG_XOUT_L          0x03
    #define IMU_REG_MAG_YOUT_H          0X06
    #define IMU_REG_MAG_YOUT_L          0x05
    #define IMU_REG_MAG_ZOUT_H          0x08
    #define IMU_REG_MAG_ZOUT_L          0x07
    #define IMU_REG_WHO_AM_I            0x75
#elif defined( A0002_REV2  )
    /* BMI270 Registers */
    #define IMU_REG_CHIP_ID             0x00
    #define IMU_REG_ERR_REG             0x02
    #define IMU_REG_STATUS              0x03
    #define IMU_REG_DATA_0              0x04    /* AUX_X (LSB) */
    #define IMU_REG_DATA_1              0x05    /* AUX_X (MSB) */
    #define IMU_REG_DATA_2              0x06    /* AUX_Y (LSB) */
    #define IMU_REG_DATA_3              0x07    /* AUX_Y (MSB) */
    #define IMU_REG_DATA_4              0x08    /* AUX_Z (LSB) */
    #define IMU_REG_DATA_5              0x09    /* AUX_Z (MSB) */
    #define IMU_REG_DATA_6              0x0A    /* AUX_R (LSB) */
    #define IMU_REG_DATA_7              0x0B    /* AUX_R (MSB) */
    #define IMU_REG_DATA_8              0x0C    /* ACC_X (LSB) */
    #define IMU_REG_DATA_9              0x0D    /* ACC_X (MSB) */
    #define IMU_REG_DATA_10             0x0E    /* ACC_Y (LSB) */
    #define IMU_REG_DATA_11             0x0F    /* ACC_Y (MSB) */
    #define IMU_REG_DATA_12             0x10    /* ACC_Z (LSB) */
    #define IMU_REG_DATA_13             0x11    /* ACC_Z (MSB) */
    #define IMU_REG_DATA_14             0x12    /* GYR_X (LSB) */
    #define IMU_REG_DATA_15             0x13    /* GYR_X (MSB) */
    #define IMU_REG_DATA_16             0x14    /* GYR_Y (LSB) */
    #define IMU_REG_DATA_17             0x15    /* GYR_Y (MSB) */
    #define IMU_REG_DATA_18             0x16    /* GYR_Z (LSB) */
    #define IMU_REG_DATA_19             0x17    /* GYR_Z (MSB) */
    #define IMU_REG_SENSORTIME_0        0x18
    #define IMU_REG_SENSORTIME_1        0x19
    #define IMU_REG_SENSORTIME_2        0x1A
    #define IMU_REG_EVENT               0x1B
    #define IMU_REG_INT_STATUS_0        0x1C
    #define IMU_REG_INT_STATUS_1        0x1D
    #define IMU_REG_SC_OUT_0            0x1E
    #define IMU_REG_SC_OUT_1            0x1F
    #define IMU_REG_WR_GEST_ACT         0x20
    #define IMU_REG_INTERNAL_STATUS     0x21
    #define IMU_REG_TEMPERATURE_0       0x22
    #define IMU_REG_TEMPERATURE_1       0x23
    #define IMU_REG_FIFO_LENGTH_0       0x24
    #define IMU_REG_FIFO_LENGTH_1       0x25
    #define IMU_REG_FIFO_DATA           0x26
    #define IMU_REG_FEAT_PAGE           0x2F
    #define IMU_REG_FEATURES            0x30
    #define IMU_REG_ACC_CONF            0x40
    #define IMU_REG_ACC_RANGE           0x41
    #define IMU_REG_GYR_CONF            0x42
    #define IMU_REG_GYR_RANGE           0x43
    #define IMU_REG_AUX_CONF            0x44
    #define IMU_REG_FIFO_DOWNS          0x45
    #define IMU_REG_FIFO_WTM_0          0x46
    #define IMU_REG_FIFO_WTM_1          0x47
    #define IMU_REG_FIFO_CONFIG_0       0x48
    #define IMU_REG_FIFO_CONFIG_1       0x49
    #define IMU_REG_SATURATION          0x4A
    #define IMU_REG_AUX_DEV_ID          0x4B
    #define IMU_REG_AUX_IF_CONF         0x4C
    #define IMU_REG_AUX_RD_ADDR         0x4D
    #define IMU_REG_AUX_WR_ADDR         0x4E
    #define IMU_REG_AUX_WR_DATA         0x4F
    #define IMU_REG_ERR_REG_MSK         0x52
    #define IMU_REG_INT1_IO_CTRL        0x53 
    #define IMU_REG_INT2_IO_CTRL        0x54
    #define IMU_REG_INT_LATCH           0x55
    #define IMU_REG_INT1_MAP_FEAT       0x56
    #define IMU_REG_INT2_MAP_FEAT       0x57
    #define IMU_REG_INT_MAP_DATA        0x58
    #define IMU_REG_INIT_CTRL           0x59
    #define IMU_REG_INIT_ADDR_0         0x5B
    #define IMU_REG_INIT_ADDR_1         0x5C
    #define IMU_REG_INIT_DATA           0x5E
    #define IMU_REG_INTERNAL_ERROR      0x5F
    #define IMU_REG_AUX_IF_TRIM         0x68
    #define IMU_REG_GYR_CRT_CONF        0x69
    #define IMU_REG_NVM_CONF            0x6A
    #define IMU_REG_IF_CONF             0x6B
    #define IMU_REG_DRV                 0x6C
    #define IMU_REG_ACC_SELF_TEST       0x6D
    #define IMU_REG_GYR_SELF_TEST_AXES  0x6E
    #define IMU_REG_NV_CONF             0x70
    #define IMU_REG_OFFSET_0            0x71
    #define IMU_REG_OFFSET_1            0x72
    #define IMU_REG_OFFSET_2            0x73
    #define IMU_REG_OFFSET_3            0x74
    #define IMU_REG_OFFSET_4            0x75
    #define IMU_REG_OFFSET_5            0x76
    #define IMU_REG_OFFSET_6            0x77
    #define IMU_REG_PWR_CONF            0x7C
    #define IMU_REG_PWR_CTRL            0x7D
    #define IMU_REG_CMD                 0x7E

    /* BMM150 Registers */
    #define MAG_REG_CHIP_ID             0x40
    #define MAG_REG_DATAX_L             0x42
    #define MAG_REG_DATAX_H             0x43
    #define MAG_REG_DATAY_L             0x44
    #define MAG_REG_DATAY_H             0x45
    #define MAG_REG_DATAZ_L             0x46
    #define MAG_REG_DATAZ_H             0x47
    #define MAG_REG_HALLR_L             0x48
    #define MAG_REG_HALLR_H             0x49
    #define MAG_REG_INT                 0x4A
    #define MAG_REG_PWR_CTRL            0x4B
    #define MAG_REG_CTRL1               0x4C
    #define MAG_REG_CTRL2               0x4D
    #define MAG_REG_CTRL3               0x4E
    #define MAG_REG_LOW_THRESH          0x4F
    #define MAG_REG_HIGH_THRESH         0x50
    #define MAG_REG_REP_CTRL_XY         0x51
    #define MAG_REG_REP_CTRL_Z          0x52
#endif

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Structure for imu containing all accel, gyro, and mag data */
typedef struct _IMU_DATA 
	{
    uint16_t    accel_x;
    uint16_t    accel_y;
    uint16_t    accel_z;
    uint16_t    gyro_x ;
    uint16_t    gyro_y ;
    uint16_t    gyro_z ;
    uint16_t    mag_x  ;
    uint16_t    mag_y  ;
    uint16_t    mag_z  ;
	uint16_t    temp   ;
	} IMU_DATA;

/* Sensor Enable Configuration */
typedef enum _IMU_SENSOR_ENABLE
    {
    IMU_DISABLE_SENSORS      = 0b00000000,
    IMU_ENABLE_AUX           = 0b00000001,
    IMU_ENABLE_GYRO          = 0b00000010, 
    IMU_ENABLE_ACC           = 0b00000100,
    IMU_ENABLE_TEMP          = 0b00001000,
    IMU_ENABLE_AUX_GYRO      = 0b00000011,
    IMU_ENABLE_AUX_ACC       = 0b00000101,
    IMU_ENABLE_AUX_GYRO_ACC  = 0b00000111,
    IMU_ENABLE_GYRO_ACC_TEMP = 0b00001110,
    IMU_ENABLE_ALL           = 0b00001111
    } IMU_SENSOR_ENABLE;

/* Output Data Rate Configuration */
typedef enum _IMU_ODR_SETTING
    {
    IMU_ODR_0P78 = 1, /* 25/32 Hz */ 
    IMU_ODR_1P5     , /* 25/16 Hz */
    IMU_ODR_3P1     , /* 25/8  Hz */
    IMU_ODR_6P25    , /* 25/4  Hz */
    IMU_ODR_12P5    , /* 25/2  Hz */
    IMU_ODR_25      , /* 25    Hz */
    IMU_ODR_50      , /* 50    Hz */
    IMU_ODR_100     , /* 100   Hz */
    IMU_ODR_200     , /* 200   Hz */
    IMU_ODR_400     , /* 400   Hz */
    IMU_ODR_800     , /* 800   Hz */
    IMU_ODR_1K6     , /* 1.6  kHz */
    IMU_ODR_3K2
    } IMU_ODR_SETTING;

/* Filtering Configuration */
typedef enum _IMU_FILTER_CONFIG
    {
    IMU_FILTER_OSR4_AVG1 = 0            , /* OSR4 Filter, No Average       */
    IMU_FILTER_OSR2_AVG2 = ( 0x01 << 4 ), /* OSR2 Filter, 2 Sample Average */
    IMU_FILTER_NORM_AVG4 = ( 0x02 << 4 ), /* Normal Mode, 4 Sample Average */
    IMU_FILTER_CIC_AVG8  = ( 0x03 << 4 ), /* CIC Filter , 8 Sample Average */
    IMU_FILTER_AVG16     = ( 0x04 << 4 ), /* 16 Sample Average             */
    IMU_FILTER_AVG32     = ( 0x05 << 4 ), /* 32 Sample Average             */
    IMU_FILTER_AVG64     = ( 0x06 << 4 ), /* 64 Sample Average             */
    IMU_FILTER_AVG128    = ( 0x07 << 4 )  /* 128 Sample Average            */
    } IMU_FILTER_CONFIG;

/* Filter Mode */
typedef enum _IMU_FILTER_MODE
    {
    IMU_FILTER_FILTER_MODE  = ( 0x01 << 7 ),
    IMU_FILTER_AVERAGE_MODE = ( 0x00 << 7 )
    } IMU_FILTER_MODE;

/* Accelerometer Measurement Range */
typedef enum _IMU_ACC_RANGE
    {
    IMU_ACC_RANGE_2G = 0, /* +- 2g  */ 
    IMU_ACC_RANGE_4G    , /* +- 4g  */
    IMU_ACC_RANGE_8G    , /* +- 8g  */
    IMU_ACC_RANGE_16G     /* +- 16g */
    } IMU_ACC_RANGE;

/* Gyroscope Measurement Range */
typedef enum _IMU_GYRO_RANGE
    {
    IMU_GYRO_RANGE_2000 = 0, /* +- 2000 deg/s */
    IMU_GYRO_RANGE_1000    , /* +- 1000 deg/s */
    IMU_GYRO_RANGE_500     , /* +- 500  deg/s */
    IMU_GYRO_RANGE_250     , /* +- 250  deg/s */
    IMU_GYRO_RANGE_125       /* +- 125  deg/s */
    } IMU_GYRO_RANGE;

/* Magnetometer Output Data Rates */
typedef enum _MAG_ODR_SETTING
    {
    MAG_ODR_10HZ = ( 0b000 << 3 ),
    MAG_ODR_2HZ  = ( 0b001 << 3 ),
    MAG_ODR_6HZ  = ( 0b010 << 3 ),
    MAG_ODR_8HZ  = ( 0b011 << 3 ),
    MAG_ODR_15HZ = ( 0b100 << 3 ),
    MAG_ODR_20HZ = ( 0b101 << 3 ),
    MAG_ODR_25HZ = ( 0b110 << 3 ),
    MAG_ODR_30HZ = ( 0b111 << 3 )
    } MAG_ODR_SETTING;

/* Magnetometer Operation Mode */
typedef enum _MAG_OP_MODE
    {
    MAG_NORMAL_MODE = ( 0b00 << 1 ),
    MAG_FORCED_MODE = ( 0b01 << 1 ),
    MAG_SLEEP_MODE  = ( 0b11 << 1 )
    } MAG_OP_MODE;

/* User IMU configuration settings */
typedef struct _IMU_CONFIG 
	{
    IMU_SENSOR_ENABLE sensor_enable;      /* Enabled Sensors                    */
    IMU_ODR_SETTING   acc_odr;            /* Accelerometer Output Data Rate     */ 
    IMU_ODR_SETTING   gyro_odr;           /* Gyroscope Output Data Rate         */
    MAG_ODR_SETTING   mag_odr;            /* Magnetometer Output Data Rate      */
    IMU_FILTER_CONFIG acc_filter;         /* Accelerometer Filter Config        */
    IMU_FILTER_CONFIG gyro_filter;        /* Gyroscope Filter Config            */
    IMU_FILTER_MODE   acc_filter_mode;    /* Accelerometer Filtering Mode       */
    IMU_FILTER_MODE   gyro_filter_mode;   /* Gyroscope Filtering Mode           */
    IMU_ACC_RANGE     acc_range;          /* Accelerometer Measurement Range    */
    IMU_GYRO_RANGE    gyro_range;         /* Gyroscope Measurement Range        */
    MAG_OP_MODE       mag_op_mode;        /* Magnetometer Operation Mode        */
    uint8_t           mag_xy_repititions; /* Magnetometer XY Measurement Reps   */
    uint8_t           mag_z_repititions;  /* Magnetometer Z  Measurement Reps   */
	} IMU_CONFIG;

/* IMU Status */
typedef enum IMU_STATUS
	{
    IMU_OK              = 0,
    IMU_FAIL               ,
    IMU_UNSUPPORTED_OP     ,
    IMU_UNRECOGNIZED_OP    ,
    IMU_TIMEOUT            , 
    IMU_I2C_ERROR          ,
    IMU_MAG_ERROR          ,
    IMU_ERROR              ,
    IMU_INIT_FAIL          ,
    IMU_CONFIG_FAIL        ,
    IMU_MAG_UNRECOGNIZED_ID,
    IMU_MAG_INIT_FAIL
	} IMU_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Initialize the IMU */
IMU_STATUS imu_init
    (
    IMU_CONFIG* imu_config_ptr /* IMU Configuration Settings */
    );

/* Return the pointer to structure that updates the x,y,z acceleration values 
   from the IMU */
IMU_STATUS imu_get_accel_xyz
    (
    IMU_DATA *pIMU
    );

/* Return the pointer to structure that updates the x,y,z gyro values from the IMU */
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_DATA *pIMU
    );

/* Return the pointer to structure that updates the x,y,z magnetometer values from 
   the IMU */
IMU_STATUS imu_get_mag_xyz
    (
    IMU_DATA *pIMU
    );

/* return the device ID of the IMU to verify that the IMU registers are accessible */
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    );

/* Change configuration of accel, gyro, mag */
void IMU_config
    (
    IMU_CONFIG *pimu_config,
    uint8_t accel_setting,
    uint16_t gyro_setting,
    uint16_t mag_setting
    );


#ifdef __cplusplus
}
#endif
#endif /* IMU_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/