/*******************************************************************************
*
* FILE: 
* 		sensor_fusion.h
*
* DESCRIPTION: 
* 		Contains functions to obtain sensor fusion and post processing
*       calculation of the sensor data
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

void sf_eliminate_accel_offset(IMU_DATA* imu_data);
void sf_eliminate_gyro_offset(IMU_DATA* imu_data);
void sf_get_roll_angle(IMU_DATA* imu_data_ptr);
void sf_get_pitch_angle(IMU_DATA* imu_data_ptr);
void sf_get_roll_rate(IMU_DATA* imu_data_ptr);
void sf_get_pitch_rate(IMU_DATA* imu_data_ptr);
void sf_get_imu_velo_x(IMU_DATA* imu_data_ptr);
void sf_get_imu_velo_y(IMU_DATA* imu_data_ptr);
void sf_get_imu_velo_z(IMU_DATA* imu_data_ptr);
void sf_get_imu_velocity(IMU_DATA* imu_data_ptr);
void sf_get_baro_alt(SENSOR_DATA* sensor_data_ptr);
void sf_get_baro_velo(SENSOR_DATA* sensor_data_ptr);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_FUSION_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/