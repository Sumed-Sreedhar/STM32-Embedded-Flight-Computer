/*
 * sensor_data.h
 *
 *  Created on: 30-Jul-2026
 *      Author: sumed
 */

#ifndef INC_SENSOR_DATA_H_
#define INC_SENSOR_DATA_H_

#include "stdint.h"

typedef struct
{
	/* BMP280 Measurements*/
    int32_t temperature_raw;
    int32_t pressure_raw;
    float temperature;
    float pressure;
    float altitude;

    /* MPU9250 Measurements*/
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
    int16_t temp_raw;

    /* MLX90393 Measurements */
    int16_t mag_raw_x;
    int16_t mag_raw_y;
    int16_t mag_raw_z;

    float mag_x_uT;
    float mag_y_uT;
    float mag_z_uT;

} sensor_data_t;

#define SENSOR_FRAME_IMU_VALID   (1U << 0)
#define SENSOR_FRAME_BARO_VALID  (1U << 1)
#define SENSOR_FRAME_MAG_VALID   (1U << 2)

/* Latest sensor values and the tick of each sensor's last successful read. */
typedef struct
{
	uint32_t imu_tick_ms;
	uint32_t baro_tick_ms;
	uint32_t mag_tick_ms;
	uint8_t valid_mask;

	float accel_g[3];
	float gyro_dps[3];
	float mag_uT[3];
	float temperature_c;
	float pressure_pa;

	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	int16_t imu_temp_raw;
	int16_t mag_raw[3];
	int32_t baro_temp_raw;
	int32_t baro_pressure_raw;

} sensor_frame_t;

extern sensor_data_t sensor_data;


#endif /* INC_SENSOR_DATA_H_ */
