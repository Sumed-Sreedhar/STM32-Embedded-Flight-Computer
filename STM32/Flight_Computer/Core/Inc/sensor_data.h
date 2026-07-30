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



} sensor_data_t;

extern sensor_data_t sensor_data;


#endif /* INC_SENSOR_DATA_H_ */
