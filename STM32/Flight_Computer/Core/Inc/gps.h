#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stdint.h>

typedef struct {
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    uint32_t tick_ms;
    uint8_t fix;
    uint8_t satellites;
} gps_data_t;
extern volatile gps_data_t gps_data;

HAL_StatusTypeDef GPS_Init(void);
void GPS_Task(void *argument);
uint8_t GPS_ParseLine(char *line, gps_data_t *data);

#endif
