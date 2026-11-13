#include "gps.h"
#include "usart.h"
#include "system_health.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

#define GPS_LINE_SIZE 96U
#define GPS_RX_BUFFER_SIZE 256U
#define GPS_RX_MASK (GPS_RX_BUFFER_SIZE - 1U)
#define GPS_RX_FLAG 1U

volatile gps_data_t gps_data;
static uint8_t rx_byte;
static uint8_t rx_buffer[GPS_RX_BUFFER_SIZE];
static volatile uint16_t rx_head, rx_tail;
static char line_buffer[GPS_LINE_SIZE];
static uint8_t line_length;
extern osThreadId_t GPSTaskHandle;

HAL_StatusTypeDef GPS_Init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    line_length = 0U;
    return HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) {
        uint16_t next = (uint16_t)((rx_head + 1U) & GPS_RX_MASK);
        if (next != rx_tail) {
            rx_buffer[rx_head] = rx_byte;
            rx_head = next;
            (void)osThreadFlagsSet(GPSTaskHandle, GPS_RX_FLAG);
        }
        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

static double NmeaCoordinate(const char *value, char hemisphere)
{
    double raw = atof(value);
    double degrees = (double)((int)(raw / 100.0));
    double coordinate = degrees + (raw - degrees * 100.0) / 60.0;
    return (hemisphere == 'S' || hemisphere == 'W') ? -coordinate : coordinate;
}

uint8_t GPS_ParseLine(char *line, gps_data_t *data)
{
    char *fields[16];
    uint8_t count = 0U;
    char *field = strtok(line, ",");
    while (field != NULL && count < 16U) {
        fields[count++] = field;
        field = strtok(NULL, ",");
    }
    if (count < 10U || (strcmp(fields[0], "$GPGGA") && strcmp(fields[0], "$GNGGA"))) return 0U;
    data->fix = (uint8_t)atoi(fields[6]);
    data->satellites = (uint8_t)atoi(fields[7]);
    if (data->fix == 0U || fields[2][0] == '\0' || fields[4][0] == '\0') return 0U;
    data->latitude_deg = NmeaCoordinate(fields[2], fields[3][0]);
    data->longitude_deg = NmeaCoordinate(fields[4], fields[5][0]);
    data->altitude_m = (float)atof(fields[9]);
    data->tick_ms = HAL_GetTick();
    return 1U;
}

static void GPS_ProcessByte(uint8_t byte)
{
    if (byte == '\n') {
        line_buffer[line_length] = '\0';
        gps_data_t parsed = {0};
        if (GPS_ParseLine(line_buffer, &parsed)) {
            taskENTER_CRITICAL();
            gps_data = parsed;
            taskEXIT_CRITICAL();
            SystemHealth_ReportSensor(SENSOR_GPS, HAL_OK);
        }
        line_length = 0U;
    } else if (byte != '\r') {
        if (line_length < GPS_LINE_SIZE - 1U) line_buffer[line_length++] = (char)byte;
        else line_length = 0U;
    }
}

void GPS_Task(void *argument)
{
    (void)argument;
    for (;;) {
        (void)osThreadFlagsWait(GPS_RX_FLAG, osFlagsWaitAny, osWaitForever);
        while (rx_tail != rx_head) {
            uint8_t byte = rx_buffer[rx_tail];
            rx_tail = (uint16_t)((rx_tail + 1U) & GPS_RX_MASK);
            GPS_ProcessByte(byte);
        }
    }
}
