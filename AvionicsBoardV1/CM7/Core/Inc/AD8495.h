#ifndef AD8495_H
#define AD8495_H

#include "main.h"

/* Calibrate ADC1 and start the first single conversion in foreground. */
HAL_StatusTypeDef AD8495_Init(void);

/* Consume a completed conversion and start the next. Never waits. */
#define AD8495_CHANNEL_COUNT 2U
typedef struct {
    uint16_t raw;
    float temp_c;
} AD8495_Reading;

HAL_StatusTypeDef AD8495_Read(AD8495_Reading readings[AD8495_CHANNEL_COUNT]);

#endif
