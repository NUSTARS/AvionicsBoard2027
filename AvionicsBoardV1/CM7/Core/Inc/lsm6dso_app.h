#ifndef LSM6DSO_APP_H
#define LSM6DSO_APP_H

#include "main.h"
#include "lsm6dso.h"

/* Exported Functions */
int32_t LSM6DSO_App_Init(void);
void    LSM6DSO_App_SetHardwareFlag(void);
extern volatile uint32_t lsm_drdy_count, lsm_drdy_busy_count, lsm_spi_error_count;

#endif /* LSM6DSO_APP_H */
