#ifndef LSM6DSO_APP_H
#define LSM6DSO_APP_H

#include "main.h"
#include "lsm6dso.h"

/* Exported Functions */
int32_t LSM6DSO_App_Init(void);
void    LSM6DSO_App_Process(FlightData_t *data);
void    LSM6DSO_App_SetHardwareFlag(void);

#endif /* LSM6DSO_APP_H */
