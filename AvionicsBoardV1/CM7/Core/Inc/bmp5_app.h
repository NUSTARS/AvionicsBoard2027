#ifndef BMP5_APP_H
#define BMP5_APP_H

#include "main.h"
#include "bmp5.h"

/* App Layer Return Status Codes */
#define BMP5_APP_OK      0
#define BMP5_APP_ERROR  -1

/* Public Module APIs */
int32_t BMP5_App_Init(void);
void BMP5_App_Process(FlightData_t *data);

#endif /* BMP5_APP_H */
