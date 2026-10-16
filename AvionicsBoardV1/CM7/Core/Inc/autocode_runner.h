#ifndef AUTOCODE_RUNNER_H
#define AUTOCODE_RUNNER_H

#include "main.h"
#include "autocodeModel.h"
#include <stdint.h>

void Autocode_Init(void);
void Autocode_Run100Hz(const FlightData_t *flight_data);

uint32_t Autocode_GetPeriodUs(void);
const real_T *Autocode_GetControlDeg(void);
const navigation_bus *Autocode_GetNavigation(void);
const controller_bus *Autocode_GetController(void);

#endif
