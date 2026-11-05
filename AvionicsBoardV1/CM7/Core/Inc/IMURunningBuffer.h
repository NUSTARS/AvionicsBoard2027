#ifndef IMU_RUNNING_BUFFER_H
#define IMU_RUNNING_BUFFER_H
#include "main.h"

/* Initialize before enabling the producer IRQ. Units: m/s^2 per accel LSB,
 * deg/s per gyro LSB. The scales stay constant during acquisition. */
void IMURunningBuffer_Init(double accel_units_per_lsb, double gyro_units_per_lsb);
/* Producer API: call from serialized acquisition IRQs; six raw little-endian
 * bytes per sensor. No SPI access, heap, or floating-point work in this path. */
void IMURunningBuffer_AddAccel(const uint8_t raw[6]);
void IMURunningBuffer_AddGyro(const uint8_t raw[6]);
/* Main-loop consumer: discard startup samples, or consume one complete window.
 * ReadAverage returns zero without modifying data if either sensor is empty.
 * A short bank swap masks IRQs; division/conversion run with IRQs restored. */
void IMURunningBuffer_Reset(void);
uint8_t IMURunningBuffer_ReadAverage(FlightData_t *data);
/* Existing debugger symbols retained for continuity. */
extern volatile uint32_t lsm_accel_sample_count, lsm_gyro_sample_count;
extern volatile uint32_t lsm_last_accel_samples, lsm_last_gyro_samples;
extern volatile uint32_t lsm_empty_window_count, lsm_accumulator_overflow_count;
#endif
