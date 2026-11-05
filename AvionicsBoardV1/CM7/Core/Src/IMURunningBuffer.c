#include "IMURunningBuffer.h"
#include <string.h>

/* IRQ producer / model-loop consumer: six exact raw integer sums, two counts.
 * No ring storage, per-sample division, floating point, or heap allocation. */
typedef struct { int32_t sum[6]; uint32_t count[2]; } ImuSums;
static ImuSums imu_sums[2];
static volatile unsigned active_bank;
static double accel_scale, gyro_scale;
volatile uint32_t lsm_accel_sample_count, lsm_gyro_sample_count;
volatile uint32_t lsm_last_accel_samples, lsm_last_gyro_samples;
volatile uint32_t lsm_empty_window_count, lsm_accumulator_overflow_count;

void IMURunningBuffer_Init(double accel_units_per_lsb, double gyro_units_per_lsb)
{
    accel_scale = accel_units_per_lsb;
    gyro_scale = gyro_units_per_lsb;
    IMURunningBuffer_Reset();
}

static void AccumulateAxes(const uint8_t *raw, unsigned first, unsigned sensor)
{
    ImuSums *bank = &imu_sums[active_bank];
    /* 65535 * -32768 still fits int32. Bound long SD stalls without overflow. */
    if (bank->count[sensor] >= 65535U) {
        lsm_accumulator_overflow_count++;
        return;
    }
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        uint16_t bits = (uint16_t)raw[2U * axis] | ((uint16_t)raw[2U * axis + 1U] << 8);
        int32_t value = bits < 32768U ? (int32_t)bits : (int32_t)bits - 65536;
        bank->sum[first + axis] += value;
    }
    bank->count[sensor]++;
    if (sensor == 0U) lsm_accel_sample_count++;
    else lsm_gyro_sample_count++;
}

void IMURunningBuffer_AddAccel(const uint8_t raw[6])
{
    AccumulateAxes(raw, 0U, 0U);
}
void IMURunningBuffer_AddGyro(const uint8_t raw[6])
{
    AccumulateAxes(raw, 3U, 1U);
}

void IMURunningBuffer_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(imu_sums, 0, sizeof(imu_sums));
    active_bank = 0U;
    __set_PRIMASK(primask);
}

uint8_t IMURunningBuffer_ReadAverage(FlightData_t *data)
{
    if (data == NULL) return 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    unsigned old = active_bank;
    if (imu_sums[old].count[0] == 0U || imu_sums[old].count[1] == 0U) {
        __set_PRIMASK(primask);
        lsm_empty_window_count++;
        return 0U;
    }
    /* Inactive bank was cleared by the previous consumer. Only the pointer
     * swap is protected: IRQs resume while conversion/division runs below. */
    active_bank = old ^ 1U;
    __DMB();
    __set_PRIMASK(primask);
    ImuSums batch = imu_sums[old];
    memset(&imu_sums[old], 0, sizeof(imu_sums[old]));
    lsm_last_accel_samples = batch.count[0];
    lsm_last_gyro_samples = batch.count[1];
    double ascale = accel_scale / (double)batch.count[0];
    double gscale = gyro_scale / (double)batch.count[1];
    data->lsm_accel.x = (double)batch.sum[0] * ascale;
    data->lsm_accel.y = (double)batch.sum[1] * ascale;
    data->lsm_accel.z = (double)batch.sum[2] * ascale;
    data->lsm_gyro.x = (double)batch.sum[3] * gscale;
    data->lsm_gyro.y = (double)batch.sum[4] * gscale;
    data->lsm_gyro.z = (double)batch.sum[5] * gscale;
    return 1U;
}
