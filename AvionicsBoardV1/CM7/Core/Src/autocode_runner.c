#include "main.h"
#include "autocode_runner.h"
#include <stdint.h>
#include <stdbool.h>

/* Defined by CubeMX in your project */
extern TIM_HandleTypeDef htim5;

/* Generated model data */
static RT_MODEL rtM_;
static RT_MODEL *const rtM = &rtM_;
static DW rtDW;

static sensor_bus rtU_sensor_bus_e;
static navigation_bus rtY_navigation_bus_d;
static controller_bus rtY_controller_bus_f;
static real_T rtY_control_deg[4];

/* 100 Hz = 10,000 us */
#define MODEL_PERIOD_US 10000U

/* Debugger-visible timing statistics */
volatile uint32_t model_step_count = 0;
volatile uint32_t model_step_duration_us = 0;
volatile uint32_t model_step_duration_max_us = 0;
volatile uint32_t model_period_actual_us = 0;
volatile uint32_t model_overrun_count = 0;
volatile uint32_t model_deadline_miss_count = 0;
volatile uint32_t model_last_start_us = 0;
volatile uint32_t model_last_finish_us = 0;

static uint32_t next_model_deadline_us = 0;
static uint32_t previous_model_start_us = 0;
static bool model_initialized = false;


/*
 * Copy only the LSM6DSO fields into the generated sensor bus.
 *
 * Fill this in after checking the sensor_bus definition
 * in autocodeModel.h.
 */
static void MapFlightDataToSensorBus(
    const FlightData_t *flight_data, sensor_bus *sensorBus)
{
    sensorBus->fBdyMeas_mps2[0] = flight_data->lsm_accel.x;
    sensorBus->fBdyMeas_mps2[1] = flight_data->lsm_accel.y;
    sensorBus->fBdyMeas_mps2[2] = flight_data->lsm_accel.z;

    // Gyroscope: convert degrees/s to radians/s
    const double DEG_TO_RAD = 0.017453292519943295;

    sensorBus->wBdyMeas_rps[0] = flight_data->lsm_gyro.x * DEG_TO_RAD;
    sensorBus->wBdyMeas_rps[1] = flight_data->lsm_gyro.y * DEG_TO_RAD;
    sensorBus->wBdyMeas_rps[2] = flight_data->lsm_gyro.z * DEG_TO_RAD;
}


/* Initialize the generated Simulink model */
void Autocode_Init(void)
{
    /* Zero the generated input and output structures */
    rtU_sensor_bus_e = (sensor_bus){0};
    rtY_navigation_bus_d = (navigation_bus){0};
    rtY_controller_bus_f = (controller_bus){0};

    for (uint32_t i = 0; i < 4; i++) {
        rtY_control_deg[i] = 0.0;
    }

    /* Connect generated model state */
    rtM->dwork = &rtDW;

    /* Initialize the model */
    autocodeModel_initialize(
        rtM,
        &rtU_sensor_bus_e,
        &rtY_navigation_bus_d,
        &rtY_controller_bus_f,
        rtY_control_deg
    );

    /*
     * TIM5 must be configured for a 1 MHz counter,
     * meaning one count per microsecond.
     *
     * Start it only if it has not already been started.
     */
    if ((htim5.Instance->CR1 & TIM_CR1_CEN) == 0U) {
        HAL_TIM_Base_Start(&htim5);
    }

    uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);

    next_model_deadline_us = now + MODEL_PERIOD_US;
    previous_model_start_us = now;

    model_step_count = 0;
    model_step_duration_us = 0;
    model_step_duration_max_us = 0;
    model_period_actual_us = 0;
    model_overrun_count = 0;
    model_deadline_miss_count = 0;

    model_initialized = true;
}


/*
 * Call repeatedly from the main loop.
 *
 * Returns immediately if the next 10 ms deadline has
 * not arrived. Executes at most one model step per call.
 */
void Autocode_Run100Hz(const FlightData_t *flight_data)
{
    if (!model_initialized || flight_data == NULL) {
        return;
    }

    uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);

    /* Signed subtraction handles timer counter wraparound. */
    if ((int32_t)(now - next_model_deadline_us) < 0) {
        return;
    }

    /*
     * If the loop missed one or more deadlines, count the
     * missed periods rather than executing catch-up steps.
     */
    uint32_t lateness = now - next_model_deadline_us;

    if (lateness >= MODEL_PERIOD_US) {
        model_overrun_count += lateness / MODEL_PERIOD_US;
    }

    if (previous_model_start_us != 0U) {
        model_period_actual_us = now - previous_model_start_us;
    }

    previous_model_start_us = now;
    model_last_start_us = now;

    /* Transfer current LSM6DSO measurements to the model. */
    MapFlightDataToSensorBus(flight_data, &rtU_sensor_bus_e);

    /* Execute one generated model step. */
    autocodeModel_step(
        rtM,
        &rtU_sensor_bus_e,
        &rtY_navigation_bus_d,
        &rtY_controller_bus_f,
        rtY_control_deg
    );

    uint32_t finish = __HAL_TIM_GET_COUNTER(&htim5);

    model_last_finish_us = finish;
    model_step_duration_us = finish - now;

    if (model_step_duration_us > model_step_duration_max_us) {
        model_step_duration_max_us = model_step_duration_us;
    }

    model_step_count++;

    if (model_step_duration_us >= MODEL_PERIOD_US) {
        model_deadline_miss_count++;
    }

    /*
     * Advance the deadline by whole periods, avoiding drift
     * from the time spent executing the model.
     */
    next_model_deadline_us += MODEL_PERIOD_US;

    if ((int32_t)(finish - next_model_deadline_us) >= 0) {
        uint32_t missed =
            (finish - next_model_deadline_us) / MODEL_PERIOD_US + 1U;

        model_overrun_count += missed;
        next_model_deadline_us += missed * MODEL_PERIOD_US;
    }
}

uint32_t Autocode_GetPeriodUs(void)
{
    return model_period_actual_us;
}

/* Optional model output accessors */
const real_T *Autocode_GetControlDeg(void)
{
    return rtY_control_deg;
}

const navigation_bus *Autocode_GetNavigation(void)
{
    return &rtY_navigation_bus_d;
}

const controller_bus *Autocode_GetController(void)
{
    return &rtY_controller_bus_f;
}
