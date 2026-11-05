#include "main.h"
#include "autocode_runner.h"
#include "autocodeModel_initialize.h"
#include <stdint.h>
#include <stdbool.h>

/* Defined by CubeMX in your project */
extern TIM_HandleTypeDef htim5;

/* Generated model data */
static RT_MODEL *rtM;

static sensor_bus rtU_sensor_bus_e;
static navigation_bus navBus;
static controller_bus controllerBus;
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
static bool model_gnss_fix_ok = false;

void Autocode_SetGPSFix(uint8_t gnss_fix_ok)
{
    model_gnss_fix_ok = gnss_fix_ok != 0U;
}


/*
 * Map the live IMU and temporary 1000 Pa barometer into the sensor bus.
 *
 */
static void MapFlightDataToSensorBus(const FlightData_t *data, sensor_bus *bus)
{
    Vector3f_t accel __attribute__((aligned(8))) = data ? data->lsm_accel : (Vector3f_t){0};
    Vector3f_t gyro __attribute__((aligned(8))) = data ? data->lsm_gyro : (Vector3f_t){0};
    autocodeModel_initializeSensorBus(bus, &accel, &gyro, NULL, 1000.0);
    bus->gpsFix_b = data != NULL && model_gnss_fix_ok;
    bus->gpsLLA_na[0] = data ? data->latitude : 0.0;
    bus->gpsLLA_na[1] = data ? data->longitude : 0.0;
    bus->gpsLLA_na[2] = data ? data->altitude : 0.0;
    bus->velNedMeas_mps[0] = data ? data->velocity_north : 0.0;
    bus->velNedMeas_mps[1] = data ? data->velocity_east : 0.0;
    bus->velNedMeas_mps[2] = data ? data->velocity_down : 0.0;
}


/* Initialize the generated Simulink model */
void Autocode_Init(void)
{
    model_gnss_fix_ok = false;
    /* Zero the generated input and output structures */
    rtU_sensor_bus_e = (sensor_bus){0};
    navBus = (navigation_bus){0};
    controllerBus = (controller_bus){0};

    for (uint32_t i = 0; i < 4; i++) {
        rtY_control_deg[i] = 0.0;
    }


    MapFlightDataToSensorBus(NULL, &rtU_sensor_bus_e);

    /* Initialize the model using the current generated wrapper API. */
    autocodeModel_setup(&rtU_sensor_bus_e);
    rtM = autocodeModel_getModel();

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

    Autocode_Step(flight_data);
    uint32_t finish = model_last_finish_us;

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
    return &navBus;
}

const controller_bus *Autocode_GetController(void)
{
    return &controllerBus;
}

void Autocode_Step(const FlightData_t *flight_data)
{
    if (!model_initialized || flight_data == NULL) return;
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
    if (model_step_count != 0U) model_period_actual_us = now - model_last_start_us;
    model_last_start_us = now;
    /* Transfer current LSM6DSO measurements to the model. */
    MapFlightDataToSensorBus(flight_data, &rtU_sensor_bus_e);

    /* Execute one generated model step. */
    autocodeModel_step(
        rtM,
        &rtU_sensor_bus_e,
        &navBus,
        &controllerBus,
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

}
