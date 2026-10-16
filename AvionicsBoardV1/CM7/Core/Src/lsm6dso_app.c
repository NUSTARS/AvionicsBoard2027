#include "lsm6dso_app.h"
#include <stdio.h>

/* Private Variables */
static LSM6DSO_Object_t MotionSensorObj;

/* External Handles from main.c */
extern SPI_HandleTypeDef hspi1;

/* Low-Level Platform Bus Components Hooks Prototypes */
static int32_t Platform_SPI_Init(void);
static int32_t Platform_SPI_DeInit(void);
static int32_t Platform_SPI_Write(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t Platform_SPI_Read(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t Platform_GetTick(void);

/**
  * @brief  Public API to initialize and configure the IMU
  */
int32_t LSM6DSO_App_Init(void)
{
    LSM6DSO_IO_t io_ctx;
    uint8_t who_am_i_id = 0;

    io_ctx.Init     = Platform_SPI_Init;
    io_ctx.DeInit   = Platform_SPI_DeInit;
    io_ctx.BusType  = LSM6DSO_SPI_4WIRES_BUS;
    io_ctx.Address  = 0;
    io_ctx.WriteReg = Platform_SPI_Write;
    io_ctx.ReadReg  = Platform_SPI_Read;
    io_ctx.GetTick  = Platform_GetTick;

    if (LSM6DSO_RegisterBusIO(&MotionSensorObj, &io_ctx) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    if (LSM6DSO_ReadID(&MotionSensorObj, &who_am_i_id) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    if (who_am_i_id != 0x6C)
        return LSM6DSO_ERROR;

    if (LSM6DSO_Init(&MotionSensorObj) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    if (LSM6DSO_ACC_Enable(&MotionSensorObj) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    if (LSM6DSO_GYRO_Enable(&MotionSensorObj) != LSM6DSO_OK)
        return LSM6DSO_ERROR;

    return LSM6DSO_OK;
}

/**
  * @brief  Public API called inside main's while(1) loop to parse FIFO data
  */
void LSM6DSO_App_Process(FlightData_t *data)
{
    LSM6DSO_Axes_t accel;
    LSM6DSO_Axes_t gyro;

    if (LSM6DSO_ACC_GetAxes(&MotionSensorObj, &accel) == LSM6DSO_OK)
    {
    	// Getting m/s^2 from mg
        data->lsm_accel.x = (double)accel.x * 0.00980665f;
        data->lsm_accel.y = (double)accel.y * 0.00980665f;
        data->lsm_accel.z = (double)accel.z * 0.00980665f;
    }

    if (LSM6DSO_GYRO_GetAxes(&MotionSensorObj, &gyro) == LSM6DSO_OK)
    {
    	// Degrees / sec
        data->lsm_gyro.x = (double)gyro.x * 0.001f;
        data->lsm_gyro.y = (double)gyro.y * 0.001f;
        data->lsm_gyro.z = (double)gyro.z * 0.001f;
    }
}

static int32_t Platform_SPI_Init(void) { return LSM6DSO_OK; }
static int32_t Platform_SPI_DeInit(void) { return LSM6DSO_OK; }

/* Note: Addr param can be ignored for SPI because CS pins handle device addressing */
static int32_t Platform_SPI_Write(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  uint8_t reg_8bit = (uint8_t)Reg;
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &reg_8bit, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, pData, Length, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
  return LSM6DSO_OK;
}

static int32_t Platform_SPI_Read(
    uint16_t Addr,
    uint16_t Reg,
    uint8_t *pData,
    uint16_t Length)
{
    uint8_t tx[Length + 1];
    uint8_t rx[Length + 1];

    tx[0] = ((uint8_t)Reg) | 0x80;

    for (uint16_t i = 0; i < Length; i++)
        tx[i + 1] = 0x00;

    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, Length + 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

    for (uint16_t i = 0; i < Length; i++)
        pData[i] = rx[i + 1];

    return LSM6DSO_OK;
}

static int32_t Platform_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}
