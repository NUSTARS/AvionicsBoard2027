#include "bmp5_app.h"
#include <stdio.h>

/* Private Driver Instances */
static struct bmp5_dev BaroSensorObj;

static struct bmp5_osr_odr_press_config BMP585_OsrOdrPressConfig = {
    .osr_t = BMP5_OVERSAMPLING_1X,
    .osr_p = BMP5_OVERSAMPLING_16X,
    .press_en = BMP5_ENABLE,
    .odr = BMP5_ODR_80_HZ
};
/* External SPI Peripheral Linkage from main.c */
extern SPI_HandleTypeDef hspi1;

/* Low-Level Platform Interface Bridging Prototypes */
static int8_t Platform_SPI_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t Platform_SPI_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void Platform_DelayUs(uint32_t period, void *intf_ptr);

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  Public API to initialize and configure the BMP581 Barometer
  */
int32_t BMP5_App_Init(void)
{
    int8_t rslt;

    DWT_Init();

    /* Bind hardware interface contexts to Bosch structure */
    BaroSensorObj.intf = BMP5_SPI_INTF;
    BaroSensorObj.read = Platform_SPI_Read;
    BaroSensorObj.write = Platform_SPI_Write;
    BaroSensorObj.delay_us = Platform_DelayUs;
    BaroSensorObj.intf_ptr = NULL;

    /* Initialize sensor */
    rslt = bmp5_init(&BaroSensorObj);
    if (rslt != BMP5_OK)
        return BMP5_APP_ERROR;

    /* Configure pressure/temperature oversampling and ODR */
    rslt = bmp5_set_osr_odr_press_config(
        &BMP585_OsrOdrPressConfig,
        &BaroSensorObj);

    if (rslt != BMP5_OK)
        return BMP5_APP_ERROR;

    /* Set sensor to continuous normal operation */
    rslt = bmp5_set_power_mode(
        BMP5_POWERMODE_NORMAL,
        &BaroSensorObj);

    if (rslt != BMP5_OK)
        return BMP5_APP_ERROR;

    return BMP5_APP_OK;
}

/**
  * @brief  Public API called inside main's while(1) loop to parse pressure/temp FIFO data
  */
void BMP585_App_Process(FlightData_t *data)
{
    struct bmp5_sensor_data sensor_data = {0};

    if (bmp5_get_sensor_data(
            &sensor_data,
            &BMP585_OsrOdrPressConfig,
            &BaroSensorObj) == BMP5_OK)
    {
        data->bmp585_pressure = sensor_data.pressure;
        data->bmp585_temperature = sensor_data.temperature;
    }
}
static int8_t Platform_SPI_Read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    (void)intf_ptr;

    uint8_t tx[len + 1];
    uint8_t rx[len + 1];

    tx[0] = reg_addr | 0x80;

    for (uint32_t i = 0; i < len; i++)
        tx[i + 1] = 0x00;

    HAL_GPIO_WritePin(BMP585_CS_GPIO_Port,
                      BMP585_CS_Pin,
                      GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1,
                                tx,
                                rx,
                                len + 1,
                                HAL_MAX_DELAY);

    HAL_GPIO_WritePin(BMP585_CS_GPIO_Port,
                      BMP585_CS_Pin,
                      GPIO_PIN_SET);

    if (status != HAL_OK)
        return BMP5_E_COM_FAIL;

    for (uint32_t i = 0; i < len; i++)
        reg_data[i] = rx[i + 1];

    return BMP5_OK;
}

static int8_t Platform_SPI_Write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    (void)intf_ptr;

    uint8_t write_cmd = reg_addr & 0x7F;

    HAL_GPIO_WritePin(BMP585_CS_GPIO_Port,
                      BMP585_CS_Pin,
                      GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1,
                        &write_cmd,
                        1,
                        HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        status = HAL_SPI_Transmit(&hspi1,
                                  (uint8_t *)reg_data,
                                  len,
                                  HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(BMP585_CS_GPIO_Port,
                      BMP585_CS_Pin,
                      GPIO_PIN_SET);

    if (status != HAL_OK)
        return BMP5_E_COM_FAIL;

    return BMP5_OK;
}

/**
  * @brief Microsecond timing helper interface
  */
static void Platform_DelayUs(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (period * SystemCoreClock) / 1000000U;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}
