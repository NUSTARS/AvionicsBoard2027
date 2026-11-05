#include "lsm6dso_app.h"
#include "IMURunningBuffer.h"
#include <string.h>

/* Private Variables */
static LSM6DSO_Object_t MotionSensorObj;

/* External Handles from main.c */
extern SPI_HandleTypeDef hspi1;

static DMA_HandleTypeDef imu_rx_dma, imu_tx_dma;
static uint8_t imu_tx[32] __attribute__((aligned(32)));
static uint8_t imu_rx[32] __attribute__((aligned(32)));
enum { IMU_IDLE, IMU_STATUS, IMU_GYRO, IMU_ACCEL, IMU_BOTH };
static volatile uint8_t acquisition_enabled, acquisition_phase, acquisition_pending;
volatile uint32_t lsm_drdy_count, lsm_drdy_busy_count, lsm_spi_error_count;
static int32_t Acquisition_DMA_Init(void);
static void Acquisition_Read(uint8_t reg, uint16_t length, uint8_t phase);


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
    LSM6DSO_IO_t io_ctx = {0};
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

    /* Configure while stopped; these bounded register accesses are startup only. */
    if (LSM6DSO_ACC_SetOutputDataRate(&MotionSensorObj, 1667.0f) != LSM6DSO_OK ||
        LSM6DSO_GYRO_SetOutputDataRate(&MotionSensorObj, 1667.0f) != LSM6DSO_OK)
        return LSM6DSO_ERROR;
    float_t accel_mg_per_lsb, gyro_mdps_per_lsb;
    if (LSM6DSO_ACC_GetSensitivity(&MotionSensorObj, &accel_mg_per_lsb) != LSM6DSO_OK ||
        LSM6DSO_GYRO_GetSensitivity(&MotionSensorObj, &gyro_mdps_per_lsb) != LSM6DSO_OK)
        return LSM6DSO_ERROR;
    IMURunningBuffer_Init((double)accel_mg_per_lsb * 0.00980665,
                          (double)gyro_mdps_per_lsb * 0.001);
    if (Acquisition_DMA_Init() != LSM6DSO_OK) return LSM6DSO_ERROR;
    lsm6dso_pin_int1_route_t int1 = {0};
    lsm6dso_pin_int2_route_t int2 = {0};
    /* Route both sources to both sensor pins: PG9 may be wired to either INT. */
    int1.drdy_xl = int1.drdy_g = 1U;
    int2.drdy_xl = int2.drdy_g = 1U;
    if (lsm6dso_pin_mode_set(&MotionSensorObj.Ctx, LSM6DSO_PUSH_PULL) != LSM6DSO_OK ||
        lsm6dso_pin_polarity_set(&MotionSensorObj.Ctx, LSM6DSO_ACTIVE_HIGH) != LSM6DSO_OK ||
        lsm6dso_data_ready_mode_set(&MotionSensorObj.Ctx, LSM6DSO_DRDY_PULSED) != LSM6DSO_OK ||
        lsm6dso_pin_int1_route_set(&MotionSensorObj.Ctx, int1) != LSM6DSO_OK ||
        lsm6dso_pin_int2_route_set(&MotionSensorObj.Ctx, NULL, int2) != LSM6DSO_OK)
        return LSM6DSO_ERROR;
    if (LSM6DSO_ACC_Enable(&MotionSensorObj) != LSM6DSO_OK ||
        LSM6DSO_GYRO_Enable(&MotionSensorObj) != LSM6DSO_OK) {
        acquisition_enabled = 0U;
        return LSM6DSO_ERROR;
    }
    /* No async SPI client is allowed until the last startup register write. */
    acquisition_enabled = 1U;
    __HAL_GPIO_EXTI_GENERATE_SWIT(LSM_INT_Pin);
    return LSM6DSO_OK;
}

/* DMA buffers occupy separate full CM7 cache lines in AXI SRAM (.bss), not DTCM.
 * SD mode has D-cache off; explicit maintenance also supports D-cache enabled.
 * No other SPI1 client may access the bus after acquisition is enabled. */
static int32_t Acquisition_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    imu_rx_dma.Instance = DMA2_Stream0;
    imu_rx_dma.Init.Request = DMA_REQUEST_SPI1_RX;
    imu_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    imu_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    imu_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    imu_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    imu_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    imu_rx_dma.Init.Mode = DMA_NORMAL;
    imu_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    imu_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    imu_tx_dma = imu_rx_dma;
    imu_tx_dma.Instance = DMA2_Stream1;
    imu_tx_dma.Init.Request = DMA_REQUEST_SPI1_TX;
    imu_tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    if (HAL_DMA_Init(&imu_rx_dma) != HAL_OK || HAL_DMA_Init(&imu_tx_dma) != HAL_OK)
        return LSM6DSO_ERROR;
    __HAL_LINKDMA(&hspi1, hdmarx, imu_rx_dma);
    __HAL_LINKDMA(&hspi1, hdmatx, imu_tx_dma);
    /* Same preemption priority serializes EXTI, DMA and SPI state transitions. */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1U, 0U);
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1U, 0U);
    HAL_NVIC_SetPriority(SPI1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    return LSM6DSO_OK;
}

static void Acquisition_Read(uint8_t reg, uint16_t length, uint8_t phase)
{
    /* Called only by equal-priority acquisition IRQs, never by the model loop. */
    imu_tx[0] = reg | 0x80U;
    acquisition_phase = phase;
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr((uint32_t *)imu_tx, sizeof(imu_tx));
        SCB_InvalidateDCache_by_Addr((uint32_t *)imu_rx, sizeof(imu_rx));
    }
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, imu_tx, imu_rx, length + 1U) != HAL_OK) {
        HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
        acquisition_phase = IMU_IDLE;
        lsm_spi_error_count++;
    }
}

void LSM6DSO_App_SetHardwareFlag(void)
{
    if (!acquisition_enabled) return;
    lsm_drdy_count++;
    if (acquisition_phase != IMU_IDLE) {
        acquisition_pending = 1U;
        lsm_drdy_busy_count++;
        return;
    }
    Acquisition_Read(LSM6DSO_STATUS_REG, 1U, IMU_STATUS);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == LSM_INT_Pin) LSM6DSO_App_SetHardwareFlag();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi != &hspi1 || !acquisition_enabled) return;
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_InvalidateDCache_by_Addr((uint32_t *)imu_rx, sizeof(imu_rx));
    uint8_t phase = acquisition_phase;
    acquisition_phase = IMU_IDLE;
    if (phase == IMU_STATUS) {
        uint8_t ready = imu_rx[1] & 3U;
        /* Do not read an unready sensor: doing so could consume a sample that
         * arrives between the status read and data burst without counting it. */
        if (ready == 3U) {
            Acquisition_Read(LSM6DSO_OUTX_L_G, 12U, IMU_BOTH);
            return;
        }
        if (ready == 2U) {
            Acquisition_Read(LSM6DSO_OUTX_L_G, 6U, IMU_GYRO);
            return;
        }
        if (ready == 1U) {
            Acquisition_Read(LSM6DSO_OUTX_L_A, 6U, IMU_ACCEL);
            return;
        }
    } else if (phase == IMU_BOTH) {
        IMURunningBuffer_AddGyro(imu_rx + 1U);
        IMURunningBuffer_AddAccel(imu_rx + 7U);
    } else if (phase == IMU_GYRO) {
        IMURunningBuffer_AddGyro(imu_rx + 1U);
    } else if (phase == IMU_ACCEL) {
        IMURunningBuffer_AddAccel(imu_rx + 1U);
    }
    if (acquisition_pending) {
        acquisition_pending = 0U;
        Acquisition_Read(LSM6DSO_STATUS_REG, 1U, IMU_STATUS);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi != &hspi1) return;
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    lsm_spi_error_count++;
    /* Fail closed; never enter a blocking abort/reinitialization from an ISR. */
    acquisition_enabled = 0U;
    acquisition_phase = IMU_IDLE;
}

void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&imu_rx_dma); }
void DMA2_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&imu_tx_dma); }
void SPI1_IRQHandler(void)
{
    /* At EOT, TX DMA may have completed but its same-priority IRQ is still
     * pending. Retire it before chaining another DMA transaction in callback. */
    HAL_DMA_IRQHandler(&imu_tx_dma);
    HAL_SPI_IRQHandler(&hspi1);
}

static int32_t Platform_SPI_Init(void) { return LSM6DSO_OK; }
static int32_t Platform_SPI_DeInit(void) { return LSM6DSO_OK; }

/* Note: Addr param can be ignored for SPI because CS pins handle device addressing */
static int32_t Platform_SPI_Write(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  uint8_t reg_8bit = (uint8_t)Reg;
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &reg_8bit, 1, 20U);
  if (status == HAL_OK) status = HAL_SPI_Transmit(&hspi1, pData, Length, 20U);
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
  return status == HAL_OK ? LSM6DSO_OK : LSM6DSO_ERROR;
}

static int32_t Platform_SPI_Read(
    uint16_t Addr,
    uint16_t Reg,
    uint8_t *pData,
    uint16_t Length)
{
    /* Only short configuration reads use this startup-only synchronous path. */
    uint8_t tx[32];
    uint8_t rx[32];
    if (Length > sizeof(tx) - 1U) return LSM6DSO_ERROR;

    tx[0] = ((uint8_t)Reg) | 0x80;

    for (uint16_t i = 0; i < Length; i++)
        tx[i + 1] = 0x00;

    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, Length + 1, 20U);

    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    if (status != HAL_OK) return LSM6DSO_ERROR;

    for (uint16_t i = 0; i < Length; i++)
        pData[i] = rx[i + 1];

    return LSM6DSO_OK;
}

static int32_t Platform_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}
