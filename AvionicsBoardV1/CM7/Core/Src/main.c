/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lsm6dso_app.h"
#include "bmp5_app.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include "autocode_runner.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

#define NAV_PVT_SIZE 100
#define UBX_MAX_PAYLOAD 256
#define USBBUF_MAXLEN 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;
// -----------------------------------
// GNSS
// -----------------------------------
uint8_t gnss_rx[256];
uint8_t nav_rx[NAV_PVT_SIZE];

uint8_t ubx_class;
uint8_t ubx_id;
uint16_t ubx_length;
uint8_t ubx_payload[UBX_MAX_PAYLOAD];

uint8_t ubx_ck_a;
uint8_t ubx_ck_b;

uint32_t iTOW;
uint16_t year;

uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;

uint8_t fixType;
uint8_t numSV;

int32_t longitude_raw;
int32_t latitude_raw;
int32_t height_raw;
int32_t heightMSL_raw;

int32_t velN_raw;
int32_t velE_raw;
int32_t velD_raw;
int32_t groundSpeed_raw;


uint8_t gnss_poll[] = {
    0xB5, 0x62,   // UBX sync
    0x0A, 0x04,   // MON-VER
    0x00, 0x00,   // Payload length = 0
    0x0E, 0x34    // Checksum
};

uint8_t nav_pvt_poll[] = {
    0xB5, 0x62,
    0x01, 0x07,
    0x00, 0x00,
    0x08, 0x19
};

volatile uint8_t nav_pvt_rx[100];
volatile uint16_t nav_pvt_rx_len = 0;
volatile uint8_t nav_pvt_status = 0;

uint8_t disable_nmea[] = {
      0xB5, 0x62,
      0x06, 0x8A,
      0x27, 0x00,

      0x00, 0x01, 0x00, 0x00,

      0xBB, 0x00, 0x91, 0x20, 0x00,
      0xCA, 0x00, 0x91, 0x20, 0x00,
      0xC0, 0x00, 0x91, 0x20, 0x00,
      0xC5, 0x00, 0x91, 0x20, 0x00,
      0xAC, 0x00, 0x91, 0x20, 0x00,
      0xB1, 0x00, 0x91, 0x20, 0x00,

      0x07, 0x00, 0x91, 0x20, 0x00,

      0x00, 0x00
};

// -----------------------------------
// DATA STRUCT
// -----------------------------------
FlightData_t flight_data;

// -----------------------------------
// FATFS
// -----------------------------------
UINT bytes_written;
UINT bytes_read;

uint32_t packet_counter = 0;

const char *flight_filename = "flight.bin";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void ubx_checksum(uint8_t *msg, uint16_t len,
                         uint8_t *cka, uint8_t *ckb)
{
    uint8_t a = 0;
    uint8_t b = 0;

    // Start at CLASS, skip B5 62
    for (uint16_t i = 2; i < len; i++)
    {
        a += msg[i];
        b += a;
    }

    *cka = a;
    *ckb = b;
}

void GNSS_PrintRaw(uint32_t timeout_ms)
{
    uint8_t byte;
    uint32_t start = HAL_GetTick();

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "GNSS RX:\r\n");

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
    {
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
    }

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK)
        {
            usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                                   "%02X ", byte);

            if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
            {
                CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
            }
        }
    }

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "\r\n--- END RX ---\r\n");

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
    {
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
    }
}

//void GNSS_PrintRaw(uint32_t timeout_ms)
//{
//    uint8_t byte;
//
//    uint32_t start = HAL_GetTick();
//
//    printf("GNSS RX:\r\n");
//
//    while ((HAL_GetTick() - start) < timeout_ms)
//    {
//        if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK)
//        {
//            printf("%02X ", byte);
//        }
//    }
//
//    printf("\r\n--- END RX ---\r\n");
//}

//void ParseNAVPVT(void)
//{
//    memcpy(&iTOW,          &ubx_payload[0],  4);
//    memcpy(&year,          &ubx_payload[4],  2);
//
//    month  = ubx_payload[6];
//    day    = ubx_payload[7];
//    hour   = ubx_payload[8];
//    minute = ubx_payload[9];
//    second = ubx_payload[10];
//
//    fixType = ubx_payload[20];
//    numSV   = ubx_payload[23];
//
//    memcpy(&longitude_raw,   &ubx_payload[24], 4);
//    memcpy(&latitude_raw,    &ubx_payload[28], 4);
//    memcpy(&height_raw,      &ubx_payload[32], 4);
//    memcpy(&heightMSL_raw,   &ubx_payload[36], 4);
//    memcpy(&velN_raw,        &ubx_payload[48], 4);
//    memcpy(&velE_raw,        &ubx_payload[52], 4);
//    memcpy(&velD_raw,        &ubx_payload[56], 4);
//    memcpy(&groundSpeed_raw, &ubx_payload[60], 4);
//
//    printf("\r\n===== NAV-PVT =====\r\n");
//
//    printf("Time: %04u-%02u-%02u %02u:%02u:%02u\r\n",
//           year, month, day,
//           hour, minute, second);
//
//    printf("Fix Type: %u\r\n", fixType);
//    printf("Satellites: %u\r\n", numSV);
//
//    printf("Latitude:  %.7f deg\r\n",
//           latitude_raw / 10000000.0);
//
//    printf("Longitude: %.7f deg\r\n",
//           longitude_raw / 10000000.0);
//
//    printf("Altitude: %.3f m\r\n",
//           height_raw / 1000.0);
//
//    printf("MSL Alt: %.3f m\r\n",
//           heightMSL_raw / 1000.0);
//
//    printf("Vel N: %.3f m/s\r\n",
//           velN_raw / 1000.0);
//
//    printf("Vel E: %.3f m/s\r\n",
//           velE_raw / 1000.0);
//
//    printf("Vel D: %.3f m/s\r\n",
//           velD_raw / 1000.0);
//
//    printf("Ground Speed: %.3f m/s\r\n",
//           groundSpeed_raw / 1000.0);
//
//    printf("===================\r\n");
//}
void ParseNAVPVT(void)
{
    memcpy(&iTOW,          &ubx_payload[0],  4);
    memcpy(&year,          &ubx_payload[4],  2);

    month  = ubx_payload[6];
    day    = ubx_payload[7];
    hour   = ubx_payload[8];
    minute = ubx_payload[9];
    second = ubx_payload[10];

    fixType = ubx_payload[20];
    numSV   = ubx_payload[23];

    memcpy(&longitude_raw,   &ubx_payload[24], 4);
    memcpy(&latitude_raw,    &ubx_payload[28], 4);
    memcpy(&height_raw,      &ubx_payload[32], 4);
    memcpy(&heightMSL_raw,   &ubx_payload[36], 4);
    memcpy(&velN_raw,        &ubx_payload[48], 4);
    memcpy(&velE_raw,        &ubx_payload[52], 4);
    memcpy(&velD_raw,        &ubx_payload[56], 4);
    memcpy(&groundSpeed_raw, &ubx_payload[60], 4);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "\r\n===== NAV-PVT =====\r\n");

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "Time: %04u-%02u-%02u %02u:%02u:%02u\r\n",
                           year, month, day, hour, minute, second);

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "Fix Type: %u\r\nSatellites: %u\r\n",
                           fixType, numSV);

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "Latitude:  %.7f deg\r\n"
                           "Longitude: %.7f deg\r\n",
                           latitude_raw / 10000000.0,
                           longitude_raw / 10000000.0);

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "Altitude: %.3f m\r\n"
                           "MSL Alt: %.3f m\r\n",
                           height_raw / 1000.0,
                           heightMSL_raw / 1000.0);

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                           "Vel N: %.3f m/s\r\n"
                           "Vel E: %.3f m/s\r\n"
                           "Vel D: %.3f m/s\r\n"
                           "Ground Speed: %.3f m/s\r\n"
                           "===================\r\n",
                           velN_raw / 1000.0,
                           velE_raw / 1000.0,
                           velD_raw / 1000.0,
                           groundSpeed_raw / 1000.0);

    if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
        CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
}

//HAL_StatusTypeDef GNSS_PollNAVPVT(void)
//{
//    uint8_t byte;
//    uint8_t sync[2];
//
//    // Send poll request
//    if (HAL_UART_Transmit(
//            &huart2,
//            nav_pvt_poll,
//            sizeof(nav_pvt_poll),
//            100) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    // Find B5 62
//    do
//    {
//        if (HAL_UART_Receive(&huart2, &byte, 1, 150) != HAL_OK)
//            return HAL_TIMEOUT;
//
//    } while (byte != 0xB5);
//
//    if (HAL_UART_Receive(&huart2, &byte, 1, 150) != HAL_OK)
//        return HAL_TIMEOUT;
//
//    if (byte != 0x62)
//        return HAL_ERROR;
//
//    nav_rx[0] = 0xB5;
//    nav_rx[1] = 0x62;
//
//    if (HAL_UART_Receive(
//            &huart2,
//            &nav_rx[2],
//            98,
//            150) != HAL_OK)
//    {
//        return HAL_TIMEOUT;
//    }
//
//    // Verify message type
//    if (nav_rx[2] != 0x01 ||
//        nav_rx[3] != 0x07)
//    {
//        return HAL_ERROR;
//    }
//
//    // Payload should be 92 bytes
//    if (nav_rx[4] != 0x5C ||
//        nav_rx[5] != 0x00)
//    {
//        return HAL_ERROR;
//    }
//
//    // Copy payload
//    memcpy(ubx_payload, &nav_rx[6], 92);
//
//    return HAL_OK;
//}
HAL_StatusTypeDef GNSS_PollNAVPVT(void)
{
    uint8_t rx[100];

    // 1. Clear pending bytes from the UART receive buffer
//    uint8_t dummy;
//    while (HAL_UART_Receive(&huart2, &dummy, 1, 2) == HAL_OK)
//    {
//    }

    // 2. Send UBX-NAV-PVT poll
    if (HAL_UART_Transmit(&huart2,
                          (uint8_t *)nav_pvt_poll,
                          sizeof(nav_pvt_poll),
                          100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 3. Read response
    if (HAL_UART_Receive(&huart2,
                         rx,
                         sizeof(rx),
                         3000) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    // 4. Validate UBX header
    if (rx[0] != 0xB5 || rx[1] != 0x62)
    {
        return HAL_ERROR;
    }

    // 5. Validate message class and ID
    if (rx[2] != 0x01 || rx[3] != 0x07)
    {
        return HAL_ERROR;
    }

    // 6. Validate payload length (92 bytes)
    if (rx[4] != 0x5C || rx[5] != 0x00)
    {
        return HAL_ERROR;
    }

    // 7. Verify UBX checksum over class through payload
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (int i = 2; i < 98; i++)
    {
        ck_a += rx[i];
        ck_b += ck_a;
    }

    if (rx[98] != ck_a || rx[99] != ck_b)
    {
        return HAL_ERROR;
    }

    // 8. Copy complete message and payload
    memcpy(nav_rx, rx, sizeof(rx));
    memcpy(ubx_payload, &rx[6], 92);

    return HAL_OK;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int __io_putchar(int ch)
//{
//    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//    return ch;
//}
//int __io_putchar(int ch)
//{
//    uint8_t c = (uint8_t)ch;
//
//    while (CDC_Transmit_FS(&c, 1) == USBD_BUSY)
//    {
//        HAL_Delay(1);
//    }
//
//    return ch;
//}

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint8_t usbRxBuf[USBBUF_MAXLEN];

uint16_t usbTxBufLen = 0;
uint16_t usbRxBufLen = 0;

uint8_t  usbRxFlag 	 = 0;

void USB_RXCallback(uint8_t* Buf, uint32_t *Len) {

	memcpy(usbRxBuf, Buf, *Len);
	usbRxBufLen = *Len;
	usbRxFlag = 1;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
//  printf("\n================================\r\n");
//  printf("CODE STARTING BEEP BOOP\n");
//  printf("================================\r\n");
  HAL_Delay(5000);
  HAL_TIM_Base_Start(&htim5);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMI_CSG_GPIO_Port, BMI_CSG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BMP585_CS_Pin|BMI_CSA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GNSS_NRST_GPIO_Port, GNSS_NRST_Pin, GPIO_PIN_SET);


  FRESULT fres;


  fres = f_mount(&SDFatFS, SDPath, 1);

  if (fres != FR_OK)
  {
      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                             "SD f_mount failed: %d\r\n", fres);
  }
  else
  {
      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                             "SD mounted successfully\r\n");
  }

  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
  {
      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
  }

  fres = f_open(&SDFile,
                "flight.bin",
                FA_WRITE | FA_CREATE_ALWAYS);

  if (fres != FR_OK)
  {
      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                             "SD f_open failed: %d\r\n", fres);
  }
  else
  {
      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
                             "Flight file opened successfully\r\n");
  }

  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
  {
      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
  }


//  if (fres != FR_OK)
//  {
//      printf("SD f_mount failed: %d\r\n", fres);
//  }
//  else
//  {
//      printf("SD mounted successfully\r\n");
//  }
//
//  fres = f_open(&SDFile,
//                "flight.bin",
//                FA_WRITE | FA_CREATE_ALWAYS);
//
//  if (fres != FR_OK)
//  {
//      printf("SD f_open failed: %d\r\n", fres);
//  }
//  else
//  {
//      printf("Flight file opened successfully\r\n");
//  }

  if (LSM6DSO_App_Init() != LSM6DSO_OK)
  {
//	  printf("failed init LSM6DSO\r\n"); // Initialization failed
  }
//  if (BMP5_App_Init() != BMP5_APP_OK)
//  {
//	  printf("Failed init BMP585\r\n"); // Initialization failed
//  }

//  uint8_t rx;
//  uint8_t state = 0;
//
//  uint8_t cka, ckb;
//
//  ubx_checksum(disable_nmea, sizeof(disable_nmea) - 2,
//               &cka, &ckb);
//
//  disable_nmea[sizeof(disable_nmea) - 2] = cka;
//  disable_nmea[sizeof(disable_nmea) - 1] = ckb;

//  printf("CFG-VALSET packet:\r\n");
//
//  for (uint16_t i = 0; i < sizeof(disable_nmea); i++)
//  {
//      printf("%02X ", disable_nmea[i]);
//  }
//
//  printf("\r\n");
//
//  printf("Sending CFG-VALSET...\r\n");
//  usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
//                         "CFG-VALSET packet:\r\n");
//
//  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
//      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//
//  for (uint16_t i = 0; i < sizeof(disable_nmea); i++)
//  {
//      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
//                             "%02X ", disable_nmea[i]);
//
//      if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
//          CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//  }
//
//  usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
//                         "\r\nSending CFG-VALSET...\r\n");
//
//  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
//      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//
//  while (HAL_UART_Receive(&huart2, &rx, 1, 0) == HAL_OK)
//  {
//      // Flush stale bytes
//  }
//
//  HAL_StatusTypeDef status = HAL_UART_Transmit(
//      &huart2,
//      disable_nmea,
//      sizeof(disable_nmea),
//      1000
//  );
//
//  GNSS_PrintRaw(3000);

//  Autocode_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  LSM6DSO_App_Process(&flight_data);
//      Autocode_Run100Hz(&flight_data);
//      const navigation_bus *nav = Autocode_GetNavigation();
//
//      const char *modeStr;
//
//      switch (nav->flightMode_enum)
//      {
//          case IDLE:
//              modeStr = "IDLE";
//              break;
//
//          case BOOST:
//              modeStr = "BOOST";
//              break;
//
//          case ACTIVE:
//              modeStr = "ACTIVE";
//              break;
//
//          case LOCKOUT:
//              modeStr = "LOCKOUT";
//              break;
//
//          default:
//              modeStr = "UNKNOWN";
//              break;
//      }
//      uint32_t period_us = Autocode_GetPeriodUs();
//
//      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
//          "MODE: %s | Period: %.3f ms | EULER: Phi=%.4f rad | Theta=%.4f rad | Psi=%.4f rad\r\n",
//          modeStr,
//          period_us / 1000.0f,
//          nav->phiEst_rad,
//          nav->thetaEst_rad,
//          nav->psiEst_rad);
//
//      if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
//      {
//    	  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//      }


      LSM6DSO_App_Process(&flight_data);

//	  printf("ACC: X=%.3f Y=%.3f Z=%.3f m/s^2 | "
//			  "GYRO: X=%.3f Y=%.3f Z=%.3f deg/s\r\n",
//			  flight_data.lsm_accel.x,
//			  flight_data.lsm_accel.y,
//			  flight_data.lsm_accel.z,
//			  flight_data.lsm_gyro.x,
//			  flight_data.lsm_gyro.y,
//			  flight_data.lsm_gyro.z);

	  usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
	      "ACC: X=%.3f Y=%.3f Z=%.3f m/s^2 | "
	      "GYRO: X=%.3f Y=%.3f Z=%.3f deg/s\r\n",
	      flight_data.lsm_accel.x,
	      flight_data.lsm_accel.y,
	      flight_data.lsm_accel.z,
	      flight_data.lsm_gyro.x,
	      flight_data.lsm_gyro.y,
	      flight_data.lsm_gyro.z);

	  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
	  {
	      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
	  }

//	  BMP585_App_Process(&flight_data);
//
//	  float altitude = 44330.0f *
//	                   (1.0f - powf(flight_data.bmp585_pressure / 101325.0f, 0.1903f));
//
//	  float altitude_ft = altitude * 3.28084f;

//	  printf("PRESSURE: %.2f hPa | TEMP: %.2f C | ALT: %.2f ft\r\n",
//	         flight_data.bmp585_pressure / 100.0f,
//	         flight_data.bmp585_temperature,
//			 altitude_ft);


//	  printf("NAV-PVT poll packet:\r\n");
//
//	  for (uint16_t i = 0; i < sizeof(nav_pvt_poll); i++)
//	  {
//		  printf("%02X ", nav_pvt_poll[i]);
//	  }
//
//	  printf("\r\n");
//
//	  HAL_UART_Transmit(
//			  &huart2,
//			  nav_pvt_poll,
//			  sizeof(nav_pvt_poll),
//			  1000
//	  );
//
//	  uint8_t nav_rx[128];
//	  uint16_t nav_count = 0;
//	  uint8_t byte;
//
//	  uint32_t start = HAL_GetTick();
//
//	  while ((HAL_GetTick() - start) < 3000)
//	  {
//		  if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK)
//		  {
//			  if (nav_count < sizeof(nav_rx))
//			  {
//				  nav_rx[nav_count++] = byte;
//			  }
//		  }
//	  }
//
//	  printf("Received %u bytes:\r\n", nav_count);
//
//	  for (uint16_t i = 0; i < nav_count; i++)
//	  {
//		  printf("%02X ", nav_rx[i]);
//	  }
//
//	  printf("\r\n");
//
//	  if (nav_count >= 100 &&
//			  nav_rx[0] == 0xB5 &&
//			  nav_rx[1] == 0x62 &&
//			  nav_rx[2] == 0x01 &&
//			  nav_rx[3] == 0x07)
//	  {
//		  printf("Valid NAV-PVT packet found!\r\n");
//
//		  memcpy(ubx_payload, &nav_rx[6], 92);
//	  }
//	  else
//	  {
//		  printf("NAV-PVT packet not found\r\n");
//	  }
//
//	  ParseNAVPVT();

	  flight_data.timestamp_us = __HAL_TIM_GET_COUNTER(&htim5);
	  flight_data.packet_counter = packet_counter++;
	  bytes_written = 0;

	  fres = f_write(&SDFile,
			  &flight_data,
			  sizeof(FlightData_t),
			  &bytes_written);

	  if (fres != FR_OK || bytes_written != sizeof(FlightData_t))
	  {
	      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
	                             "SD write failed: %d, wrote %u bytes\r\n",
	                             fres,
	                             (unsigned)bytes_written);

	      if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
	          CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
	  }

	  usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
	                         "sizeof FlightData_t = %u, bytes_written = %u\r\n",
	                         (unsigned)sizeof(FlightData_t),
	                         (unsigned)bytes_written);

	  if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
	      CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

	  fres = f_sync(&SDFile);

	  if (fres != FR_OK)
	  {
	      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
	                             "Sync failed: %d\r\n", fres);

	      if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
	          CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
	  }

	  if (packet_counter >= 100)
	  {
	      f_close(&SDFile);

	      usbTxBufLen = snprintf((char *)usbTxBuf, USBBUF_MAXLEN,
	                             "Logging finished.\r\n"
	                             "Expected file size: %lu\r\n"
	                             "packet_counter = %lu\r\n"
	                             "bytes_written = %u\r\n",
	                             (unsigned long)(packet_counter * sizeof(FlightData_t)),
	                             (unsigned long)packet_counter,
	                             (unsigned)bytes_written);

	      if (usbTxBufLen > 0 && usbTxBufLen < USBBUF_MAXLEN)
	      {
	    	  while (CDC_Transmit_FS(usbTxBuf, usbTxBufLen) == USBD_BUSY)
	    	  {
	    		  HAL_Delay(1);
	    	  }
	      }
	      while (1);
	  }

//	  if (fres != FR_OK || bytes_written != sizeof(FlightData_t))
//	  {
//		  printf("SD write failed: %d, wrote %u bytes\r\n",
//				  fres,
//				  bytes_written);
//	  }
//
//	  printf("sizeof FlightData_t = %u, bytes_written = %u\r\n",
//	         (unsigned)sizeof(FlightData_t),
//	         (unsigned)bytes_written);
//
//	  fres = f_sync(&SDFile);
//
//	  if (fres != FR_OK) {
//		  printf("Sync failed: %d\r\n", fres);
//	  }
//
//	  if (packet_counter >= 6000) {
//		  f_close(&SDFile);
//		  printf("Logging finished.\r\n");
//		  printf("Expected file size: %lu\r\n",
//		         (unsigned long)(packet_counter * sizeof(FlightData_t)));
//		  printf("packet_counter = %lu\r\n",
//		         (unsigned long)packet_counter);
//
//		  printf("bytes_written = %u\r\n",
//		         (unsigned)bytes_written);
//		  while (1);
//	  }

//	  if (GNSS_PollNAVPVT() == HAL_OK)
//	  {
//		  ParseNAVPVT();
//	  }


	  HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_SMPS_2V5_SUPPLIES_LDO);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x007074AF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 59;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GNSS_NRST_GPIO_Port, GNSS_NRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP581_CS_GPIO_Port, BMP581_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LSM_CS_Pin|BMP585_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMI_CSA_GPIO_Port, BMI_CSA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMI_CSG_GPIO_Port, BMI_CSG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BMP581_INT_Pin */
  GPIO_InitStruct.Pin = BMP581_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP581_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_NRST_Pin */
  GPIO_InitStruct.Pin = GNSS_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP581_CS_Pin */
  GPIO_InitStruct.Pin = BMP581_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMP581_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LSM_INT_Pin BMP585_INT_Pin */
  GPIO_InitStruct.Pin = LSM_INT_Pin|BMP585_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LSM_CS_Pin BMP585_CS_Pin */
  GPIO_InitStruct.Pin = LSM_CS_Pin|BMP585_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI_INTG_Pin BMI_INTA_Pin */
  GPIO_InitStruct.Pin = BMI_INTG_Pin|BMI_INTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BMI_CSA_Pin */
  GPIO_InitStruct.Pin = BMI_CSA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMI_CSA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMI_CSG_Pin */
  GPIO_InitStruct.Pin = BMI_CSG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMI_CSG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(BMI_INTG_EXTI_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(BMI_INTG_EXTI_IRQn);

  HAL_NVIC_SetPriority(BMP581_INT_EXTI_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(BMP581_INT_EXTI_IRQn);

  HAL_NVIC_SetPriority(BMP585_INT_EXTI_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(BMP585_INT_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
