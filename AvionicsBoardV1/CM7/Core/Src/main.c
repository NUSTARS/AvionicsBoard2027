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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "usbd_def.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#include "autocodeModel_initialize.h"
#include "autocodePlant_initialize.h"
#include "autocodePlant_feedback.h"
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
#undef DUAL_CORE_BOOT_SYNC_SEQUENCE /* Standalone CM7: CM4 is disabled. */

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

#define NAV_PVT_SIZE 100
#define UBX_MAX_PAYLOAD 256
#define USBBUF_MAXLEN 128
#define SIMULATION_STEP_PERIOD_US 10000U
#define SIMULATION_STEP_PERIOD_S 0.01
#define SIMULATION_DURATION_SECONDS 35U
#define SIMULATION_DURATION_STEPS \
    (((SIMULATION_DURATION_SECONDS * 1000000U) / SIMULATION_STEP_PERIOD_US) + 1U)
#define SIMULATION_LOG_FRAME_BUFFER_SIZE 800U
#define SIMULATION_LOG_COMMAND_BUFFER_SIZE 32U
#define SIMULATION_LOG_MAX_ATTEMPTS 3U
#define SIMULATION_LOG_ACK_TIMEOUT_MS 3000U
#define SIMULATION_LOG_UART_TIMEOUT_MS 1000U
#define SIMULATION_LOG_ACK_LINE_LENGTH 12U
#define SIMULATION_START_COMMAND "START_LOG_SIM"
#define SIMULATION_ECHO_COMMAND "ECHO"
#define SIMULATION_LOG_ACK_ACCEPTED 1U
#define SIMULATION_LOG_ACK_RETRY 2U

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
/* USER CODE BEGIN PV */
static sensor_bus simulated_sensor_bus;
static navigation_bus simulated_navigation_bus;
static controller_bus simulated_controller_bus;
static plant_bus simulated_plant_bus;
static real_T simulated_control_deg[4];
/* Capture in SRAM; UART at 115200 cannot carry 28 CSV columns at 100 Hz.
 * Float storage uses 364104 bytes; generated calculations remain real_T. */
static float simulation_samples[SIMULATION_DURATION_STEPS][26];
static uint32_t simulation_sample_count;
static uint32_t simulation_next_step_us;
static RT_MODEL *simulation_model;
static RT_MODEL_autocodePlant_T *simulation_plant;
volatile uint32_t simulation_deadline_misses;
volatile uint32_t simulation_step_max_us;
static uint32_t simulation_start_us;
static uint32_t simulation_next_ready_ms;
static uint32_t simulation_transfer_row;
static uint8_t simulation_phase;
static uint8_t simulation_frame_pending;
static uint8_t simulation_waiting_ack;
static uint8_t simulation_attempts;
static uint32_t simulation_ack_tick;
static uint16_t simulation_tx_length;
static uint16_t simulation_tx_offset;
static uint8_t simulation_transport_usb;
static uint8_t simulation_usb_tx_started;
static volatile uint16_t simulation_usb_rx_head;
static volatile uint16_t simulation_usb_rx_tail;
static uint8_t simulation_usb_rx[256];
static uint32_t simulation_tx_tick;
static char simulation_command[SIMULATION_LOG_COMMAND_BUFFER_SIZE];
static uint16_t simulation_command_length;
static uint8_t simulation_command_overflow;
enum { SIM_READY, SIM_RUNNING, SIM_HEADER, SIM_ROWS, SIM_END, SIM_DONE, SIM_ERROR };
static char simulation_log_line[768];
static char simulation_log_frame[SIMULATION_LOG_FRAME_BUFFER_SIZE];
static uint32_t simulation_log_next_sequence;
static uint32_t simulation_log_rows_sent;
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
//static void ubx_checksum(uint8_t *msg, uint16_t len,
//                         uint8_t *cka, uint8_t *ckb)
//{
//    uint8_t a = 0;
//    uint8_t b = 0;
//
//    // Start at CLASS, skip B5 62
//    for (uint16_t i = 2; i < len; i++)
//    {
//        a += msg[i];
//        b += a;
//    }
//
//    *cka = a;
//    *ckb = b;
//}


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

uint8_t usbRxBuf[USBBUF_MAXLEN];

uint16_t usbRxBufLen = 0;

uint8_t  usbRxFlag 	 = 0;

/* USB IRQ producer / main-loop consumer. Never parse commands in the ISR. */
void USB_RXCallback(uint8_t* Buf, uint32_t *Len)
{
    for (uint32_t i = 0; i < *Len; ++i)
    {
        uint16_t next = (simulation_usb_rx_head + 1U) % sizeof(simulation_usb_rx);
        if (next == simulation_usb_rx_tail) break;
        simulation_usb_rx[simulation_usb_rx_head] = Buf[i];
        __DMB();
        simulation_usb_rx_head = next;
    }
}



/* UART is polled one byte per loop: no HAL_MAX_DELAY, RX wait, or IRQ dependency.
 * The shared TX buffer is owned until TC, including the final stop bit. */
static uint8_t SimulationCom_Queue(const char *text)
{
    if (simulation_tx_length != 0U) return 0U;
    size_t length = strlen(text);
    if (length == 0U || length >= sizeof(simulation_log_frame)) return 0U;
    if (text != simulation_log_frame) memcpy(simulation_log_frame, text, length);
    simulation_tx_length = (uint16_t)length;
    simulation_tx_offset = 0U;
    simulation_usb_tx_started = 0U;
    simulation_tx_tick = HAL_GetTick();
    return 1U;
}

static void SimulationLog_Abort(const char *reason)
{
    simulation_phase = SIM_ERROR;
    simulation_frame_pending = 0U;
    simulation_waiting_ack = 0U;
    simulation_tx_length = 0U;
    snprintf(simulation_log_line, sizeof(simulation_log_line), "ERR,%s\r\n", reason);
    (void)SimulationCom_Queue(simulation_log_line);
}

static uint16_t SimulationLog_Crc16Update(uint16_t crc, uint8_t value)
{
    crc ^= (uint16_t)value << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        crc = ((crc & 0x8000U) != 0U)
                  ? (uint16_t)((crc << 1) ^ 0x1021U)
                  : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint8_t SimulationLog_ParseAck(const char *line,
                                      uint32_t expected_sequence)
{
    if (strlen(line) != SIMULATION_LOG_ACK_LINE_LENGTH)
    {
        return 0U;
    }

    uint8_t result = 0U;
    if (strncmp(line, "ACK,", 4U) == 0)
    {
        result = SIMULATION_LOG_ACK_ACCEPTED;
    }
    else if (strncmp(line, "NAK,", 4U) == 0)
    {
        result = SIMULATION_LOG_ACK_RETRY;
    }
    else
    {
        return 0U;
    }

    uint32_t received_sequence = 0U;
    for (uint8_t digit_index = 0U; digit_index < 8U; digit_index++)
    {
        const char digit = line[4U + digit_index];
        uint8_t value;
        if (digit >= '0' && digit <= '9')
        {
            value = (uint8_t)(digit - '0');
        }
        else if (digit >= 'A' && digit <= 'F')
        {
            value = (uint8_t)(digit - 'A' + 10);
        }
        else if (digit >= 'a' && digit <= 'f')
        {
            value = (uint8_t)(digit - 'a' + 10);
        }
        else
        {
            return 0U;
        }
        received_sequence = (received_sequence << 4) | value;
    }

    return (received_sequence == expected_sequence) ? result : 0U;
}


static uint8_t SimulationLog_SendFrame(uint32_t sequence, const char *payload)
{
    int length = snprintf(simulation_log_frame, sizeof(simulation_log_frame),
                          "@%08lX|%s", (unsigned long)sequence, payload);
    if (length <= 0 || (size_t)length + 8U >= sizeof(simulation_log_frame)) return 0U;
    uint16_t crc = 0xFFFFU;
    for (int i = 1; i < length; ++i)
        crc = SimulationLog_Crc16Update(crc, (uint8_t)simulation_log_frame[i]);
    snprintf(&simulation_log_frame[length], sizeof(simulation_log_frame) - (size_t)length,
             "|%04X\r\n", (unsigned int)crc);
    simulation_frame_pending = 1U;
    simulation_attempts = 1U;
    return SimulationCom_Queue(simulation_log_frame);
}

static uint8_t SimulationLog_SendHeader(void)
{
    static const char header[] =
        "model_sample_time_s,plant_state_time_s,pos_n_m,pos_e_m,pos_d_m,q0,q1,q2,q3,"
        "vel_body_x_mps,vel_body_y_mps,vel_body_z_mps,"
        "rate_body_x_rps,rate_body_y_rps,rate_body_z_rps,"
        "acc_body_x_mps2,acc_body_y_mps2,acc_body_z_mps2,"
        "wdot_body_x_rps2,wdot_body_y_rps2,wdot_body_z_rps2,"
        "nav_phi_rad,nav_theta_rad,nav_psi_rad,"
        "control_0_deg,control_1_deg,control_2_deg,control_3_deg";


    return SimulationLog_SendFrame(0U, header);
}

static uint8_t SimulationLog_WriteRow(uint32_t row)
{
    int length = snprintf(simulation_log_line, sizeof(simulation_log_line),
                          "%.6f,%.6f", row * SIMULATION_STEP_PERIOD_S,
                          row * SIMULATION_STEP_PERIOD_S);
    for (uint32_t i = 0U; i < 26U; ++i)
    {
        if (length <= 0 || (size_t)length >= sizeof(simulation_log_line)) return 0U;
        int added = snprintf(&simulation_log_line[length],
                             sizeof(simulation_log_line) - (size_t)length,
                             ",%.6f", (double)simulation_samples[row][i]);
        if (added <= 0 || (size_t)added >= sizeof(simulation_log_line) - (size_t)length)
            return 0U;
        length += added;
    }
    return SimulationLog_SendFrame(simulation_log_next_sequence, simulation_log_line);
}

static void SimulationLog_CaptureRow(void)
{
    const eom_bus *e = &simulated_plant_bus.eom_bus;
    float *row = simulation_samples[simulation_sample_count];
    uint32_t column = 0U;
    for (uint32_t i = 0; i < 3; ++i) row[column++] = (float)e->posNed_m[i];
    for (uint32_t i = 0; i < 4; ++i) row[column++] = (float)e->q_na[i];
    for (uint32_t i = 0; i < 3; ++i) row[column++] = (float)e->velBdy_mps[i];
    for (uint32_t i = 0; i < 3; ++i) row[column++] = (float)e->wBdy_rps[i];
    for (uint32_t i = 0; i < 3; ++i) row[column++] = (float)e->accBdy_mps2[i];
    for (uint32_t i = 0; i < 3; ++i) row[column++] = (float)e->wDotBdy_rps2[i];
    row[column++] = (float)simulated_navigation_bus.phiEst_rad;
    row[column++] = (float)simulated_navigation_bus.thetaEst_rad;
    row[column++] = (float)simulated_navigation_bus.psiEst_rad;
    for (uint32_t i = 0; i < 4; ++i) row[column++] = (float)simulated_control_deg[i];
}

/* The plant invokes this at its major output/update boundary: sensors and
 * truth are at t; the controller updates u(t) before actuator/discrete updates
 * and integration. A final sample at 35.00 s returns without advancing time. */
static boolean_T Simulation_RunController(sensor_bus *sensors, plant_bus *plant,
                                           real_T control_deg[4])
{
    (void)plant; /* Both bus pointers refer to the application's live buses. */
    autocodeModel_step(simulation_model, sensors,
                       &simulated_navigation_bus, &simulated_controller_bus,
                       control_deg);
    SimulationLog_CaptureRow();
    return simulation_sample_count + 1U < SIMULATION_DURATION_STEPS;
}

static void Simulation_Start(void)
{
    memset(&simulated_sensor_bus, 0, sizeof(simulated_sensor_bus));
    memset(simulated_control_deg, 0, sizeof(simulated_control_deg));
    autocodePlant_setup();
    autocodeModel_setup(&simulated_sensor_bus);
    simulation_model = autocodeModel_getModel();
    simulation_plant = autocodePlant_getModel();
    autocodePlant_SetFeedbackCallback(Simulation_RunController);
    simulation_sample_count = 0U;
    simulation_deadline_misses = 0U;
    simulation_step_max_us = 0U;
    simulation_start_us = __HAL_TIM_GET_COUNTER(&htim5);
    simulation_next_step_us = simulation_start_us;
    simulation_phase = SIM_RUNNING;
}

static void SimulationCom_OnLine(const char *line)
{
    if (simulation_waiting_ack != 0U)
    {
        uint8_t ack = SimulationLog_ParseAck(line, simulation_log_next_sequence);
        if (ack == SIMULATION_LOG_ACK_RETRY)
        {
            /* Service loop handles retransmission without waiting. */
            simulation_ack_tick = HAL_GetTick() - SIMULATION_LOG_ACK_TIMEOUT_MS;
        }
        else if (ack == SIMULATION_LOG_ACK_ACCEPTED)
        {
            simulation_waiting_ack = 0U;
            simulation_frame_pending = 0U;
            simulation_log_next_sequence++;
            if (simulation_phase == SIM_HEADER) simulation_phase = SIM_ROWS;
            else if (simulation_phase == SIM_ROWS)
            {
                simulation_transfer_row++;
                simulation_log_rows_sent++;
                if (simulation_transfer_row == SIMULATION_DURATION_STEPS) simulation_phase = SIM_END;
            }
            else if (simulation_phase == SIM_END) simulation_phase = SIM_DONE;
        }
        return;
    }
    if (simulation_phase != SIM_READY || simulation_tx_length != 0U) return;
    if (strcmp(line, SIMULATION_ECHO_COMMAND) == 0)
        (void)SimulationCom_Queue("ECHO_OK\r\n");
    else if (strcmp(line, SIMULATION_START_COMMAND) == 0)
    {
        (void)SimulationCom_Queue("STARTED\r\n");
        Simulation_Start();
    }
    else (void)SimulationCom_Queue("ERR,UNKNOWN_COMMAND\r\n");
}

static void SimulationCom_ReceiveByte(uint8_t byte, uint8_t from_usb)
{
    /* A host owns the link from its command through DONE. */
    if (simulation_phase != SIM_READY && from_usb != simulation_transport_usb) return;
    if (byte == '\n')
    {
        simulation_command[simulation_command_length] = '\0';
        if (!simulation_command_overflow && simulation_command_length)
        {
            if (simulation_phase == SIM_READY) simulation_transport_usb = from_usb;
            SimulationCom_OnLine(simulation_command);
        }
        simulation_command_length = 0U;
        simulation_command_overflow = 0U;
    }
    else if (byte != '\r')
    {
        if (simulation_command_length < sizeof(simulation_command) - 1U)
            simulation_command[simulation_command_length++] = (char)byte;
        else simulation_command_overflow = 1U;
    }
}

static void SimulationCom_Service(void)
{
    uint32_t now = HAL_GetTick();
    /* Clear line errors, including ORE after a disconnected host. */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)) __HAL_UART_CLEAR_FEFLAG(&huart3);
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE)) __HAL_UART_CLEAR_NEFLAG(&huart3);
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
        SimulationCom_ReceiveByte((uint8_t)huart3.Instance->RDR, 0U);
    if (simulation_usb_rx_tail != simulation_usb_rx_head)
    {
        uint8_t byte = simulation_usb_rx[simulation_usb_rx_tail];
        simulation_usb_rx_tail = (simulation_usb_rx_tail + 1U) % sizeof(simulation_usb_rx);
        SimulationCom_ReceiveByte(byte, 1U);
    }
    if (simulation_tx_length != 0U)
    {
        uint8_t complete = 0U;
        if (simulation_transport_usb)
        {
            USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL)
            {
                if (!simulation_usb_tx_started && cdc->TxState == 0U &&
                    CDC_Transmit_FS((uint8_t *)simulation_log_frame, simulation_tx_length) == USBD_OK)
                    simulation_usb_tx_started = 1U;
                else if (simulation_usb_tx_started && cdc->TxState == 0U) complete = 1U;
            }
        }
        else
        {
        if (simulation_tx_offset < simulation_tx_length)
        {
            if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE))
                huart3.Instance->TDR = (uint8_t)simulation_log_frame[simulation_tx_offset++];
        }
        else if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)) complete = 1U;
        }
        if (complete)
        {
            simulation_tx_length = 0U;
            if (simulation_frame_pending)
            {
                simulation_waiting_ack = 1U;
                simulation_ack_tick = now;
            }
        }
        if (simulation_tx_length && simulation_phase != SIM_READY && simulation_phase != SIM_ERROR &&
            (uint32_t)(now - simulation_tx_tick) >= SIMULATION_LOG_UART_TIMEOUT_MS)
            SimulationLog_Abort("UART_TX_TIMEOUT");
        return;
    }
    if (simulation_waiting_ack)
    {
        if ((uint32_t)(now - simulation_ack_tick) >= SIMULATION_LOG_ACK_TIMEOUT_MS)
        {
            if (simulation_attempts >= SIMULATION_LOG_MAX_ATTEMPTS)
                SimulationLog_Abort("ACK_TIMEOUT");
            else
            {
                simulation_attempts++;
                simulation_waiting_ack = 0U;
                (void)SimulationCom_Queue(simulation_log_frame);
            }
        }
        return;
    }
    if (simulation_phase == SIM_READY && (int32_t)(now - simulation_next_ready_ms) >= 0)
    {
        /* Prefer the enumerated board USB COM port; UART remains available. */
        simulation_transport_usb = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
        (void)SimulationCom_Queue("READY\r\n");
        simulation_next_ready_ms = now + 1000U;
    }
    else if (simulation_phase == SIM_HEADER)
    {
        if (!SimulationLog_SendHeader()) SimulationLog_Abort("HEADER_FORMAT_FAILED");
    }
    else if (simulation_phase == SIM_ROWS)
    {
        if (!SimulationLog_WriteRow(simulation_transfer_row)) SimulationLog_Abort("ROW_FORMAT_FAILED");
    }
    else if (simulation_phase == SIM_END)
    {
        snprintf(simulation_log_line, sizeof(simulation_log_line), "END,%lu", (unsigned long)simulation_log_rows_sent);
        if (!SimulationLog_SendFrame(simulation_log_next_sequence, simulation_log_line)) SimulationLog_Abort("END_FORMAT_FAILED");
    }
    else if (simulation_phase == SIM_DONE)
    {
        snprintf(simulation_log_line, sizeof(simulation_log_line), "DONE,%lu\r\n", (unsigned long)simulation_log_rows_sent);
        (void)SimulationCom_Queue(simulation_log_line);
        simulation_phase = SIM_READY;
        simulation_next_ready_ms = now + 1000U;
    }
}

static void Simulation_Service(void)
{
    if (simulation_phase != SIM_RUNNING) return;
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
    if ((int32_t)(now - simulation_next_step_us) < 0) return;
    /* Plant outputs -> controller -> plant state updates/integration, all in
     * the same sample. Capture occurs in the callback before minor solver stages. */
    if (simulation_sample_count < SIMULATION_DURATION_STEPS)
    {
        if (now - simulation_next_step_us >= SIMULATION_STEP_PERIOD_US)
        {
            simulation_deadline_misses++;
            SimulationLog_Abort("SIMULATION_DEADLINE_MISSED");
            return;
        }
        autocodePlant_step(simulation_plant, simulated_control_deg,
                           &simulated_sensor_bus, &simulated_plant_bus);
        simulation_sample_count++;
        uint32_t elapsed = __HAL_TIM_GET_COUNTER(&htim5) - now;
        if (elapsed > simulation_step_max_us) simulation_step_max_us = elapsed;
        if ((int32_t)(__HAL_TIM_GET_COUNTER(&htim5) - (simulation_next_step_us + SIMULATION_STEP_PERIOD_US)) >= 0)
        {
            simulation_deadline_misses++;
            SimulationLog_Abort("SIMULATION_STEP_OVERRUN");
            return;
        }
        if (simulation_sample_count < SIMULATION_DURATION_STEPS)
            simulation_next_step_us += SIMULATION_STEP_PERIOD_US;
    }
    /* Keep the run phase alive to its 35 s boundary before starting UART output. */
    else
    {
        simulation_log_next_sequence = 0U;
        simulation_log_rows_sent = 0U;
        simulation_transfer_row = 0U;
        simulation_phase = SIM_HEADER;
    }
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
  /* Generated double-precision plant code needs M7 caches to meet 10 ms. USB
   * uses CPU-driven FIFO transfers (PCD DMA disabled), so no DMA cache alias. */
  SCB_EnableICache();
  SCB_EnableDCache();

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

  /* UART simulation needs no sensor, USB, SD, CAN or CM4 initialization. */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start(&htim5) != HAL_OK) Error_Handler();
  simulation_phase = SIM_READY;
  simulation_next_ready_ms = HAL_GetTick();
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Simulation_Service();
    SimulationCom_Service();
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
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_2V5_SUPPLIES_LDO) != HAL_OK) Error_Handler();

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  const uint32_t voltage_start = HAL_GetTick();
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
    if ((uint32_t)(HAL_GetTick() - voltage_start) >= 100U) Error_Handler();
  }

  /* HSI / 8 * 60 = 480 MHz VCO; CPU 240 MHz, AHB 120 MHz, USB 48 MHz. */
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
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
  /* HSI: no external-clock security interrupt required. */
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
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0U) timer_clock *= 2U;
  htim5.Init.Prescaler = timer_clock / 1000000U - 1U;
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
