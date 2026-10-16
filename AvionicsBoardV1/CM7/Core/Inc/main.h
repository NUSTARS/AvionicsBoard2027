/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMP581_INT_Pin GPIO_PIN_6
#define BMP581_INT_GPIO_Port GPIOC
#define BMP581_INT_EXTI_IRQn EXTI9_5_IRQn
#define GNSS_NRST_Pin GPIO_PIN_15
#define GNSS_NRST_GPIO_Port GPIOA
#define BMP581_CS_Pin GPIO_PIN_1
#define BMP581_CS_GPIO_Port GPIOD
#define LSM_INT_Pin GPIO_PIN_9
#define LSM_INT_GPIO_Port GPIOG
#define LSM_INT_EXTI_IRQn EXTI9_5_IRQn
#define LSM_CS_Pin GPIO_PIN_10
#define LSM_CS_GPIO_Port GPIOG
#define BMP585_CS_Pin GPIO_PIN_11
#define BMP585_CS_GPIO_Port GPIOG
#define BMP585_INT_Pin GPIO_PIN_12
#define BMP585_INT_GPIO_Port GPIOG
#define BMP585_INT_EXTI_IRQn EXTI15_10_IRQn
#define BMI_INTG_Pin GPIO_PIN_3
#define BMI_INTG_GPIO_Port GPIOB
#define BMI_INTG_EXTI_IRQn EXTI3_IRQn
#define BMI_CSA_Pin GPIO_PIN_4
#define BMI_CSA_GPIO_Port GPIOB
#define BMI_INTA_Pin GPIO_PIN_8
#define BMI_INTA_GPIO_Port GPIOB
#define BMI_INTA_EXTI_IRQn EXTI9_5_IRQn
#define BMI_CSG_Pin GPIO_PIN_0
#define BMI_CSG_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#pragma pack(push, 1)

typedef struct {
    double x;
    double y;
    double z;
} Vector3f_t;

typedef struct {

    // Timing
    uint32_t timestamp_us;
    uint32_t packet_counter;

    // BMI088
    Vector3f_t bmi_accel;
    Vector3f_t bmi_gyro;

    // LSM6DSO
    Vector3f_t lsm_accel;
    Vector3f_t lsm_gyro;

    // BMP581
    float bmp581_pressure;
    float bmp581_temperature;

    // BMP585
    float bmp585_pressure;
    float bmp585_temperature;

    // GNSS
    double latitude;
    double longitude;
    float altitude;

    float velocity_north;
    float velocity_east;
    float velocity_down;

    // GNSS status
    uint8_t gnss_fix_type;
    uint8_t gnss_satellites;

} FlightData_t;

#pragma pack(pop)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
