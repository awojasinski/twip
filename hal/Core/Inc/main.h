/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(char*, int);
void SystemClock_Config(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_L_direction1_Pin GPIO_PIN_0
#define Motor_L_direction1_GPIO_Port GPIOF
#define Motor_L_direction2_Pin GPIO_PIN_1
#define Motor_L_direction2_GPIO_Port GPIOF
#define Motor_R_hall1_Pin GPIO_PIN_0
#define Motor_R_hall1_GPIO_Port GPIOA
#define Motor_R_hall2_Pin GPIO_PIN_1
#define Motor_R_hall2_GPIO_Port GPIOA
#define ST_LINK_RX_Pin GPIO_PIN_2
#define ST_LINK_RX_GPIO_Port GPIOA
#define ST_LINK_TX_Pin GPIO_PIN_3
#define ST_LINK_TX_GPIO_Port GPIOA
#define Sensor_data_INT_Pin GPIO_PIN_4
#define Sensor_data_INT_GPIO_Port GPIOA
#define Sensor_data_INT_EXTI_IRQn EXTI4_IRQn
#define SD_card_SCK_Pin GPIO_PIN_5
#define SD_card_SCK_GPIO_Port GPIOA
#define Motor_L_hall1_Pin GPIO_PIN_6
#define Motor_L_hall1_GPIO_Port GPIOA
#define Motor_L_hall2_Pin GPIO_PIN_7
#define Motor_L_hall2_GPIO_Port GPIOA
#define Motor_R_direction2_Pin GPIO_PIN_0
#define Motor_R_direction2_GPIO_Port GPIOB
#define Motor_L_control_Pin GPIO_PIN_8
#define Motor_L_control_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_9
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOA
#define Motor_R_control_Pin GPIO_PIN_11
#define Motor_R_control_GPIO_Port GPIOA
#define Motor_R_direction1_Pin GPIO_PIN_12
#define Motor_R_direction1_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Sensor_SCL_Pin GPIO_PIN_15
#define Sensor_SCL_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define SD_card_MISO_Pin GPIO_PIN_4
#define SD_card_MISO_GPIO_Port GPIOB
#define SD_card_MOSI_Pin GPIO_PIN_5
#define SD_card_MOSI_GPIO_Port GPIOB
#define SD_card_detection_Pin GPIO_PIN_6
#define SD_card_detection_GPIO_Port GPIOB
#define SD_card_detection_EXTI_IRQn EXTI9_5_IRQn
#define Sensor_SDA_Pin GPIO_PIN_7
#define Sensor_SDA_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_CS_GPIO_Port LD2_GPIO_Port
#define SD_CS_Pin LD2_Pin

#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
