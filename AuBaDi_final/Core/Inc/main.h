/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define TIM4_PERIOD 850
#define TIM4_PRESCALER 200
#define TIM5_PRESCALER 700
#define TIM5_PERIOD 700
#define TIM10_PERIOD 850
#define TIM10_PRESCALER 850
#define TIM3_PERIOD 300
#define TIM3_PRESCALER 238
#define button_left_Pin GPIO_PIN_3
#define button_left_GPIO_Port GPIOE
#define button_left_EXTI_IRQn EXTI3_IRQn
#define button_select_Pin GPIO_PIN_5
#define button_select_GPIO_Port GPIOE
#define button_select_EXTI_IRQn EXTI9_5_IRQn
#define USB_power_Pin GPIO_PIN_0
#define USB_power_GPIO_Port GPIOC
#define TIM5_CH2_Pin GPIO_PIN_1
#define TIM5_CH2_GPIO_Port GPIOA
#define TIM5_CH3_Pin GPIO_PIN_2
#define TIM5_CH3_GPIO_Port GPIOA
#define TIM5_CH4_Pin GPIO_PIN_3
#define TIM5_CH4_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define pump_relay1_Pin GPIO_PIN_7
#define pump_relay1_GPIO_Port GPIOE
#define pump_relay2_Pin GPIO_PIN_9
#define pump_relay2_GPIO_Port GPIOE
#define pump_relay3_Pin GPIO_PIN_11
#define pump_relay3_GPIO_Port GPIOE
#define pump_relay4_Pin GPIO_PIN_13
#define pump_relay4_GPIO_Port GPIOE
#define pump_relay5_Pin GPIO_PIN_15
#define pump_relay5_GPIO_Port GPIOE
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define endstop_Pin GPIO_PIN_11
#define endstop_GPIO_Port GPIOD
#define endstop_EXTI_IRQn EXTI15_10_IRQn
#define LED_green_Pin GPIO_PIN_12
#define LED_green_GPIO_Port GPIOD
#define LED_orange_Pin GPIO_PIN_13
#define LED_orange_GPIO_Port GPIOD
#define LED_red_Pin GPIO_PIN_14
#define LED_red_GPIO_Port GPIOD
#define LED_blue_Pin GPIO_PIN_15
#define LED_blue_GPIO_Port GPIOD
#define dir_Pin GPIO_PIN_6
#define dir_GPIO_Port GPIOC
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define step_Pin GPIO_PIN_8
#define step_GPIO_Port GPIOC
#define slp_rst_Pin GPIO_PIN_8
#define slp_rst_GPIO_Port GPIOA
#define I2S3_CK_Pin GPIO_PIN_10
#define I2S3_CK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define I2C2_SDA_Pin GPIO_PIN_3
#define I2C2_SDA_GPIO_Port GPIOB
#define I2X1_SCL_Pin GPIO_PIN_6
#define I2X1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define button_right_Pin GPIO_PIN_1
#define button_right_GPIO_Port GPIOE
#define button_right_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
