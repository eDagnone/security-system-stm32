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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Window_Switch_1_Pin GPIO_PIN_0
#define Window_Switch_1_GPIO_Port GPIOC
#define Window_Switch_2_Pin GPIO_PIN_1
#define Window_Switch_2_GPIO_Port GPIOC
#define Window_Switch_3_Pin GPIO_PIN_2
#define Window_Switch_3_GPIO_Port GPIOC
#define Window_Switch_4_Pin GPIO_PIN_3
#define Window_Switch_4_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Distance_Sensor_1_Pin GPIO_PIN_6
#define Distance_Sensor_1_GPIO_Port GPIOA
#define Distance_sensor_2_Pin GPIO_PIN_7
#define Distance_sensor_2_GPIO_Port GPIOA
#define Window_2_Status_LED_Pin GPIO_PIN_4
#define Window_2_Status_LED_GPIO_Port GPIOC
#define Window_2_Status_LEDC5_Pin GPIO_PIN_5
#define Window_2_Status_LEDC5_GPIO_Port GPIOC
#define Window_3_Status_LED_Pin GPIO_PIN_0
#define Window_3_Status_LED_GPIO_Port GPIOB
#define Window_3_Status_LEDB1_Pin GPIO_PIN_1
#define Window_3_Status_LEDB1_GPIO_Port GPIOB
#define Window_4_Status_LED_Pin GPIO_PIN_2
#define Window_4_Status_LED_GPIO_Port GPIOB
#define Window_4_Status_LEDB10_Pin GPIO_PIN_10
#define Window_4_Status_LEDB10_GPIO_Port GPIOB
#define Keypad_Output___Row_D_Pin GPIO_PIN_12
#define Keypad_Output___Row_D_GPIO_Port GPIOB
#define Keypad_Output___Row_C_Pin GPIO_PIN_13
#define Keypad_Output___Row_C_GPIO_Port GPIOB
#define Keypad_Output___Row_B_Pin GPIO_PIN_14
#define Keypad_Output___Row_B_GPIO_Port GPIOB
#define Keypad_Output___Row_A_Pin GPIO_PIN_15
#define Keypad_Output___Row_A_GPIO_Port GPIOB
#define Keypad_Input___Column_2_Pin GPIO_PIN_9
#define Keypad_Input___Column_2_GPIO_Port GPIOC
#define Keypad_Input___Column_1_Pin GPIO_PIN_8
#define Keypad_Input___Column_1_GPIO_Port GPIOA
#define Door_2_Status_LED_Pin GPIO_PIN_9
#define Door_2_Status_LED_GPIO_Port GPIOA
#define Door_2_Status_LEDA10_Pin GPIO_PIN_10
#define Door_2_Status_LEDA10_GPIO_Port GPIOA
#define Door_1_Status_LED_Pin GPIO_PIN_11
#define Door_1_Status_LED_GPIO_Port GPIOA
#define Door_1_Status_LEDA12_Pin GPIO_PIN_12
#define Door_1_Status_LEDA12_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Window_1_Status_LED_Pin GPIO_PIN_10
#define Window_1_Status_LED_GPIO_Port GPIOC
#define Window_1_Status_LEDC11_Pin GPIO_PIN_11
#define Window_1_Status_LEDC11_GPIO_Port GPIOC
#define SystemAlarm_Pin GPIO_PIN_12
#define SystemAlarm_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Lighting_Relay_2_Pin GPIO_PIN_4
#define Lighting_Relay_2_GPIO_Port GPIOB
#define Lighting_Relay_1_Pin GPIO_PIN_5
#define Lighting_Relay_1_GPIO_Port GPIOB
#define Alarm_Relay_Pin GPIO_PIN_6
#define Alarm_Relay_GPIO_Port GPIOB
#define Door_Switch_2_Pin GPIO_PIN_8
#define Door_Switch_2_GPIO_Port GPIOB
#define Door_Switch_1_Pin GPIO_PIN_9
#define Door_Switch_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
