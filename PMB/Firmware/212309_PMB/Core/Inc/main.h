/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define DOORB_Pin GPIO_PIN_13
#define DOORB_GPIO_Port GPIOC
#define DOORB_EXTI_IRQn EXTI15_10_IRQn
#define DOORA_Pin GPIO_PIN_14
#define DOORA_GPIO_Port GPIOC
#define DOORA_EXTI_IRQn EXTI15_10_IRQn
#define TEST_LINK_Pin GPIO_PIN_15
#define TEST_LINK_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOD
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOD
#define SECOND_PC_LED_Pin GPIO_PIN_0
#define SECOND_PC_LED_GPIO_Port GPIOC
#define RS485_CTRL_Pin GPIO_PIN_1
#define RS485_CTRL_GPIO_Port GPIOC
#define BATTERY_CHGR_EN_Pin GPIO_PIN_2
#define BATTERY_CHGR_EN_GPIO_Port GPIOC
#define FAIL_12V_Pin GPIO_PIN_3
#define FAIL_12V_GPIO_Port GPIOC
#define FAN1_TACHO_Pin GPIO_PIN_0
#define FAN1_TACHO_GPIO_Port GPIOA
#define FAN1_PWM_Pin GPIO_PIN_1
#define FAN1_PWM_GPIO_Port GPIOA
#define MODEM_TX_Pin GPIO_PIN_2
#define MODEM_TX_GPIO_Port GPIOA
#define MODEM_RX_Pin GPIO_PIN_3
#define MODEM_RX_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define PSU2_DC_OK_Pin GPIO_PIN_4
#define PSU2_DC_OK_GPIO_Port GPIOC
#define PSU3_DC_OK_Pin GPIO_PIN_5
#define PSU3_DC_OK_GPIO_Port GPIOC
#define PSU1_AC_OK_Pin GPIO_PIN_0
#define PSU1_AC_OK_GPIO_Port GPIOB
#define PSU2_AC_OK_Pin GPIO_PIN_1
#define PSU2_AC_OK_GPIO_Port GPIOB
#define PSU3_AC_OK_Pin GPIO_PIN_2
#define PSU3_AC_OK_GPIO_Port GPIOB
#define BUS_I2C_SCL_Pin GPIO_PIN_10
#define BUS_I2C_SCL_GPIO_Port GPIOB
#define BUS_I2C_SDA_Pin GPIO_PIN_11
#define BUS_I2C_SDA_GPIO_Port GPIOB
#define PSU1_DC_OK_Pin GPIO_PIN_14
#define PSU1_DC_OK_GPIO_Port GPIOB
#define SIM_PIN_Pin GPIO_PIN_15
#define SIM_PIN_GPIO_Port GPIOB
#define SIM_PIN_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_LED_Pin GPIO_PIN_6
#define STATUS_LED_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_7
#define ERROR_LED_GPIO_Port GPIOC
#define FAN2_TACHO_Pin GPIO_PIN_8
#define FAN2_TACHO_GPIO_Port GPIOC
#define FAN2_PWM_Pin GPIO_PIN_9
#define FAN2_PWM_GPIO_Port GPIOC
#define ECA_HEATER_RLY_Pin GPIO_PIN_8
#define ECA_HEATER_RLY_GPIO_Port GPIOA
#define CHASSIS_HEATER_RLY_Pin GPIO_PIN_9
#define CHASSIS_HEATER_RLY_GPIO_Port GPIOA
#define ENERGY_METER_Pin GPIO_PIN_10
#define ENERGY_METER_GPIO_Port GPIOA
#define ENERGY_METER_EXTI_IRQn EXTI15_10_IRQn
#define JTMS_SWDIO_Pin GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port GPIOA
#define JTCK_SWCLK_Pin GPIO_PIN_14
#define JTCK_SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define PC_UART3_TX_Pin GPIO_PIN_10
#define PC_UART3_TX_GPIO_Port GPIOC
#define PC_UART3_RX_Pin GPIO_PIN_11
#define PC_UART3_RX_GPIO_Port GPIOC
#define RS485_TX_Pin GPIO_PIN_12
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_2
#define RS485_RX_GPIO_Port GPIOD
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define NJTRST_Pin GPIO_PIN_4
#define NJTRST_GPIO_Port GPIOB
#define PC_LED_Pin GPIO_PIN_5
#define PC_LED_GPIO_Port GPIOB
#define PC_UART1_TX_Pin GPIO_PIN_6
#define PC_UART1_TX_GPIO_Port GPIOB
#define PC_UART1_RX_Pin GPIO_PIN_7
#define PC_UART1_RX_GPIO_Port GPIOB
#define BATT_I2C_SCL_Pin GPIO_PIN_8
#define BATT_I2C_SCL_GPIO_Port GPIOB
#define BATT_I2C_SDA_Pin GPIO_PIN_9
#define BATT_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
