/**
  ******************************************************************************
  * @file    app_ips_conf.h
  * @author  SRA Application Team
  * @brief   BSP IP configuration file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_IPS_CONF_H__
#define __APP_IPS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_exti.h"

/** @addtogroup OUT_16_Example OUT_16_Example
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Exported_Constants OUT_16_Example Exported Constants
  * @{
  */

#define IPS_TIM_Handle htim3
#define IPS_TIM_Init MX_TIM3_Init
#define IPS_TIM_INSTANCE TIM3
#define PWM_TIMER_FREQ 10000

#define IPS_GUARD_TIM_Handle htim15
#define IPS_GUARD_TIM_Init MX_TIM15_Init
#define IPS_GUARD_TIM_INSTANCE TIM15

#define IPS_WD_TIM_Handle htim2
#define IPS_WD_TIM_Init MX_TIM2_Init
#define IPS_WD_TIM_INSTANCE TIM2

#define IPS_BUTTON_PORT GPIOC
#define IPS_BUTTON_PIN  GPIO_PIN_13
#define IPS_BUTTON_LINE 13
#define IPS_BUTTON_IRQn EXTI15_10_IRQn

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Exported_Variables OUT_16_Example Exported Variables
  * @{
  */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim2;

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* __APP_IPS_CONF_H__ */

