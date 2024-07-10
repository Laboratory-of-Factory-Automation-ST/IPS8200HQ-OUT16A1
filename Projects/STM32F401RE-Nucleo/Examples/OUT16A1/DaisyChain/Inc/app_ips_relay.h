/**
  ******************************************************************************
  * @file           : app_ips_relay.h
  * @author         : AMS IPC Application Team
  * @brief          : Header file for app_ips_relay.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_IPS_RELAY_H
#define APP_IPS_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "out16a1.h"

/** @addtogroup OUT_16_Example OUT_16_Example
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @addtogroup OUT_16_Example_Exported_Functions OUT_16_Example Exported Functions
  * @{
  */

void custom_app_init(void);
void custom_app_process(void);

void MX_IPS_Relay_Init(void);
void MX_IPS_Relay_Process(void);

uint8_t getTimerClkSrc(TIM_TypeDef *tim);
uint32_t getTimerClk(TIM_TypeDef *tim);

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
void IPS_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif /* #if (USE_HAL_TIM_REGISTER_CALLBACKS == 1) */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* APP_IPS_RELAY_H */

