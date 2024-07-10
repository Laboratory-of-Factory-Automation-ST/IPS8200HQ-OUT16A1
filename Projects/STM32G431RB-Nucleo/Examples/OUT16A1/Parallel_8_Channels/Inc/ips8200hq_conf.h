/**
  ******************************************************************************
  * @file    ips8200hq_conf.h
  * @author  AMS IPC IO-Link Application Team
  * @brief   Predefines values for the IPS8200HQ parameters
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
#ifndef IPS8200HQ_CONF_H
#define IPS8200HQ_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32g4xx_hal.h"
#include "app_ips_version.h"

/** @addtogroup OUT_16_Example OUT_16_Example
  * @{
  */

/** @addtogroup OUT_16_Example_Conf OUT_16_Example Conf
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Conf_Exported_Constants OUT_16_Example Conf Exported Constants
  * @{
  */

/** Maximum number of instances supported */
#define IPS8200HQ_INSTANCES_NBR (1U)

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* IPS8200HQ_CONF_H */
