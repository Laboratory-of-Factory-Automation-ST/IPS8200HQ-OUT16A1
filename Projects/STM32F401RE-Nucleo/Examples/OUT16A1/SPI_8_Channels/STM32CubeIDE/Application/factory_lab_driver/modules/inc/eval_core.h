/**
  ******************************************************************************
  * @file    eval_core.h
  * @brief   This file contains the definition of core hardware evaluation
  * 		 utilities.
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
#ifndef APPLICATION_FACTORY_LAB_DRIVER_MODULES_INC_EVAL_CORE_H_
#define APPLICATION_FACTORY_LAB_DRIVER_MODULES_INC_EVAL_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum Board_ctx_t {
	OUT16A1
} BoardContext_Typedef;

typedef enum Setup_ctx_t {
	_default
} SetupContext_Typedef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void usr_btn_routine(BoardContext_Typedef board, SetupContext_Typedef setup);
void usr_btn_pressed_callback();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_FACTORY_LAB_DRIVER_MODULES_INC_EVAL_CORE_H_ */
