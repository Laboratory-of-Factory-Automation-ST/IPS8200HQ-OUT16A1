/**
  ******************************************************************************
  * @file    eval_core.c
  * @brief   This file containts the implementation of core hardware evaluation
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_ips_custom.h"
#include "out16a1.h"
#include "eval_core.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define ROUTINE_CONTEXT(board_ctx, setup_ctx, step) ((board_ctx << 16) + (setup_ctx << 8) + step)
/* Private variables ---------------------------------------------------------*/
_Bool usr_btn_pressed = 0;
uint8_t usr_btn_routine_step = 0;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/


/*
 * @brief Main implementation of user button routine
 * @param Board context specifier
 * @param Setup context specifier
 * @retval None
 */
void usr_btn_routine(BoardContext_Typedef board_ctx, SetupContext_Typedef setup_ctx) {
	uint8_t ctrlMode;
	if (usr_btn_pressed != 0) {
		/* Debouncing */
		HAL_Delay(50);

		/* Wait until the button is released */
		while ((HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET));

		/* Debouncing */
		HAL_Delay(50);

		usr_btn_pressed = 0;

		switch (ROUTINE_CONTEXT(board_ctx, setup_ctx, usr_btn_routine_step)) {
			case ROUTINE_CONTEXT(OUT16A1, _default, 0):
				OUT16_RELAY_SetOperatingMode(OUT16_BOARD_0, &ctrlMode);
				OUT16_RELAY_SetCtrlPinStatus(OUT16_BOARD_0, OUT16_RELAY_0_OUT_EN, 1U);
				OUT16_RELAY_SetAllChannelStatus(OUT16_BOARD_0, 0xFFU);
				usr_btn_routine_step += 1;
				break;
			case ROUTINE_CONTEXT(OUT16A1, _default, 1):
				OUT16_RELAY_SetAllChannelStatus(OUT16_BOARD_0, 0x00U);
				usr_btn_routine_step += 1;
				break;
			case ROUTINE_CONTEXT(OUT16A1, _default, 2):
				OUT16_RELAY_SetChannelFreq(OUT16_BOARD_0, OUT16_RELAY_0_IN1, 100U);
				OUT16_RELAY_SetChannelDc(OUT16_BOARD_0, OUT16_RELAY_0_IN1, 20U);
				OUT16_RELAY_SetPwmEnable(OUT16_BOARD_0, OUT16_RELAY_0_IN1, 1U);
				usr_btn_routine_step += 1;
				break;
			default:
				OUT16_RELAY_SetCtrlPinStatus(OUT16_BOARD_0, OUT16_RELAY_0_OUT_EN, 0U);
				usr_btn_routine_step = 0;
				break;
		}
	}
}

/*
 * @brief Custom callback handler of user button presses
 * @retval None
 */
void usr_btn_pressed_callback() {
	usr_btn_pressed = 1;
}
