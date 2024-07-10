/**
  ******************************************************************************
  * @file           : out16a1_conf.h
  * @author         : AMS IPC Application Team
  * @brief          : Header file for the project configuration
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
#ifndef OUT16A1_CONF_H
#define OUT16A1_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "ips8200hq.h"

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

#define USE_BOARD_0 1

#define H_EXTI_LINE_FAULT_L EXTI_LINE_8
#define H_EXTI_8 hexti8
#define H_EXTI_FAULT_L hexti8
/** This is the HW configuration section: FAULT_L_Pin */
#define FAULT_L_Pin GPIO_PIN_8
/** This is the HW configuration section: FAULT_L_GPIO_Port */
#define FAULT_L_GPIO_Port GPIOB
/** This is the HW configuration section: FAULT_L_EXTI_IRQn */
#define FAULT_L_EXTI_IRQn EXTI9_5_IRQn

#define H_EXTI_LINE_PGOOD_L EXTI_LINE_9
#define H_EXTI_9 hexti9
#define H_EXTI_PGOOD_L hexti9
/** This is the HW configuration section: PGOOD_L_Pin */
#define PGOOD_L_Pin GPIO_PIN_9
/** This is the HW configuration section: PGOOD_L_GPIO_Port */
#define PGOOD_L_GPIO_Port GPIOB
/** This is the HW configuration section: PGOOD_L_EXTI_IRQn */
#define PGOOD_L_EXTI_IRQn EXTI9_5_IRQn

#define H_EXTI_LINE_TWARN_L EXTI_LINE_1
#define H_EXTI_1 hexti1
#define H_EXTI_TWARN_L hexti1
/** This is the HW configuration section: TWARN_L_Pin */
#define TWARN_L_Pin GPIO_PIN_1
/** This is the HW configuration section: TWARN_L_GPIO_Port */
#define TWARN_L_GPIO_Port GPIOC
/** This is the HW configuration section: TWARN_L_EXTI_IRQn */
#define TWARN_L_EXTI_IRQn EXTI1_IRQn

/** This is the HW configuration section: IN1_Pin */
#define IN1_Pin GPIO_PIN_7
/** This is the HW configuration section: IN1_GPIO_Port */
#define IN1_GPIO_Port GPIOC
/** This is the HW configuration section: IN2_Pin */
#define IN2_Pin GPIO_PIN_9
/** This is the HW configuration section: IN2_GPIO_Port */
#define IN2_GPIO_Port GPIOA
/** This is the HW configuration section: IN3_Pin */
#define IN3_Pin GPIO_PIN_8
/** This is the HW configuration section: IN3_GPIO_Port */
#define IN3_GPIO_Port GPIOA
/** This is the HW configuration section: IN4_Pin */
#define IN4_Pin GPIO_PIN_10
/** This is the HW configuration section: IN4_GPIO_Port */
#define IN4_GPIO_Port GPIOB
/** This is the HW configuration section: IN5_Pin */
#define IN5_Pin GPIO_PIN_0
/** This is the HW configuration section: IN5_GPIO_Port */
#define IN5_GPIO_Port GPIOB
/** This is the HW configuration section: IN6_Pin */
#define IN6_Pin GPIO_PIN_5
/** This is the HW configuration section: IN6_GPIO_Port */
#define IN6_GPIO_Port GPIOB
/** This is the HW configuration section: IN7_Pin */
#define IN7_Pin GPIO_PIN_6
/** This is the HW configuration section: IN7_GPIO_Port */
#define IN7_GPIO_Port GPIOB
/** This is the HW configuration section: IN8_Pin */
#define IN8_Pin GPIO_PIN_10
/** This is the HW configuration section: IN8_GPIO_Port */
#define IN8_GPIO_Port GPIOA

/** This is the HW configuration section: SEL2_L_Pin */
#define SEL2_L_Pin GPIO_PIN_0
/** This is the HW configuration section: SEL2_L_GPIO_Port */
#define SEL2_L_GPIO_Port GPIOA
/** This is the HW configuration section: SEL1_Pin */
#define SEL1_Pin GPIO_PIN_0
/** This is the HW configuration section: SEL1_GPIO_Port */
#define SEL1_GPIO_Port GPIOC
/** This is the HW configuration section: WD_Pin */
#define WD_Pin GPIO_PIN_4
/** This is the HW configuration section: WD_GPIO_Port */
#define WD_GPIO_Port GPIOA
/** This is the HW configuration section: WDEN_Pin */
#define WDEN_Pin GPIO_PIN_3
/** This is the HW configuration section: WDEN_GPIO_Port */
#define WDEN_GPIO_Port GPIOB

/** CHIP ID  */
#define IPS_SWITCH_CHIP_ID IPS8200HQ_CHIP_ID

/** Maximum number of devices supported */
#define IPS_SWITCH_DEVICES_NBR IPS8200HQ_DEVICES_NBR

/** Maximum number of instances supported */
#define IPS_INSTANCES_NBR IPS8200HQ_INSTANCES_NBR
/* Control Mode settings */

/** Undefined Control mode setting */
#define IPS_UNDEF_CTRL_MODE IPS8200HQ_UNDEF_CTRL_MODE

/** Parallel Control mode setting */
#define IPS_PAR_CTRL_MODE IPS8200HQ_PAR_CTRL_MODE

/** SPI Control mode setting */
#define IPS_SPI_CTRL_MODE IPS8200HQ_SPI_CTRL_MODE

/** SPI Daisy Chain Control mode setting */
#define IPS_SPI_DC_CTRL_MODE IPS8200HQ_SPI_DC_CTRL_MODE

/* SPI Width settings */

/** Undefined SPI Width setting */
#define IPS_SPI_W_NONE IPS8200HQ_SPI_W_NONE

/** SPI 8bit Width setting */
#define IPS_SPI_W_8BIT IPS8200HQ_SPI_W_8BIT

/** SPI 16bit Width setting */
#define IPS_SPI_W_16BIT IPS8200HQ_SPI_W_16BIT

/* Dev pins for IPS8200HQ (Parallel Interface) */
/* Board 0 */
/** Input Pin: IN1 */
#define IPS_SWITCH_0_IN1 IPS8200HQ_0_IN1
/** Input Pin: IN2 */
#define IPS_SWITCH_0_IN2 IPS8200HQ_0_IN2
/** Input Pin: IN3 */
#define IPS_SWITCH_0_IN3 IPS8200HQ_0_IN3
/** Input Pin: IN4 */
#define IPS_SWITCH_0_IN4 IPS8200HQ_0_IN4
/** Input Pin: IN5 */
#define IPS_SWITCH_0_IN5 IPS8200HQ_0_IN5
/** Input Pin: IN6 */
#define IPS_SWITCH_0_IN6 IPS8200HQ_0_IN6
/** Input Pin: IN7 */
#define IPS_SWITCH_0_IN7 IPS8200HQ_0_IN7
/** Input Pin: IN8 */
#define IPS_SWITCH_0_IN8 IPS8200HQ_0_IN8

/** Control Pin: SEL2_L */
#define IPS_SWITCH_0_SEL2_L IPS8200HQ_0_SEL2_L
/** Control Pin: SEL1 */
#define IPS_SWITCH_0_SEL1 IPS8200HQ_0_SEL1
/** Control Pin: WD */
#define IPS_SWITCH_0_WD IPS8200HQ_0_WD
/** Control Pin: WDEN */
#define IPS_SWITCH_0_WDEN IPS8200HQ_0_WDEN

/** IN1 Port 0 */
#define IPS_SWITCH_0_IN1_GPIO_PORT IPS8200HQ_0_IN1_GPIO_PORT
/** IN1 Pin 0 */
#define IPS_SWITCH_0_IN1_GPIO_PIN IPS8200HQ_0_IN1_GPIO_PIN
/** IN2 Port 0 */
#define IPS_SWITCH_0_IN2_GPIO_PORT IPS8200HQ_0_IN2_GPIO_PORT
/** IN2 Pin 0 */
#define IPS_SWITCH_0_IN2_GPIO_PIN IPS8200HQ_0_IN2_GPIO_PIN
/** IN3 Port 0 */
#define IPS_SWITCH_0_IN3_GPIO_PORT IPS8200HQ_0_IN3_GPIO_PORT
/** IN3 Pin 0 */
#define IPS_SWITCH_0_IN3_GPIO_PIN IPS8200HQ_0_IN3_GPIO_PIN
/** IN4 Port 0 */
#define IPS_SWITCH_0_IN4_GPIO_PORT IPS8200HQ_0_IN4_GPIO_PORT
/** IN4 Pin 0 */
#define IPS_SWITCH_0_IN4_GPIO_PIN IPS8200HQ_0_IN4_GPIO_PIN
/** IN5 Port 0 */
#define IPS_SWITCH_0_IN5_GPIO_PORT IPS8200HQ_0_IN5_GPIO_PORT
/** IN5 Pin 0 */
#define IPS_SWITCH_0_IN5_GPIO_PIN IPS8200HQ_0_IN5_GPIO_PIN
/** IN6 Port 0 */
#define IPS_SWITCH_0_IN6_GPIO_PORT IPS8200HQ_0_IN6_GPIO_PORT
/** IN6 Pin 0 */
#define IPS_SWITCH_0_IN6_GPIO_PIN IPS8200HQ_0_IN6_GPIO_PIN
/** IN7 Port 0 */
#define IPS_SWITCH_0_IN7_GPIO_PORT IPS8200HQ_0_IN7_GPIO_PORT
/** IN7 Pin 0 */
#define IPS_SWITCH_0_IN7_GPIO_PIN IPS8200HQ_0_IN7_GPIO_PIN
/** IN8 Port 0 */
#define IPS_SWITCH_0_IN8_GPIO_PORT IPS8200HQ_0_IN8_GPIO_PORT
/** IN8 Pin 0 */
#define IPS_SWITCH_0_IN8_GPIO_PIN IPS8200HQ_0_IN8_GPIO_PIN

/** SEL2_L Port 0 */
#define IPS_SWITCH_0_SEL2_L_GPIO_PORT IPS8200HQ_0_SEL2_L_GPIO_PORT
/** SEL2_L Pin 0 */
#define IPS_SWITCH_0_SEL2_L_GPIO_PIN IPS8200HQ_0_SEL2_L_GPIO_PIN
/** SEL1 Port 0 */
#define IPS_SWITCH_0_SEL1_GPIO_PORT IPS8200HQ_0_SEL1_GPIO_PORT
/** SEL1 Pin 0 */
#define IPS_SWITCH_0_SEL1_GPIO_PIN IPS8200HQ_0_SEL1_GPIO_PIN
/** WD Port 0 */
#define IPS_SWITCH_0_WD_GPIO_PORT IPS8200HQ_0_WD_GPIO_PORT
/** WD Pin 0 */
#define IPS_SWITCH_0_WD_GPIO_PIN IPS8200HQ_0_WD_GPIO_PIN
/** WDEN Port 0 */
#define IPS_SWITCH_0_WDEN_GPIO_PORT IPS8200HQ_0_WDEN_GPIO_PORT
/** WDEN Pin 0 */
#define IPS_SWITCH_0_WDEN_GPIO_PIN IPS8200HQ_0_WDEN_GPIO_PIN

/** FAULT_L Port 0 */
#define IPS_SWITCH_0_FAULT_L_GPIO_PORT IPS8200HQ_0_FAULT_L_GPIO_PORT
/** FAULT_L Pin 0 */
#define IPS_SWITCH_0_FAULT_L_GPIO_PIN IPS8200HQ_0_FAULT_L_GPIO_PIN
/** PGOOD_L Port 0 */
#define IPS_SWITCH_0_PGOOD_L_GPIO_PORT IPS8200HQ_0_PGOOD_L_GPIO_PORT
/** PGOOD_L Pin 0 */
#define IPS_SWITCH_0_PGOOD_L_GPIO_PIN IPS8200HQ_0_PGOOD_L_GPIO_PIN
/** TWARN_L Port 0 */
#define IPS_SWITCH_0_TWARN_L_GPIO_PORT IPS8200HQ_0_TWARN_L_GPIO_PORT
/** TWARN_L Pin 0 */
#define IPS_SWITCH_0_TWARN_L_GPIO_PIN IPS8200HQ_0_TWARN_L_GPIO_PIN

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Conf_Exported_Variables OUT_16_Example Conf Exported Variables
  * @{
  */

extern EXTI_HandleTypeDef hexti1;
extern EXTI_HandleTypeDef hexti8;
extern EXTI_HandleTypeDef hexti9;

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Conf_Exported_Macros OUT_16_Example Conf Exported Macros
  * @{
  */

/** Customization of generic driver function */
#define OUT16_GetTick         BSP_GetTick
/** Customization of generic driver function */
#define OUT16_WriteChan       OUT16_SetChanInputPin
/** Customization of generic driver function */
#define OUT16_ReadChan        OUT16_GetChanInputPin
/** Customization of generic driver function */
#define OUT16_WriteAllChan    OUT16_SetAllChanInputPin
/** Customization of generic driver function */
#define OUT16_ReadAllChan     OUT16_GetAllChanInputPin
/** Customization of generic driver function */
#define OUT16_ReadFault       OUT16_ReadFaultPin
/** Customization of generic driver function */
#define OUT16_SetPwm          OUT16_SetTimerForPwm
/** Customization of generic driver function */
#define OUT16_WritePin        OUT16_SetControlPin
/** Customization of generic driver function */
#define OUT16_ReadPin         OUT16_GetControlPin

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

/** @addtogroup OUT_16_Example_Conf_Exported_Functions OUT_16_Example Conf Exported Functions
  * @{
  */

int32_t OUT16_SetTimerForPwm(uint8_t pwmEnable);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* OUT16A1_CONF_H */

