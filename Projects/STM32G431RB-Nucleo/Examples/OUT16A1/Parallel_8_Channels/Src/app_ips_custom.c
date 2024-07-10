/**
  ******************************************************************************
  * @file           : app_ips_custom.c
  * @author         : AMS IPC Application Team
  * @brief          : Custom application code
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
#include "app_ips_custom.h"

/** @addtogroup OUT_16_Example OUT_16_Example
  * @{
  */

/* Global variables ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Private_defines OUT_16_Example Private defines
  * @{
  */

/** Number of states in the example application */
#define MAX_STEPS (11U)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Private_Variables OUT_16_Example Private Variables
  * @{
  */

/** Button press status */
static volatile uint8_t gButtonPressed = 0U;
static int32_t PushButtonState = GPIO_PIN_RESET;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @addtogroup OUT_16_Example_Private_Function_Prototypes OUT_16_Example Private Function Prototypes
  * @{
  */

static void TIM_Config(uint32_t Freq);
#ifdef IPS_DEBUG
void IPS_debug_GPIO_Init(void);
#endif /* #ifdef IPS_DEBUG */

/**
  * @}
  */

/* Functions Definition ------------------------------------------------------*/

/** @addtogroup OUT_16_Example_Functions_Definition OUT_16_Example Functions Definition
  * @{
  */

/**
  * @brief  Get the FW version of the current project
  * @param  pFwVersion pointer to the FW version
  * @retval 0 in case of success, an error code otherwise
  */
int32_t OUT16_GetFwVersion(uint32_t *pFwVersion)
{
  *pFwVersion = OUT16_FW_VERSION;
  return IPS_DEVICE_OK;
}

#ifdef IPS_DEBUG
/**
  * @brief IPS Debug Initialization Function
  * @retval None
  */
void IPS_debug_GPIO_Init(void)
{
  GPIO_InitTypeDef gpioInitStruct;

  /* Configure the debug pin PB4 (CN9.6) */
  gpioInitStruct.Pin = GPIO_PIN_4;
  gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInitStruct.Pull = GPIO_PULLUP;
  gpioInitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpioInitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}
#endif /* #ifdef IPS_DEBUG */

/**
  * @brief  Start/Stop timer which generates the PWM
  * @param  pwmEnable 0 to stop else to start
  * @retval 0 in case of success, an error code otherwise
  */
int32_t OUT16_SetTimerForPwm(uint8_t pwmEnable)
{
  int32_t ret;
  if (pwmEnable != 0U)
  {
    ret = (int32_t)HAL_TIM_Base_Start_IT(&IPS_TIM_Handle);
  }
  else
  {
    ret = (int32_t)HAL_TIM_Base_Stop_IT(&IPS_TIM_Handle);
  }
  return ret;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim timer handler
  * @retval None
  */
void IPS_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t Instance;
  IPS8200HQ_Object_t *pObj;

  if (htim == &IPS_TIM_Handle)
  {
    for (Instance = 0; Instance < OUT16_INSTANCES_NBR; Instance++)
    {
      pObj = (IPS8200HQ_Object_t *)SWITCH_CompObj[Instance];
      if ((pObj != NULL) && (pObj->isPresent == 0x1U))
      {
        (void)OUT16_SWITCH_PwmTick(Instance);
      }
    }
  }
}

/* Use HAL_EXTI_RegisterCallback */
static void fault_isr(void);
static void pgood_isr(void);
static void twarn_isr(void);

/**
  * @brief IRQ Callback register service
  * @retval None
  */
void set_out16_int_pin(void)
{
  /* register event irq handler */
  (void)OUT16_RegisterCallBack(OUT16_BOARD_0, fault_isr, pgood_isr, twarn_isr);
}

/**
  * @brief FAULT_L Interrupt Service Request
  * @retval None
  */
static void fault_isr(void)
{
  IPS8200HQ_Object_t *pObj;

  pObj = (IPS8200HQ_Object_t *)SWITCH_CompObj[OUT16_BOARD_0];
  if ((pObj != NULL) && (pObj->is_initialized != 0U))
  {
    /* Action to be customized */
    /* Fault on FAULT_L board 0 */
  }
}

/**
  * @brief PGOOD_L Interrupt Service Request
  * @retval None
  */
static void pgood_isr(void)
{
  IPS8200HQ_Object_t *pObj;

  pObj = (IPS8200HQ_Object_t *)SWITCH_CompObj[OUT16_BOARD_0];
  if ((pObj != NULL) && (pObj->is_initialized != 0U))
  {
    /* Action to be customized */
    /* Fault on PGOOD_L board 0 */
  }
}

/**
  * @brief TWARN_L Interrupt Service Request
  * @retval None
  */
static void twarn_isr(void)
{
  IPS8200HQ_Object_t *pObj;

  pObj = (IPS8200HQ_Object_t *)SWITCH_CompObj[OUT16_BOARD_0];
  if ((pObj != NULL) && (pObj->is_initialized != 0U))
  {
    /* Action to be customized */
    /* Fault on TWARN_L board 0 */
  }
}

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
/**
  * @brief Timer IRQ Callback register service
  * @retval None
  */
void set_ips_int_tim(void)
{
  HAL_TIM_RegisterCallback(&IPS_TIM_Handle, HAL_TIM_PERIOD_ELAPSED_CB_ID, IPS_TIM_PeriodElapsedCallback);
}
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

/**
  * @brief IPS Switch Initialization Function
  * @retval None
  */
void custom_app_init(void)
{
  OUT16_SWITCH_Init_Parameters_t switchInitParam;

  /* Initialize Timer */
  IPS_TIM_Init();

  /* Configure Timer to run with desired algorithm frequency */
  TIM_Config(PWM_TIMER_FREQ);

  /* Reset PWM status for all boards */
  Device_System_PWMEnable_bitmap = 0U;
  Device_System_CtrlMode[0U] = (uint8_t)OUT16_UNDEF_CTRL_MODE;
  Device_System_State_bitmap = 0U;
  Device_System_WdMode[0U] = 0U;
  IPS_NbInstances = 0U;
  IPS_NbDevices = OUT16_SWITCH_DEVICES_NBR;

  switchInitParam.pwmFreq = PWM_TIMER_FREQ;

  (void)OUT16_SWITCH_Init(OUT16_BOARD_0, OUT16_SWITCH_CHIP_ID, IPS_NbDevices, &switchInitParam);

#ifdef IPS_DEBUG
  IPS_debug_GPIO_Init();
#endif /* #ifdef IPS_DEBUG */

  /* Configure User Key Button */
  (void)BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;

  set_out16_int_pin();
  set_ips_int_tim();
}

/**
  * @brief IPS Switch custom operations execution function
  * @retval None
  */
void custom_app_process(void)
{
  /** Example step index */
  static volatile uint8_t gStep = MAX_STEPS;
  uint8_t ctrlMode;

  /* Each time the user button is pressed, the step is increased by 1 */
  if (gButtonPressed != 0U)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the button is released */
    while ((BSP_PB_GetState(BUTTON_KEY) == PushButtonState));

    /* Debouncing */
    HAL_Delay(50);

    /* Reset Interrupt flag */
    gButtonPressed = 0;

    gStep++;
    if (gStep > MAX_STEPS)
    {
      gStep = 0;
    }

    switch (gStep)
    {
      case 0:
        /*********** Step 0 ************/
        /* Parallel (SEL2=L) or SPI (SEL2=H) selection thru' SEL2_L signal (PA0-CN8.1) */
        (void)OUT16_SWITCH_SetOperatingMode(OUT16_BOARD_0, &ctrlMode);

        /* Set chan 0, 3, 4, 7 */
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 1U);
        break;

      case 1:
        /*********** Step 1 ************/
        /* Set chan 1, 2, 5, 6 */
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 1U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 1U);
        break;

      case 2:
        /*********** Step 2 ************/
        /* Clear chan 0, 1, 4, 5 */
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 0U);
        break;

      case 3:
        /*********** Step 3 ************/
        /* Clear chan 2, 3, 6, 7 */
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 0U);
        (void)OUT16_SWITCH_SetChannelStatus(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 0U);
        break;

      case 4:
        /*********** Step 4 ************/
        /* Set four channels in board 0 */
        (void)OUT16_SWITCH_SetAllChannelStatus(OUT16_BOARD_0, 0xF0U);
        break;

      case 5:
        /*********** Step 5 ************/
        /* Set four channels in board 0 */
        (void)OUT16_SWITCH_SetAllChannelStatus(OUT16_BOARD_0, 0x0FU);
        break;

      case 6:
        /*********** Step 6 ************/
        /* Clear all channels in  board 0 */
        (void)OUT16_SWITCH_SetAllChannelStatus(OUT16_BOARD_0, 0U);

        /* Set frequency for 2Hz for chan 0, 2, 4, 6 */
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 20U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 20U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 20U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 20U);

        /* Set frequency for 1Hz for chan 1, 3, 5, 7 */
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 10U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 10U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 10U);
        (void)OUT16_SWITCH_SetChannelFreq(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 10U);

        /* Set duty cycle at 25% for chan 0, 2, 4, 6 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 25U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 25U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 25U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 25U);

        /* Set duty cycle at 50% for chan 1, 3, 5, 7 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 50U);

        /* Start PWM mode all chans */
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 1U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 1U);
        break;

      case 7:
        /*********** Step 7 ************/
        /* Set duty cycle at 50% for chan 0, 2, 4, 6 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 50U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 50U);
        break;

      case 8:
        /*********** Step 8 ************/
        /* Set duty cycle at 75% for chan 1, 3, 5, 7 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 75U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 75U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 75U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 75U);
        break;

      case 9:
        /*********** Step 9 ************/
        /* Set duty cycle at 100% for chan 0, 2, 4, 6 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 100U);
        break;

      case 10:
        /*********** Step 10 ************/
        /* Set duty cycle at 100% for chan 1, 3, 5, 7 */
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 100U);
        (void)OUT16_SWITCH_SetChannelDc(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 100U);
        break;

      case 11:
      default:
        /*********** Step 11 ************/
        /* Stop PWM mode all chans */
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN1, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN2, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN3, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN4, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN5, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN6, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN7, 0U);
        (void)OUT16_SWITCH_SetPwmEnable(OUT16_BOARD_0, OUT16_SWITCH_0_IN8, 0U);
        break;
    }
  }
}

/**
  * @brief  Timer configuration
  * @param  Freq the desired Timer frequency
  * @retval None
  */
static void TIM_Config(uint32_t Freq)
{
  uint32_t timer_clock;
  uint32_t prescaler_value = 0;

  timer_clock = getTimerClk(IPS_TIM_INSTANCE);
  const uint32_t tim_counter_clock = timer_clock; /* TIM counter clock */
  uint32_t period = ((tim_counter_clock / (Freq * (prescaler_value + 1))) - 1);

  IPS_TIM_Handle.Init.Prescaler = prescaler_value;
  IPS_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  IPS_TIM_Handle.Init.Period = period;
  IPS_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  IPS_TIM_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&IPS_TIM_Handle) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  gButtonPressed = 1;
}

/**
  * @}
  */

/**
  * @}
  */
