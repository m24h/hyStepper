/*
 * inputter_dirpul.c
 *
 *  Created on: 2023.10.2
 *      Author: johnny
 */
#include "inputter.h"
#include "motion.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"
#include "air001xx_ll_exti.h"
#include "air001xx_ll_tim.h"

#ifdef INPUTTER_DIRPUL

#error deprecated, uncompleted, not tested

#define STEPIO_LEVEL_VALID   0           /* 0 for opendrain optocoupler */

static void init(void)
{
  // TIM1;
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_TIM1);
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_TIM1);

  LL_TIM_DisableCounter(TIM1);
  LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetPrescaler(TIM1, 0);
  LL_TIM_SetRepetitionCounter(TIM1, 0);
  LL_TIM_SetAutoReload(TIM1, 0xFFFF);
  LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X2_TI2);
  LL_TIM_IC_Config(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV8_N6 | LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_Config(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1 | LL_TIM_IC_POLARITY_RISING);
  //LL_TIM_IC_Config(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1 | LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_EnableXORCombination(TIM1); // make T1 Xor T2 (T3 is unknown), maybe can use PB6 (also used by BOOT0/PF4)

  LL_TIM_GenerateEvent_UPDATE(TIM1); // load ARR
  TIM1->DIER=0; // disable all intr.
  TIM1->SR=0;   // clear all flags
  LL_TIM_SetCounter(TIM1, 0);
  LL_TIM_EnableCounter(TIM1);

  // PA0:EN PA3:DIR/TIM1_CH1 PB3:PUL/TIM1_CH2
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA|LL_IOP_GRP1_PERIPH_GPIOB);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, STEPIO_LEVEL_VALID?LL_GPIO_PULL_DOWN:LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, STEPIO_LEVEL_VALID?LL_GPIO_PULL_DOWN:LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF13_TIM1);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, STEPIO_LEVEL_VALID?LL_GPIO_PULL_DOWN:LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF1_TIM1);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

  LL_EXTI_EnableFallingTrig(LL_EXTI_LINE_0);
  LL_EXTI_EnableRisingTrig(LL_EXTI_LINE_0);
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_LINE_0);
  LL_EXTI_EnableIT(LL_EXTI_LINE_0);

  NVIC_SetPriority(EXTI0_1_IRQn, 2);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

int32_t  inputter_get()
{
  return (int32_t)LL_TIM_GetCounter(TIM1);
}

void __attribute((interrupt("IRQ"))) EXTI0_1_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_0)) {
    LL_EXTI_ClearFlag(LL_EXTI_LINE_0);
    if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0))
      if (STEPIO_LEVEL_VALID)
        motion_enable();
      else
        motion_disable();
    else
      if (STEPIO_LEVEL_VALID)
        motion_disable();
      else
        motion_enable();
  }
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  (void)cmdstr;
  app_resp_printf(resp, "DirPul: POS=%ld\r\n", inputter_get());
}

APP_REG_SUBMODULE("inp", &init, NULL, &cmd, "traditional DIR/PUL interface")

#endif
