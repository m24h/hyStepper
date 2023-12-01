/*
 * encoder_MT6701ACAK.c
 *
 *  Created on: 2023.9.30
 *      Author: johnny
 */

#include "encoder.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"
#include "air001xx_ll_exti.h"
#include "air001xx_ll_tim.h"

#if defined(ENCODER_MT6701AC) || defined(ENCODER_MT6701AK)

#error deprecated, uncompleted, not tested

#if defined(ENCODER_MT6701AC)
#define STEPS 4096U
#define CHIP_NAME "MT6701AC"
#elif defined(ENCODER_MT6701AK)
#define STEPS 4000U
#define CHIP_NAME "MT6701AK"
#else
#error unknown chip
#endif

static uint32_t z_adj;

static void init(void)
{
  // 5us delayed according to MT6701 datasheet
  // 4400ns added for encoder_get() itself
  // in fact, variable values above 300us are measured in dynamic scene
  encoder_delay_ticks=APP_CLK_SYS/1000*300/1000;

  // delay a while for MT6701 power-up A/B pulses, according to datasheet, 50ms
  uint32_t t=app_cnt_1ms+80;
  while((int32_t)(t-app_cnt_1ms)>0);

  // init TIM3
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);

  LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetPrescaler(TIM3, 0);
  LL_TIM_SetRepetitionCounter(TIM3, 0);
  LL_TIM_SetAutoReload(TIM3, STEPS-1);
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_Config(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1 | LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_Config(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1 | LL_TIM_IC_POLARITY_FALLING);
  LL_TIM_GenerateEvent_UPDATE(TIM3); // load ARR
  TIM3->DIER=0; // disable all intr.
  TIM3->SR=0;   // clear all flags
  LL_TIM_SetCounter(TIM3, 0);
  LL_TIM_EnableCounter(TIM3);

  // PA4:Z PA2:TIM3_CH1(A) PA5:TIM3_CH2(B)
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF13_TIM3);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF13_TIM3);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);

  // init Z EXTI4
  LL_EXTI_EnableRisingTrig(LL_EXTI_LINE_4);
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE4);
  LL_EXTI_EnableIT(LL_EXTI_LINE_4);

  NVIC_SetPriority(EXTI4_15_IRQn, 2);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  encoder_status=ENCODER_MODERATE; // always
  z_adj=0;
}

// for Z pulse
void __attribute__ ((interrupt("IRQ"))) EXTI4_15_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_4)) {
    LL_EXTI_ClearFlag(LL_EXTI_LINE_4);
    // for timer counting done
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    if (z_adj>=STEPS)
      z_adj=LL_TIM_GetCounter(TIM3);
    LL_TIM_SetCounter(TIM3, 0);
  }
}

int32_t encoder_get(void)
{
  uint32_t t=LL_TIM_GetCounter(TIM3)+z_adj;
  if (t>STEPS)
    t-=STEPS;

#if defined(ENCODER_MT6701AC)
  return (int32_t)(t<<20);
#elif defined(ENCODER_MT6701AK)
  return (int32_t)(t*1073741U);
#endif
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  (void)cmdstr;
  app_resp_printf(resp, CHIP_NAME ": POS=0x%08lX, STEPS=%u\r\n", encoder_get(), STEPS);
}

APP_REG_SUBMODULE("enc", &init, NULL, &cmd, "motor encoder " CHIP_NAME)

#endif
