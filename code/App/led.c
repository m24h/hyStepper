/*
 * led.c
 *
 *  Created on: 2023.10.3
 *      Author: johnny
 */

#include "led.h"
#include "cali.h"
#include "motion.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"

#define PERIOD_MS 295

static void init(void)
{
  // PF4:LED+
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

  LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_4);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
}

#define ON  LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_4);
#define OFF LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_4);

static void loop(void)
{
  static uint32_t timer=0;
  static uint8_t  stage=0;

  if (timer && (int32_t)(timer-app_cnt_1ms)>0)
    return;
  timer=app_cnt_1ms+PERIOD_MS;

  if (app_error & APP_ERROR_FATAL) {
    OFF
  } else {
    motion_state_t _motion_state=motion_state; // motion_state maybe changes during comparison
    if (cali_status!=CALI_OK || _motion_state==MOTION_LOST || _motion_state==MOTION_BLOCKED || app_error) {
      stage=(stage+1) & 0x01;
      if (stage)
        ON
      else
        OFF
    } else {
      ON
    }
  }
}

APP_REG_SUBMODULE("led", &init, &loop, NULL, NULL)
