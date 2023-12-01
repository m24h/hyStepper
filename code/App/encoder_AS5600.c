/*
 * encoder_AS5600.c
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#include "encoder.h"
#include "i2c.h"
#include "driver.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"

#ifdef ENCODER_AS5600

#error not tested, AS5600 is too slow for motor control, make sure you can use this

#define ADDR_W 0x6C /*slave address of the AS5600 is 0x36*/

static volatile uint32_t status_data;

APP_REG_ERROR(static, encoder_err, "Encoder")

static void init(void)
{
  encoder_delay_ticks=((APP_CLK_SYS>>10)*315)>>10; // // AS5600 is very slow, nearly 300us
  encoder_status=ENCODER_NONE;

  // PA4:DIR PA2:I2C_SDA(SOFT) PA5:I2C_SCL(SOFT)
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5); // CW (face to back of chip) is positive direction
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

  // delay a while before AS5600 is stable
  uint32_t t=app_cnt_1ms+50;
  while((int32_t)(t-app_cnt_1ms)>0);

  // config i2c
  i2c_conf(GPIOA, LL_GPIO_PIN_2, GPIOA, LL_GPIO_PIN_5, APP_CLK_SYS/800000/3); // nearly 800kHz

  app_irq_disable();
  // try to correct last interrupted operation
  i2c_start();
  i2c_delay();
  i2c_stop();
  if (!i2c_set(ADDR_W, 0x07, (const uint8_t*)"\x07\0", 2)) // for highest speed, address 0x07:CONF -> FTH=1 SF=3
    app_error|=encoder_err;
  app_irq_resume();

  encoder_status=ENCODER_NONE;
  status_data=0;

  if (app_error & encoder_err)
    app_printf("AS5600: Configuration failed\r\n");
}

int32_t encoder_get(void)
{
  uint8_t buff[2];
  static int32_t last_value=0;

#if APP_CLK_APB1/(DRIVER_PWM_REPT<<DRIVER_PWM_BITS)>10000
#error driver frequency is too high, AS5600 need more time to get data
#endif

  app_irq_disable();
  if (i2c_get(ADDR_W, 0x0C, buff, 2U)) { // address 0x0C : raw angle
    last_value=(int32_t)((((uint32_t)buff[0]<<8) | (uint32_t)buff[1])<<20);
    app_error&=~encoder_err;
  } else {
    app_error|=encoder_err;
    encoder_status=ENCODER_NONE;
  }
  app_irq_resume();

  return last_value;
}

static void loop(void)
{
  static uint32_t timer=0;
  if (timer && (int32_t)(app_cnt_1ms-timer)<0)
    return;
  timer=app_cnt_1ms+169;

  uint8_t buff[4];
  app_irq_disable();
  if (i2c_get(ADDR_W, 0x0B, buff, 1) // address 0x0B : status
   && i2c_get(ADDR_W, 0x1A, buff+1, 3)) { // address 0x1A:AGC 0x1B~0x1C:MAG
    app_error&=~encoder_err;
    if (!(buff[0] & 0x20)) {
      encoder_status=ENCODER_LOST;
    } else {
      if (buff[0] & 0x10)
        encoder_status=ENCODER_WEAK;
      else if (buff[0] & 0x08)
        encoder_status=ENCODER_STRONG;
      else
        encoder_status=ENCODER_MODERATE;
    }
    status_data=((uint32_t)buff[1]<<16)|((uint32_t)buff[2]<<8)|((uint32_t)buff[3]);
  } else {
    encoder_status=ENCODER_NONE;
    status_data=0;
    app_error|=encoder_err;
  }
  app_irq_resume();
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  (void)cmdstr;
  app_resp_printf(resp, "AS5600: %s, POS=0x%08lX, MAG=0x%lu, AGC=%lu\r\n", encoder_status_str(), encoder_get(),
            status_data&0xFFFU, (status_data>>16)&0xFFU);
}

APP_REG_SUBMODULE("enc", &init, &loop, &cmd, "motor encoder AS5600")

#endif
