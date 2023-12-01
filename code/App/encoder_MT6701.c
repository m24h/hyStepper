/*
 * encoder_MT6701.c
 *
 * the gap between MT6701 and magnet is important, and it's tolerance is very poor,
 * only a range of 0.3mm is useful for a high accuracy need
 *
 * MT6701 seems to have a bad compensation for dynamic angle error compensation,
 * nearly 0.1~0.3 deg, but if the sampling period becomes bigger, it becomes bigger too.
 * and the DC response has an bad offset
 *
 * 4LSB Hysteresis (HYST=2) setting can give a good DC response
 * 2LSB Hysteresis (HYST=1) setting can give a good dynamic response
 *
 *  Created on: 2023.9.30
 *      Author: johnny
 */

#include "encoder.h"
#include "i2c.h"
#include "flash.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"

#ifdef ENCODER_MT6701

//#define ENCODER_TEST

#define ADDR_W      0x0C /*slave address of the MT6701 is 0x06*/
#define HYST        2    /* 2LSB */

static volatile uint32_t status_data;

APP_REG_ERROR(static, encoder_err, "Encoder")

typedef struct {
  int32_t hyst;   // stepping electric current, 1000 means CONF_CURRENT_MAX
} params_t;

static const params_t params_def={
  .hyst=1, // 2LSB, balanced between static and dynamic performance
};

const char * tab_hyst_str [] = {"1LSB","2LSB","4LSB","8LSB","0LSB","0.25LSB","0.5LSB","1LSB"};

// there's 5us delayed according to MT6701 datasheet
// and 4400ns added for function encoder_get() itself
// but in fact, variable values above 300us are measured in dynamic scene
// I didn't do a rigorous test, only part of this (2LSB/4LSB) is empirical data and the rest is guesswork
const uint32_t tab_delay_ticks [] = {
    APP_CLK_SYS/1000*220/1000,
    APP_CLK_SYS/1000*250/1000,
    APP_CLK_SYS/1000*300/1000,
    APP_CLK_SYS/1000*350/1000,
    APP_CLK_SYS/1000*180/1000,
    APP_CLK_SYS/1000*180/1000,
    APP_CLK_SYS/1000*200/1000,
    APP_CLK_SYS/1000*220/1000
};

static const volatile params_t __attribute__((used,__aligned__(4),section(".foc_param"))) params_flash=params_def;

static __attribute__((__aligned__(4))) params_t params; // a used version of params_flash, no need to be volatile, for IRQ handler, it will not be optimized

static void trans_params(void)
{
  uint32_t err=1U;
  uint8_t b;

  app_irq_disable();
  do {
    // try to correct interrupted operation
    i2c_start();
    i2c_delay();
    i2c_stop();
    i2c_delay();
    if (!i2c_get(ADDR_W, 0x29, &b, 1U))  // address 0x29/bit1 : DIR
      break;
    i2c_delay();
    b &= 0xFD; // CCW is positive
    if (!i2c_set(ADDR_W, 0x29, &b, 1))
      break;
    i2c_delay();
    if (!i2c_get(ADDR_W, 0x32, &b, 1))  // address 0x32/bit7 : HYST[2]
      break;
    i2c_delay();
    b = (uint8_t)((b & 0x7F) | ((params.hyst & 0x04)<<5));
    if (!i2c_set(ADDR_W, 0x32, &b, 1))
      break;
    i2c_delay();
    if (!i2c_get(ADDR_W, 0x34, &b, 1))  // address 0x34/bit6-7 : HYST[0-1]
      break;
    i2c_delay();
    b = (uint8_t)((b & 0x3F) | ((params.hyst & 0x03)<<6));
    if (!i2c_set(ADDR_W, 0x34, &b, 1))
      break;
    err=0U;
  } while(0);
  i2c_stop();
  if (err)
    app_error|=encoder_err;
  encoder_delay_ticks=tab_delay_ticks[params.hyst&0x07];
  app_irq_resume();

  if (err)
    app_printf("MT6701: Configuration failed\r\n");
}

static void init(void)
{
  memcpy(&params, (const void*)&params_flash, sizeof(params_t));

  encoder_status=ENCODER_NONE;

  // PA4:SSI_CSN PA2:SSI_DO/I2C_SDA PA5:SSI_SCL/I2C_SCL --- encoder
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4); // default CSN : 1, for I2C
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

  // for both I2C and SSI, so OPENDRAIN and PULL_UP
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL); // MT6701 does not drive CLK
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

  // delay a while before MT6701 is stable, according to datasheet, 32ms
  uint32_t t=app_cnt_1ms+50;
  while((int32_t)(t-app_cnt_1ms)>0);

  // config i2c
  i2c_conf(GPIOA, LL_GPIO_PIN_2, GPIOA, LL_GPIO_PIN_5, APP_CLK_SYS/500000/3); // nearly 500kHz

  trans_params();

  status_data=0;
}

#define CSN0        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
#define CSN1        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
#define SCL0        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
#define SCL1        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);

int32_t encoder_get(void)
{
  register uint32_t t=0;

  app_irq_disable();
  CSN1
  SCL1
  __NOP(); __NOP();  __NOP(); // 60ns
  CSN0
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); // 165ns, tested, should not be less this
  SCL0
  __NOP(); __NOP();  __NOP(); __NOP(); // 80ns
  SCL1
  __NOP(); __NOP(); __NOP(); // 60ns, tested, should not be less this
  for (register uint32_t i=23; i; i--) { // at least 60ns at the tail of each cycle
    SCL0                 // 20ns
    t=(t<<1)|(GPIOA->IDR & LL_GPIO_PIN_2);   // at least 80ns
    SCL1                 // 20ns
  }
  t=(t<<1)|(GPIOA->IDR & LL_GPIO_PIN_2);   // at least 80ns
  CSN1;
  app_irq_resume();

  status_data=t;
  return (int32_t)((t<<6)&0xFFFC0000); // (GPIOA->IDR & LL_GPIO_PIN_2) is originally 2 bits left-shifted
}

static void loop(void)
{
  static  uint32_t timer=0;
  if (timer && (int32_t)(app_cnt_1ms-timer)<0)
    return;
  timer=app_cnt_1ms+163;

  encoder_get();
  switch((status_data>>8)&3) { // (GPIOA->IDR & LL_GPIO_PIN_2) is originally 2 bits left-shifted
  case 1:
    encoder_status=ENCODER_STRONG;
    break;
  case 2:
    encoder_status=ENCODER_WEAK;
    break;
  default:
    encoder_status=ENCODER_MODERATE;
  }
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);
  if (!str || !*str)
    app_resp_printf(resp, "MT6701: %s, POS=0x%08lX, MAG=0x%lX, HYST=%ld (%s)\r\n",
        encoder_status_str(), encoder_get(), (status_data>>6)&15,
        params.hyst, tab_hyst_str[params.hyst & 0x07]);
  else if (strcasecmp("hyst", str)==0) {
    params.hyst=app_sscan_int(&cmdstr, params.hyst, 0, 7);
    trans_params(); // put it into effect
    app_resp_printf(resp, "MT6701: HYST=%ld (%s)\r\n", params.hyst, tab_hyst_str[params.hyst & 0x07]);
  }
  else if (strcasecmp("save", str)==0) {
    if (flash_write((void*)&params_flash, &params, sizeof(params_t))) {
      app_resp_printf(resp, "MT6701: Configuration is saved to FLASH\r\n");
    } else
      app_resp_printf(resp, "MT6701: Failed to write FLASH\r\n");
  }
  else if (strcasecmp("test", str)==0)
    encoder_test("MT6701", 20480U, resp);
  else
    app_resp_printf(resp, "MT6701: <hyst [0-7]> <save> <test>\r\n");
}

APP_REG_SUBMODULE("enc", &init, &loop, &cmd, "motor encoder MT6701")

#endif
