/*
 * i2c.c
 *
 *  Created on: 2023.10.31日
 *      Author: johnny
 */

#include "i2c.h"

#include "air001xx_ll_gpio.h"

static GPIO_TypeDef * port_sda;
static uint32_t       pin_sda;
static GPIO_TypeDef * port_scl;
static uint32_t       pin_scl;
static uint32_t       delay;

#define SDA0        LL_GPIO_ResetOutputPin(port_sda, pin_sda);
#define SDA1        LL_GPIO_SetOutputPin(port_sda, pin_sda);
#define SCL0        LL_GPIO_ResetOutputPin(port_scl, pin_scl);
#define SCL1        LL_GPIO_SetOutputPin(port_scl, pin_scl);

void i2c_conf(GPIO_TypeDef *GPIO_sda, uint32_t Pin_sda, GPIO_TypeDef *GPIO_scl, uint32_t Pin_scl, uint32_t delay_clk)
{
  port_sda=GPIO_sda;
  pin_sda=Pin_sda;
  port_scl=GPIO_scl;
  pin_scl=Pin_scl;
  delay=delay_clk>=12?(delay_clk-12):0; // subtract the clocks it takes to make the call and return
}

void i2c_delay(void)
{
  app_delay_clk(delay);
}

void i2c_ack(void)
{
  SDA0
  i2c_delay();
  SCL1
  i2c_delay();
  SCL0
  i2c_delay();
  SDA1 // release SDA
}

void i2c_nack(void)
{
  SDA1
  i2c_delay();
  SCL1
  i2c_delay();
  SCL0
  i2c_delay();
}

void i2c_start(void)
{
  SDA1
  i2c_delay();
  SCL1
  i2c_delay();
  SDA0
  i2c_delay();
  SCL0
  i2c_delay();
}

void i2c_stop(void)
{
  SDA0
  i2c_delay();
  SCL1
  i2c_delay();
  SDA1
  i2c_delay();
}

void i2c_recv (uint8_t *b, uint32_t len)
{
  if (len) for(;;) {
    for (register uint32_t i=8U; i; i--) {
      i2c_delay();
      SCL1
      i2c_delay();
      *b = (uint8_t)(((*b)<<1)|((port_sda->IDR & pin_sda)?1U:0));
      SCL0
      i2c_delay();
    }
    if (!(--len))
      return;
    b++;
    i2c_ack();
  }
}

uint32_t i2c_send (const uint8_t *b, uint32_t len)
{
  register uint32_t ret=1U;

  for(;ret && len; len--, b++) {
    for (register uint32_t i=0x80; i; i>>=1) {
      if (*b & i) {
        SDA1
      } else {
        SDA0
      }
      i2c_delay();
      SCL1
      i2c_delay();
      SCL0
      i2c_delay();
    }
    i2c_delay();
    SCL1
    i2c_delay();
    if (port_sda->IDR & pin_sda) //NACK
      ret=0U;
    SCL0
    i2c_delay();
  }

  return ret;
}

uint32_t i2c_get(uint8_t addr_w, uint8_t idx, uint8_t *b, uint32_t len)
{
  uint32_t ret=0U;
  i2c_start();
  do {
    if (!i2c_send(&addr_w, 1U))
      break;
    if (!i2c_send(&idx, 1U))
      break;
    i2c_start();
    addr_w|=1U;
    if (!i2c_send(&addr_w, 1U))
      break;
    i2c_recv(b, len);
    i2c_nack();
    ret=1U;
  } while(0);
  i2c_stop();
  return ret;
}

// use I2C write address and register idx to set bytes
uint32_t i2c_set(uint8_t addr_w, uint8_t idx, const uint8_t *b, uint32_t len)
{
  uint32_t ret=0;
  i2c_start();
  do {
    if (!i2c_send(&addr_w, 1U))
      break;
    if (!i2c_send(&idx, 1U))
      break;
     if (!i2c_send(b, len))
      break;
    ret=1U;
  } while(0);
  i2c_stop();
  return ret;
}
