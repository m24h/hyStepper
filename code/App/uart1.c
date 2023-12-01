/*
 * uart1.c
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#include "uart1.h"

#include "air001xx_ll_system.h"
#include "air001xx_ll_bus.h"
#include "air001xx_ll_usart.h"
#include "air001xx_ll_dma.h"
#include "air001xx_ll_gpio.h"

#include <stdio.h>

#define RX_BUFF_SIZE     128U
static char      rx_buff [RX_BUFF_SIZE];

static void send(const char *str, uint32_t len)
{
  while (len-->0 && LL_USART_IsEnabled(USART1)) {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, (uint8_t)*(str++));
  }
}

static void init (void)
{
  // USART1
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_USART1);
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_USART1);

  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
  LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_16);
  LL_USART_SetBaudRate(USART1, APP_CLK_APB1, LL_USART_OVERSAMPLING_16, CONF_UART1_BAUDRATE);
  LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
  LL_USART_EnableDMAReq_RX(USART1);

  // DMA for RX (CH1)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
                      LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                      LL_DMA_MODE_CIRCULAR |
                      LL_DMA_PERIPH_NOINCREMENT  |
                      LL_DMA_MEMORY_INCREMENT  |
                      LL_DMA_PDATAALIGN_BYTE |
                      LL_DMA_MDATAALIGN_BYTE |
                      LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)rx_buff);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&(USART1->DR));
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);

  // DMA remap
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetDMARemap_CH3(LL_SYSCFG_DMA_MAP_USART1_RX);

  // PF0:USART1_RX PF1:USART1_TX
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

  LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOF, LL_GPIO_PIN_0, LL_GPIO_AF8_USART1);
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOF, LL_GPIO_PIN_1, LL_GPIO_AF8_USART1);
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);

  // start listening
  LL_USART_Enable(USART1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RX_BUFF_SIZE);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

// support printf ... in stdio.h
#ifndef __GNUC__
#error following code is for gcc only, otherwise, it is better to overwrite "int fputc(int ch, FILE *f)"
#endif
int _write (int fd, char *pstr, int size)
{
  (void)fd;
  send(pstr, (uint32_t)size);
  return size;
}

int uart1_vprintf (const char * fmt, va_list va)
{
  char *buff=NULL;
  int len=vasiprintf(&buff, fmt, va);
  send(buff, (uint32_t)len);
  free(buff);
  return len;
}

int uart1_printf (const char * fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  int len=uart1_vprintf(fmt, va);
  va_end(va);
  return len;
}

// overwrite weak app_printf()
int app_printf (const char * fmt, ...) __attribute__ ((alias ("uart1_printf")));

static void loop(void)
{
  static uint32_t  rx_head=0;

  while(rx_head!=RX_BUFF_SIZE-LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3)) {
    app_shell(rx_buff[rx_head], &send);
    if ((++rx_head)>=RX_BUFF_SIZE)
      rx_head=0;
  }
}

APP_REG_INIT(&init) // before other modules, so they can use printf during initialization
APP_REG_SUBMODULE("com", NULL, &loop, NULL, "UART1")
