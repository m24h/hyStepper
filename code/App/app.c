/*
 * app.c
 *
 *  Created on: 2023.9.24
 *      Author: johnny
 */
#include "app.h"

#include "air001xx_ll_system.h"
#include "air001xx_ll_rcc.h"
#include "air001xx_ll_pwr.h"
#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"
#include "air001xx_ll_tim.h"
#include "air001xx_ll_exti.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>

#define SHELL_LINE_SIZE 127

static        char    uid_store[12];
const char * const    app_uid=uid_store;

volatile uint32_t     app_error;
volatile uint32_t     app_mem_free;
volatile uint32_t     app_cnt_1ms;
volatile uint32_t     app_loops_1ms;
volatile uint32_t     app_irq_disabled;

static volatile uint32_t loop_cnt;
static volatile uint64_t clk_cnt;

volatile app_shell_sig_t app_shell_sig;

// registered init functions
extern void (* const app_init_start)(void);
extern void (* const app_init_end)(void);

// registered submodules
extern const app_submodule_t app_submodule_start;
extern const app_submodule_t app_submodule_end;

// registered error
extern const char * const app_error_start;
extern const char * const app_error_end;

// if linked with option -nostartfiles -nostdlib
void __attribute__((weak)) _init(void)
{
}

// default _sbrk() will intrude the stack greedily, this one won't
void * _sbrk(ptrdiff_t incr)
{
  extern uint8_t _end; // in app.ld
  extern uint8_t _estack; // in app.ld
  extern uint32_t _Min_Stack_Size; // in app.ld
  const  uint8_t * heap_end_max = & _estack - (uint32_t)&_Min_Stack_Size;
  static uint8_t * heap_end = &_end;

  register uint8_t * old_end=heap_end;
  heap_end+=incr;
  register int32_t mf=heap_end_max-heap_end;
  if (mf>0) {
    app_mem_free=(uint32_t)mf;
    return (void*)old_end;
  }

  // no mem
  heap_end=old_end;
  errno = ENOMEM;
  return (void *)-1;
}


// for all uncatched, unexpected IRQ
void __attribute((__noreturn__, interrupt("IRQ"))) Default_Handler(void)
{
  app_printf("App: Unexpected IRQ\r\n");

  // don't known how to clear intr. flag, so stop all
  __disable_irq();
  app_error|=APP_ERROR_FATAL;
  // send break to all timer
  LL_TIM_GenerateEvent_BRK(TIM1);
  LL_TIM_GenerateEvent_BRK(TIM16);
  LL_TIM_GenerateEvent_BRK(TIM17);
  // play dead
  for(;;);
}

void __attribute((interrupt("IRQ"))) PVD_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_16)) {
    LL_EXTI_ClearFlag(LL_EXTI_LINE_16);
    if (LL_PWR_IsActiveFlag_PVDO()) {
      __disable_irq();
      app_error|=APP_ERROR_PVD;
      __enable_irq();
      app_printf("App: Power down\r\n");
    } else {
#ifndef CONF_PVD_NO_UP
      __disable_irq();
      app_error&=~APP_ERROR_PVD;
      __enable_irq();
      app_printf("App: Power OK\r\n");
#endif
    }
  }
}

// for systick IRQ
void __attribute((interrupt("IRQ"))) SysTick_Handler(void)
{
  // following is atomical, because systick got the highest priority
  ++app_cnt_1ms;
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)  // also clear COUNTFLAG, indicate that clk_cnt is the newest value
      clk_cnt+=APP_CLK_SYS/1000U;
  clk_cnt+=APP_CLK_SYS/1000U;
  app_loops_1ms=loop_cnt;
  loop_cnt=0;
}

uint64_t app_cnt_clk(void)
{
  uint32_t val=(APP_CLK_SYS/1000U-1)-(SysTick->VAL & SysTick_VAL_CURRENT_Msk);
  if (val<(SysTick_VAL_CURRENT_Msk>>1) && (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) // COUNTFLAG is actomical
    clk_cnt+=APP_CLK_SYS/1000U;
  return clk_cnt+val;
}

static void init_system_clock(void)
{
  // set flash SetLatency to 1 for 48MHz running
#if APP_CLK_SYS>24000000U
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) ;
#endif

  // enable and set HSI to 24MHz with calibration
  LL_RCC_HSI_Enable();
#if APP_CLK_HSI==24000000U
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
#elif APP_CLK_HSI==22120000U
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_22p12MHz);
#elif APP_CLK_HSI==16000000U
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_16MHz);
#elif APP_CLK_HSI==8000000U
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_8MHz);
#elif APP_CLK_HSI==4000000U
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_4MHz);
#else
#error impossible HSI frequency
#endif
  while(!LL_RCC_HSI_IsReady());

  // set AHB clock divider
#if APP_CLK_AHB==APP_CLK_SYS
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
#elif APP_CLK_AHB==APP_CLK_SYS/2
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
#elif APP_CLK_AHB==APP_CLK_SYS/4
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_4);
#elif APP_CLK_AHB==APP_CLK_SYS/8
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_8);
#elif APP_CLK_AHB==APP_CLK_SYS/16
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_16);
#elif APP_CLK_AHB==APP_CLK_SYS/64
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_64);
#elif APP_CLK_AHB==APP_CLK_SYS/128
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_128);
#elif APP_CLK_AHB==APP_CLK_SYS/256
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_256);
#elif APP_CLK_AHB==APP_CLK_SYS/512
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_512);
#else
#error impossible AHB frequency
#endif

  // set APB1 clock divider
#if APP_CLK_APB1==APP_CLK_AHB
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
#elif APP_CLK_APB1==APP_CLK_AHB/2
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
#elif APP_CLK_APB1==APP_CLK_AHB/4
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
#elif APP_CLK_APB1==APP_CLK_AHB/8
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_8);
#elif APP_CLK_APB1==APP_CLK_AHB/16
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_16);
#else
#error impossible APB1 frequency
#endif

  // using PLL or HSISYS as sys clock
#if APP_CLK_SYS==APP_CLK_HSI*2
  // enable PLL using HSI as source
  LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1U);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
#elif APP_CLK_SYS==APP_CLK_HSI
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);
#else
#error impossible system clock frequency
#endif

  // prepare variable SystemCoreClock for HAL and some LL, but SystemCoreClock is not recommended, this App must using 48MHz APP_CLK_SYS
  SystemCoreClock=APP_CLK_SYS;
}

static void init_PVD(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_3V0); // 3.0V/2.9V
  LL_PWR_SetPVDSource(LL_PWR_PVD_SOURCE_VCC);
  LL_PWR_SetPVDFilter(LL_PWR_PVD_FILTER_1024CLOCK);
  LL_RCC_SetPVDClockSource(LL_RCC_PVD_CLKSOURCE_PCLK1);
  LL_PWR_EnablePVDFilter();

  LL_EXTI_ClearFlag(LL_EXTI_LINE_16);
  LL_EXTI_EnableRisingTrig(LL_EXTI_LINE_16);
  LL_EXTI_EnableFallingTrig(LL_EXTI_LINE_16);
  LL_EXTI_EnableIT(LL_EXTI_LINE_16);

  LL_PWR_EnablePVD();

  NVIC_SetPriority(PVD_IRQn, 0);
  NVIC_EnableIRQ(PVD_IRQn);
}

static void init_SWD(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DBGMCU);
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_WWDG_STOP|LL_DBGMCU_APB1_GRP1_IWDG_STOP);

  // PA13:SW_DIO PA14:SW_CLK
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_13, LL_GPIO_AF0_SWJ);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_14, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_14, LL_GPIO_AF0_SWJ);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
}

static void init_systick(void)
{
  clk_cnt=0;
  SysTick_Config(APP_CLK_SYS/1000U); // Setup Systick with the lowest priority
  NVIC_SetPriority(SysTick_IRQn, 0); // now set it to the highest priority, time base is important, and the IRQ handler is so short
  NVIC_EnableIRQ(SysTick_IRQn);
}

void __attribute__((__noreturn__)) app_reset(void)
{
  __disable_irq();

  // reset all peripherals
  WRITE_REG(RCC->APBRSTR2, 0xFFFFFFFF);
  WRITE_REG(RCC->APBRSTR1, 0xFFFFFFFF);
  WRITE_REG(RCC->AHBRSTR, 0xFFFFFFFF);
  // make sure BOOT0=0
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_4);
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  // delay for boot0 pin OK
  app_delay_clk(APP_CLK_SYS/100);
  // reset, never return
  NVIC_SystemReset();
  for(;;);
}

// delay clocks
void app_delay_clk(uint32_t delay)
{
  __ASM (
    ".syntax unified \n"
    ".thumb \n"
    "   LSRS %0, #2 \n"
    "   beq L2%= \n"
    "L1%=: \n"
    "   subs %0, #1 \n"
    "   bne L1%= \n"
    "L2%=: \n"
    : : "r"(delay) : "cc"
  );
}

// maybe some module really implement it
int __attribute__((weak)) app_printf (const char * fmt, ...)
{
  (void)fmt;
  return 0;
}

int32_t app_sscan_int(char **str, int32_t def, int32_t min, int32_t max)
{
  char *s=strtok_r(*str, " \t\r\n", str);
  if (!s || !*s)
    return def;

  int32_t ret=(int32_t)atol(s);
  if (ret<min || ret>max)
    return def;

  return ret;
}

// skip spaces (including " " "\t" "\r" "\n"), then return 1 if the "chk_str" exists and is skipped
uint32_t app_sscan_chk(char **str, const char * chk_str)
{
  char *p=*str;
  while (*p==' ' || *p=='\t' || *p=='\r' || *p=='\n')
    p++;

  size_t len=strlen(chk_str);
  if (strncasecmp(p, chk_str, len))
    return 0U;

  *str=p+len;
  return 1U;
}

void app_shell(char c, app_cmd_res_t resp)
{
  static char shell_line[SHELL_LINE_SIZE+1]; // leave a place for '\0'
  static uint32_t       tail=0;
  static uint32_t       cursor=0;
  static uint32_t       crlf=0;
  static uint32_t       esc=0;

  if (esc==1) {
    if (c=='[')
      esc=2;
    else
      esc=0;
  } else if (esc) {
    if (c>='@' && c<='~') { // end character according to CSI rule
      if (c=='C') { // cursor forward
        if (cursor<tail) {
          ++cursor;
          (*resp)("\x1B[C", 3); // echo back
        }
      } else if (c=='D') { // cursor back, now it does not support multi-line
        if (cursor>0) {
          --cursor;
          (*resp)("\x1B[D", 3); // echo back
        }
      } else if (c=='~') { // delete
        if (tail>cursor) {
          esc=0;
          goto SHELL_DELETE_CHAR;
        }
      }
      esc=0;
    }
  } else if (c=='\x08' || c=='\x7F') { // backspace
    crlf=0;
    if (cursor>0) {
      --cursor;
      (*resp)("\x1B[D", 3); // echo back
SHELL_DELETE_CHAR:
      --tail;
      for (uint32_t i=cursor; i<tail; i++) {
        shell_line[i]=shell_line[i+1];
      }
      (*resp)("\x1B[s\x1B[J", 6U); // push cursor position, clean display to line end
      (*resp)(shell_line+cursor, tail-cursor); // re-display
      (*resp)("\x1B[u", 3U); // pop cursor position
    }
  } else if (c=='\x1B') { // ESC
      esc=1;
      crlf=0;
  } else if (c=='\x03') { // ^C
    app_shell_sig=APP_SIG_INT;
  } else if (c=='\r' || c=='\n') { // a line is gotten
    if (tail>0 || !crlf || crlf==(uint32_t)c) {
      (*resp)("\r\n", 2U); // echo back
      shell_line[tail]='\0';
      crlf=c;
      app_shell_sig=APP_SIG_NONE;
      app_cmd(shell_line, resp);
    }
    tail=0;
    cursor=0;
  } else if ((c>=' ' && c<='~') || c=='\t') { // printable
    crlf=0;
    if (cursor<SHELL_LINE_SIZE) {
      (*resp)(&c, 1U); // echo back
      shell_line[cursor++]=c;
      if (tail<cursor)
        tail=cursor;
    }
  }
}

void app_resp_printf(app_cmd_res_t resp, const char * fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  char *buff=NULL;
  int len=vasiprintf(&buff, fmt, va);
  va_end(va);
  if (len>0)
    (*resp)(buff, (uint32_t)len);
  free(buff);
}

void app_cmd(char *cmd, app_cmd_res_t resp)
{
  char *s=strtok_r(cmd, " \t\r\n", &cmd);

  if (strcasecmp("uid", s)==0)
    app_resp_printf(resp, "UID: %s\r\n", app_uid);
  else if (strcasecmp("reset", s)==0)
    app_reset();
  else if (strcasecmp("err", s)==0) {
    app_resp_printf(resp, "Err: 0x%08lX", app_error);
    if (app_error & APP_ERROR_FATAL)
      app_resp_printf(resp, " Fatal");
    if (app_error & APP_ERROR_PVD)
      app_resp_printf(resp, " PowerLow");
    if (app_error & APP_ERROR_INIT)
      app_resp_printf(resp, " Init");
    if (app_error & APP_ERROR_SOME)
      app_resp_printf(resp, " Some");
    uint32_t bit=1U;
    for (const char * const *p=&app_error_start; p<&app_error_end; p++, bit<<=1) {
      if ((app_error & bit) && *p) {
        app_resp_printf(resp, " %s", *p);
      }
    }
    app_resp_printf(resp, "\r\n");
  }
  else if (strcasecmp("free", s)==0)
    app_resp_printf(resp, "Free: MEM=%lu, LOOP_MS=%lu\r\n", app_mem_free, app_loops_1ms);
  else {
    const app_submodule_t * p;
    for (p=&app_submodule_start; p<&app_submodule_end; p++) {
      if (p->name && p->cmd && strcasecmp(p->name, s)==0) {
        (*p->cmd)(cmd, resp);
        return;
      }
    }

    app_resp_printf(resp,
        "-------- help ----------\r\n"
        "uid - show device UID\r\n"
        "free - show free resources\r\n"
        "err - show error status\r\n"
        "reset - reset the system\r\n"
    );
    for (p=&app_submodule_start; p<&app_submodule_end; p++) {
      if (p->name && p->cmd && p->help)
        app_resp_printf(resp, "%s - %s\r\n", p->name, p->help);
    }
  }
}

int __attribute__((__noreturn__)) main(void)
{
  init_system_clock();

  uid_store[0]='H';
  uid_store[1]='S';
  uid_store[2]=((const char *)UID_BASE)[0];
  uid_store[3]=((const char *)UID_BASE)[1];
  uid_store[4]=((const char *)UID_BASE)[2];
  uid_store[5]=((const char *)UID_BASE)[3];
  uid_store[6]='A'+(((uint8_t *)UID_BASE)[4]>>4);
  uid_store[7]='A'+(((uint8_t *)UID_BASE)[4]&0x0F);
  uid_store[8]=((const char *)UID_BASE)[5];
  uid_store[9]=((const char *)UID_BASE)[6];
  uid_store[10]=((const char *)UID_BASE)[7];
  uid_store[11]='\0';

  app_error=APP_ERROR_INIT;
  app_irq_disabled=0;
  app_cnt_1ms=0;
  app_loops_1ms=0;
  loop_cnt=0;
  app_shell_sig=APP_SIG_NONE;
  _sbrk(0); // get app_mem_free

  // important system modules
  init_SWD();
  init_PVD();
  init_systick();

  // call init functions
  for (void (* const *p)(void)=&app_init_start; p<&app_init_end; p++) {
    if (*p)
      (**p)();
  }

  // init submodules
  const app_submodule_t *p;
  for (p=&app_submodule_start; p<&app_submodule_end; p++) {
    if (p->init)
      (*(p->init))();
  }

  app_printf("App: Ready\r\n");

  __disable_irq();
  app_error&=~APP_ERROR_INIT;
  app_irq_disabled=0;
  __enable_irq();

  // main loop
  for(;;) {
    for (p=&app_submodule_start; p<&app_submodule_end; p++) {
      if (p->loop) {
        (*p->loop)();
        __disable_irq();
        app_irq_disabled=0;
        __enable_irq(); // keep IRQ enabled
      }
    }
    loop_cnt++;
  }
}



