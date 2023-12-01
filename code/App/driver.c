/*
 * driver.c
 *
 * compatible chips: A4950, TB67H450FNG
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#include "driver.h"
#include "motion.h"
#include "cali.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_gpio.h"
#include "air001xx_ll_system.h"
#include "air001xx_ll_tim.h"

volatile driver_updater_t driver_updater;

static volatile uint8_t is_working; // maybe changed in a function called by IRQ handler

static void tim_init(TIM_TypeDef *TIMx)
{
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP); // just for convention safe
  LL_TIM_SetPrescaler(TIMx, 0);
  LL_TIM_SetClockDivision(TIMx, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_EnableARRPreload(TIMx);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_ConfigOutput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_SetOffStates(TIMx, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
  LL_TIM_SetAutoReload(TIMx, (1<<DRIVER_PWM_BITS)-2); // not use whole cycle
  LL_TIM_SetCounter(TIMx, (1<<DRIVER_PWM_BITS)-2);
  LL_TIM_SetRepetitionCounter(TIMx, DRIVER_PWM_REPT-1);
  LL_TIM_OC_SetCompareCH1(TIMx, 0);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIMx);
  LL_TIM_EnableCounter(TIMx);
  LL_TIM_GenerateEvent_UPDATE(TIMx);
}

static void init(void)
{
  driver_updater=NULL;

  // init timer
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16 | LL_APB1_GRP2_PERIPH_TIM17);
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_TIM16 | LL_APB1_GRP2_PERIPH_TIM17);
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_TIM16 | LL_APB1_GRP2_PERIPH_TIM17);

  tim_init(TIM16);
  tim_init(TIM17);

  // enable braking
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_EnableTIMBreakInputs(LL_SYSCFG_TIMBREAK_LOCKUP_TO_ALL | LL_SYSCFG_TIMBREAK_PVD_TO_ALL);

  // enable TIM16 update intr, high priority
  LL_TIM_EnableIT_UPDATE(TIM16);
  NVIC_SetPriority(TIM16_IRQn, 1);
  NVIC_EnableIRQ(TIM16_IRQn);

  // PA6:A_current/TIM16_CH1 PA7:B_current/TIM17_CH1 --- PWM Curren
  // PB0:A+ PA1:A- PB1:B+ PB2:B-
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA|LL_IOP_GRP1_PERIPH_GPIOB);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF5_TIM16);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF5_TIM17);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_MEDIUM);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_MEDIUM);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_MEDIUM);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_MEDIUM);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);

  is_working=0;
}

void __attribute((interrupt("IRQ"))) TIM16_IRQHandler (void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM16)) {
    LL_TIM_ClearFlag_UPDATE(TIM16);
    if (driver_updater)
      (*driver_updater)(DRIVER_UPDATE);
  }
}

static void loop(void)
{
  if (app_error) {
    if (driver_updater)
      driver_occupy(NULL);

    if (is_working)
      driver_disable();
  }
}

#define AP0 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
#define AP1 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
#define AN0 LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
#define AN1 LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
#define BP0 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
#define BP1 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
#define BN0 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
#define BN1 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);

void driver_disable(void)
{
  AP0
  AN0
  BP0
  BN0
  LL_TIM_OC_SetCompareCH1(TIM16, 0);
  LL_TIM_OC_SetCompareCH1(TIM17, 0);
  is_working=0;
}

// a table of sin(ang/512*pi/2)*65535, ang=0...512, that's enough for 9-bit PWM (error<0.1deg), defined at the tail of this file
static const uint16_t tab_sin[513];

void driver_drive(int32_t angle, uint32_t current)
{
  if (app_error)
    return;

  is_working=1;

  uint32_t cur=current>>16; // 0-65535
  register uint32_t ang=((uint32_t)angle+(1U<<20))>>21; // 0-2047 for 0-360deg
  uint32_t cos=(tab_sin[512-(ang & 511U)]*cur)>>(32-DRIVER_PWM_BITS);
  uint32_t sin=(tab_sin[ang & 511U]*cur)>>(32-DRIVER_PWM_BITS);
  if (ang & 512U) {
    LL_TIM_OC_SetCompareCH1(TIM16, sin);
    LL_TIM_OC_SetCompareCH1(TIM17, cos);
    if (ang & 1024) {
      AP1 BN1 AN0 BP0 // 1 before 0, helpful for keeping slow decay when changing phases
    } else {
      AN1 BP1 AP0 BN0
    }
  } else {
    LL_TIM_OC_SetCompareCH1(TIM16, cos);
    LL_TIM_OC_SetCompareCH1(TIM17, sin);
    if (ang & 1024) {
      AN1 BN1 AP0 BP0
    } else {
      AP1 BP1 AN0 BN0
    }
  }
}

void driver_occupy(driver_updater_t updater)
{
  if (driver_updater) {
    app_irq_disable();
    (*driver_updater)(DRIVER_EXIT);
    app_irq_resume();
  }

  if (updater)
    (*updater)(DRIVER_INIT);
  else
    driver_disable();
  driver_updater=updater;
}

void driver_release(driver_updater_t updater)
{
  if (!driver_updater)
    return;

  app_irq_disable();
  if (driver_updater==updater || updater==NULL) {
    (*driver_updater)(DRIVER_EXIT);
    driver_updater=NULL;
    driver_disable();
  }
  app_irq_resume();
}

APP_REG_SUBMODULE("drv", &init, &loop, NULL, "motor driver")

static const uint16_t tab_sin[513]={
    0,
    201,
    402,
    603,
    804,
    1005,
    1206,
    1407,
    1608,
    1809,
    2010,
    2211,
    2412,
    2613,
    2814,
    3015,
    3216,
    3416,
    3617,
    3818,
    4019,
    4219,
    4420,
    4621,
    4821,
    5022,
    5222,
    5422,
    5623,
    5823,
    6023,
    6223,
    6424,
    6624,
    6824,
    7024,
    7223,
    7423,
    7623,
    7823,
    8022,
    8222,
    8421,
    8620,
    8820,
    9019,
    9218,
    9417,
    9616,
    9815,
    10014,
    10212,
    10411,
    10609,
    10808,
    11006,
    11204,
    11402,
    11600,
    11798,
    11996,
    12193,
    12391,
    12588,
    12785,
    12982,
    13179,
    13376,
    13573,
    13770,
    13966,
    14163,
    14359,
    14555,
    14751,
    14947,
    15142,
    15338,
    15533,
    15729,
    15924,
    16119,
    16313,
    16508,
    16703,
    16897,
    17091,
    17285,
    17479,
    17673,
    17866,
    18060,
    18253,
    18446,
    18639,
    18831,
    19024,
    19216,
    19408,
    19600,
    19792,
    19984,
    20175,
    20366,
    20557,
    20748,
    20939,
    21129,
    21319,
    21509,
    21699,
    21889,
    22078,
    22267,
    22456,
    22645,
    22834,
    23022,
    23210,
    23398,
    23586,
    23773,
    23960,
    24147,
    24334,
    24521,
    24707,
    24893,
    25079,
    25265,
    25450,
    25635,
    25820,
    26005,
    26189,
    26374,
    26557,
    26741,
    26925,
    27108,
    27291,
    27473,
    27656,
    27838,
    28020,
    28201,
    28383,
    28564,
    28745,
    28925,
    29106,
    29286,
    29465,
    29645,
    29824,
    30003,
    30181,
    30360,
    30538,
    30716,
    30893,
    31070,
    31247,
    31424,
    31600,
    31776,
    31952,
    32127,
    32302,
    32477,
    32651,
    32826,
    32999,
    33173,
    33346,
    33519,
    33692,
    33864,
    34036,
    34208,
    34379,
    34550,
    34721,
    34891,
    35061,
    35231,
    35400,
    35569,
    35738,
    35906,
    36074,
    36242,
    36409,
    36576,
    36743,
    36909,
    37075,
    37241,
    37406,
    37571,
    37736,
    37900,
    38064,
    38227,
    38390,
    38553,
    38715,
    38877,
    39039,
    39200,
    39361,
    39522,
    39682,
    39842,
    40001,
    40161,
    40319,
    40478,
    40635,
    40793,
    40950,
    41107,
    41263,
    41419,
    41575,
    41730,
    41885,
    42039,
    42194,
    42347,
    42500,
    42653,
    42806,
    42958,
    43109,
    43261,
    43411,
    43562,
    43712,
    43861,
    44011,
    44159,
    44308,
    44456,
    44603,
    44750,
    44897,
    45043,
    45189,
    45334,
    45479,
    45624,
    45768,
    45912,
    46055,
    46198,
    46340,
    46482,
    46624,
    46765,
    46905,
    47046,
    47185,
    47325,
    47464,
    47602,
    47740,
    47877,
    48014,
    48151,
    48287,
    48423,
    48558,
    48693,
    48827,
    48961,
    49095,
    49228,
    49360,
    49492,
    49624,
    49755,
    49885,
    50016,
    50145,
    50274,
    50403,
    50531,
    50659,
    50787,
    50913,
    51040,
    51166,
    51291,
    51416,
    51540,
    51664,
    51788,
    51911,
    52033,
    52155,
    52277,
    52398,
    52518,
    52638,
    52758,
    52877,
    52995,
    53113,
    53231,
    53348,
    53464,
    53580,
    53696,
    53811,
    53925,
    54039,
    54153,
    54266,
    54378,
    54490,
    54602,
    54713,
    54823,
    54933,
    55042,
    55151,
    55260,
    55367,
    55475,
    55582,
    55688,
    55794,
    55899,
    56003,
    56108,
    56211,
    56314,
    56417,
    56519,
    56620,
    56721,
    56822,
    56922,
    57021,
    57120,
    57218,
    57316,
    57413,
    57510,
    57606,
    57702,
    57797,
    57891,
    57985,
    58079,
    58171,
    58264,
    58356,
    58447,
    58537,
    58628,
    58717,
    58806,
    58895,
    58983,
    59070,
    59157,
    59243,
    59329,
    59414,
    59498,
    59582,
    59666,
    59749,
    59831,
    59913,
    59994,
    60075,
    60155,
    60234,
    60313,
    60391,
    60469,
    60546,
    60623,
    60699,
    60775,
    60850,
    60924,
    60998,
    61071,
    61144,
    61216,
    61287,
    61358,
    61429,
    61498,
    61567,
    61636,
    61704,
    61772,
    61838,
    61905,
    61970,
    62035,
    62100,
    62164,
    62227,
    62290,
    62352,
    62414,
    62475,
    62535,
    62595,
    62654,
    62713,
    62771,
    62829,
    62886,
    62942,
    62998,
    63053,
    63107,
    63161,
    63214,
    63267,
    63319,
    63371,
    63422,
    63472,
    63522,
    63571,
    63620,
    63668,
    63715,
    63762,
    63808,
    63853,
    63898,
    63943,
    63986,
    64030,
    64072,
    64114,
    64155,
    64196,
    64236,
    64276,
    64315,
    64353,
    64391,
    64428,
    64464,
    64500,
    64535,
    64570,
    64604,
    64638,
    64671,
    64703,
    64734,
    64765,
    64796,
    64826,
    64855,
    64883,
    64911,
    64939,
    64966,
    64992,
    65017,
    65042,
    65066,
    65090,
    65113,
    65136,
    65158,
    65179,
    65199,
    65219,
    65239,
    65258,
    65276,
    65293,
    65310,
    65327,
    65342,
    65357,
    65372,
    65386,
    65399,
    65412,
    65424,
    65435,
    65446,
    65456,
    65466,
    65475,
    65483,
    65491,
    65498,
    65504,
    65510,
    65515,
    65520,
    65524,
    65527,
    65530,
    65532,
    65534,
    65535,
    65535,
};
