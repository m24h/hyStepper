/*
 * env.c
 *
 *  Created on: 2023.10.16
 *      Author: johnny
 */

#include "env.h"
#include "air001xx_ll_system.h"
#include "air001xx_ll_bus.h"
#include "air001xx_ll_adc.h"

volatile int32_t env_vcc_mv;
volatile int32_t env_temp_mdeg;

#define DEF_TEMP 25000

APP_REG_ERROR(, env_err_high_temp, "HighTemp")

static void init(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  LL_ADC_Reset(ADC1);

  LL_ADC_SetCalibrationMode(ADC1, LL_ADC_CAL_MODE_OFFSETANDLINEARITY);
  LL_ADC_SetCalibrationSamplingTime(ADC1, LL_ADC_CAL_SAMPLINGTIME_8CYCLES);
  LL_ADC_StartCalibration(ADC1);
  while (LL_ADC_IsCalibrationOnGoing(ADC1) || (LL_ADC_GetCalibrationStatus(ADC1) & LL_ADC_CAL_STATUS_ONGOING)); // wait
  app_delay_clk(LL_ADC_DELAY_CALIB_ENABLE_CPU_CYCLES);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV32); // 1.5MHz
  LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR|LL_ADC_PATH_INTERNAL_VREFINT);
  app_delay_clk(((LL_ADC_DELAY_VREFINT_STAB_US>LL_ADC_DELAY_TEMPSENSOR_STAB_US?LL_ADC_DELAY_VREFINT_STAB_US:LL_ADC_DELAY_TEMPSENSOR_STAB_US)*(APP_CLK_SYS>>10))>>10);

  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5); // nearly 3kHz
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_TEMPSENSOR|LL_ADC_CHANNEL_VREFINT);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT);

  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversion(ADC1);

  env_vcc_mv=CONF_VCC;
  env_temp_mdeg=DEF_TEMP;
}

static void loop(void)
{
  static uint32_t timer=0;
  if (timer && (int32_t)(app_cnt_1ms-timer)<0)
    return;
  timer=app_cnt_1ms+153;

  if (LL_ADC_IsActiveFlag_EOC(ADC1)) {
    int32_t val=(int32_t)LL_ADC_REG_ReadConversionData12(ADC1); // this also clear EOC
    if (LL_ADC_IsActiveFlag_EOS(ADC1)) { // the last one : VREFINT
       LL_ADC_ClearFlag_EOS(ADC1);
      if (val>500) // cannot over 9.8V
        env_vcc_mv=(env_vcc_mv*15+((int32_t)VREFINT_CAL_VREF*4095/val)+(1<<3))>>4;
    } else {
      val=val*3300/env_vcc_mv; // adjust for non 3.3V VCC
      env_temp_mdeg=(int32_t)(env_temp_mdeg*15+(1000*(TEMPSENSOR_CAL2_TEMP-TEMPSENSOR_CAL1_TEMP))*(val-(int32_t)*TEMPSENSOR_CAL1_ADDR)/(*TEMPSENSOR_CAL2_ADDR-*TEMPSENSOR_CAL1_ADDR)+(TEMPSENSOR_CAL1_TEMP*1000))>>4;
      if (env_temp_mdeg>=CONF_TEMP_HIGH) {
        if (!(app_error & env_err_high_temp)) {
          app_irq_disable();
          app_error |= env_err_high_temp;
          app_irq_resume();
        }
      } else if (env_temp_mdeg<=CONF_TEMP_OK) {
        if (app_error & env_err_high_temp) {
          app_irq_disable();
          app_error &= ~env_err_high_temp;
          app_irq_resume();
        }
      }
    }
  }
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  (void)cmdstr;
  app_resp_printf(resp, "Env: VCC=%ld, TEMP=%ld\r\n", env_vcc_mv, env_temp_mdeg);
}

APP_REG_SUBMODULE("env", &init, &loop, &cmd, "show working environment")
