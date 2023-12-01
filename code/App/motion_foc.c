/*
 * motion_foc.c
 *
 * this uses FOC/PID algorithm to do position control
 *
 *  Created on: 2023.10.13
 *      Author: johnny
 */
#include "motion_foc.h"
#include "motion.h"
#include "flash.h"
#include "cali.h"

static uint32_t current_max;   // maxium electric current, 2^32 means hardware full current

typedef struct {
  int32_t mc;      // stepping electric current, 1000 means CONF_CURRENT_MAX
  int32_t kp;      // Kp of PID, 1000 means that 90deg got full current in a pole
  int32_t ti;      // Ti of PID, 1 means the sample period
  int32_t td;      // Td of PID, 1 means the sample period
  int32_t kf;      // Kf of PID, The injection ratio of input in a differential algorithm, 1000 means 1
} params_t;

static const params_t params_def={
  .mc=1000,
  .kp=100,
  .ti=200,
  .td=40,
  .kf=200
};

static const volatile params_t __attribute__((used,__aligned__(4),section(".foc_param"))) params_flash=params_def;

static __attribute__((__aligned__(4))) params_t params; // a used version of params_flash, no need to be volatile, for IRQ handler, it will not be optimized

static void trans_params(void)
{
  app_irq_disable();
  current_max=((uint32_t)params.mc*(CONF_CURRENT_MAX/1000U))>>16;
  motion_pid_coef(params.kp, params.ti, params.td, params.kf);
  app_irq_resume();
}

static void init(void)
{
  memcpy(&params, (const void*)&params_flash, sizeof(params_t));
  trans_params();
}

static void update(driver_update_signal_t sig)
{
  if (!motion_prepare(sig))
    return;

  motion_pid();

  register uint32_t cur=(uint32_t)(motion_vo>0?motion_vo:-motion_vo);
  if (cur>65536)
    cur=65536;
  motion_cur=cur*current_max;
  motion_drv=motion_predict+(motion_vo>0?(1<<30):motion_vo<0?-(1<<30):0);
  cali_driver_drive(motion_drv, motion_cur);
}

void motion_foc_enable(void)
{
  driver_occupy(&update);
}

void motion_foc_disable(void)
{
  driver_release(&update);
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);

  if (!str || !*str)
    app_resp_printf(resp, "FOC: %s, DIFF=%ld, CUR=%lu, MSTEPS=%u, MC=%ld, KP=%ld, TI=%ld, TD=%ld, KF=%ld\r\n",
        driver_updater==&update?motion_state_str():"Disable",
        (motion_err)>>(16-CONF_MSTEP_EXP),
        ((motion_cur>>10)*1000+(CONF_CURRENT_MAX>>11))/(CONF_CURRENT_MAX>>10),
        (1U<<CONF_MSTEP_EXP),
        params.mc, params.kp, params.ti, params.td, params.kf);

  else if (strcasecmp("en", str)==0) {
    motion_foc_enable();
    app_resp_printf(resp, "FOC: %s\r\n", driver_updater==&update?motion_state_str():"Disable");
  }
  else if (strcasecmp("dis", str)==0) {
    motion_foc_disable();
    app_resp_printf(resp, "FOC: %s\r\n", driver_updater==&update?motion_state_str():"Disable");
  }
  else if (strcasecmp("mc", str)==0) {
    params.mc=app_sscan_int(&cmdstr, params.mc, 0, 1000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "FOC: MC=%ld, MC_DEF=%ld\r\n", params.mc, params_def.mc);
  }
  else if (strcasecmp("kp", str)==0) {
    params.kp=app_sscan_int(&cmdstr, params.kp, 1, 10000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "FOC: KP=%ld, KP_DEF=%ld, CP=%ld\r\n", params.kp, params_def.kp, motion_kp);
  }
  else if (strcasecmp("ti", str)==0) {
    params.ti=app_sscan_int(&cmdstr, params.ti, 0, 10000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "FOC: TI=%ld, TI_DEF=%ld, CI=%ld\r\n", params.ti, params_def.ti, motion_ki);
  }
  else if (strcasecmp("td", str)==0) {
    params.td=app_sscan_int(&cmdstr, params.td, 0, 10000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "FOC: TD=%ld, TD_DEF=%ld, CDE=%ld, CDI=%ld\r\n", params.td, params_def.td, motion_kd_enc, motion_kd_inp);
  }
  else if (strcasecmp("kf", str)==0) {
    params.kf=app_sscan_int(&cmdstr, params.kf, 0, 10000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "FOC: KF=%ld, KF_DEF=%ld, CDI=%ld\r\n", params.kf, params_def.kf, motion_kd_inp);
  }
  else if (strcasecmp("save", str)==0) {
    if (flash_write((void*)&params_flash, &params, sizeof(params_t))) {
      app_resp_printf(resp, "FOC: Configuration is saved to FLASH\r\n");
    } else
      app_resp_printf(resp, "FOC: Failed to write FLASH\r\n");
  }
  else
    app_resp_printf(resp, "FOC: <en> <dis> <mc [0-1000]> <kp [0-10000]> <ti [0-10000]> <td [0-10000]> <kf [0-10000]> <save>\r\n");
}

APP_REG_SUBMODULE("foc", &init, NULL, &cmd, "FOC/PID motion control")

