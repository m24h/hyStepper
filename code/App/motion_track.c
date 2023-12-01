/*
 * motion_track.c
 *
 * this uses a simple semi-open algorithm,
 * and provide highest accuracy in scenarios with light loads at low speed,
 * because motor's accuracy (even with cogging) is higher than normal encoder in scenarios with light loads.
 *
 * this algorithm does not fully utilize the power of the motor,
 * so it may lost finally if the stepping speed is too high (comparison against FOC/PID control)
 *
 *  Created on: 2023.11.16
 *      Author: johnny
 */

#include "motion_track.h"
#include "motion.h"
#include "cali.h"
#include "flash.h"

static uint32_t current_max;   // maxium electric current, 2^32 means hardware full current
static uint32_t current_hold;  // holding electric current, 2^32 means hardware full current

typedef struct {
  int32_t mc;  // maxium electric current, 1000 means CONF_CURRENT_MAX
  int32_t hc;  // holding electric current, 1000 means CONF_CURRENT_MAX
} params_t;

static const params_t params_def={
  .mc=1000,
  .hc=150,
};

static const volatile params_t __attribute__((used,__aligned__(4),section(".track_param"))) params_flash=params_def;

static __attribute__((__aligned__(4))) params_t params; // a used version of params_flash, no need to be volatile, for IRQ handler, it will not be optimized

static void trans_params(void)
{
  app_irq_disable();
  current_max=(uint32_t)params.mc*(CONF_CURRENT_MAX/1000U);
  current_hold=(uint32_t)params.hc*(CONF_CURRENT_MAX/1000U);
  if (current_max<current_hold)
    current_max=current_hold;
  app_irq_resume();
}

static void update(driver_update_signal_t sig)
{
  if (!motion_prepare(sig))
    return;

  if (motion_err>(1<<14)) { // 90deg
    motion_drv=((motion_pos+motion_pos_diff)<<16)+motion_start+(1<<30); // safe coherent catching up with light forecast
    motion_cur=current_max;
  } else if (motion_err<-(1<<14)) { // -90deg
    motion_drv=((motion_pos+motion_pos_diff)<<16)+motion_start-(1<<30); // safe coherent catching up with light forecast
    motion_cur=current_max;
  } else {
    motion_drv=(motion_tar<<16)+motion_start;
    register int32_t t;
    register uint32_t cur;
    // decide electric current
    if (motion_tar_diff) { // steping
      cur=current_max;
    } else {
      t=motion_err>=0?motion_err:-motion_err;
      if (t>8192) // 45deg
        cur=current_max;
      else if (t<=(1<<(16-CONF_MSTEP_EXP))) // 1 step
        cur=current_hold;
      else
        cur=current_hold+((current_max-current_hold)>>13)*(uint32_t)(t-(1<<(16-CONF_MSTEP_EXP))); // no need to be precise
    }
    // smooth down electric current
    static int32_t current_smooth=0;
    if (cur>motion_cur) {
      current_smooth=0;
      motion_cur=cur;
    } else {
      current_smooth+=(int32_t)(cur-motion_cur)>>2;
      t=current_smooth>>13; // smooth for nearly 3s
      current_smooth-=t<<13;
      motion_cur+=(uint32_t)t;
    }
  }

  cali_driver_drive(motion_drv, motion_cur);
}

void motion_track_enable(void)
{
  driver_occupy(&update);
}

void motion_track_disable(void)
{
  driver_release(&update);
}

static void init(void)
{
  memcpy(&params, (const void*)&params_flash, sizeof(params_t));
  trans_params();
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);

  if (!str || !*str)
    app_resp_printf(resp, "Track: %s, DIFF=%ld, CUR=%lu, MSTEPS=%u, MC=%ld, HC=%ld\r\n", driver_updater==&update?motion_state_str():"Disable",
        (motion_err)>>(16-CONF_MSTEP_EXP),
        ((motion_cur>>10)*1000+(CONF_CURRENT_MAX>>11))/(CONF_CURRENT_MAX>>10),
        (1U<<CONF_MSTEP_EXP),
        params.mc, params.hc);
  else if (strcasecmp("en", str)==0) {
    motion_track_enable();
    app_resp_printf(resp, "Track: %s\r\n", driver_updater==&update?motion_state_str():"Disable");
  }
  else if (strcasecmp("dis", str)==0) {
    motion_track_disable();
    app_resp_printf(resp, "Track: %s\r\n", driver_updater==&update?motion_state_str():"Disable");
  }
  else if (strcasecmp("mc", str)==0) {
    params.mc=app_sscan_int(&cmdstr, params.mc, 0, 1000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "Track: MC=%ld, MC_DEF=%ld\r\n", params.mc, params_def.mc);
  }
  else if (strcasecmp("hc", str)==0) {
    params.hc=app_sscan_int(&cmdstr, params.hc, 0, 1000);
    trans_params(); // put it into effect
    app_resp_printf(resp, "Track: HC=%ld, HC_DEF=%ld\r\n", params.hc, params_def.hc);
  }
  else if (strcasecmp("save", str)==0) {
    if (flash_write((void*)&params_flash, &params, sizeof(params_t))) {
      app_resp_printf(resp, "Track: Configuration is saved to FLASH\r\n");
    } else
      app_resp_printf(resp, "Track: Failed to write FLASH\r\n");
  }
  else
    app_resp_printf(resp, "Track: <en> <dis> <mc [0-1000]> <hc [0-1000]> <save>\r\n");
}

APP_REG_SUBMODULE("trk", &init, NULL, &cmd, "semi-open-loop motion control with catching up")

