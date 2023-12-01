/*
 * motion.c
 *
 * PPID uses 2DOF-PID control algorithm.
 * PidOut = Kp*error + Ki*sigma(error) + Kd*(Kf*delta(input)-delta(encoder))
 * Ki=Kp/Ti, (Ti=T-of-integral/T-sample)
 * Kd=Kp*Td, (Td=T-of-differential/T-sample)
 *
 *
 *  Created on: 2023.11.17
 *      Author: johnny
 */
#include "motion.h"
#include "encoder.h"
#include "inputter.h"
#include "driver.h"
#include "cali.h"

#include "air001xx_ll_gpio.h"

#define T_SAMPLE ((DRIVER_PWM_REPT<<DRIVER_PWM_BITS)*(APP_CLK_SYS>>16)/(APP_CLK_APB1>>16))

#define IS_NOT_HOME (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_13))

volatile motion_state_t motion_state;

int32_t   motion_inp;
int32_t   motion_enc;
int32_t   motion_drv;
uint32_t  motion_cur;

int32_t   motion_start;
int32_t   motion_predict;
int32_t   motion_pos;
int32_t   motion_tar;
int32_t   motion_err;
int32_t   motion_pos_diff;
int32_t   motion_tar_diff;

int32_t   motion_vo;
int32_t   motion_vp;
int32_t   motion_vi;
int32_t   motion_vi_frac;
int32_t   motion_vd_inp;
int32_t   motion_vd_enc;

int32_t   motion_kp;
int32_t   motion_ki;
int32_t   motion_kd_inp;
int32_t   motion_kd_enc;

typedef enum {
  TRACE_NONE=0,
  TRACE_ERR,
  TRACE_PID
} trace_t;
static volatile trace_t trace;

#define TEST_DATA_SIZE 384U
typedef struct {
  uint32_t          timer;
  uint32_t          timer_set;
  volatile uint32_t len;
  int32_t           data[TEST_DATA_SIZE];
} test_data_t;
static test_data_t * volatile ptest;

#define LIMIT(v, m) \
  if ((v)>(m)) (v)=(m); else if ((v)<-(m)) (v)=-(m);

const char * motion_state_str(void)
{
  switch(motion_state) {
    case MOTION_DISABLE: return "Disable";
    case MOTION_ENABLE: return "Enable";
    case MOTION_BLOCKED: return "Blocked";
    case MOTION_LOST: return "Lost";
    case MOTION_HOME: return "Home";
    case MOTION_LIMIT_UP: return "UpLimit";
    case MOTION_LIMIT_DOWN: return "DownLimit";
    default: return "Unknown";
  }
}

static void init(void)
{
  motion_state=MOTION_DISABLE;

  motion_inp=0;
  motion_enc=0;
  motion_drv=0;
  motion_cur=0U;
  motion_start=0;
  motion_predict=0;
  motion_pos=0;
  motion_tar=0;
  motion_err=0;
  motion_pos_diff=0;
  motion_tar_diff=0;
  motion_vo=0;
  motion_vp=0;
  motion_vi=0;
  motion_vi_frac=0;
  motion_vd_inp=0;
  motion_vd_enc=0;
  motion_kp=0;
  motion_ki=0;
  motion_kd_inp=0;
  motion_kd_enc=0;

  ptest=NULL;
  trace=TRACE_NONE;
}

#if CONF_MSTEP_EXP>=16
#error following algorithm is wrong
#endif

uint32_t motion_prepare(driver_update_signal_t sig)
{
  if (sig==DRIVER_EXIT) {
    motion_state=MOTION_DISABLE;
    motion_cur=0U;
    trace=TRACE_NONE;
    cali_driver_drive(motion_drv, motion_cur);
    return 0U;
  }

  if (sig==DRIVER_INIT) {
    motion_inp=inputter_get();
    motion_enc=cali_encoder_get()*(int32_t)CONF_MOTOR_POLES;
    motion_drv=motion_enc;
    motion_cur=CONF_CURRENT_MAX/6;
    motion_start=motion_enc;
    motion_predict=motion_enc;
    motion_pos=0;
    motion_tar=0;
    motion_err=0;
    motion_pos_diff=0;
    motion_tar_diff=0;
    cali_driver_drive(motion_drv, motion_cur);
    motion_vo=0;
    motion_vp=0;
    motion_vi=0;
    motion_vi_frac=0;
    motion_vd_inp=0;
    motion_vd_enc=0;
    motion_state=MOTION_ENABLE;
    return 0U;
  }

  if (sig!=DRIVER_UPDATE)
    return 0U;

  // check state
  motion_state_t state=motion_state;  // to be non-volatile
  if (state==MOTION_DISABLE || state==MOTION_LOST)
    return 0U;

  if (IS_NOT_HOME) {
    if (state==MOTION_HOME)
      motion_state=MOTION_ENABLE;
  } else {
    motion_state=MOTION_HOME;
    return 0U;
  }

  if (encoder_status==ENCODER_LOST) {
LOST_RETURN:
    motion_state=MOTION_LOST;
RELEASE_RETURN:
    motion_cur=0; // release force
    cali_driver_drive(motion_drv, motion_cur);
    return 0U;
  }

  register int32_t t;
  // prepare variables
  t=inputter_get();
  motion_tar_diff=t-motion_inp;
  if (motion_tar_diff>=(1<<(14+CONF_MSTEP_EXP)) || motion_tar_diff<=-(1<<(14+CONF_MSTEP_EXP)))  // it will be overflow if inputter take a very long step
    goto LOST_RETURN;
  motion_inp=t;
  motion_tar+=motion_tar_diff<<(16-CONF_MSTEP_EXP);

  t=cali_encoder_get()*(int32_t)CONF_MOTOR_POLES;
  motion_pos_diff=(t-motion_enc+(1<<15))>>16;
  motion_predict=t+motion_pos_diff*(32768+(int32_t)(((1U<<20)/T_SAMPLE)*(encoder_delay_ticks>>4)));
  motion_enc+=(motion_pos_diff<<16); // avoid accumulating error of calculation
  motion_pos+=motion_pos_diff;

  motion_err=motion_tar-motion_pos;

  test_data_t *pt=ptest;
  if (pt && pt->len<TEST_DATA_SIZE) {
    if (pt->timer) {
      pt->timer--;
    } else {
      pt->timer=pt->timer_set;
      pt->data[pt->len++]=motion_err;
    }
  }

  if (motion_err>(1<<30) || motion_err<-(1<<30))
    goto LOST_RETURN;

  // speed must be limited, or rolling encoder value may be overflow
  if (motion_pos_diff>6000 || motion_pos_diff<-6000)
    goto RELEASE_RETURN;

  return 1U;
}

void motion_pid_coef(int32_t kp, int32_t ti, int32_t td, int32_t kf)
{
  // make sure other coef. are less than 2^16
  motion_kp=((kp<<10)+500)/1000; // base:1024
  if (motion_kp>=(1<<16)) motion_kp=(1<<16)-1; // [0-2^16)
  motion_ki=ti?((motion_kp<<6)/ti):0; // base:65536
  if (motion_ki>=(1<<16)) motion_ki=(1<<16)-1; // [0-2^16)
  motion_kd_enc=(motion_kp*td+(1<<1))>>2; // base:256
  if (motion_kd_enc>=(1<<16)) motion_kd_enc=(1<<16)-1; // [0-2^16)
  motion_kd_inp=(motion_kd_enc*kf+500)/1000; // base:256
  if (motion_kd_inp>=(1<<16)) motion_kd_inp=(1<<16)-1; // [0-2^16)
}

void motion_pid()
{
#if CONF_MSTEP_EXP>=10
#error following algorithm is lack of precision
#endif

  register int32_t t;

  // proportion
  t=(motion_err+(1<<3))>>4;  // base:1024=90deg
  LIMIT(t, (1<<15)-1)   // range:(+-2^15)
  motion_vp=(t*motion_kp+(1<<5))>>4; // range:(+-2^27), base:2^16

  // limited integral
  motion_vi_frac+=(t*motion_ki)>>1;  // range:(+-2^31), base:2^25, for accuracy of integration, not to miss any error
  t=(motion_vi_frac+(1<<8))>>9;   // range:(+-2^21), base:2^16
  motion_vi_frac-=t<<9;        // range:(+-2^9), base:2^26
  t+=motion_vi;  // range:(+-2^17), base:2^16
  LIMIT(t, (1<<16)-1) // range:(+-2^16), base:2^16
  motion_vi=t;

  // differential of inputter value
  t=(motion_tar_diff+(1<<1))>>2; // base:4096=90deg
  LIMIT(t, (1<<15)-1)   //  range:(+-2^15)
  motion_vd_inp=(t*motion_kd_inp+(1<<3))>>4;   // range:(+-2^27), base:2^16

  // differential of encoder value
  t=(motion_pos_diff+(1<<1))>>2; // base:4096=90deg
  LIMIT(t, (1<<15)-1)   // range:(+-2^15)
  motion_vd_enc=(t*motion_kd_enc+(1<<3))>>4; // range:(+-2^27), base:2^16

  motion_vo=motion_vp+motion_vi+motion_vd_inp-motion_vd_enc;
}

void motion_disable(void)
{
  driver_release(NULL);
}

static void loop(void)
{
  if (motion_state!=MOTION_DISABLE) {
    if (app_error || encoder_status==ENCODER_NONE || cali_status!=CALI_OK)
      motion_disable();

    if (trace!=TRACE_NONE) {
      if (app_shell_sig==APP_SIG_INT)
        trace=TRACE_NONE;
      else if (trace==TRACE_PID)
        app_printf("Motion: VO=%ld, VP=%ld, VI=%ld, VDE=%ld, VDI=%ld\r\n", motion_vo, motion_vp, motion_vi, motion_vd_enc, motion_vd_inp);
      else
        app_printf("Motion: ERR=%ld, TAR=%ld, POS=%ld, DRV=%ld, CUR=%lu\r\n", motion_err, motion_tar, motion_pos, motion_drv, motion_cur);
    }
  }
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);

  if (!str || !*str)
    app_resp_printf(resp, "Motion: %s, DIFF=%ld, CUR=%lu, MSTEPS=%u\r\n", motion_state_str(),
        (motion_err)>>(16-CONF_MSTEP_EXP),
        ((motion_cur>>10)*1000+(CONF_CURRENT_MAX>>11))/(CONF_CURRENT_MAX>>10),
        (1U<<CONF_MSTEP_EXP));
  else if (strcasecmp("dis", str)==0) {
    motion_disable();
    app_resp_printf(resp, "Motion: %s\r\n", motion_state_str());
  }
  else if (strcasecmp("test", str)==0) {
    test_data_t * p;
    if (motion_state==MOTION_DISABLE)
      app_resp_printf(resp, "Motion: No controller is enabled\r\n");
    else if (!(p=(test_data_t*)aligned_alloc(4U, sizeof(test_data_t))))
      app_resp_printf(resp, "Motion: No memory\r\n");
    else {
      p->timer_set=(uint32_t)app_sscan_int(&cmdstr, 0, 0, 0x7FFFFFF);
      p->timer=p->timer_set;
      p->len=0;
      int32_t steps=app_sscan_int(&cmdstr, 0, -10000, 10000)<<(16-CONF_MSTEP_EXP);
      app_irq_disable();
      ptest=p;
      motion_tar+=steps;
      app_irq_resume();
       app_resp_printf(resp, "Motion: Following is error data\r\n");
      while(motion_state==MOTION_ENABLE && ptest->len<TEST_DATA_SIZE);
      ptest=NULL;
      for (uint32_t i=0;i<p->len;i++)
        app_resp_printf(resp, "%ld\r\n", p->data[i]);
      app_irq_disable(); // maybe data is still used by updater
      free(p);
      app_irq_resume();
    }
  }
  else if (strcasecmp("coef", str)==0)
    app_resp_printf(resp, "Motion: KP=%lu, KI=%ld, KDE=%ld, KDI=%ld\r\n",
        motion_kp, motion_ki, motion_kd_enc, motion_kd_inp
    );
  else if (strcasecmp("val", str)==0)
    app_resp_printf(resp, "Motion: VO=%ld, VP=%ld, VI=%ld, VDE=%ld, VDI=%ld\r\n",
        motion_vo, motion_vp, motion_vi, motion_vd_enc, motion_vd_inp
    );
  else if (strcasecmp("trace", str)==0)
    trace=app_sscan_chk(&cmdstr, "pid")?TRACE_PID:app_sscan_chk(&cmdstr, "err")?TRACE_ERR:TRACE_NONE;
  else
    app_resp_printf(resp, "Motion: <dis> <coef> <val> <test [timer] [steps]> <trace [err|pid]>\r\n");
}

APP_REG_SUBMODULE("mot", &init, &loop, &cmd, "common motion control")


