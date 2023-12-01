/*
 * cali.c
 *
 *  using such a model :
 *
 *  1. when there is no load, in 0,45,90,135,180,225,270,315 deg, <real angle> = <drive angle> + <driver offset> + <direction error>
 *  2. when there is no load, in other deg, <real angle> = <drive angle> + <driver offset> + <direction error> + <driver error>
 *  3. <real angle> = <encoder value> + <encoder error>
 *  4. <driver offset> is a DC value
 *  5. <encoder error> is in the low frequency region, the frequency is 4 / cycle
 *  6. <driver error> is mostly because of cogging, in the high frequency region, the frequency is 4*poles / cycle
 *  7. <direction error> is mostly because of internal compensation of encoder, and inertia/damping of motor, can be countervailed by CW and CCW
 *  8. <driver error> can be countervailed partly by using symmetric position of sampling points
 *
 *  Created on: 2023.10.16
 *      Author: johnny
 */

#include "cali.h"
#include "driver.h"
#include "encoder.h"
#include "flash.h"

volatile cali_status_t cali_status;

#define CURRENT       (CONF_CURRENT_MAX/2)  /* 0xFFFFFFFF means full current */
#define TIME_CYCLE    5U                    /* 5 second per cycle */

// those data was given considering poles of motor, variable's limit, memory stack usage, don't change them easily
#define ENCODER_EXP           8U /* 8 means encoder data having 256 points */
#define DRIVER_EXP            7U /* 7 means 128 microsteps in a cycle of a pole, just for measured calibration data, not for actual division */

typedef struct {
  uint32_t current;       // current used in calibration, should be CURRENT
  uint32_t invert;        // should invert trunning direction, encoder value is increased in CW direction, motor is not
  int32_t  offset;        // offset between 0 deg of encoder and 0 deg of driver
  uint32_t mse_encoder;   // MSE of encoder error, (45deg)^2/2^32
  uint32_t mse_driver;    // MSE of driver error, (180deg)^2/2^32 for a single pole
  uint32_t encoder_size;  // num of encoder calibration points, should be 1<<ENCODER_EXP
  uint32_t driver_size;   // num of driver calibration data, should be 1<<DRIVER_EXP
  int16_t  encoder  [1<<ENCODER_EXP];  // real angle - measured - offset = this * 11.25 (180/16) deg/32768
  int16_t  driver   [1<<DRIVER_EXP];   // real angle - driven = this * 90 deg/32768 (for a single pole, for whole cycle, this number should be divided by CONF_MOTOR_POLES)
} params_t;

static const volatile params_t __attribute__((used,__aligned__(4),section(".cali_param"))) params={0}; // initialized not valid, and so that volatile is needed

typedef enum {
  NONE=0,
  ABORT,
  DONE,
  MEASURE,
  TEST
} state_t;

typedef struct {
  params_t     params;
  union { // temporary data
    int32_t enc32[1<<ENCODER_EXP];
    int16_t tmp16[1<<ENCODER_EXP];
  };
  uint32_t          timer;
  app_cmd_res_t     resp;
  int32_t           step;
  int32_t           angle;
  uint8_t           stage;  // 0:CW prepare 1:CW 2:CCW prepare 3:CCW
} data_t;

static data_t * volatile pdata;
static volatile state_t  state;

static void init(void)
{
  // check if data in FLASH is valid
  if (params.current!=CURRENT
   || params.encoder_size!=(1<<ENCODER_EXP)
   || params.driver_size!=(1<<DRIVER_EXP))
  {
    app_printf("CALI: data in FLASH is invalid, new calibration is needed\r\n");
    cali_status=CALI_NODATA;
  } else
    cali_status=CALI_OK;

  pdata=NULL;
  state=NONE;
}

static void update(driver_update_signal_t sig)
{
  // make it non-volatile
  state_t st=state;
  data_t *p=pdata;
  encoder_status_t _encoder_status=encoder_status;

  // if error
  if (sig==DRIVER_EXIT || app_error || _encoder_status==ENCODER_LOST || _encoder_status==ENCODER_OVER || _encoder_status==ENCODER_NONE) {
    state=ABORT;
    return;
  }

  // if not started
  if (sig!=DRIVER_UPDATE || (st!=MEASURE && st!=TEST) || !p)
    return;

  // if it's not the time
  if (p->timer) {
    p->timer--;
    return;
  } else {
    p->timer=(TIME_CYCLE*(APP_CLK_APB1/(DRIVER_PWM_REPT<<DRIVER_PWM_BITS)))/(CONF_MOTOR_POLES<<DRIVER_EXP); // 1 cycle every 5 seconds
  }

  // get encoder data, check the direction of rotation
  int32_t enc_value;
  if (st==TEST)
    enc_value=cali_encoder_get();
  else {
    enc_value=encoder_get();
    if (p->stage==0) {
      if (enc_value-(int32_t)p->params.mse_encoder<0) // encoder value is getting smaller
        p->params.invert++;
      else
        p->params.invert--;
      p->params.mse_encoder=(uint32_t)enc_value; // temporary use this to store last encoder value
    }
  }

  // measurement
  if (p->stage & 1) {
    int32_t diff=(p->params.invert?-p->angle:p->angle)+p->params.offset-enc_value;
    if (st==TEST) {
      if ((p->step & 7)==0) // only output 1/8 data
        app_resp_printf(p->resp, "%ld\r\n", diff);
      else {
        app_delay_clk(APP_CLK_SYS/CONF_UART1_BAUDRATE*80); // just keep balance of rotate speed
      }
      int32_t d=(diff+(1<<12))>>13; // 45deg/65536
      p->params.mse_encoder+=(uint32_t)(d*d)>>(DRIVER_EXP+1); // (45deg/65536)^2, use it as total MSE, maybe overflow if MSE is too large
    } else {
      // encoder error
      uint32_t idx=((uint32_t)enc_value+(1U<<(31-ENCODER_EXP)))>>(32-ENCODER_EXP);
  #if (CONF_MOTOR_POLES<<DRIVER_EXP<<1>>ENCODER_EXP)>128
  #error maybe overflow
  #endif
      p->enc32[idx]+=(diff+(1<<7))>>8;   // 360deg/2^24
      p->params.encoder[idx]++;
      // driver error
      idx=(p->params.invert?-p->step:p->step) & ((1<<DRIVER_EXP)-1);
      if ((idx & ((1<<(DRIVER_EXP-2))-1))==0) // in those points, there should be no cogging
        p->params.mse_encoder=(uint32_t)diff; // temporary usage
      p->params.driver[idx]+=(int16_t)((int32_t)(diff-(int32_t)p->params.mse_encoder+(1<<15))>>16); // 360deg/65536, CONF_MOTOR_POLES*2 points, each element is limited to +-90deg/CONF_MOTOR_POLES, that's always OK
    }
  }

  // check the progress and change the state
  switch(p->stage) {
  case 0:
    p->step++;
    if (p->step>=(int32_t)((CONF_MOTOR_POLES>>1)<<DRIVER_EXP)) {
      p->stage=1;
      if (st==MEASURE)
        p->params.invert=((int32_t)p->params.invert>0)?1:0;
      else {
        p->params.offset=enc_value-p->angle;
        // align to 0 angle of motor pole
        p->params.offset=(int32_t)(((uint32_t)p->params.offset+((1U<<31)/CONF_MOTOR_POLES))/(((uint32_t)-(CONF_MOTOR_POLES>>1))/CONF_MOTOR_POLES+1U));
        p->params.offset=(int32_t)((uint32_t)p->params.offset*(((uint32_t)-(CONF_MOTOR_POLES>>1))/CONF_MOTOR_POLES+1U));
      }
    }
    break;
  case 1:
    p->step++;
    if (p->step>=(int32_t)(((CONF_MOTOR_POLES>>1)<<DRIVER_EXP)+(CONF_MOTOR_POLES<<DRIVER_EXP)))
       p->stage=2;
    break;
  case 2:
    p->step--;
    if (p->step<=(int32_t)(CONF_MOTOR_POLES<<DRIVER_EXP))
      p->stage=3;
    break;
  case 3:
    p->step--;
    if (p->step<=0)
      state=DONE;
    break;
  }
  p->angle=p->step*(int32_t)(((1U<<(32-DRIVER_EXP))+(CONF_MOTOR_POLES>>1))/CONF_MOTOR_POLES);
  if (st==TEST)
    cali_driver_drive(p->angle*(int32_t)CONF_MOTOR_POLES, CURRENT);
  else
    driver_drive(p->angle*(int32_t)CONF_MOTOR_POLES, CURRENT);
}

static void fir_filter(int16_t * dest, const int16_t *src, int32_t size, const int16_t* fir, int32_t half_order)
{
  for (int32_t i=0; i<size; i++) {
    int32_t t=0;
    for (int32_t j=-half_order; j<=half_order; j++) {
      int32_t k=i+j;
      if (k<0)
        k+=size;
      else if (k>=size)
        k-=size;
      t+=(int32_t)src[k]*(int32_t)fir[j+half_order];
    }
    dest[i]=(int16_t)((t+(1<<15))>>16);
  }
}

static void prepare(app_cmd_res_t resp)
{
  pdata=NULL;

  data_t * p=(data_t*)aligned_alloc(4U, sizeof(data_t));
  if (!p) {
    app_resp_printf(resp, "Cali: Out of memory\r\n");
    return;
  }

  memset(p, 0, sizeof(data_t));
  p->params.current=CURRENT;
  p->params.encoder_size=1<<ENCODER_EXP;
  p->params.driver_size=1<<DRIVER_EXP;
  p->resp=resp;

  pdata=p;
  driver_occupy(&update);
}

static void finish(void)
{
  driver_release(&update);

  app_irq_disable(); // prevent updater using pdata when detroying pdata
  free(pdata);
  pdata=NULL;
  app_irq_resume();
}

static void calibrate(app_cmd_res_t resp)
{
  prepare(resp);
  if (!pdata) return;

  app_resp_printf(resp, "Cali: Now the motor should be rotating\r\n");
  state=MEASURE; // start signal

  for (;;) {
    if (driver_updater!=&update || state==ABORT) {
      app_resp_printf(resp, "Cali: Abort\r\n");
      break;
    }
    if (state==DONE) {
      data_t *p=pdata;
      // average and offset
      int32_t avg=0;
      for (uint32_t i=0; i<(1U<<ENCODER_EXP); i++) {
#if CONF_MOTOR_POLES>128
#error maybe overflow
#endif
        avg+=(p->enc32[i]+(1<<DRIVER_EXP))>>(DRIVER_EXP+1); // 360deg/2^24
      }
      avg=(avg+(int32_t)(CONF_MOTOR_POLES>>1))/(int32_t)CONF_MOTOR_POLES;
      p->params.offset=avg<<8; // 360deg/2^32
      // encoder error and MSE
      p->params.mse_encoder=0;
      for (uint32_t i=0; i<(1U<<ENCODER_EXP); i++) {
        p->tmp16[i]=(int16_t)((int32_t)(((p->enc32[i]+(p->params.encoder[i]>>1))/p->params.encoder[i])-avg+(1<<3))>>4); // 360deg/2^24 -> 360deg/2^20 = 22.5deg/65536
        p->params.mse_encoder+=((uint32_t)((int32_t)p->tmp16[i]*(int32_t)p->tmp16[i])+(1U<<(ENCODER_EXP-1)))>>ENCODER_EXP; // (22.5deg/65536)^2
      }
      p->params.mse_encoder=(p->params.mse_encoder+(1U<<1))>>2;  // (45deg/65536)^2
      // FIR for encoder error
      static const int16_t fir_enc[]={13029,13146,13186,13146,13029};
      fir_filter(p->params.encoder, p->tmp16, 1<<ENCODER_EXP, fir_enc, 2);
      // do filter with driver error data
      static const int16_t fir_drv[]={12672,13324,13544,13324,12672};
#if DRIVER_EXP>ENCODER_EXP*2
#error following code is not suitable if DRIVER_EXP is bigger than ENCODER_EXP*2
#endif
      fir_filter(p->tmp16, p->params.driver, 1<<DRIVER_EXP, fir_drv, 2);
      // calculate driver MSE
      p->params.mse_driver=0;
      for (uint32_t i=0; i<(1U<<DRIVER_EXP); i++) {
        p->params.mse_driver+=((uint32_t)((int32_t)p->params.driver[i]*(int32_t)p->params.driver[i])+(1<<(DRIVER_EXP-1)))>>DRIVER_EXP; // (180deg)^2/2^32
        p->params.driver[i]=p->tmp16[i]; // no simple method to correct cogging, just copy it
      }
      // burn to FLASH
      if (!flash_write((void*)&params, &(p->params), sizeof(params_t))) {
        app_resp_printf(resp, "Cali: Failed to write FLASH\r\n");
        break;
      }
      // all done
      cali_status=CALI_OK;
      app_resp_printf(resp, "Cali: Done, FLASH is updated, ENC_MSE=%lu, DRV_MSE=%lu\r\n",
          (params.mse_encoder*483)>>10, (params.mse_driver*483)>>6);
      break;
    }
  }

  state=NONE;
  finish();
}

static void test(app_cmd_res_t resp)
{
  if (cali_status!=CALI_OK) {
    app_resp_printf(resp, "Cali: Do calibration first\r\n");
    return;
  }

  prepare(resp);
  if (!pdata) return;

  app_resp_printf(resp, "Cali: Following is error data\r\n");
  state=TEST; // start signal

  for(;;) {
    if (driver_updater!=&update || state==ABORT) {
      app_resp_printf(resp, "Cali: Abort\r\n");
      break;
    }
    if (state==DONE) {
      data_t *p=pdata;
      app_resp_printf(resp, "Cali: Done, TOTAL_MSE=%lu\r\n", (p->params.mse_encoder/CONF_MOTOR_POLES*483)>>10);
      break;
    }
  }

  state=NONE;
  finish();
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);
  if (!str || !*str)
    // OFFSET, MSE use unit of 0.001 deg or (0.001 deg)^2
    app_resp_printf(resp, "Cali: %s, OFFSET=%ld, ENC_MSE=%lu, DRV_MSE=%lu\r\n",
        (cali_status==CALI_NODATA?"NoData":params.invert?"Invert":"OK"), ((params.offset>>13)*5625)>>13,
        (params.mse_encoder*483)>>10, (params.mse_driver*483)>>6); // (45deg)^2/32^2 / (180deg)^2/32^2  -> (0.001deg)^2
  else if (strcasecmp("enc", str)==0) {
    app_resp_printf(resp, "Cali: encoder data\r\n");
    for (uint32_t i=0;i<(1U<<ENCODER_EXP);i++)
      app_resp_printf(resp, "%d\r\n", params.encoder[i]);
  }
  else if (strcasecmp("drv", str)==0) {
    app_resp_printf(resp, "Cali: driver data\r\n");
    for (uint32_t i=0;i<(1U<<DRIVER_EXP);i++)
      app_resp_printf(resp, "%d\r\n", params.driver[i]);
  }
  else if (strcasecmp("start",str)==0)
      calibrate(resp);
  else if (strcasecmp("test",str)==0)
      test(resp);
  else
    app_resp_printf(resp, "Cali: <enc> <drv> <start> <test>\r\n");
}

int32_t cali_encoder_get ()
{
  register int32_t angle=encoder_get();

  uint32_t idx=(uint32_t)angle>>(32-ENCODER_EXP);
  uint32_t remain=((uint32_t)angle>>(17-ENCODER_EXP)) & 0xFFFF; // limited to 15bit, err2-err1 is +-65535
  int32_t  err1=params.encoder[idx]; // 16bit  +-(90/8)deg/+-32768 => +-(180*4096)deg/+-2^31
  int32_t  err2=params.encoder[(idx+1) & ((1U<<ENCODER_EXP)-1U)];
  angle=angle+(err1<<12)+(((err2-err1)*(int32_t)remain+(1<<2))>>3)+params.offset;

  return angle;
}

void cali_driver_drive (int32_t angle, uint32_t current)
{
  // I have no idea to conquer cogging problem, but cogging error is small enough
  driver_drive((params.invert?-angle:angle), current);
}

APP_REG_SUBMODULE("cali", &init, NULL, &cmd, "calibration")
