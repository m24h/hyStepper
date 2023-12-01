/*
 * encoder.c
 *
 *  Created on: 2023.11.17
 *      Author: johnny
 */

#include "encoder.h"
#include "driver.h"

volatile encoder_status_t encoder_status;

volatile uint32_t encoder_delay_ticks=0;

#define TEST_CURRENT (CONF_CURRENT_MAX/4)

const char * encoder_status_str(void)
{
  switch(encoder_status) {
    case ENCODER_MODERATE: return "MODERATE";
    case ENCODER_WEAK: return "WEAK";
    case ENCODER_LOST: return "LOST";
    case ENCODER_STRONG: return "STRONG";
    case ENCODER_OVER: return "OVER";
    default: return "Unknown";
  }
}

void encoder_test(const char * title, uint32_t ticks_interval, app_cmd_res_t resp)
{
  typedef struct {
    int32_t drv [128];
    int32_t enc [128];
  } test_data_t;

  test_data_t * p=(test_data_t*)aligned_alloc(4U, sizeof(test_data_t));
  if (!p) {
    app_resp_printf(resp, "%s: No memory\r\n", title);
    return;
  }

  // make driving stable
  driver_drive(0, TEST_CURRENT);
  app_delay_clk(480000);

  // test driver and encoder
  app_resp_printf(resp, "%s: Following is test data\r\n", title);
  uint32_t idx=0;
  p->drv[0]=0;
  uint64_t clk=app_cnt_clk()+ticks_interval;
  for (int32_t i=1; i<=16; i++) {
    while(app_cnt_clk()<clk);
    clk+=ticks_interval;
    p->enc[idx]=encoder_get();
    p->drv[++idx]=i<<26;
    driver_drive(p->drv[idx], TEST_CURRENT);
  }
  for (int32_t i=15; i>=0; i--) {
    while(app_cnt_clk()<clk);
    clk+=ticks_interval;
    p->enc[idx]=encoder_get();
    p->drv[++idx]=i<<26;
    driver_drive(p->drv[idx], TEST_CURRENT);
  }
  for (int32_t i=1; i<=16; i++) {
    while(app_cnt_clk()<clk);
    clk+=ticks_interval;
    p->enc[idx]=encoder_get();
    p->drv[++idx]=i<<26;
    driver_drive(p->drv[idx], TEST_CURRENT);
  }
  for (int32_t i=0; i<70; i++) {
    while(app_cnt_clk()<clk);
    clk+=ticks_interval;
    p->enc[idx]=encoder_get();
    p->drv[idx++]=16<<26;
  }

  int32_t dir=(p->enc[idx-1]>p->enc[0])?(int32_t)CONF_MOTOR_POLES:-(int32_t)CONF_MOTOR_POLES;
  for (uint32_t i=0; i<idx; i++)
    app_resp_printf(resp, "%ld, %ld\r\n", p->drv[i], (p->enc[i]-p->enc[0])*dir);

  driver_disable();
  free(p);
}
