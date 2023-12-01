/*
 * encoder.h
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "app.h"

typedef enum {
  ENCODER_NONE=0,  // unknown
  ENCODER_MODERATE,
  ENCODER_WEAK,
  ENCODER_LOST,
  ENCODER_STRONG,
  ENCODER_OVER,
} encoder_status_t;

// readonly for encoder user
extern volatile encoder_status_t encoder_status;
// readonly, it indicates how many ticks delayed when it get angle value
extern volatile uint32_t encoder_delay_ticks;
// get a literal string of encoder_status
extern const char * encoder_status_str(void);
// test encoder's dynamic performance
void encoder_test(const char * title, uint32_t ticks_interval, app_cmd_res_t resp);
// return: rolling data, means -180 to 180 deg of a whole cycle, this is need to be implemented by actual encoder
int32_t encoder_get(void);

#endif /* ENCODER_H_ */

