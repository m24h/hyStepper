/*
 * cali.h
 *
 *  Created on: 2023.10.16
 *      Author: johnny
 */

#ifndef CALI_H_
#define CALI_H_

#include "app.h"

typedef enum {
  CALI_NODATA=0,
  CALI_OK,
} cali_status_t;

extern volatile cali_status_t cali_status;

// get corrected encoder value, angle 0 coincides with 0 deg of a motor pole
int32_t cali_encoder_get (void);
// drive to an angle of a pole cycle, if "angle" is gotten from from cali_encoder_get(), and multipiled with CONF_MOTOR_POLES, motor will stay in original place
void cali_driver_drive (int32_t angle, uint32_t current);

#endif /* CALI_H_ */
