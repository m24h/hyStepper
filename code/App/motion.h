/*
 * motion.h
 *
 *  this provides common parts/utils of other updaters
 *
 *  Created on: 2023.10.13
 *      Author: johnny
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "app.h"
#include "driver.h"

typedef enum {
  MOTION_DISABLE=0, // disabled
  MOTION_ENABLE,    // enabled
  MOTION_BLOCKED,   // enabled, but blocked, maybe resume later
  MOTION_LOST,      // enabled, but stopped, totally lost, out of tracking range, to solve this, to disable/enable again
  MOTION_HOME,      // enabled, but stopped, HOME touched
  MOTION_LIMIT_UP,  // enabled, but stopped, up limit touched
  MOTION_LIMIT_DOWN // enabled, but stopped, down limit touched
} motion_state_t;

// read only for motion user, writable for motion controller
extern volatile motion_state_t motion_state;

// get a literal string of motion_state
const char * motion_state_str(void);

// these variables are used in driver updater mainly, so treate these variables as non-volatile for convenience, but do not use them to do intermediate c
extern int32_t   motion_inp;      // last inputter value, 1 means 1 step
extern int32_t   motion_enc;      // last encoder value, 2^32 means a cycle of motor pole
extern int32_t   motion_drv;      // last driving angle, 2^32 means a cycle of motor pole
extern uint32_t  motion_cur;      // last driving electric current, 2^32 means hardware full current
extern int32_t   motion_start;    // the angle of motor pole at the moment motion is enabled, 2^32 means a cycle of motor pole
extern int32_t   motion_predict;  // current predicted position, 2^32 means a cycle of motor pole

extern int32_t   motion_pos;      // rolling current position, 65536 means a cycle of motor pole
extern int32_t   motion_tar;      // rolling current target, 65536 means a cycle of motor pole
extern int32_t   motion_pos_diff; // position delta value, 65536 means a cycle of motor pole
extern int32_t   motion_tar_diff; // target delta value, 65536 means a cycle of motor pole

extern int32_t   motion_err;      // current error, motion_tar-motion_pos, 65536 means a cycle of motor pole

extern int32_t   motion_vo;      // PID output, base:2^16
extern int32_t   motion_vp;      // PID proportion of error, base:2^16
extern int32_t   motion_vi;      // PID integral of error, base:2^16
extern int32_t   motion_vi_frac; // PID remain fraction of pid_vi, for accuracy of integration
extern int32_t   motion_vd_inp;  // PID differential of inputter value, base:2^16
extern int32_t   motion_vd_enc;  // PID differential of encoder value, base:2^16

extern int32_t   motion_kp;      // coefficient of PID proportion part, [0-2^16), base:1024
extern int32_t   motion_ki;      // coefficient of PID integral part, [0-2^16), base:65536
extern int32_t   motion_kd_inp;  // coefficient of PID differential part of input, [0-2^16), base:256
extern int32_t   motion_kd_enc;  // coefficient of PID differential part of encoder, [0-2^16), base:256

// deal with INIT/EXIT signal, prepare motion_xxx variables, check lost, return 0 if there's no need to do more detailed process
uint32_t motion_prepare(driver_update_signal_t sig);
// transform and set PID coef. base of "kp"/"kf" is 1000, base of "ti"/"td" is 1
void motion_pid_coef(int32_t kp, int32_t ti, int32_t td, int32_t kf);
// calculate and update PID variables
void motion_pid(void);
// disable current motion control, release driver
void motion_disable(void);

#endif /* MOTION_H_ */
