/*
 * env.h
 *
 *  Created on: 2023.10.16
 *      Author: johnny
 */

#ifndef ENV_H_
#define ENV_H_

#include "app.h"

// readonly, err mask
extern uint32_t env_err_high_temp;

// current voltage of vcc, in mV
extern volatile int32_t env_vcc_mv;
// current temperature, in milli-degree
extern volatile int32_t env_temp_mdeg;

#endif /* ENV_H_ */
