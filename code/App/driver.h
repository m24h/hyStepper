/*
 * driver.h
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include "app.h"

// frequency is (APP_CLK_APB1/(DRIVER_PWM_REPT<<DRIVER_PWM_BITS)),

#ifdef ENCODER_AS5600
// AS5600 need 70us to read from I2C, so sampling can not be too fast
#define DRIVER_PWM_BITS 10U
#define DRIVER_PWM_REPT 5U
#else
// for 48M MCU, freq. is 15625Hz, for 50 poles motor, nearly 900RPM
#define DRIVER_PWM_BITS 10U
#define DRIVER_PWM_REPT 3U
#endif

typedef enum {
  DRIVER_EXIT=0,
  DRIVER_INIT,
  DRIVER_UPDATE
} driver_update_signal_t;

// updater style
typedef void (*driver_updater_t) (driver_update_signal_t);

// relase output into high impedance state, does not change driver_angle and driver_current
void driver_disable(void);
// enable driver, and set it to the position specified by driver_angle and driver_current
void driver_drive(int32_t angle, uint32_t current);

// readonly for checking, because updater maybe occupied by other modules, must use driver_occupy()/driver_release() to set updater
extern volatile driver_updater_t driver_updater;
// occupy driver, set updater function "updater", if "updater" is NULL, current updater is released
void driver_occupy(driver_updater_t updater);
// release driver only when current updater is the same as specified "updater" or "updater" is NULL
void driver_release(driver_updater_t updater);

#endif /* DRIVER_H_ */
