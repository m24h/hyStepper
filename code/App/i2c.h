/*
 * i2c.h
 *
 * functions in this file does not disable IRQ
 *
 *  Created on: 2023.10.31
 *      Author: johnny
 */

#ifndef I2C_H_
#define I2C_H_

#include "app.h"

// setup I2C port/pin and delay clocks number, (an I2C bit need nearly 3 delay_clk clocks)
void i2c_conf(GPIO_TypeDef *GPIO_sda, uint32_t Pin_sda, GPIO_TypeDef *GPIO_scl, uint32_t Pin_scl, uint32_t delay_clk);
// i2c delay, nearly "delay_clk" MCU clocks, which is defined in i2c_set();
void i2c_delay(void);
// send I2C ACK signal
void i2c_ack(void);
// send I2C NACK signal
void i2c_nack(void);
// send I2C start signal
void i2c_start(void);
// send I2C stop signal
void i2c_stop(void);
// read bytes from I2C, send ACK after each bytes got, excluding the last one (maybe NACK for STOP)
void i2c_recv (uint8_t *b, uint32_t len);
// write bytes to I2C, if NACK is got, will return 0 as failure, this function will not send the last STOP
uint32_t i2c_send (const uint8_t *b, uint32_t len);
// use I2C write address and register idx to get bytes, return 0 as failure
uint32_t i2c_get(uint8_t addr_w, uint8_t idx, uint8_t *b, uint32_t len);
// use I2C write address and register idx to set bytes, return 0 as failure
uint32_t i2c_set(uint8_t addr_w, uint8_t idx, const uint8_t *b, uint32_t len);


#endif /* I2C_H_ */
