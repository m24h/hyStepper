/*
 * conf.h
 *
 *  Created on: 2023.11.17
 *      Author: johnny
 */

#ifndef CONF_H_
#define CONF_H_

#define CONF_MOTOR_POLES  50U /* number of magnetic poles of the driven motor, normally 50 for 200 steps motor (1.8 deg 2-line-4-phase motor)*/
#define CONF_MSTEP_EXP    5U  /* means 1<<6=64 steps in a cycle of motor pole, or in 4 full steps of motor */
#define CONF_CURRENT_MAX  0xD0000000U          /* about 1.2A */

#define CONF_UART1_BAUDRATE  115200U /* baudrate of uart1 */

#define CONF_VCC        3300 /* designed voltage of MCU power, in mV */
#define CONF_TEMP_HIGH 70000 /* it will cause error if the temperature reaches this, in 0.001 Degree Celsius */
#define CONF_TEMP_OK   60000 /* it will revoke error if the temperature falls down to this, in 0.001 Degree Celsius */

// selection of one among several same function modules

#define ENCODER_MT6701
// for MT6701, it's accuracy becomes very pool if power lost, real re-power is needed
#define CONF_PVD_NO_UP

//#define ENCODER_AS5600
//#define ENCODER_MT6701AC
//#define ENCODER_MT6701AK

#define INPUTTER_RS485
// known issue : if baudrate exceeds, bit error occurs due to communication with the encoder
#define CONF_RS485_DEF_BAUDRATE 250000

//#define INPUTTER_DIRPUL

#endif /* CONF_H_ */

// resource usage:
// PA13:SW_DIO PA14:SW_CLK --- SWD
// PF0:USART1_RX PF1:USART1_TX --- Config
// PA6:A_current/TIM16_CH1 PA7:B_current/TIM17_CH1 --- PWM Current
// PB0:A+ PA1:A- PB1:B+ PB2:B- --- driver
// PA4:SSI_CSN/AS5600_DIR/Z PA2:I2C_SDA(SOFT)/SSI_DO/TIM3_CH1(A) PA5:I2C_SCL(SOFT)/SSI_SCL/TIM3_CH2(B)
// PA0:EN/USART2_TX PA3:DIR/TIM1_CH1/USART2_RX PB3:PUL/TIM1_CH2/RS_485_DIR --- inputter
// PF4:LED+ (PB6)
// DMA1/ch3 ---- USART1
// USART2 DMA/ch2 --- RS485 inputter
// TIM1 --- inputter
// TIM16 TIM17 --- driver/pwm
// TIM3 --- encoder
//
// IRQ Priority:
// time and PVD : 0
// driving : 1
// encoder and inputter : 2
// usart1 : 3
//
