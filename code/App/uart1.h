/*
 * uart1.h
 *
 *  Created on: 2023.9.26
 *      Author: johnny
 */

#ifndef UART1_H_
#define UART1_H_

#include "app.h"

#include <stdarg.h>

// print to USART1
int uart1_printf  (const char * fmt, ...) __attribute__((format(__printf__, 1, 2)));
// print to USART1
int uart1_vprintf (const char * fmt, va_list va) __attribute__((format(__printf__, 1, 0)));

#endif /* UART1_H_ */
