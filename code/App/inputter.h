/*
 * inputter.h
 *
 *  Created on: 2023.10.2
 *      Author: johnny
 */

#ifndef INPUTTER_H_
#define INPUTTER_H_

#include "app.h"

// return current position, it's not absolute but relative, and rolling changed,
int32_t  inputter_get(void);

#endif /* INPUTTER_H_ */
