/*
 * flash.h
 *
 *  Created on: 2023.10.17
 *      Author: johnny
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "app.h"

// flash is busy if returned value is not zero
static inline __attribute__((always_inline)) uint32_t flash_is_busy(void) {return READ_BIT(FLASH->SR, FLASH_SR_BSY)?1:0;}
// OK if returned value is not zero
uint32_t flash_unlock(void);
// OK if returned value is not zero
uint32_t flash_lock(void);
// flash is locked if returned value is not zero
static inline __attribute__((always_inline)) uint32_t flash_is_locked(void) {return READ_BIT(FLASH->CR, FLASH_CR_LOCK)?1:0;}
// OK if returned value is not zero
uint32_t flash_erase_page(void *addr, uint32_t size);
// OK if returned value is not zero
uint32_t flash_erase_sector(void *addr, uint32_t size);
// OK if returned value is not zero, both "addr" and "data" must be 32bit aligned
uint32_t flash_program(void *addr, const void *data, uint32_t len);
// unlock, erase, then program, lock again
uint32_t flash_write(void *addr, const void *data, uint32_t len);

#endif /* FLASH_H_ */
