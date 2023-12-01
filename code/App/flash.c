/*
 * flash.c
 *  only for MCU AIR001
 *  Created on: 2023.10.17
 *      Author: johnny
 */

#include "flash.h"

#ifndef APP_CLK_HSI
#error APP_CLK_HSI is not defined
#endif

#define PAGE_SIZE     128U /* must be power of 2 */
#define SECTOR_SIZE  4096U /* must be power of 2 */

static void init(void)
{
#if APP_CLK_HSI==24000000U
  WRITE_REG(FLASH->TS0, *(uint32_t*)0x1FFF0F6C);
  WRITE_REG(FLASH->TS1, *(uint32_t*)0x1FFF0F6C);
  WRITE_REG(FLASH->TS2P, *(uint32_t*)0x1FFF0F70);
  WRITE_REG(FLASH->TPS3, *(uint32_t*)0x1FFF0F70);
  WRITE_REG(FLASH->TS3, *(uint32_t*)0x1FFF0F6C);
  WRITE_REG(FLASH->PERTPE, *(uint32_t*)0x1FFF0F74);
  WRITE_REG(FLASH->SMERTPE, *(uint32_t*)0x1FFF0F78);
  WRITE_REG(FLASH->PRGTPE, *(uint32_t*)0x1FFF0F7C);
  WRITE_REG(FLASH->PRETPE, *(uint32_t*)0x1FFF0F7C);
#elif APP_CLK_HSI==22120000U
  WRITE_REG(FLASH->TS0, *(uint32_t*)0x1FFF0F58);
  WRITE_REG(FLASH->TS1, *(uint32_t*)0x1FFF0F58);
  WRITE_REG(FLASH->TS2P, *(uint32_t*)0x1FFF0F5C);
  WRITE_REG(FLASH->TSS3, *(uint32_t*)0x1FFF0F5C);
  WRITE_REG(FLASH->TS3, *(uint32_t*)0x1FFF0F58);
  WRITE_REG(FLASH->PERTPE, *(uint32_t*)0x1FFF0F60);
  WRITE_REG(FLASH->SMERTPE, *(uint32_t*)0x1FFF0F64);
  WRITE_REG(FLASH->PRGTPE, *(uint32_t*)0x1FFF0F68);
  WRITE_REG(FLASH->PRETPE, *(uint32_t*)0x1FFF0F68);
#elif APP_CLK_HSI==16000000U
  WRITE_REG(FLASH->TS0, *(uint32_t*)0x1FFF0F44);
  WRITE_REG(FLASH->TS1, *(uint32_t*)0x1FFF0F44);
  WRITE_REG(FLASH->TS2P, *(uint32_t*)0x1FFF0F48);
  WRITE_REG(FLASH->TPS3, *(uint32_t*)0x1FFF0F48);
  WRITE_REG(FLASH->TS3, *(uint32_t*)0x1FFF0F44);
  WRITE_REG(FLASH->PERTPE, *(uint32_t*)0x1FFF0F4C);
  WRITE_REG(FLASH->SMERTPE, *(uint32_t*)0x1FFF0F50);
  WRITE_REG(FLASH->PRGTPE, *(uint32_t*)0x1FFF0F54);
  WRITE_REG(FLASH->PRETPE, *(uint32_t*)0x1FFF0F54);
#elif APP_CLK_HSI==8000000U
  WRITE_REG(FLASH->TS0, *(uint32_t*)0x1FFF0F30);
  WRITE_REG(FLASH->TS1, *(uint32_t*)0x1FFF0F30);
  WRITE_REG(FLASH->TS2P, *(uint32_t*)0x1FFF0F34);
  WRITE_REG(FLASH->TPS3, *(uint32_t*)0x1FFF0F34);
  WRITE_REG(FLASH->TS3, *(uint32_t*)0x1FFF0F30);
  WRITE_REG(FLASH->PERTPE, *(uint32_t*)0x1FFF0F38);
  WRITE_REG(FLASH->SMERTPE, *(uint32_t*)0x1FFF0F3C);
  WRITE_REG(FLASH->PRGTPE, *(uint32_t*)0x1FFF0F40);
  WRITE_REG(FLASH->PRETPE, *(uint32_t*)0x1FFF0F40);
#elif APP_CLK_HSI==4000000U
  WRITE_REG(FLASH->TS0, *(uint32_t*)0x1FFF0F1C);
  WRITE_REG(FLASH->TS1, *(uint32_t*)0x1FFF0F1C);
  WRITE_REG(FLASH->TS2P, *(uint32_t*)0x1FFF0F20);
  WRITE_REG(FLASH->TPS3, *(uint32_t*)0x1FFF0F20);
  WRITE_REG(FLASH->TS3, *(uint32_t*)0x1FFF0F1C);
  WRITE_REG(FLASH->PERTPE, *(uint32_t*)0x1FFF0F24);
  WRITE_REG(FLASH->SMERTPE, *(uint32_t*)0x1FFF0F28);
  WRITE_REG(FLASH->PRGTPE, *(uint32_t*)0x1FFF0F2C);
  WRITE_REG(FLASH->PRETPE, *(uint32_t*)0x1FFF0F2C);
#else
#error Not supported APP_CLK_HSI
#endif
}

uint32_t flash_unlock(void)
{
  if (READ_BIT(FLASH->SR, FLASH_SR_BSY))
    return 0U;

  if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    __NOP();
    __NOP();
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) // failed
      return 0U;
  }
  return 1U;
}

uint32_t flash_lock()
{
  if (READ_BIT(FLASH->SR, FLASH_SR_BSY))
    return 0;

  SET_BIT(FLASH->CR, FLASH_CR_LOCK);
  __NOP();
  __NOP();
  return READ_BIT(FLASH->CR, FLASH_CR_LOCK)?1U:0U;
}

uint32_t flash_erase_page(void *addr, uint32_t size)
{
  if (READ_BIT(FLASH->SR, FLASH_SR_BSY))
    return 0;

  uint32_t from=(uint32_t)addr & (~(PAGE_SIZE-1U));
  uint32_t to=((uint32_t)addr+size-1U) & (~(PAGE_SIZE-1U));

  SET_BIT(FLASH->CR, FLASH_CR_PER | FLASH_CR_EOPIE);
  while (from<=to) {
    CLEAR_BIT(FLASH->SR, FLASH_SR_WRPERR | FLASH_SR_EOP);
    *(uint32_t *)from=0xffffffff;
    __NOP();
    __NOP();
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY));
    if (READ_BIT(FLASH->SR, FLASH_SR_WRPERR) || !READ_BIT(FLASH->SR, FLASH_SR_EOP)) { // failed
      CLEAR_BIT(FLASH->CR, FLASH_CR_PER | FLASH_CR_EOPIE);
      CLEAR_BIT(FLASH->SR, FLASH_SR_EOP | FLASH_SR_WRPERR);
      return 0U;
    }
    from+=PAGE_SIZE;
  }
  CLEAR_BIT(FLASH->CR, FLASH_CR_PER | FLASH_CR_EOPIE);
  CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);
  return 1U;
}

uint32_t flash_erase_sector(void *addr, uint32_t size)
{
  if (READ_BIT(FLASH->SR, FLASH_SR_BSY))
    return 0;

  uint32_t from=(uint32_t)addr & (~(SECTOR_SIZE-1U));
  uint32_t to=((uint32_t)addr+size-1U) & (~(SECTOR_SIZE-1U));

  SET_BIT(FLASH->CR, FLASH_CR_SER | FLASH_CR_EOPIE);
  while (from<=to) {
    CLEAR_BIT(FLASH->SR, FLASH_SR_WRPERR | FLASH_SR_EOP);
    *(uint32_t *)from=0xffffffff;
    __NOP();
    __NOP();
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY));
    if (READ_BIT(FLASH->SR, FLASH_SR_WRPERR) || !READ_BIT(FLASH->SR, FLASH_SR_EOP)) { // failed
      CLEAR_BIT(FLASH->CR, FLASH_CR_SER | FLASH_CR_EOPIE);
      CLEAR_BIT(FLASH->SR, FLASH_SR_EOP | FLASH_SR_WRPERR);
      return 0;
    }
    from+=SECTOR_SIZE;
  }

  CLEAR_BIT(FLASH->CR, FLASH_CR_SER | FLASH_CR_EOPIE);
  CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);
  return 1U;
}

uint32_t flash_program(void *addr, const void *data, uint32_t len)
{
  if (READ_BIT(FLASH->SR, FLASH_SR_BSY))
    return 0U;

  if (((uint32_t)addr&3)!=0 || ((uint32_t)data&3)!=0)
    return 0U;

  SET_BIT(FLASH->CR, FLASH_CR_PG | FLASH_CR_EOPIE);
  for (len=(len+3U)>>2;len;len--) {
    if (((uint32_t)addr & (PAGE_SIZE-1U))==(PAGE_SIZE-4) || len==1) { // page end or all end
      SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
      *(uint32_t*)addr=*(uint32_t*)data;
      while (READ_BIT(FLASH->SR, FLASH_SR_BSY));
      if (READ_BIT(FLASH->SR, FLASH_SR_WRPERR) || !READ_BIT(FLASH->SR, FLASH_SR_EOP)) { // failed
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG | FLASH_CR_EOPIE);
        CLEAR_BIT(FLASH->SR, FLASH_SR_EOP | FLASH_SR_WRPERR);
        return 0U;
      }
    } else
      *(uint32_t*)addr=*(uint32_t*)data;
    addr=(void*)((uint32_t*)addr+1);
    data=(const void *)((uint32_t*)data+1);
  }
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG | FLASH_CR_EOPIE);
  CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);
  return 1U;
}

uint32_t flash_write(void *addr, const void *data, uint32_t len)
{
    return (flash_unlock()
       && flash_erase_page(addr, len)
       && flash_program(addr, data, len)
       && flash_lock())?1U:0U;
}

APP_REG_INIT(&init)
