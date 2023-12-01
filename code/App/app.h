/*
 * app.h
 *
 *  a common platform for submodule, note:
 *
 *  error masks are limited (32 bits including system error bits), register with caution
 *  it is impossible to strictly enforce the rules, but submodules should abide by them (some value is readonly)
 *  compiler is gcc (GNU C11), toolchain is "GNU Tools for STM32"
 *  this app is only for the MCU AIR001, producer: openluat.com
 *  platform takes over everything of systick, don't touch it
 *  disable/enable IRQ only for atomicity operation, don't keep IRQ disabled in a long time, use app_irq_disable()/app_irq_resume()
 *
 *  Created on: 2023.9.24
 *      Author: johnny
 */

#ifndef APP_H_
#define APP_H_

// useful almost every where
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "conf.h"

#include "air001xx.h"
#include "air001_dev.h"
#include "system_air001xx.h"
#include "core_cm0plus.h"
#include "cmsis_gcc.h"

#define APP_MCU_AIR001  /* only for this MCU */

// clocks, fixed, do not change them
#define APP_CLK_HSI   24000000U
#define APP_CLK_SYS   48000000U
#define APP_CLK_AHB   48000000U
#define APP_CLK_APB1  48000000U

// UID, readonly
extern const char * const        app_uid;

// current error status, bits are defined in APP_ERROR_xxx, using atomic operations to modify it
extern volatile uint32_t         app_error;
// predefined system error
#define APP_ERROR_FATAL          0x80000000U /* meet a fatal error, which is impossible to recover it */
#define APP_ERROR_PVD            0x40000000U /* power down */
#define APP_ERROR_INIT           0x20000000U /* platform is not ready */
#define APP_ERROR_SOME           0x10000000U /* some unknown failure */

// register an error bitmask, with caution, totally only 28 bitmasks are available, and "name" is the "qualifier" uint32_t variable name of error
#define APP_REG_ERROR(qualifier, name, text) \
  static __attribute__((unused,used,section(".app_error"))) const char * const _app_error_##name=text; \
  qualifier uint32_t name; \
  static void  __attribute__((constructor)) _app_error_init_##name (void) {extern const char * const app_error_start; name=1U<<(&_app_error_##name - &app_error_start);} // put into section .init_array

// cmd response handler type
typedef void (*app_cmd_res_t) (const char *, uint32_t);

// submodule should provide these informations
typedef struct {
  const char * name;
  void (*init) (void);
  void (*loop) (void);
  void (*cmd)  (char *, app_cmd_res_t); // Interact with human, the string parameter's content may be changed during process
  const char * help;
} app_submodule_t;

// register a submodule, it can be used only once in a source file
#define APP_REG_SUBMODULE(_name, _init, _loop, _cmd, _help) \
  static __attribute__((unused,used,section(".app_submodule"))) const app_submodule_t _app_submodle_reg={.name=_name, .init=_init, .loop=_loop, .cmd=_cmd, .help=_help};

// register an initialization function, it will run before any submodule initialization, it can be used only once in a source file
#define APP_REG_INIT(func) \
  static __attribute__((unused,used,section(".app_init"))) void (* const _app_init_reg)(void)=(func);

typedef enum {
  APP_SIG_NONE=0,
  APP_SIG_INT=3
} app_shell_sig_t;

// read only, if shell receives ^C
extern volatile app_shell_sig_t app_shell_sig;
// send text commands to platform and submodules, and get the response
void app_cmd(char *cmd, app_cmd_res_t resp);
// a simple shell, input a char, repsonse using (*resp)()
void app_shell(char c, app_cmd_res_t resp);
// use this to send response in printf style
void app_resp_printf(app_cmd_res_t resp, const char * fmt, ...) __attribute__((format(__printf__, 2, 3)));

// memory remained, not used by stack nor heap
extern volatile uint32_t    app_stack_free;
// increased every millisecond, should read it only, the repetition period is nearly 49 days
extern volatile uint32_t    app_cnt_1ms;
// how many main loops are done in last 1ms, should read it only
extern volatile uint32_t    app_loops_1ms;
// how many clocks elapsed from systick started
extern uint64_t             app_cnt_clk(void);

// delay clocks
void app_delay_clk(uint32_t delay);

// return an integer from string, or the default value if failure/out of range, and spaces (including " " "\t" "\r" "\n") is skipped
// note: it will use strtok_r() to modify "*str"
// note: to scan a token, just using strtok_r()
// note: it does not check if "str" is valid
int32_t app_sscan_int(char **str, int32_t def, int32_t min, int32_t max);
// skip spaces (including " " "\t" "\r" "\n"), then return 1 if the "chk_str" exists and is skipped
// note: it does not check if "str" is valid
uint32_t app_sscan_chk(char **str, const char * chk_str);

// submodules should use these to print informations
int app_printf (const char * fmt, ...)  __attribute__((format(__printf__, 1, 2)));
#define printf  app_printf

// submodules use these to enter/exit critical region
extern volatile uint32_t app_irq_disabled;
static inline __attribute__((always_inline)) void app_irq_disable(void) {__disable_irq(); app_irq_disabled++;}
static inline __attribute__((always_inline)) void app_irq_resume(void) {if (!(--app_irq_disabled)) __enable_irq(); }

// reset whole platform
void app_reset(void);

#endif /* APP_H_ */


