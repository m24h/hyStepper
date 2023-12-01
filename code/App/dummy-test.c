/*
 * test.c
 *
 * temporary test
 *
 *  Created on: 2023.11.12
 *      Author: johnny
 */
#include "app.h"

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  (void)cmdstr;
  (void)resp;

  app_resp_printf(resp, "Dummy\r\n");
 }

APP_REG_SUBMODULE("test", NULL, NULL, &cmd, "a temporay test")

