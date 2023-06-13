#ifndef _APP_ERROR_H
#define _APP_ERROR_H

#include "stm32g0xx_hal.h"

void error_mode_init(void);
void error_mode_tick(void);
void error_check(void);
#endif