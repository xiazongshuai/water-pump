#ifndef _APP_STANDBY_H
#define _APP_STANDBY_H

#include "stm32g0xx_hal.h"

void restore_factory_settings(void);
void standy_mode_init(void);
void standy_mode_tick(void);

#endif