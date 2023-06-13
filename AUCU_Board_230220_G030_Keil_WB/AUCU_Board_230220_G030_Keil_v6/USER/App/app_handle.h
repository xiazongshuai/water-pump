#ifndef _APP_HANDLE_H
#define _APP_HANDLE_H

#include "stm32g0xx_hal.h"

typedef enum work_mode_e
{
	MODE_NULL = 0,    //空
  MODE_STANDLY,     //待机
  MODE_MANUAL,     //手动
  MODE_AUTO,        //自动
  MODE_ERROR,       //故障状态
}WORK_MODE_E;

extern WORK_MODE_E  g_work_mode;
extern WORK_MODE_E  g_work_mode_ppre;

void work_mode_init(void);
void work_mode_perfrom(void);
void work_mode_switch(WORK_MODE_E State);

void send_hide_function_data(void);
#endif