#ifndef _APP_HANDLE_H
#define _APP_HANDLE_H

#include "stm32g0xx_hal.h"

#if 0
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

/*******************************
指示灯   档位
绿+1      1     AUTO
绿+2      2     速度档
绿+3      3     恒压 5m 档
绿+1+3    4     恒压 7m 档
绿+2+3    5     恒压 9m 档
绿+1+2+3  6     最大功率 12m 档
橙+1+2+3 PWM    PWM 调速和功率反馈
*******************************/
typedef enum work_mode_e
{
	MODE_STANDLY = 0,
  MODE_AUTO,  //1
  MODE_SPEED,	
  MODE_PRESSURE_5M,       
	MODE_PRESSURE_7M,      
	MODE_PRESSURE_9M,        
	MODE_MAX_POWER_12M,        
	MODE_PWM,       
  MODE_ERROR, 
  MODE_NULL,   
}WORK_MODE_E;

extern WORK_MODE_E  g_work_mode;
extern WORK_MODE_E  g_work_mode_ppre;

void work_mode_init(void);
void work_mode_perfrom(void);
void work_mode_switch(WORK_MODE_E State);
#endif