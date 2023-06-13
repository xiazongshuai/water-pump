#ifndef _APP_HANDLE_H
#define _APP_HANDLE_H

#include "stm32g0xx_hal.h"

#if 0
typedef enum work_mode_e
{
	MODE_NULL = 0,    //��
  MODE_STANDLY,     //����
  MODE_MANUAL,     //�ֶ�
  MODE_AUTO,        //�Զ�
  MODE_ERROR,       //����״̬
}WORK_MODE_E;

extern WORK_MODE_E  g_work_mode;
extern WORK_MODE_E  g_work_mode_ppre;

void work_mode_init(void);
void work_mode_perfrom(void);
void work_mode_switch(WORK_MODE_E State);

void send_hide_function_data(void);
#endif

/*******************************
ָʾ��   ��λ
��+1      1     AUTO
��+2      2     �ٶȵ�
��+3      3     ��ѹ 5m ��
��+1+3    4     ��ѹ 7m ��
��+2+3    5     ��ѹ 9m ��
��+1+2+3  6     ����� 12m ��
��+1+2+3 PWM    PWM ���ٺ͹��ʷ���
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