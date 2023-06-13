#ifndef LED_KEY_H
#define LED_KEY_H

#include "main.h"

typedef struct KEY
{
  int down_time;
	int up_time;
	uint8_t down_flag;
	uint8_t up_flag;
	uint8_t state;
}KEY;
extern KEY key;

typedef enum 
{
//	null_err=0,
	overcurrent_err,      //过流
	startup_err,          //启动失败
	locked_rotor_err,     //堵转故障
	undervoltage_err,     //欠压故障
	overtemperature_err,  //过温故障
	overvoltage_err,      //过压故障
	mc_err,               //电机库自带故障
}ERROR_ENUM;

extern uint8_t MOTOR_ERROR_INFO;

typedef enum 
{
  null_func=0,
  auto_func,
	speed_func,
	pressure_5m_func,
	pressure_7m_func,
	pressure_9m_func,
	max_power_12m_func,
	pwm_func,
}FUNC_ENUM;

void exe_key_func(void); 
uint8_t get_key_state(void);
void led_clear(void);
void led_show_err(ERROR_ENUM motor_err);
void led_show_function(FUNC_ENUM func_type);

#endif