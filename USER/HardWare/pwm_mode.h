#ifndef _PWM_MODE_H
#define _PWM_MODE_H

#include "main.h"

#define  pwm_max_speed  4500
#define  pwm_min_speed  800

extern TIM_HandleTypeDef  htim16;

extern  int rise1_cnt;
extern  int rise2_cnt;
extern  int fall_cnt;

extern float pwm_arr;
extern float pwm_ccr;	
extern int pwm_duty;
extern uint8_t pwm_out_config;

void pwm_mode_init(void);
int get_duty_period(void);
void tim_pwminput_config(void);
void pwm_speed_control(uint8_t duty_data);
void pwm_output_config(void);
#endif