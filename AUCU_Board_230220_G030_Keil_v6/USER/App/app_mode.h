#ifndef  _APP_MODE_H
#define  _APP_MODE_H

#include "main.h"

#define voltage_0_10   
//#define LIN_CONTROL
//#define PWM_CONTROL

extern uint8_t mrx_data;
extern int mspeed;

void mode_standby_init(void);
void mode_standby_tick(void);

void mode_auto_init(void);
void mode_auto_tick(void);

void mode_speed_init(void);
void mode_speed_tick(void);

void mode_pressure_5m_init(void);
void mode_pressure_5m_tick(void);

void mode_pressure_7m_init(void);
void mode_pressure_7m_tick(void);

void mode_pressure_9m_init(void);
void mode_pressure_9m_tick(void);

void mode_max_power_12m_init(void);
void mode_max_power_12m_tick(void);

void mode_pwm_init(void);
void mode_pwm_tick(void);

void mode_error_init(void);
void mode_error_tick(void);

#endif
