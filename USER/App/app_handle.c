#include "app_handle.h"
#include "app_error.h"
#include "app_mode.h"


typedef struct work_mode_state
{
	WORK_MODE_E work_mode;
	void (*ctrinit)(void);
	void (*ctrloop)(void);
}WORK_MODE_STATE;

static WORK_MODE_STATE g_work_mode_state[]=
{
   {MODE_STANDLY,         mode_standby_init,         mode_standby_tick},
	 {MODE_AUTO,            mode_auto_init,            mode_auto_tick},
	 {MODE_SPEED,           mode_speed_init,           mode_speed_tick},
	 {MODE_PRESSURE_5M,     mode_pressure_5m_init,     mode_pressure_5m_tick},
	 {MODE_PRESSURE_7M,     mode_pressure_7m_init,     mode_pressure_7m_tick},
	 {MODE_PRESSURE_9M,     mode_pressure_9m_init,     mode_pressure_9m_tick},
	 {MODE_MAX_POWER_12M,   mode_max_power_12m_init,   mode_max_power_12m_tick},
	 {MODE_PWM,             mode_pwm_init,             mode_pwm_tick},
	 {MODE_ERROR,           mode_error_init,           mode_error_tick},
};
WORK_MODE_E    g_work_mode_ppre =  MODE_NULL;
WORK_MODE_E    g_work_mode_pre  =  MODE_NULL;
WORK_MODE_E    g_work_mode      =  MODE_STANDLY;


void work_mode_init(void)
{
  g_work_mode_ppre =  MODE_NULL;
	g_work_mode_pre  =  MODE_NULL;
	g_work_mode      =  MODE_STANDLY;
}

void work_mode_perfrom(void)
{
  for(uint8_t i=0; i<sizeof(g_work_mode_state)/sizeof(g_work_mode_state[0]); i++)
  {
     if(g_work_mode == g_work_mode_state[i].work_mode)
     {
			if(g_work_mode_pre != g_work_mode)
			{
				g_work_mode_state[i].ctrinit();
				g_work_mode_ppre = g_work_mode_pre;
				g_work_mode_pre = g_work_mode;
			}
		 g_work_mode_state[i].ctrloop();
     break;
    }
  }
}

void work_mode_switch(WORK_MODE_E State)
{
	if (g_work_mode != State)
	{
		g_work_mode = State;
	}
}

