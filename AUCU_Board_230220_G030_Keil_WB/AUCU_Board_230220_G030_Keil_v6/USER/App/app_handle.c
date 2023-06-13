#include "app_handle.h"
#include "app_standby.h"
#include "app_auto.h"
#include "app_error.h"
#include "app_manual.h"

typedef struct work_mode_state
{
	WORK_MODE_E work_mode;
	void (*ctrinit)(void);
	void (*ctrloop)(void);
}WORK_MODE_STATE;

static WORK_MODE_STATE g_work_mode_state[]=
{
   {MODE_STANDLY, standy_mode_init,     standy_mode_tick},
	 {MODE_AUTO,    auto_mode_init,       auto_mode_tick},
	 {MODE_MANUAL,  manual_mode_init,     manual_mode_tick},
	 {MODE_ERROR,   error_mode_init,      error_mode_tick},
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
