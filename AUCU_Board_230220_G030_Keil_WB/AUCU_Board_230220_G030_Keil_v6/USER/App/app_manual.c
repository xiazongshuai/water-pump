#include <stdio.h>
#include "app_manual.h"
#include "app_type.h"
#include "usart_handle.h"

uint16_t tmep_speed;
uint16_t work_manual_time;

//手动控制模式
void manual_mode_init(void)
{
	tmep_speed = 0;
	work_manual_time = 0;
//	printf("manual\r\n");
}

void manual_mode_tick(void)
{
	work_manual_time++;
	if(target_speed != tmep_speed)
	{
	  tmep_speed = target_speed;
//		printf("target_speed = %d\r\n",target_speed);
		MC_ProgramSpeedRampMotor1(target_speed/6,1000);
	  MC_StartMotor1();
	}

	if(work_manual_time > 500)
	{
	  work_manual_time = 0;
	  int16_t tmp = MC_GetMecSpeedAverageMotor1();
	  actual_motor_speed = tmp*6;  //RPM 转/分
//		printf("speed = %d\r\n",actual_motor_speed);
	}
}



