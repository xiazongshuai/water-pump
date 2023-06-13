#include "app_standby.h"
#include "at24c02.h"
#include "usart_handle.h"
#include "app_handle.h"
#include "app_type.h"

extern uint8_t err_flag;
extern MCI_State_t msta;


void standy_mode_init(void)
{
	//停止工作
	MC_StopMotor1();
	MC_AcknowledgeFaultMotor1();  //清除故障
	
	err_flag = 0;
	target_speed = 800;
  target_pressure = 2.0;
	
//	printf("standby\r\n");
	
  //异常断电重启--读取EE里面存储的工作模式
//	ee_read_state = ee_read_data();
//	if(ee_read_state == 0) //读取失败
//	{
//	  usart_send_err(parameter_reading_err);
//	}
//	if(ee_work_mode != MODE_STANDLY)
//	{
//		global_motor_speed = ee_motor_speed;
//		global_hydraulic_pressure = ee_hydraulic_pressure/10; //扩大了10倍存储
//	  work_mode_switch(ee_work_mode);
//	}
}

void standy_mode_tick(void)
{
  if(msta != 8) //状态不是STOP
	{
	 MC_StopMotor1();
	}
	if(msta==10 || msta==11) //状态是Fault
	{
    MC_AcknowledgeFaultMotor1();  //清除故障
	}
}