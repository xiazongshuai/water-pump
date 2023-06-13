#include "app_error.h"
#include "mc_config.h"
#include "usart_handle.h"
#include "app_handle.h"
#include "led_key.h"
#include "adc_cin.h"

 uint16_t BusVoltage;
static uint16_t BusVoltage_LTime;
static uint16_t BusVoltage_HTime;

static uint16_t Temp_Time;
signed char  VTS_Temperature;

static uint16_t LockedRotor_Time; 

extern MCI_State_t msta;
float vts;
//int myabs(int16_t num)
//{
//  if(num>0) return num;
//	else return -num;
//}


void error_check_init(void)
{
	
}

void error_check(void)
{
	BusVoltage = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
	BusVoltage = BusVoltage/1.414;
	if(BusVoltage < 135)
	{
		BusVoltage_LTime++;
		if(BusVoltage_LTime > 2000)
		{
			BusVoltage_LTime = 0;
		  MOTOR_ERROR_INFO |= 1<<undervoltage_err;
		}
	}
	else
	{
	  BusVoltage_LTime = 0;
	}
	
	if(BusVoltage > 295) //295V/2S
	{
		BusVoltage_HTime++;
		if(BusVoltage_HTime > 2000)
		{
			BusVoltage_HTime = 0;
		  MOTOR_ERROR_INFO |= 1<<overvoltage_err;
		}
	}
	else
	{
	  BusVoltage_HTime = 0;
	}
	
	if(BusVoltage > 140 && BusVoltage < 290 && (MOTOR_ERROR_INFO&(1<<undervoltage_err)) != 0) //电压由低恢复正常--需要自动恢复运行
	{
		MOTOR_ERROR_INFO &= ~(1<<undervoltage_err);  //故障恢复
	  work_mode_switch(g_work_mode_ppre);
	}
	
	if(BusVoltage < 265 && BusVoltage > 135 && (MOTOR_ERROR_INFO&(1<<overvoltage_err)) != 0) //电压由高恢复正常
	{
		MOTOR_ERROR_INFO &= ~(1<<overvoltage_err);
	  work_mode_switch(g_work_mode_ppre);
	}
	
	
	//过流故障
	if(GET_ADC_CIN_Digital() > 26809) //1.35A*0.1*10*19859 = 26809.65
	{
		R3_1_OVERCURRENT_IRQHandler(&PWM_Handle_M1);
	  MOTOR_ERROR_INFO |= (1<<overcurrent_err);
	}
	
	//启动故障
	if((mccurrent_err & MC_START_UP) !=0 )
	{
	  MOTOR_ERROR_INFO |= (1<<startup_err);
	}
	
	//电机库自带的其他故障
	if((mccurrent_err & (~MC_START_UP)) !=0 )
	{
	  MOTOR_ERROR_INFO |= (1<<mc_err);
	}
	
	//过温故障--需要自动恢复运行
	float vts_temp = GET_ADC_VTS_Voltage();
	VTS_Temperature = vts_temp*60 -21;
	if(VTS_Temperature > 60)
	{
		Temp_Time++;
		if(Temp_Time>10)
		{
			Temp_Time = 0;
	    MOTOR_ERROR_INFO |= (1<<overtemperature_err);
		}
	}
	else if(VTS_Temperature < 60 )
	{
		Temp_Time = 0;
		if((MOTOR_ERROR_INFO & (1<<overtemperature_err))!= 0)
		{
	  MOTOR_ERROR_INFO &= ~(1<<overtemperature_err);
		work_mode_switch(g_work_mode_ppre);
		}
	}
	
//	//堵转  1985=>1A   电流、时间(待定)
//	if(myabs(PWM_Handle_M1._Super.Ia) > 1985*7 || myabs(PWM_Handle_M1._Super.Ib) > 1985*7 || myabs(PWM_Handle_M1._Super.Ic) > 1985*7)
//	{
//		LockedRotor_Time++; 
//		if(LockedRotor_Time > 3000)
//		{
//		 LockedRotor_Time = 0;
//		 ERR_INFO_SEND |= 1<<locked_rotor_err;
//		}
//	}
//	else
//	{
//		LockedRotor_Time = 0;
//		LockedRotor_Start_Time = 0;
//	  ERR_INFO_SEND &= ~(1<<locked_rotor_err);
//	}
	
	if(MOTOR_ERROR_INFO!=0)// && g_work_mode!=MODE_STANDLY) 
	{
	  work_mode_switch(MODE_ERROR);
	}
}


void error_mode_init(void)
{
   MC_StopMotor1();
	 led_clear();
	 HAL_Delay(300);
	
	if(MOTOR_ERROR_INFO & 0x01)
	 led_show_err(overcurrent_err);
	
	else if(MOTOR_ERROR_INFO & 0x02)
	 led_show_err(startup_err);
	
	else if(MOTOR_ERROR_INFO & 0x04)
	 led_show_err(locked_rotor_err);
	
	 else if(MOTOR_ERROR_INFO & 0x08)
	 led_show_err(undervoltage_err);
	
	else if(MOTOR_ERROR_INFO & 0x10)
	 led_show_err(overtemperature_err);
	
	else if(MOTOR_ERROR_INFO & 0x20)
	 led_show_err(overvoltage_err);
	
	else if(MOTOR_ERROR_INFO & 0x40) 
	 led_show_err(mc_err);
}

void error_mode_tick(void)
{
	if(msta != STOP)
	{
	 MC_StopMotor1();
	}
	if(MOTOR_ERROR_INFO == 0)
	{
		//work_mode_switch(MODE_STANDLY);
	}
}