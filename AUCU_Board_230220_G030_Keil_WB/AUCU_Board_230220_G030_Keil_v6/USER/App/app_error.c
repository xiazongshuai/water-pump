#include "app_error.h"
#include "app_info.h"
#include "mc_config.h"
#include "usart_handle.h"
#include "app_handle.h"
#include "app_type.h"
#include <math.h>


uint16_t BusVoltage;
uint16_t BusVoltage_LTime;
uint16_t BusVoltage_HTime;
uint8_t IPM_ErrFlag;

uint16_t Loss_PhaseTime;

uint16_t LockedRotor_Time; //��ת����ʱ��
uint16_t LockedRotor_Start_duration; //��ת��೤ʱ������һ��
uint16_t LockedRotor_Start_Time;  //��ת���������Ĵ���

uint16_t Re_Start_duration; //�೤ʱ������һ��
uint16_t Re_Start_Time;  //���������Ĵ���

uint8_t EeSave_Flag;

extern MCI_State_t msta;

int myabs(int16_t num)
{
  if(num>0) return num;
	else return -num;
}

void error_mode_init(void)
{
   MC_StopMotor1();
}

void error_check_init(void)
{
	
}

void error_check(void)
{
  //������ȡ�����Ϣ
	
	//��ѹ����
	BusVoltage = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
	if(BusVoltage < 135)
	{
		BusVoltage_LTime++;
		if(BusVoltage_LTime > 3000)
		{
			BusVoltage_LTime = 0;
		  ERR_INFO_SEND |= 1<<voltage_err;
		}
	}
	
	if(BusVoltage > 290)
	{
		BusVoltage_HTime++;
		if(BusVoltage_HTime > 3000)
		{
			BusVoltage_HTime = 0;
		  ERR_INFO_SEND |= 1<<voltage_err;
		}
	}
	
	if(BusVoltage > 140 && BusVoltage < 290 && (ERR_INFO_SEND&(1<<voltage_err)) != 0) //��ѹ�ɵͻָ�����
	{
		ERR_INFO_SEND &= ~(1<<voltage_err);  //���ϻָ�
	  work_mode_switch(g_work_mode_ppre);
	}
	
	if(BusVoltage < 265 && BusVoltage > 135 && (ERR_INFO_SEND&(1<<voltage_err)) != 0) //��ѹ�ɸ߻ָ�����
	{
		ERR_INFO_SEND &= ~(1<<voltage_err);
	  work_mode_switch(g_work_mode_ppre);
	}
	
	//IPM����
	if(IPM_ErrFlag == 1)
	{
		IPM_ErrFlag = 0;
		ERR_INFO_SEND |= 1<<IPM_voltage_err;
	}
	
	//����������  XXXXXX
	
	//ȱ��
//	if(myabs(PWM_Handle_M1._Super.Ia) ==0 || myabs(PWM_Handle_M1._Super.Ib) ==0 || myabs(PWM_Handle_M1._Super.Ic) ==0)
//	{
//	  if(myabs(PWM_Handle_M1._Super.Ia - PWM_Handle_M1._Super.Ib) <300 ||
//		   myabs(PWM_Handle_M1._Super.Ia - PWM_Handle_M1._Super.Ic) <300 ||
//			 myabs(PWM_Handle_M1._Super.Ib - PWM_Handle_M1._Super.Ic) <300 )
//		{
//			Loss_PhaseTime++;
//			if(Loss_PhaseTime > 2000)
//			{
//				Loss_PhaseTime = 0;
//			  ERR_INFO_SEND |= 1<<motor_phase_loss_err;
//			}
//		}
//		else
//		{
//		   Loss_PhaseTime = 0;
//			 ERR_INFO_SEND &= ~(1<<motor_phase_loss_err);
//		}
//	}
	
	if(msta == 4) //�˶������Ż�ȥ��ȡƫ�õ�ѹ
	{
	 if(PWM_Handle_M1.PhaseAOffset>AD_bias_upvoltage || PWM_Handle_M1.PhaseBOffset>AD_bias_upvoltage || PWM_Handle_M1.PhaseCOffset>AD_bias_upvoltage ||
		 PWM_Handle_M1.PhaseAOffset<AD_bias_lowvoltage || PWM_Handle_M1.PhaseBOffset<AD_bias_lowvoltage || PWM_Handle_M1.PhaseCOffset<AD_bias_lowvoltage)
	{
		ERR_INFO_SEND |= 1<<AD_bias_voltage_err;
	}
	else
  {
		ERR_INFO_SEND &= ~(1<<AD_bias_voltage_err);
  }
 }
	
//	//��ת  1985=>1A   ������ʱ��(����)
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
	
	if(ERR_INFO_SEND != 0  || ERR_INFO_REC != 0)  //��塢������ֹ���
	{
	  work_mode_switch(MODE_ERROR);
	}
}


void error_mode_tick(void)
{
	if(msta != 8) //״̬����STOP
	{
	 MC_StopMotor1();
	}
	if(ERR_INFO_SEND == 0  || ERR_INFO_REC == 0)
	{
		work_mode_switch(MODE_STANDLY);
	}
	
//	if((ERR_INFO_SEND & (1<<locked_rotor_err)) != 0) //��ת
//	{
//	  LockedRotor_Start_duration++;
//		if(LockedRotor_Start_duration>30*1000) //ÿ30������һ��
//		{
//			LockedRotor_Start_Time++;
//			if(LockedRotor_Start_Time <= 5)
//			{
//		   work_mode_switch(g_work_mode_ppre);  //�ָ�����ǰ��״̬
//			}
//		}
//	}
	
//	if((ERR_INFO_SEND & (1<<startup_err)) != 0 || (ERR_INFO_SEND & (1<<speedfbk_err)) != 0)
//	{
//	  Re_Start_duration++;
//		if(Re_Start_duration>30*1000) //ÿ30������һ��
//		{
//			Re_Start_Time++;
//			if(Re_Start_Time <= 5)
//			{
//		   work_mode_switch(g_work_mode_ppre);  //�ָ�����ǰ��״̬
//			}
//		}
//	}
}