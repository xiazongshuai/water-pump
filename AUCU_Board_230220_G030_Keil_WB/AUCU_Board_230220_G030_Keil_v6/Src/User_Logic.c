/**
  ******************************************************************************
  * @file    User_Logic.c 
  * @author  Dennis GE - MMS MC Team
  * @version 4.3.0
  * @date    28-May-2014 10:45
  * @brief   Inverter Aircon Project
  * @Modified by dsg based on Ver 4.3 FOC Library 
  *******************************************************************************/ 

#include "User_Logic.h"
#include "stdlib.h" // For abs function
//#include "Timer.h"
#include "MC_type.h"
#include "MC_Math.h"
//#include "PFCApplication.h"  //+ by victor 20171019
//#include "ipm_temp_sense.h"
//#include "ipm2_temp_sense.h"
#include "mc_tasks.h"
#include "mc_config.h"
//#include "VariableParams.h"
#include "mc_api.h"
#include "motorcontrol.h"
#include "mc_interface.h"
//#include "LSI_IWDG.h"
//#include "userAPI.h"
#include "main.h"

S_Lib_DUBUG		lib_debug;

//volatile int16_t UARTSpeedCMD_M1_rpm ;//rpm
volatile int16_t UARTSpeedCMD_M1 ;	//#SPEED_UNIT, U_01HZ
volatile int16_t UARTSpeedCMDLast_M1 ;
//volatile int16_t PowerCMD_M1=0; //added by zn 20191225

volatile int16_t Acc_M1 = 10;		//#SPEED_UNIT, U_01HZ//add or decrease 60rpm/s
uint8_t M1EnableStartFlag = false;

void UART_Emulator_Debug(void)
{
  int16_t current_speed_ref;
  int16_t i_out; 
  int16_t speed_acc;
  int32_t duration_time;
  int16_t current_speed_error;
  
  /*Process motor 1*/
	UARTSpeedCMD_M1 = (int16_t)((float)lib_debug.UARTSpeedCMD_M1_rpm / 6.0);
	
//	UARTSpeedCMD_M1 = (int16_t)((float)UARTSpeedCMD_M1_rpm / 6.0);
  if(UARTSpeedCMD_M1 != UARTSpeedCMDLast_M1)
  {
    current_speed_ref = MC_GetMecSpeedReferenceMotor1();
    current_speed_error = UARTSpeedCMD_M1 - current_speed_ref;
    UARTSpeedCMDLast_M1 = UARTSpeedCMD_M1;
    speed_acc = Acc_M1;

    /*duration time calculate*/
    if(UARTSpeedCMD_M1 != 0)	// can be modified to: if(UARTEmulatorSpeedCMD_M1 != 0)
    {
      duration_time = (int32_t)abs(UARTSpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
    }
    else
    {
      duration_time = 65535;	//Is this for stop motor using? ask Jenny
    }
    if(duration_time > 65535)
    {
      duration_time = 65535;
    }
    /*Set acture speed ref*/
   // MCI_ExecSpeedRamp(oMCI[M1],UARTSpeedCMDLast_M1,duration_time);
    MC_ProgramSpeedRampMotor1(UARTSpeedCMD_M1,duration_time);
  }
}


/*victor add 20191023*/
int16_t report_speed = 0;
int32_t report_speed_buf = 0;          
#define REPORT_SPEED_T_LPF  (1.0) 
#define REPORT_SPEED_FILTER_CONST (short int)  (32767/(REPORT_SPEED_T_LPF*100))  /*100Hz=10ms this filter was called every 10ms*/ 

int32_t lib_debug_compressor_speed_fdbk = 0;
  int16_t speed_cmd=0;
//  int16_t M1_fault_code;

void User_Motor_Control_M1(void)
{
//  int16_t speed_cmd;
//  int16_t M1_fault_code;

//  MCI_State_t m1_state;
  lib_debug.M1_state = MCI_GetSTMState(pMCI[M1]);
  
  speed_cmd = MCI_GetLastRampFinalSpeed(pMCI[M1]);	//#SPEED_UNIT
  lib_debug.M1_fault_code = MCI_GetOccurredFaults(pMCI[M1]);

  lib_debug_compressor_speed_fdbk = MCI_GetAvrgMecSpeedUnit(pMCI[M1]);	//#SPEED_UNIT
  
  report_speed_buf = report_speed_buf + (lib_debug_compressor_speed_fdbk - (report_speed_buf >> 15))*REPORT_SPEED_FILTER_CONST;
  report_speed = report_speed_buf >> 15;

  if(lib_debug.M1_fault_code == 0)
  {		
    /*** added by zn 20191225 start***/ 
	
      if(speed_cmd == 0)
      {
        if((lib_debug.M1_state != IDLE) && (lib_debug.M1_state != ICLWAIT) && (lib_debug.M1_state != STOP))// && (m1_state != STOP_IDLE))
        {
					MC_StopMotor1();  //影响转矩模式
        }
        M1EnableStartFlag = true;
      }
      else
      {
        if((lib_debug.M1_state == IDLE) && (M1EnableStartFlag == true))
        {
          MC_StartMotor1();      	//START motor
          M1EnableStartFlag = false;
        }
      }
  }
  else
  {
    if((speed_cmd == 0)) //revised by zn 20191226
    {
      M1EnableStartFlag = true;
			MC_AcknowledgeFaultMotor1();			//ack Fault
//      MCI_FaultAcknowledged(pMCI[M1]);			//ack Fault
    }
  }
}



void UI_Process(void)
{   
    UART_Emulator_Debug();

    User_Motor_Control_M1();
}




/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
