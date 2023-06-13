/**
  ******************************************************************************
  * @file    User_Logic.h 
  * @author  Dennis GE - MMS MC Team
  * @version 4.3.0
  * @date    28-May-2014 10:45
  * @brief   Inverter Aircon Project
  * @Modified by dsg based on Ver 4.3 FOC Library 
  *******************************************************************************/ 

#ifndef __USER_LOGIC_H
#define __USER_LOGIC_H

#include "MC_type.h"
#include "mc_interface.h"

extern volatile int16_t Acc_M1 ;
extern volatile int16_t UARTSpeedCMD_M1 ;
//extern uint8_t M1_StartPreHeatingEnableFlag ;
//void BSP_HW_Init(void);
void UI_Process(void);
void User_Motor_Control_M1(void);
void UART_Emulator_Debug(void);
//void MC_Boot(void) ;
/*User interface framwork*/


typedef struct
{
	int16_t 		UARTSpeedCMD_M1_rpm;
	int16_t 		hTargetSpeed_rpm;		//unit:rpm
	int16_t		 	hMeasuredSpeed_rpm;	//unit:rpm	
	MCI_State_t M1_state;
	int16_t 		M1_fault_code;
	
	
} S_Lib_DUBUG;

extern	S_Lib_DUBUG		lib_debug;



#endif /* __USERLOGIC_H */
