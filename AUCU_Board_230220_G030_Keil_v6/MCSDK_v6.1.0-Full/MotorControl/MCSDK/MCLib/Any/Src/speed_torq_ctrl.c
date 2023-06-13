/**
  ******************************************************************************
  * @file    speed_torq_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Speed & Torque Control component of the Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup SpeednTorqCtrl
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_torq_ctrl.h"
#include "speed_pos_fdbk.h"

#include "mc_type.h"
#include "User_Logic.h"
#include "lin.h"

#define CHECK_BOUNDARY

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednTorqCtrl Speed & Torque Control
  * @brief Speed & Torque Control component of the Motor Control SDK
  *
  * The MCSDK is able to control the startup phase of the motor in a torque Mode.
  * In this case the STC component will be used to manage a torque reference and a torque ramp.
  *
  * @{
  */

/**
  * @brief  Initializes all the object variables.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @param  pPI: the PI object used as controller for the speed regulation.
  *         It can be equal to MC_NULL if the STC is initialized in torque mode
  *         and it will never be configured in speed mode.
  * @param  SPD_Handle: the speed sensor used to perform the speed regulation.
  *         It can be equal to MC_NULL if the STC is used only in torque mode.
  * @retval none.
  *
  * - Called once right after object creation at initialization of the whole MC core.
  */
__weak void STC_Init(SpeednTorqCtrl_Handle_t *pHandle, PID_Handle_t *pPI, SpeednPosFdbk_Handle_t *SPD_Handle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->PISpeed = pPI;
    pHandle->SPD = SPD_Handle;
    pHandle->Mode = pHandle->ModeDefault;
    pHandle->SpeedRefUnitExt = ((int32_t)pHandle->MecSpeedRefUnitDefault) * 65536; //默认速度*65536
    pHandle->TorqueRef = ((int32_t)pHandle->TorqueRefDefault) * 65536;
    pHandle->TargetFinal = 0;
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief Sets in real time the speed sensor utilized by the STC.
  * @param pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @param SPD_Handle Speed sensor component to be set.
  * @retval none
  *
  * - Called during tasks execution of the MC state machine into MediumFrequencyTask.
  */
__weak void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *SPD_Handle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->SPD = SPD_Handle;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief Returns the speed sensor utilized by the FOC.
  * @param pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval SpeednPosFdbk_Handle_t speed sensor utilized by the FOC.
  *
  * - Called as soon as component parameters are required by MC FW.
  */
__weak SpeednPosFdbk_Handle_t *STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL ==  pHandle) ? MC_NULL : pHandle->SPD);
#else
  return (pHandle->SPD);
#endif
}

/**
  * @brief  Resets the integral term of speed regulator if STC is set in speed mode.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval none.
  *
  * - Called before each motor restart.
  */
__weak void STC_Clear(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      PID_SetIntegralTerm(pHandle->PISpeed, 0);
    }
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief  Gets the current mechanical rotor speed reference
  *         @ref SpeednTorqCtrl_Handle_t::SpeedRefUnitExt "SpeedRefUnitExt" expressed in the unit
  *         defined by [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval int16_t current mechanical rotor speed reference expressed in 
  *         the unit defined by [SPEED_UNIT](measurement_units.md).
  *
  * - Called at MC boot procedure and for speed monitoring through MotorPilote.
  */
__weak int16_t STC_GetMecSpeedRefUnit(SpeednTorqCtrl_Handle_t *pHandle)  //获取当前机械转子的参考速度
{
#ifndef FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
#ifdef NULL_PTR_SPD_TRQ_CTL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->SpeedRefUnitExt >> 16));
#else
  return ((int16_t)(pHandle->SpeedRefUnitExt >> 16));  //先乘了65536，返回速度时除65536
#endif
#else
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->SpeedRefUnitExt / 65536));
#else
  return ((int16_t)(pHandle->SpeedRefUnitExt / 65536));
#endif
#endif
}

/**
  * @brief  Gets the current motor torque reference
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval int16_t current motor torque reference. This value represents
  *         actually the Iq current expressed in digit.
  *
  * - @ref SpeednTorqCtrl_Handle_t::TorqueRef "TorqueRef" represents
  * actually the Iq current reference expressed in digit.
  * - To convert current expressed in digit to current expressed in Amps
  * is possible to use the formula:\n
  * Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * - Called during #STC_ExecRamp execution.
  */
__weak int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifndef FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
#ifdef NULL_PTR_SPD_TRQ_CTL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->TorqueRef >> 16));
#else
  return ((int16_t)(pHandle->TorqueRef >> 16));
#endif
#else
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->TorqueRef / 65536));
#else
  return ((int16_t)(pHandle->TorqueRef / 65536));
#endif
#endif
}

/**
  * @brief  Sets the modality of the speed and torque controller
  *         @ref SpeednTorqCtrl_Handle_t::Mode "Mode".
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @param  bMode:  modality of STC. It can be one of these two settings:
  *         MCM_TORQUE_MODE to enable the Torque mode or MCM_SPEED_MODE to
  *         enable the Speed mode.
  * @retval none
  *
  * - Two modality are available Torque mode and Speed mode.\n
  * -- In Torque mode is possible to set directly the motor torque
  * reference or execute a motor torque ramp. This value represents
  * actually the Iq current reference expressed in digit.\n
  * -- In Speed mode is possible to set the mechanical rotor speed
  * reference or execute a speed ramp. The required motor torque is
  * automatically calculated by the STC.\n
  * - Interrupts the execution of any previous ramp command
  * maintaining the last value of Iq by clearing
  *  @ref SpeednTorqCtrl_Handle_t::RampRemainingStep "RampRemainingStep".
  * - Called generally before Starting the execution of a ramp.
  */
__weak void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, MC_ControlMode_t bMode)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Mode = bMode;     //设置模式速度、转矩.....
    pHandle->RampRemainingStep = 0u; /* Interrupts previous ramp. */   //斜坡步数清零
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief  Gets the modality of the speed and torque controller
  *           @ref SpeednTorqCtrl_Handle_t::Mode "Mode".
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval MC_ControlMode_t  modality of STC. It can be one of
  *         these two values: MCM_TORQUE_MODE or MCM_SPEED_MODE.
  *
  * - Called by @ref SpeedRegulatorPotentiometer Speed potentiometer component to manage new speed reference.
  */
__weak MC_ControlMode_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? MCM_TORQUE_MODE : pHandle->Mode);
#else
  return (pHandle->Mode);
#endif
}

/**
  * @brief  Starts the execution of a ramp using new target and duration.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @param  hTargetFinal: final value of command. This is different accordingly
  *         the STC modality.
  *         If STC is in Torque mode hTargetFinal is the value of motor torque
  *         reference at the end of the ramp. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:\n
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro\n
  *         If STC is in Speed mode hTargetFinal is the value of mechanical
  *         rotor speed reference at the end of the ramp expressed in the unit 
  *         defined by [SPEED_UNIT](measurement_units.md).
  * @param  hDurationms: the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool returning false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or max
  *         application torque or below min application speed depending on
  *         current modality of TSC) in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true.
  *
  * - This command interrupts the execution of any previous ramp command.
  * The generated ramp will be in the modality previously set by
  * #STC_SetControlMode method.
  * - Called during @ref motor profiling, @ref RevUpCtrl "Rev-Up Control" phase,
  * @ref EncAlignCtrl "Encoder Alignment Control",
  * @ref PositionControl "Position Control" loop or
  * speed regulation with @ref SpeedRegulatorPotentiometer Speed potentiometer.
  */
	
    int16_t hCurrentReference;	                                 //using new target and duration
__weak bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms) //执行斜坡
{
  bool allowedRange = true;
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    allowedRange = false;
  }
  else
  {
#endif
    uint32_t wAux;
    int32_t wAux1;
//    int16_t hCurrentReference;

    /* Check if the hTargetFinal is out of the bound of application. */
    if (MCM_TORQUE_MODE == pHandle->Mode)  //转矩模式
    {
      hCurrentReference = STC_GetTorqueRef(pHandle);  //获取当前设定转矩
#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxPositiveTorque)  //电机扭矩的最大正值。该值实际上表示以数字表示的最大Iq电流
      {
        allowedRange = false;
      }
      if ((int32_t)hTargetFinal < (int32_t)pHandle->MinNegativeTorque) //最小负值
      {
        allowedRange = false;
      }
#endif
    }
    else  //速度模式
    {
#ifndef FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt >> 16);     //获取当前设定速度
#else
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt / 65536);
#endif

#ifdef CHECK_BOUNDARY    //检查速度范围
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxAppPositiveMecSpeedUnit) //大于最大设定速度
      {
        allowedRange = false;
      }
      else if (hTargetFinal < pHandle->MinAppNegativeMecSpeedUnit) //小于 负的最大设定速度
      {
        allowedRange = false;
      }
      else if ((int32_t)hTargetFinal < (int32_t)pHandle->MinAppPositiveMecSpeedUnit)
      {
        if (hTargetFinal > pHandle->MaxAppNegativeMecSpeedUnit)
        {
          allowedRange = false;
        }
      }
      else
      {
        /* Nothing to do */
      }
#endif
    }

    if (true == allowedRange)
    {
      /* Interrupts the execution of any previous ramp command */
      if (0U == hDurationms) //设定时间斜坡时间完成
      {
        if (MCM_SPEED_MODE == pHandle->Mode)
        {
          pHandle->SpeedRefUnitExt = ((int32_t)hTargetFinal) * 65536;  //把设定速度赋给当前速度参考值
        }
        else
        {
          pHandle->TorqueRef = ((int32_t)hTargetFinal) * 65536; //参考转矩
        }
        pHandle->RampRemainingStep = 0U;
        pHandle->IncDecAmount = 0;
      }
      else
      {
        /* Store the hTargetFinal to be applied in the last step */
        pHandle->TargetFinal = hTargetFinal;

        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        wAux = ((uint32_t)hDurationms) * ((uint32_t)pHandle->STCFrequencyHz);   //hDurationms-->ms  STCFrequencyHz-->hz   所以还要除1000
        wAux /= 1000U;
        pHandle->RampRemainingStep = wAux;  //斜坡步数
        pHandle->RampRemainingStep++;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcTorqueReference. */
        wAux1 = (((int32_t)hTargetFinal) - ((int32_t)hCurrentReference)) * 65536;  //当前和目标的差值
        wAux1 /= ((int32_t)pHandle->RampRemainingStep);   
        pHandle->IncDecAmount = wAux1;  //每步加减量
      }
    }
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
  return (allowedRange);
}

/**
  * @brief  Interrupts the execution of any previous ramp command in particular by clearing
  *         the number of steps remaining to complete the ramp
  *         @ref SpeednTorqCtrl_Handle_t::RampRemainingStep "RampRemainingStep".
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval none
  *
  * - If STC has been set in Torque mode the last value of Iq is maintained.\n
  * - If STC has been set in Speed mode the last value of mechanical
  * rotor speed reference is maintained.
  * - Called by MCI_StopSpeedRamp execution command.
  */
__weak void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle)  //停止斜坡
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief  Computes the new value of motor torque reference.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval int16_t motor torque reference. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:\n
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  *
  * - Must be called at fixed time equal to hSTCFrequencyHz. It is called
  * passing as parameter the speed sensor used to perform the speed regulation.
  * - Called during START and ALIGNEMENT states of the MC state machine into MediumFrequencyTask.
  */
#include "watch_data.h"
uint8_t speed_pi_swich = 0;
extern  uint32_t time_30s;
int16_t speed_pi_iq;
__weak int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle) //计算新的参考转矩
{
  int16_t hTorqueReference;
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    hTorqueReference = 0;
  }
  else
  {
#endif
    int32_t wCurrentReference;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (MCM_TORQUE_MODE == pHandle->Mode)  //转矩模式
    {
      wCurrentReference = pHandle->TorqueRef;
    }
    else
    {
      wCurrentReference = pHandle->SpeedRefUnitExt;  //机械转子参考速度
    }

    /* Update the speed reference or the torque reference according to the mode
       and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1U) //完成斜坡剩余步骤数
    {
      /* Increment/decrement the reference value. */
      wCurrentReference += pHandle->IncDecAmount;   //增减量

      /* Decrement the number of remaining steps */
      pHandle->RampRemainingStep--;  //步数减一
    }
    else if (1U == pHandle->RampRemainingStep)
    {
      /* Set the backup value of hTargetFinal. */
      wCurrentReference = ((int32_t)pHandle->TargetFinal) * 65536;  
      pHandle->RampRemainingStep = 0U;
    }
    else
    {
      /* Do nothing. */
    }

    if (MCM_SPEED_MODE == pHandle->Mode)  //速度闭环
    {
      /* Run the speed control loop */
      /* Compute speed error */
#ifndef FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hTargetSpeed = (int16_t)(wCurrentReference >> 16);
#else
      hTargetSpeed = (int16_t)(wCurrentReference / 65536);
#endif
      hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->SPD);
      hError = hTargetSpeed - hMeasuredSpeed;
	
      hTorqueReference = PI_Controller(pHandle->PISpeed, (int32_t)hError);  //PI控制   速度闭环后 iq_ref 由速度PI计算得到

      pHandle->SpeedRefUnitExt = wCurrentReference;
      pHandle->TorqueRef = ((int32_t)hTorqueReference) * 65536;
    }
    else  
    {
      pHandle->TorqueRef = wCurrentReference;  //速度没闭环 iq_ref 由启动阶段设定
#ifndef FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL.
			
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hTorqueReference = (int16_t)(wCurrentReference >> 16);
#else
      hTorqueReference = (int16_t)(wCurrentReference / 65536);
#endif
    }
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
		
//	      hError = hTargetSpeed - hMeasuredSpeed;
	global_target_speed = hTargetSpeed*6;
	global_actual_speed = hMeasuredSpeed*6;
		
	watch_data.w_hTargetSpeed_rpm = hTargetSpeed*6;
	watch_data.w_hMeasuredSpeed_rpm = hMeasuredSpeed*6;
	
	lib_debug.hTargetSpeed_rpm = 	watch_data.w_hTargetSpeed_rpm;
	lib_debug.hMeasuredSpeed_rpm = watch_data.w_hMeasuredSpeed_rpm;
		
	#if 0
		if( (millis() - time_30s > 30000) && (speed_pi_swich == 0) )
	  {
			speed_pi_swich = 1;
			PIDSpeedHandle_M1.hKpGain = 2898;
			PIDSpeedHandle_M1.hKiGain = 37;
			
			PIDSpeedHandle_M1.hKpDivisor = 4096;
		//	PIDSpeedHandle_M1.hKpDivisorPOW2 = LOG2((4096));
			PIDSpeedHandle_M1.hKiDivisor = 16384;
	//		PIDSpeedHandle_M1.hKiDivisorPOW2 = LOG2((16384));
//			PIDSpeedHandle_M1.wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)16384;
//			PIDSpeedHandle_M1.wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)16384;
			
	    PIDSpeedHandle_M1.hDefKpGain          = (int16_t)2898;
//		PIDSpeedHandle_M1.hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,

			
//			PID_HandleInit(&PIDSpeedHandle_M1);
//			STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_PLL_M1._Super);
			
			speed_pi_iq = hTorqueReference;
		}	
	#endif	
  return (hTorqueReference);
}

/**
  * @brief  Gets the Default mechanical rotor speed reference
  *         @ref SpeednTorqCtrl_Handle_t::MecSpeedRefUnitDefault "MecSpeedRefUnitDefault" expressed in
  *         the unit defined by [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval int16_t Default mechanical rotor speed.
  *
  * - It is the first command to STC after the start of speed ramp execution.
  * - Called during the boot phase of the MC process.
  */
__weak int16_t STC_GetMecSpeedRefUnitDefault(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? 0 : pHandle->MecSpeedRefUnitDefault);
#else
  return (pHandle->MecSpeedRefUnitDefault);
#endif
}

/**
  * @brief  Returns the Application maximum positive value of rotor speed
  *         @ref SpeednTorqCtrl_Handle_t::MaxAppPositiveMecSpeedUnit "MaxAppPositiveMecSpeedUnit".
  *         Expressed in the unit defined by [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  *
  * - Not used into current implementation.
  */
__weak uint16_t STC_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? 0U : pHandle->MaxAppPositiveMecSpeedUnit);
#else
  return (pHandle->MaxAppPositiveMecSpeedUnit);
#endif
}

/**
  * @brief  Returns the Application minimum negative value of rotor speed
  *         @ref SpeednTorqCtrl_Handle_t::MinAppNegativeMecSpeedUnit "MinAppNegativeMecSpeedUnit".
  *         Expressed in the unit defined by [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  *
  * - Not used into current implementation.
  */
__weak int16_t STC_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  return ((MC_NULL == pHandle) ? 0 : pHandle->MinAppNegativeMecSpeedUnit);
#else
  return (pHandle->MinAppNegativeMecSpeedUnit);
#endif
}

/**
  * @brief  Checks if the settled speed or torque ramp has been completed by checking zero value of
  *           @ref SpeednTorqCtrl_Handle_t::RampRemainingStep "RampRemainingStep".
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval bool returning true if the ramp is completed, false otherwise.
  *
  * - Called during motor profiler tuning of HALL sensor.
  */
__weak bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle) //检查斜坡是否完成
{
  bool retVal = false;
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (0U == pHandle->RampRemainingStep)
    {
      retVal = true;
    }
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  Stops the execution of speed ramp by clearing the number of steps remaining to complete the ramp.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval bool returning true if the command is executed, false otherwise.
  *
  * - Not used into current implementation.
  */
__weak bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle) //停止速度斜坡
{
  bool retVal = false;
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      pHandle->RampRemainingStep = 0u;
      retVal = true;
    }
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  Returns the default values of Iqdref.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval default values of Iqdref.
  *
  * - Called during the boot phase of the MC process.
  */
__weak qd_t STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle) //获取默认的Id、Iq
{
  qd_t IqdRefDefault;
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    IqdRefDefault.q = 0;
    IqdRefDefault.d = 0;
  }
  else
  {
#endif
    IqdRefDefault.q = pHandle->TorqueRefDefault;
    IqdRefDefault.d = pHandle->IdrefDefault;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
  return (IqdRefDefault);
}

/**
  * @brief  Changes the nominal current by setting new values of
  *         @ref SpeednTorqCtrl_Handle_t::MaxPositiveTorque "MaxPositiveTorque" and
  *         @ref SpeednTorqCtrl_Handle_t::MinNegativeTorque "MinNegativeTorque".
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @param  hNominalCurrent: represents actually the maximum Iq current expressed in digit.
  * @retval none
  *
  * - Not used into current implementation.
  */
__weak void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->MaxPositiveTorque = hNominalCurrent;     //MaxPositiveTorque---最大电流
    pHandle->MinNegativeTorque = -(int16_t)hNominalCurrent;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @brief  Forces the speed reference
  *         @ref SpeednTorqCtrl_Handle_t::SpeedRefUnitExt "SpeedRefUnitExt" to the current speed.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component.
  * @retval none
  *
  * - Called during the CHARGE_BOOT_CAP, SWITCH_OVER and WAIT_STOP_MOTOR states of the MC state machine
  * into MediumFrequencyTask to initialize the speed reference.
  */
__weak void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_TRQ_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->SpeedRefUnitExt = ((int32_t)SPD_GetAvrgMecSpeedUnit(pHandle->SPD)) * (int32_t)65536;
#ifdef NULL_PTR_SPD_TRQ_CTL
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
