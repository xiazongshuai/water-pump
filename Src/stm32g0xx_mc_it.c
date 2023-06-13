
/**
  ******************************************************************************
  * @file    stm32g0xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control
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
  * @ingroup STM32G0xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g0xx_ll_exti.h"
#include "mcp_config.h"
#include "User_Logic.h"
#include "usart_handle.h"
#include "app_error.h"
#include "led_key.h"
#include "app_handle.h"
#include "main.h"
#include "pwm_mode.h"
/* USER CODE END Includes */
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

#define USER_LOGIC_COUNTER (20) //10ms counter, 10ms = 500us * 20
int16_t User_Logic_Handle_Counter = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

void DMA1_Channel1_IRQHandler (void);

void TIM1_BRK_UP_TRG_COM_IRQHandler (void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI4_15_IRQHandler (void);

/**
  * @brief  This function handles current regulation interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler (void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */
    /* Debug High frequency task duration
     * LL_GPIO_SetOutputPin (GPIOB, LL_GPIO_PIN_3);
     */
	
	//过流故障
//	if(GET_ADC_CIN_Digital() > 1985*5) //1.35A*0.1*10*19859 = 26809.65
//	{
//		MC_StopMotor1();
//		work_mode_switch(MODE_STANDLY);
//		R3_1_OVERCURRENT_IRQHandler(&PWM_Handle_M1);
//	  MOTOR_ERROR_INFO |= (1<<overcurrent_err);
//	}
	
  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  /* Clear Flags */
  DMA1->IFCR = (LL_DMA_ISR_GIF1|LL_DMA_ISR_TCIF1|LL_DMA_ISR_HTIF1);
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */

  TSK_HighFrequencyTask();

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler (void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */

  if(LL_TIM_IsActiveFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_UPDATE(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
    /* USER CODE BEGIN PWM_Update */

    /* USER CODE END PWM_Update */
  }
  if(LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_BRK(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
    R3_1_OVERCURRENT_IRQHandler(&PWM_Handle_M1);
    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  if (LL_TIM_IsActiveFlag_BRK2(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_BRK(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK2(PWM_Handle_M1.pParams_str->TIMx);
    R3_1_OVERVOLTAGE_IRQHandler(&PWM_Handle_M1);
    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
   /* No other interrupts are routed to this handler */
  }
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */
}

/**
  * @brief This function handles DMA_RX_A channel DMACH_RX_A global interrupt.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQHandler 0 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQHandler 0 */

  /* Buffer is ready by the HW layer to be processed */
  if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A) ){
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT (&aspepOverUartA);
  }
  /* USER CODE BEGIN DMA1_Channel2_3_IRQHandler 1 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQHandler 1 */

}

/* This section is present only when serial communication is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
extern uint8_t mrx_data;
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQHandler 0 */

  /* USER CODE END USART2_IRQHandler 0 */

  if ( LL_USART_IsActiveFlag_TC (USARTA) )
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
    LL_DMA_DisableChannel( DMA_TX_A, DMACH_TX_A );
    LL_USART_ClearFlag_TC (USARTA);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
  }
  if ( (LL_USART_IsActiveFlag_ORE (USARTA) || LL_USART_IsActiveFlag_FE (USARTA) || LL_USART_IsActiveFlag_NE (USARTA))
        && LL_USART_IsEnabledIT_ERROR (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/
    WRITE_REG(USARTA->ICR, USART_ICR_FECF|USART_ICR_ORECF|USART_ICR_NECF);
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR (USARTA);
    LL_USART_EnableIT_IDLE (USARTA);
  }
  if ( LL_USART_IsActiveFlag_IDLE (USARTA) && LL_USART_IsEnabledIT_IDLE (USARTA) )
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_DisableIT_IDLE (USARTA);
    /* Once the complete unexpected data are received, we enable back the error IT*/
    LL_USART_EnableIT_ERROR (USARTA);
    /* To be sure we fetch the potential pendig data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX (USARTA);
    LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX (USARTA);
    ASPEP_HWDMAReset (&aspepOverUartA);
  }

  /* USER CODE BEGIN USART2_IRQHandler 1 */

  /* USER CODE END USART2_IRQHandler 1 */
}


//void USART1_IRQHandler(void)
//{
//		if(LL_USART_IsActiveFlag_RXNE_RXFNE(USART1) && LL_USART_IsEnabledIT_RXNE_RXFNE(USART1))
//    {
//		  mrx_data = LL_USART_ReceiveData8(USART1);
//	 } 
//}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {

  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(User_Logic_Handle_Counter == USER_LOGIC_COUNTER)
	{
		UI_Process();
		User_Logic_Handle_Counter = 0;
	}
	User_Logic_Handle_Counter++;	
	
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();
  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/**
  * @brief  This function handles Button IRQ on PIN PC13.
  */

__IO uint8_t rise_time = 0;
extern int timer17_updata_time;

void EXTI4_15_IRQHandler (void)
{
	/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadRisingFlag_0_31(LL_EXTI_LINE_13) ||
       LL_EXTI_ReadFallingFlag_0_31(LL_EXTI_LINE_13) )
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_13);
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_13);
    UI_HandleStartStopButton_cb ();
  }
	
	if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_7) != RESET) //上升沿
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_7);
		HAL_TIM_Base_Start_IT(&htim17);
		
		LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_7);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);
		rise_time++;
		
		if(rise_time == 1)
		{
			htim17.Instance->CNT = 0; //清零定时器
			timer17_updata_time = 0;
		  rise1_cnt = 0;
			rise2_cnt = 0;
			fall_cnt = 0;
		}
		else if(rise_time == 2)
		{
		  rise2_cnt = timer17_updata_time*65535 + htim17.Instance->CNT;
			timer17_updata_time = 0;
			rise_time = 0;
			htim17.Instance->CNT = 0;
		}

  }
	else if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_7) != RESET) //下降沿
  {
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_7);
		
		fall_cnt = timer17_updata_time*65535 + htim17.Instance->CNT;
		LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_7);
		LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_7);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
