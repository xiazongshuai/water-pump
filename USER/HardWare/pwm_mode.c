#include "pwm_mode.h"
#include "app_mode.h"
#include "stdlib.h" 

TIM_HandleTypeDef  htim16;

void pwmoutput_gpio_init(void) ;
void pwm_output_config(void);
void pwminput_gpio_init(void);

void pwm_mode_init(void)
{
	#ifdef PWM_CONTROL
  pwmoutput_gpio_init();
	#endif
  pwminput_gpio_init();
}

// pwm输出配置
void pwmoutput_gpio_init(void)  //TX--PB6--TIM16_CH1N
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
   
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

float pwm_arr = 0; //volatile
float pwm_ccr = 0;
void pwm_output_config(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	__HAL_RCC_TIM16_CLK_ENABLE();
 
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = pwm_arr;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void set_pwm_output(void)
{
 __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwm_ccr);
	//HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
}



void pwminput_gpio_init(void) 
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB, LL_EXTI_CONFIG_LINE7);

  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);

  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

int timer17_updata_time = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM17)
   {
     timer17_updata_time++;
   }
}


int rise1_cnt = 0;
int rise2_cnt = 0;
int fall_cnt = 0;
int pwm_tmp;
int pwm_duty;
int get_duty_period(void)
{
  if(rise2_cnt != 0 && fall_cnt != 0)
	{
	  pwm_ccr = fall_cnt;
		pwm_arr = rise2_cnt;
		pwm_tmp = (int)(((float)(pwm_ccr/pwm_arr))*100);
	}
	return pwm_tmp;
}

static uint8_t temp = 0;
uint8_t start_or_stop =0;
void pwm_speed_control(uint8_t duty_data)
{
	 int16_t current_speed_ref;
	 int32_t SpeedCMD_M1;
  int16_t speed_acc = 400;
  int32_t duration_time;
	current_speed_ref = MC_GetMecSpeedReferenceMotor1();
//	SpeedCMD_M1 = //(int16_t)((float)watch_data.w_hTargetSpeed_rpm);
//	 duration_time = (int32_t)abs(SpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
	if(temp == duty_data)
	{
	  return;
	}
	else
	{
		temp = duty_data;
	}
   set_pwm_output();
   if(duty_data <= 5)
	{
	  //最大速度
		start_or_stop = 1;
		SpeedCMD_M1 = pwm_max_speed/6;
	  duration_time = (int32_t)abs(SpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
		MC_ProgramSpeedRampMotor1(SpeedCMD_M1,duration_time);
		//MC_ProgramSpeedRampMotor1(pwm_max_speed/6,10000);
	  MC_StartMotor1();
	}
	else if(duty_data > 5 && duty_data <= 85)
	{
	  //根据占空比调速
		start_or_stop = 1;
		SpeedCMD_M1 = (pwm_max_speed - ((pwm_max_speed-pwm_min_speed)*duty_data/80))/6;  //4300/6;//3585/6;//
	  duration_time = (int32_t)abs(SpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
		MC_ProgramSpeedRampMotor1( SpeedCMD_M1  ,duration_time);
	  MC_StartMotor1();
	}
	else if(duty_data > 85 && duty_data <= 91)
	{
	  //最小速度
		start_or_stop = 1;
		SpeedCMD_M1 = pwm_min_speed/6;
	  duration_time = (int32_t)abs(SpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
		MC_ProgramSpeedRampMotor1(SpeedCMD_M1,duration_time);
	  MC_StartMotor1();
	}
	else if(duty_data > 91 && duty_data <= 95)
	{
	  //迟滞区：启动、停止
		if(start_or_stop == 1)
		{
		 SpeedCMD_M1 = pwm_min_speed/6;
	   duration_time = (int32_t)abs(SpeedCMD_M1 - current_speed_ref) * 1000 / speed_acc;
		 MC_ProgramSpeedRampMotor1(SpeedCMD_M1,duration_time);
	   MC_StartMotor1();
		}
		
		else if(start_or_stop == 2)
		{
		  MC_StopMotor1();
		}
	}
	else if(duty_data > 95  && duty_data <= 100)
	{
	  //停止
		MC_StopMotor1();
		start_or_stop = 2;
	}
}




