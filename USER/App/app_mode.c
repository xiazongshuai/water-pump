#include "app_mode.h"
#include "app_error.h"
#include "led_key.h"
#include "pwm_mode.h"
#include "main.h"
/*******************************
ָ指示灯       档位
绿+1     1   AUTO
绿+2     2   速度挡
绿+3     3   恒压 5m 挡
绿+1+3   4   恒压 7m 挡
绿+2+3   5   恒压 9m 挡
绿+1+2+3 6   最大功率 12m 挡
绿+1+2+3 7   PWM调速和功率反馈
*******************************/

uint8_t mrx_data;
int mspeed=1000;
extern MCI_State_t msta;
uint8_t need_stop_flag = 1;

int myabs(int16_t num)
{
  if(num>0) return num;
	else return -num;
}

//待机
void mode_standby_init(void)
{
	led_clear();
	need_stop_flag = 1;
	MC_StopMotor1(); //停止
	MOTOR_ERROR_INFO = 0;
	mccurrent_err = 0;
	MC_AcknowledgeFaultMotor1();  //清故障
}

void mode_standby_tick(void)
{
	if(msta != STOP) //状态不是STOP
	{
	 MC_StopMotor1();
	}
	if(msta==FAULT_NOW || msta==FAULT_OVER) //状态是Fault
	{
    MC_AcknowledgeFaultMotor1();  //清故障
	}
}


//AUTO
void mode_auto_init(void)
{ 
	led_show_function(auto_func);
	
	need_stop_flag = 1;
	MC_ProgramSpeedRampMotor1(1000/6,10000);
	MC_StartMotor1();
}

void mode_auto_tick(void)
{

}

//速度挡
void mode_speed_init(void)
{
	led_show_function(speed_func);
	
	MC_StopMotor1();
	
}

int speedmode = 0;
void mode_speed_tick(void)
{
//	if((MC_GetSTMStateMotor1() != STOP) && need_stop_flag == 1)
//	{
//	  MC_StopMotor1();
//		return;
//	}
//	if(MC_GetSTMStateMotor1() == STOP)
//	{
//	  need_stop_flag = 0;
//	}

}

//恒压 5m 挡
void mode_pressure_5m_init(void)
{
	led_show_function(pressure_5m_func);
	
	MC_ProgramSpeedRampMotor1(1200/6,10000);
	MC_StartMotor1();
}

void mode_pressure_5m_tick(void)
{
	
}

//恒压 7m 挡
void mode_pressure_7m_init(void)
{
	led_show_function(pressure_7m_func);
	
	MC_ProgramSpeedRampMotor1(1400/6,10000);
	MC_StartMotor1();
}

void mode_pressure_7m_tick(void)
{
	
}

//恒压 9m 挡
void mode_pressure_9m_init(void)
{
	led_show_function(pressure_9m_func);
	
	MC_ProgramSpeedRampMotor1(1500/6,10000);
	MC_StartMotor1();
}

void mode_pressure_9m_tick(void)
{
	
}

//最大功率 12m 挡
void mode_max_power_12m_init(void)
{
	led_show_function(max_power_12m_func);
	need_stop_flag = 1;
	MC_ProgramSpeedRampMotor1(1800/6,10000);
	MC_StartMotor1();
}

void mode_max_power_12m_tick(void)
{
	
}

#ifdef LIN_CONTROL
void mode_pwm_init(void)
{
}
void mode_pwm_tick(void)
{
}
#endif

#ifdef PWM_CONTROL
//volatile
//PWM 调速和功率反馈
int pwm_time; 
uint8_t pwm_out_config=1;
uint16_t motor_power;     //功率
void mode_pwm_init(void)
{
  pwm_time = 0;
	pwm_out_config = 1;
	led_show_function(pwm_func);
	
	MC_StopMotor1();
	
	//使用PWM方式
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_6|GPIO_PIN_7);
	__NVIC_DisableIRQ(USART1_IRQn);
	LL_USART_DisableIT_RXNE_RXFNE(USART1);
	LL_USART_Disable(USART1);

  MX_TIM17_Init();
  pwm_mode_init();
	
	pwm_output_config();
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
}

uint8_t temp_duty[10]={98,98,98,98,98,98,98,98,98,98};
uint8_t temp_duty_time;
int duty_average;
void mode_pwm_tick(void)
{
//	if((MC_GetSTMStateMotor1() == RUN) && need_stop_flag == 1)
//	{
//	  MC_StopMotor1();
//		return;
//	}
//	if(MC_GetSTMStateMotor1() == STOP)
//	{
//	  need_stop_flag = 0;
//	}
//	
	
	//pwm_speed_control(20);
	
	pwm_time++;
	pwm_duty = get_duty_period();
	temp_duty[temp_duty_time] = pwm_duty;
	temp_duty_time++;
	
	if(temp_duty_time == 10)
	{
		temp_duty_time = 0;
		for(uint8_t i=0; i<10 ;i++)
		{
	   duty_average += temp_duty[i];
		}
		duty_average = duty_average/10;
	}
	if(myabs(temp_duty[0] - duty_average) > 2)
	{
		duty_average = 0;
	  return;
	}
	duty_average = 0;

	pwm_speed_control(temp_duty[0]);
	motor_power = PQD_GetAvrgElMotorPowerW(pMPM[M1]);
	

	if(pwm_out_config == 1 && pwm_time > 1000)
	{
		pwm_time = 0;
		pwm_out_config = 0;
	  pwm_output_config();
		HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
		//__HAL_TIM_SET_AUTORELOAD(&htim16,pwm_arr);
	}
}
#endif


#ifdef voltage_0_10
//0到10V调速
void mode_pwm_init(void)
{
	led_show_function(pwm_func);

	//使用PWM方式
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_6|GPIO_PIN_7);
	__NVIC_DisableIRQ(USART1_IRQn);
	LL_USART_DisableIT_RXNE_RXFNE(USART1);
	LL_USART_Disable(USART1);

  MX_TIM17_Init();
  pwm_mode_init();
	//HAL_TIM_Base_Start_IT(&htim17);
}

uint8_t temp_duty[10]={98,98,98,98,98,98,98,98,98,98};
uint8_t temp_duty_time;
int duty_average;
void mode_pwm_tick(void)
{
	pwm_duty = get_duty_period();
	if(pwm_duty == 0)
	{
	  return;
	}
	temp_duty[temp_duty_time] = pwm_duty;
	temp_duty_time++;
	
	if(temp_duty_time == 10)
	{
		temp_duty_time = 0;
		for(uint8_t i=0; i<10 ;i++)
		{
	   duty_average += temp_duty[i];
		}
		duty_average = duty_average/10;
	}
	if(myabs(temp_duty[0] - duty_average) > 2)
	{
		duty_average = 0;
	  return;
	}
	duty_average = 0;

	pwm_speed_control(temp_duty[0]);
}
#endif

void mode_error_init(void)
{
  error_mode_init();
}
void mode_error_tick(void)
{
  error_mode_tick();
}
