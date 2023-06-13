#include "led_key.h"
#include "main.h"
#include "app_handle.h"
//红绿一起亮--橙色

/* 
ָ指示灯       档位
绿+1     1   AUTO
绿+2     2   速度挡
绿+3     3   恒压 5m 挡
绿+1+3   4   恒压 7m 挡
绿+2+3   5   恒压 9m 挡
绿+1+2+3 6   最大功率 12m 挡
绿+1+2+3 7   PWM调速和功率反馈
*/

KEY key;
uint8_t switch_mode_tmp = 66;
uint8_t MOTOR_ERROR_INFO;

uint8_t check_key_state(void)
{
  if(LL_GPIO_IsInputPinSet(KEY1_GPIO_Port,KEY1_Pin) == SET)
	{
	  key.down_time++;
		key.up_time = 0;
		if(key.down_time > 100)
		{
		  key.down_flag = 1;
		}
		
		if(key.down_time > 1500)
		{
			key.down_flag = 2;
		  return 2;
		}
	}
	
	if(LL_GPIO_IsInputPinSet(KEY1_GPIO_Port,KEY1_Pin) == RESET && key.down_flag != 0)
	{
		 key.up_time++;
		 key.down_time = 0;
		 if(key.up_time > 20)
		  {
				if(key.down_flag == 1)
				{
					key.down_flag = 0;
			   return 1;
				}
				key.down_flag = 0;
		  }
	}
	return 0;
}

//static uint16_t key_time;
//static uint16_t key_tmp;

uint8_t get_key_state(void)
{
  if(check_key_state() == 1)
	{
//		key_time = 0;
	  key.state++;
		if(key.state > 7)
		{
		  key.state = 0;
		}
	}
	
	if(check_key_state() == 2)
	{
		key.state = 0;
	}
	return key.state;
}

static uint8_t key_num;

void exe_key_func(void)
{
	key_num = get_key_state();
//	led_show_function(key_num);
//	key_time++;
//	if(key_time>1000)
//	{
//		key_time = 0;
//		key_tmp = get_key_state();
//	}
	
	if(switch_mode_tmp != key_num)// && key_num == key_tmp)
	{
    switch_mode_tmp = key_num;
		work_mode_switch(switch_mode_tmp);
	}
}


void led_clear(void)
{
	LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
	LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
	LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
	LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
	LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
}



void led_show_err(ERROR_ENUM motor_err)
{
	led_clear();
	for(uint8_t i=0; i<=motor_err; i++)
	{
	 LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
	 LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
	 LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
	 HAL_Delay(300);
	 LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
	 LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
	 LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
	 HAL_Delay(300);
	}
}

	
void led_show_function(FUNC_ENUM func_type)
{
	func_type = func_type;
	led_clear();
  if(func_type == pwm_func)
	{
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
	  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);//红+绿=橙
		
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
	}
	else
	{
	  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin); //绿
		
		if(func_type == auto_func)
		{
			LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		}
		else if(func_type == speed_func)
		{
			LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		}
		else if(func_type == pressure_5m_func)
		{
			LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		}
		else if(func_type == pressure_7m_func)
		{
			LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
			LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		}
		else if(func_type == pressure_9m_func)
		{
			LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
			LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		}
		else if(func_type == max_power_12m_func)
		{
			LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
			LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
			LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		}
	}
}
