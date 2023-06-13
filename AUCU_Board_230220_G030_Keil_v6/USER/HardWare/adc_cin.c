#include "adc_cin.h"
#include "regular_conversion_manager.h"

int cin_digital;
RegConv_t ADC_UserConv_CIN;
uint8_t ADC_UserHander_CIN;

int vts_digital;
float vts_v;
RegConv_t ADC_UserConv_VTS;
uint8_t ADC_UserHander_VTS;

uint8_t user_adc_flag;

void ADC_CIN_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  ADC_UserConv_CIN.regADC = ADC1;
	ADC_UserConv_CIN.channel = MC_ADC_CHANNEL_16;  
	ADC_UserConv_CIN.samplingTime = ADC_SAMPLETIME_12CYCLES_5;
	ADC_UserHander_CIN = RCM_RegisterRegConv(&ADC_UserConv_CIN);
}

int GET_ADC_CIN_Digital(void)
{
	if(user_adc_flag == 1)
	{
  if(RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{ 
	  RCM_RequestUserConv(ADC_UserHander_CIN);
	}
	else if(RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{ 
	  cin_digital = RCM_GetUserConv();
		user_adc_flag = 0;
	}
  }
	return cin_digital;
}


// temperature = 60*V - 21
void ADC_VTS_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  ADC_UserConv_VTS.regADC = ADC1;
	ADC_UserConv_VTS.channel = MC_ADC_CHANNEL_10; 
	ADC_UserConv_VTS.samplingTime = ADC_SAMPLETIME_12CYCLES_5;
	ADC_UserHander_VTS = RCM_RegisterRegConv(&ADC_UserConv_VTS);
}

float GET_ADC_VTS_Voltage(void)
{
	if(user_adc_flag == 0)
	{
		if(RCM_GetUserConvState() == RCM_USERCONV_IDLE)
		{ 
			RCM_RequestUserConv(ADC_UserHander_VTS);
		}
		else if(RCM_GetUserConvState() == RCM_USERCONV_EOC)
		{ 
			vts_digital = RCM_GetUserConv();
			vts_v = vts_digital*3.3/65536;
			user_adc_flag = 1;
		}
  }
	return vts_v;
}
