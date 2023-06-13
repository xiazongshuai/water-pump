#include <stdio.h>
#include <string.h>
#include "usart_handle.h"
#include "main.h"
#include "app_handle.h"


#if 0

uint8_t send_data[20] = {0};
uint8_t rx_data[20] = {0};
uint8_t rx_tmp_data;
uint8_t rx_sta = 0;
uint8_t recvlen = 0;
uint8_t rec_data_finish = 0;

uint8_t mode_tmp=0;

void sendUartBytes(char * data, unsigned short datalen)
{
	for(int i=0; i<datalen; i++)
	{
	  LL_USART_TransmitData8(USART2, data[i]);
	  while(!(USART2->ISR&USART_CR1_TXEIE_TXFNFIE))  //注意判断不然会漏数据
    {;}
	}
}


void usart2_send_lenth(uint8_t *data, uint8_t lenth)
{
	for(uint8_t i=0; i<lenth; i++)
	{
		LL_USART_TransmitData8(USART2, data[i]);
		while(!(USART2->ISR&USART_CR1_TXEIE_TXFNFIE))  //注意判断不然会漏数据
    {;}
	}
}

void uart_send_data_compose(void)
{
	
  send_data[0] = main_panle_id;
  send_data[1] = display_panel_id;
  send_data[2] = control_cmd;
	
	send_data[3] = ERR_INFO_SEND;           //故障信息
	send_data[4] = ERR_INFO_SEND>>8;
	send_data[5] = ERR_INFO_SEND>>16;
	
	send_data[6] =  actual_motor_speed>>8;   //转速
	send_data[7] =  actual_motor_speed;
	
	send_data[8] =  data_power>>8;           //功率
	send_data[9] =  data_power;
	
	send_data[10] =   data_day>>8;          //通电时长(天)
	send_data[11] =  data_day;
	
	send_data[12] =  data_version>>8;       //版本号  1.0.0
	send_data[13] =  data_version;
	
	send_data[14] =  data_temperature+51;   //模块温度(以50为零刻度)
	
	send_data[15] =  data_voltage;         //输出电压
	send_data[16] =  data_voltage>>8;
	
	//保留位
	send_data[17] =  0x00;
	send_data[18] =  0x00;
	
	send_data[19] = 0;
	for(uint8_t i=0; i<19; i++)
	{
	  send_data[19] += send_data[i];
	}
	usart2_send_lenth(send_data,20);
}



void usart_handle_receive_data(void)
{
  uint8_t check=0;
 
	for(uint8_t i=0; i<18; i++)
	{
		check += rx_data[i];
	}
	uart_send_data_compose();
	if(check == rx_data[19])
	{
		ERR_INFO_REC = rx_data[3];        //故障信息
		ERR_INFO_REC = (ERR_INFO_REC << 8) | rx_data[4];
	  ERR_INFO_REC = (ERR_INFO_REC << 8) | rx_data[5];
		
		if(ERR_INFO_REC != 0)
		{
		  work_mode_switch(MODE_STANDLY); //MODE_STANDLY  MODE_ERROR
			return;
		}
		
		target_speed = rx_data[6]*100;    //设定转速/100
		target_pressure = rx_data[7];     //设定水压
		ac_inerpressure = rx_data[8];     //实际进水压
		ac_outpressure = rx_data[9];      //实际出水压
		set_mode = rx_data[10];            //手自动、开关复位、加速运行
		
		water_temperature =  rx_data[11];  //水温值+50
		ke_temperature =  rx_data[12];     //壳温值+50
		discharge_water =  rx_data[13];    //水流量*10
		
		//memset(rx_data,0,sizeof(rx_data));
		
		if((set_mode & system_reset) == system_reset) //复位
		{
		 restore_factory_settings();
		}
		
		if(g_work_mode != MODE_ERROR )
		{
		if((set_mode & mode_state) == manual_mode  && g_work_mode != MODE_MANUAL) //手动
		{
		 // work_mode_switch(MODE_MANUAL);
			mode_tmp = MODE_MANUAL;
		}
		if((set_mode & mode_state) == auto_mode  && g_work_mode != MODE_AUTO ) //自动
		{
		 // work_mode_switch(MODE_AUTO);
			mode_tmp = MODE_AUTO;
		}
		
		if((set_mode & acc_run) == acc_run) //加速运行
		{
			
		}
		
		if((set_mode & power_state) == power_on) //开机
		{
			if(g_work_mode == MODE_STANDLY && mode_tmp  == MODE_AUTO)
			{
		   work_mode_switch(MODE_MANUAL); 
			}
			
			if(g_work_mode == MODE_STANDLY && mode_tmp  == MODE_MANUAL)
			{
		   work_mode_switch(MODE_MANUAL); 
			}
			
			if(g_work_mode != MODE_STANDLY && mode_tmp  != g_work_mode)
			{
		   work_mode_switch(mode_tmp); 
			}
			
		}
	  if((set_mode & power_state) == power_off)  //待机
		{
		  work_mode_switch(MODE_STANDLY);
		}
	}
	}
}

#endif