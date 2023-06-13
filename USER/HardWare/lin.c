#include "lin.h"
#include "app_error.h"
#include "app_handle.h"
#include "led_key.h"

int global_target_speed;
int global_actual_speed;

/*********************** 作为主机 ***********************/

//主机帧头部分
//1、同步间隔段，13位显性电平
void Lin_SendBreak(void)
{
   LL_USART_RequestBreakSending(USART1);
}

//2、同步段，发送0X55
void Lin_SendSyncSegment(void)
{
	LL_USART_TransmitData8(USART1, 0x55);
	while(!(USART1->ISR&USART_CR1_TXEIE_TXFNFIE));
}

//3、发送PID（protect ID）
//异或：不同为1，相同为0
uint8_t Lin_CheckPID(uint8_t id)
{
	uint8_t returnpid;
	uint8_t P0 ;
	uint8_t P1;
	P0 = ( ((id)^(id>>1)^(id>>2)^(id>>4)) & 0x01 ) << 6;
	P1 = ( ( ~((id>>1)^(id>>3)^(id>>4)^(id>>5))) & 0x01 ) << 7;
	returnpid = id|P0|P1;
	return returnpid;
}

//该函数体就是单片机作为主机发送的帧头，可以指定ID发送帧头，接收从机返回的数据；也可以发送帧头+数据，让从机接收。
void Lin_SendHead(uint8_t id)
{
	Lin_SendBreak();
	Lin_SendSyncSegment();
	LL_USART_TransmitData8(USART1,Lin_CheckPID(id));
	while(!(USART1->ISR&USART_CR1_TXEIE_TXFNFIE));
}


//输入ID+数据，返回校验和段，里面有调用返回PID函数。
// 是经典校验还是增强校验，另：诊断帧只能经典校验
uint8_t Lin_Checksum(uint8_t id , uint8_t data[])
{
	uint8_t t ;
	uint16_t sum;

	sum = data[0];
	if(id == 0x3c)          // 如果是诊断帧，用经典校验
	{
		for(t=1;t<8;t++)
		{
			sum += data[t];
			if(sum&0xff00)   //大于255
			{
					sum&=0x00ff;  //=> sum -= 255
					sum+=1;
			}
		}
		sum = ~sum;
		return (uint8_t)sum;
	}
 
	for(t=1;t<8;t++)
	{
			sum += data[t];
			if(sum&0xff00)
			{
					sum&=0x00ff;
					sum+=1;
			}
	}
	 sum+=Lin_CheckPID(id);
	if(sum&0xff00)
	{
			sum&=0x00ff;
			sum+=1;
	}
	sum = ~sum;
	return (uint8_t)sum;
}

//数据发送
void Lin_SentData(uint8_t data[])
{
	uint8_t t ;
	for(t=0;t<8;t++)
	{
		LL_USART_TransmitData8(USART1,data[t]);
		while(!(USART1->ISR&USART_CR1_TXEIE_TXFNFIE));
	}
}

//主机响应函数调用
void Lin_SendAnswer(uint8_t id ,uint8_t data[])
{
   Lin_SentData(data);
   LL_USART_TransmitData8(USART1,Lin_Checksum(id,data));
   while(!(USART1->ISR&USART_CR1_TXEIE_TXFNFIE));
}


/*********************** 作为从机 ***********************/
uint8_t DataProcess=0;
uint8_t send_lin_data[8];
uint8_t Data_Index;
uint8_t LinReceiveData[20];
uint8_t DataReceiveflag;
uint8_t ReceiveCheckSum;
uint8_t FrameReceiveOverFlag;
uint8_t ReceivePID;
uint8_t SumCheck;

WORK_MODE_E LIN_MODE;
int LIN_SPEED;

//对接收到的报文进行解析执行。然后就是在中断函数体内，当接收到需要本从机反馈数据时，进行数据反馈
void Lin_DataProcess(void)
{
	uint8_t ReceiveID;
	uint8_t PIDChecksum;
	
	if(FrameReceiveOverFlag == 1)
	{
		FrameReceiveOverFlag = 0;
		ReceiveID = ReceivePID&0x3f;
		PIDChecksum = Lin_CheckPID(ReceiveID);
		if (PIDChecksum != ReceivePID)
		{
				return;
		}
		else
		{
		}
	
			SumCheck = Lin_Checksum(ReceiveID,LinReceiveData);
			if(ReceiveCheckSum != SumCheck)
				{
					return;
				}
				else
				{
				}
							 
		  if(ReceiveID == 0x23)
			{
				LIN_MODE = LinReceiveData[0];
				if(LIN_MODE != MODE_SPEED)    //接收LIN指令
				{
				 work_mode_switch(LIN_MODE);
				}
				else
				{
				  LIN_SPEED = LinReceiveData[1]<<8 |LinReceiveData[2];
					if(LIN_SPEED != 0)
					{
					 g_work_mode = MODE_NULL;
				   MC_ProgramSpeedRampMotor1(LIN_SPEED/6,10000);
	         MC_StartMotor1();
					}
					else
					{
					   work_mode_switch(MODE_STANDLY);
					}
				}
			}    
	}
}

uint8_t test_data;
uint8_t Lin_LBD_Flag;
void USART1_IRQHandler(void) //串口1中断服务程序
{
  uint8_t ReceiveData;
  uint8_t ReceiveID;
	
	if(LL_USART_IsActiveFlag_LBD(USART1)!=RESET)//LIN中断检测标志
	{
	  LL_USART_ClearFlag_LBD(USART1);
//	if(Lin_Sending_Flag==DISABLE)//没有发送数据
//	{
		 Lin_LBD_Flag=ENABLE;
//	}
	}
	if(LL_USART_IsActiveFlag_RXNE_RXFNE(USART1) && LL_USART_IsEnabledIT_RXNE_RXFNE(USART1))
  {
	 if(Lin_LBD_Flag==ENABLE)//lin中断使能可接收数据
		{
    ReceiveData = LL_USART_ReceiveData8(USART1); //读取接收到的数据
			test_data = ReceiveData;
		if(DataProcess == 0)
		{
			if(ReceiveData != 0x55)
			{
					return;
			}
			if(ReceiveData == 0x55)
			{
					DataProcess = 1;
					return;
			}
		}
		
		if(DataProcess == 1)
		{
			ReceivePID = ReceiveData;
			ReceiveID = ReceivePID&0x3f;
			if(ReceiveID == 0x33)   //从机需要反馈信号
			{
				send_lin_data[0] = MOTOR_ERROR_INFO;   //故障信息
				send_lin_data[1] = mccurrent_err;      //电机库自带故障
				send_lin_data[2] = g_work_mode;        //工作模式
				send_lin_data[3] = global_target_speed>>8;   //目标速度高位
				send_lin_data[4] = global_target_speed;      //目标速度低位
				send_lin_data[5] = global_actual_speed>>8;   //实际速度高位
				send_lin_data[6] = global_actual_speed;      //实际速度低位
				
				Lin_SentData(send_lin_data);
				LL_USART_TransmitData8(USART1,Lin_Checksum(ReceiveID,send_lin_data));
				while(!(USART1->ISR&USART_CR1_TXEIE_TXFNFIE));
			 
				DataProcess = 0;
				return ;
			}
			DataProcess = 2;
			return;
		}
		
		if(DataProcess == 2)
		{
			if(Data_Index<8)
			{
				LinReceiveData[Data_Index] = ReceiveData;
				Data_Index += 1;
				if(Data_Index == 8)
				{
					Data_Index = 0;
					DataProcess = 3;
					return ;
				}
			}	 
		}
		
		if(DataProcess == 3)
		{
			ReceiveCheckSum = ReceiveData;
			FrameReceiveOverFlag = 1;
			DataProcess = 0;
			Lin_LBD_Flag=DISABLE;//接收数据失能
		}
  }
 }
}
