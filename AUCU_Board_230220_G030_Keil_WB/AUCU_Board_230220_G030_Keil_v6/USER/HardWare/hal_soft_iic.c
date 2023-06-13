#include "hal_soft_iic.h"


/*****************************************
  SDA引脚转变为 OUT输出模式(输出模式给停止 开始信号) 
******************************************/
void IIC_SDA_Mode_OUT(void)
{
  GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}
/*****************************************
  SDA引脚转变为 输入模式(输入模式传输具体的数据) 
******************************************/
void IIC_SDA_Mode_IN(void)
{
  GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}


void iic_delay_us(int num)
{
  while(64*num--)
	{
	
	}
}


//注意，在正常传输数据过程中，当SCL处于高电平时，SDA上的值不应该变化，防止意外产生一个停止条件
//所以每处理一个数据最后都将SCL拉低-->钳住总线  最好开始处理数据前也将SCL拉低

void I2C_Start(void)
{
	SDA_Out();
	SDA_H;
	SCL_H;
	I2C_delay();
	SDA_L;
	I2C_delay(); //延时稳定
	SCL_L;    //拉低SCL，钳主IIC总线
}


void I2C_Stop(void)
{
	SCL_L;
	SDA_Out();
	SDA_L; 
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_L;
}


//主机产生应答信号ACK
void I2C_Ack(void)
{
	SCL_L;
	SDA_Out();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
}

//主机不产生应答信号NACK
void I2C_NoAck(void)
{
	SCL_L;
	SDA_Out();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
}

//等待应答信号到来
//每当主机向从机发送完一个字节的数据，主机总是需要等待从机给出一个应答信号，以确认从机是否成功接收到了数据
//**每发送一个字节（8个bit）**在一个字节传输的8个时钟后的第九个时钟期间，接收器接收数据后必须回一个ACK应答信号给发送器，这样才能进行数据传输
uint8_t I2C_WaitAck(void) 
{
	uint8_t timeout = 0;
	SCL_L;
	SDA_H;
	I2C_delay();
	SDA_In(); 
	SCL_H;
	I2C_delay();
	while(SDA_Read)  //应答0  非应答1
	{
		timeout++;
		if (timeout > 250) 
		{
			I2C_Stop();
			return 0;
		}
	}
	SCL_L; 
	return 1;
}


void I2C_WriteByte(uint8_t txd) 
{
	uint8_t t;
	SCL_L;
	SDA_Out();								
	for (t = 0; t < 8; t++) 
	{
		if ((txd & 0x80) >> 7) 
		{
			SDA_H;
		}
		else
			SDA_L;

		txd <<= 1;
		I2C_delay();
		SCL_H;
		I2C_delay();
		SCL_L;
	}
}

uint8_t I2C_ReadByte(void)
{
	uint8_t i, receive = 0;
	SCL_L;
	SDA_In();	
	for (i = 0; i < 8; i++) 
	{
		SCL_L;
		I2C_delay();
		SCL_H;
		receive <<= 1;
		I2C_delay();
		if (SDA_Read)
			receive++;
	}
	SCL_L;
	return receive; 
}
