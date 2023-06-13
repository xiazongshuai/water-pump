#include "hal_soft_iic.h"


/*****************************************
  SDA����ת��Ϊ OUT���ģʽ(���ģʽ��ֹͣ ��ʼ�ź�) 
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
  SDA����ת��Ϊ ����ģʽ(����ģʽ������������) 
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


//ע�⣬�������������ݹ����У���SCL���ڸߵ�ƽʱ��SDA�ϵ�ֵ��Ӧ�ñ仯����ֹ�������һ��ֹͣ����
//����ÿ����һ��������󶼽�SCL����-->ǯס����  ��ÿ�ʼ��������ǰҲ��SCL����

void I2C_Start(void)
{
	SDA_Out();
	SDA_H;
	SCL_H;
	I2C_delay();
	SDA_L;
	I2C_delay(); //��ʱ�ȶ�
	SCL_L;    //����SCL��ǯ��IIC����
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


//��������Ӧ���ź�ACK
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

//����������Ӧ���ź�NACK
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

//�ȴ�Ӧ���źŵ���
//ÿ��������ӻ�������һ���ֽڵ����ݣ�����������Ҫ�ȴ��ӻ�����һ��Ӧ���źţ���ȷ�ϴӻ��Ƿ�ɹ����յ�������
//**ÿ����һ���ֽڣ�8��bit��**��һ���ֽڴ����8��ʱ�Ӻ�ĵھŸ�ʱ���ڼ䣬�������������ݺ�����һ��ACKӦ���źŸ����������������ܽ������ݴ���
uint8_t I2C_WaitAck(void) 
{
	uint8_t timeout = 0;
	SCL_L;
	SDA_H;
	I2C_delay();
	SDA_In(); 
	SCL_H;
	I2C_delay();
	while(SDA_Read)  //Ӧ��0  ��Ӧ��1
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
