#include<stdio.h>
#include "at24c02.h"
#include "app_handle.h"
#include "app_type.h"

uint8_t at24c02_writeonebyte(uint8_t writeaddr, uint8_t writedata)
{
  I2C_Start();
	I2C_WriteByte(0XA0);
	I2C_WaitAck();
	
	I2C_WriteByte(writeaddr); //24c02只有256字节，因此一个字节就够了，其他型号需要连发多个地址
	I2C_WaitAck();
	
	I2C_WriteByte(writedata);
	I2C_WaitAck();
	
	I2C_Stop();
	//HAL_Delay(10); //EEPROM 写入过程比较慢，需等待一点时间，再写下一次
}

uint8_t at24c02_readonebyte(uint8_t readaddr)
{
  uint8_t temp;
	I2C_Start();
	
	I2C_WriteByte(0XA0);  //0为发送模式
	I2C_WaitAck();
	
	I2C_WriteByte(readaddr); 
	I2C_WaitAck();
	
	I2C_Start();
	I2C_WriteByte(0XA1); //1为读取模式
	I2C_WaitAck();
	
	temp = I2C_ReadByte();
	I2C_Stop();
	return temp;
}


void at24c02_writelenbyte(uint8_t writeaddr, uint8_t* writedata, uint8_t writelen)
{
  for(int i=0; i<writelen;i++)
	{
	  at24c02_writeonebyte(writeaddr, writedata[i]);
		writeaddr++;
	}
}


void at24c02_readlenbyte(uint8_t readaddr, uint8_t *readdata, uint8_t readlen)
{
  for(int i=0; i<readlen;i++)
	{
	  readdata[i] = at24c02_readonebyte(readaddr);
		readaddr++;
	}
}


uint8_t at24c02_test(void)
{
  uint8_t temp;
	temp=at24c02_readonebyte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X66)
	{
		//printf("at24c02 test ok\r\n");
		return 0;		
	}		
	else//排除第一次初始化的情况
	{
		at24c02_writeonebyte(255,0X66);
	  temp=at24c02_readonebyte(255);	  
		if(temp==0X66)
		{
			//printf("at24c02 test ok\r\n");
			return 0;
		}
	}
	printf("at24c02 test fail\r\n");
	return 1;
}



/*******************************************
EEPROM地址----数据对应表
0     工作状态
1-2   转速
3-4   水压
*******************************************/
uint8_t ee_write_state; //存储是否故障
uint8_t ee_read_state; //读取是否故障

uint8_t ee_work_mode = MODE_NULL;
uint16_t ee_motor_speed = 0;
uint8_t ee_hydraulic_pressure = 0; //水压为小数0.5-6.0 转为整数存储5-60

//数据发生改变就重新存入
uint8_t ee_save_data(void)  //0--失败  1--成功
{
  if(ee_work_mode != g_work_mode)
	{
		ee_work_mode = g_work_mode;
	  at24c02_writeonebyte(0,ee_work_mode);
	}
	if(ee_motor_speed != target_speed)
	{
		ee_motor_speed = target_speed;
	  at24c02_writeonebyte(1, (uint8_t)ee_motor_speed>>8);
		at24c02_writeonebyte(2, (uint8_t)ee_motor_speed);
	}
	if(ee_hydraulic_pressure != target_pressure)
	{
		ee_hydraulic_pressure = target_pressure;
	  at24c02_writeonebyte(3, ee_hydraulic_pressure>>8);
		at24c02_writeonebyte(3, ee_hydraulic_pressure);
	}
	return 1;
}

uint8_t ee_data[10];
uint8_t ee_read_data(void)  //0--失败  1--成功
{
  at24c02_readlenbyte(0, ee_data, 6); //具体大小待定
	
	ee_work_mode = ee_data[0];
	ee_motor_speed = ee_data[1]<<8 | ee_data[2];
	ee_hydraulic_pressure = ee_data[3]<<8 | ee_data[4];
}

