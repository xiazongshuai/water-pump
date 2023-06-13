#ifndef _HAL_SOFT_IIC_H
#define _HAL_SOFT_IIC_H

#include "main.h"
#include "stm32g0xx_hal.h"  //放#include "main.h"前面报错？？？
//#include "gpio.h"


static GPIO_InitTypeDef GPIO_InitStruct;

#define SCL_PORT     GPIOB
#define SDA_PORT     GPIOB
 
#define SCL_PIN    GPIO_PIN_6
#define SDA_PIN    GPIO_PIN_7

void IIC_SDA_Mode_IN(void);
void IIC_SDA_Mode_OUT(void);	
void iic_delay_us(int num);
									 
#define SDA_H				HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)		 
#define SDA_L       HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
#define SCL_H       HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define SCL_L				HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)						 
#define SDA_Out()		IIC_SDA_Mode_OUT()									 
#define SDA_In()	  IIC_SDA_Mode_IN()
#define SDA_Read		HAL_GPIO_ReadPin(SDA_PORT,SDA_PIN)
#define  I2C_delay()  iic_delay_us(100)
	
void I2C_Start(void);
void I2C_Stop(void);		
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void I2C_WriteByte(uint8_t txd);
uint8_t I2C_ReadByte(void);

//uint8_t Single_Write(uint8_t Address, uint8_t REG_Address, uint8_t REG_data);
//uint8_t Single_Read(uint8_t Address, uint8_t REG_Address, uint8_t *data);
//uint8_t SequenceRead(uint8_t Address, uint8_t REG_Address, uint8_t *data, uint32_t size);
//uint8_t SequenceWrite(uint8_t addr, uint8_t reg, uint8_t len, const uint8_t *buf);

#endif
						 