#ifndef _AT24C02_H
#define _AT24C02_H

#include "main.h"
#include "hal_soft_iic.h"

extern uint8_t ee_write_state; //存储是否故障
extern uint8_t ee_read_state; //读取是否故障

extern uint8_t ee_work_mode;
extern uint16_t ee_motor_speed;
extern uint8_t ee_hydraulic_pressure; //水压为小数0.5-6.0 转为整数存储5-60

uint8_t at24c02_writeonebyte(uint8_t writeaddr, uint8_t writedata);
uint8_t at24c02_readonebyte(uint8_t readaddr);
void at24c02_writelenbyte(uint8_t writeaddr, uint8_t* writedata, uint8_t writelen);
void at24c02_readlenbyte(uint8_t readaddr, uint8_t *readdata, uint8_t readlen);

uint8_t at24c02_test(void);

uint8_t ee_save_data(void);  //0--失败  1--成功
uint8_t ee_read_data(void);  //0--失败  1--成功

#endif