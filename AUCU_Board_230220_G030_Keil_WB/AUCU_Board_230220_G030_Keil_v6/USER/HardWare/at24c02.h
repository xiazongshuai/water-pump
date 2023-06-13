#ifndef _AT24C02_H
#define _AT24C02_H

#include "main.h"
#include "hal_soft_iic.h"

extern uint8_t ee_write_state; //�洢�Ƿ����
extern uint8_t ee_read_state; //��ȡ�Ƿ����

extern uint8_t ee_work_mode;
extern uint16_t ee_motor_speed;
extern uint8_t ee_hydraulic_pressure; //ˮѹΪС��0.5-6.0 תΪ�����洢5-60

uint8_t at24c02_writeonebyte(uint8_t writeaddr, uint8_t writedata);
uint8_t at24c02_readonebyte(uint8_t readaddr);
void at24c02_writelenbyte(uint8_t writeaddr, uint8_t* writedata, uint8_t writelen);
void at24c02_readlenbyte(uint8_t readaddr, uint8_t *readdata, uint8_t readlen);

uint8_t at24c02_test(void);

uint8_t ee_save_data(void);  //0--ʧ��  1--�ɹ�
uint8_t ee_read_data(void);  //0--ʧ��  1--�ɹ�

#endif