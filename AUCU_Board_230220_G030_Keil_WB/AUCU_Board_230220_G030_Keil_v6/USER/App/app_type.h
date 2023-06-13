#ifndef _APP_TYPE_H
#define _APP_TYPE_H

#include "stm32g0xx_hal.h"
#include <stdio.h>

#define  power_limit    700     //��������700W

#define  display_panel_id  0XAA    //��ʾ�����
#define  main_panle_id     0X55    //���ذ����
#define  control_cmd       0X10    //����ָ��

//����---->���
extern uint16_t data_power;     //����
extern uint16_t data_speed;     //ת��
extern uint16_t data_pressure;  //ˮѹ
extern uint8_t data_temperature; //ģ���¶�
extern uint16_t data_voltage;  //�����ѹ
extern uint16_t data_version;  //�汾
extern uint16_t data_day;      //ͨ������
extern uint16_t actual_motor_speed;  //ʵ��ת��
extern uint32_t ERR_INFO_SEND;   //������Ϣ


//���---->����
extern uint16_t target_speed;    //�趨ת��/100
extern uint8_t target_pressure;  //�趨ˮѹ*10
extern uint8_t ac_inerpressure;  //ʵ�ʽ�ˮѹ
extern uint8_t ac_outpressure;   //ʵ�ʳ�ˮѹ
extern uint32_t ERR_INFO_REC;    //������Ϣ
extern uint8_t set_mode;          //ģʽ����
extern uint8_t water_temperature;  //ˮ��
extern uint8_t ke_temperature;     //����
extern uint8_t discharge_water;   //ˮ����


#endif