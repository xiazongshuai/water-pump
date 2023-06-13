#include "app_type.h"
#include "main.h"


//����---->���
uint16_t data_power;     //����
uint16_t data_speed;     //ת��
uint16_t data_pressure;  //ˮѹ
uint8_t  data_temperature; //ģ���¶�
uint16_t data_voltage;  //�����ѹ
uint16_t data_version = 100;   //�汾
uint16_t data_day;      //ͨ������

uint16_t actual_motor_speed;  //ʵ��ת��
uint32_t ERR_INFO_SEND;   //������Ϣ


//���---->����
uint16_t target_speed;    //�趨ת��/100
uint8_t target_pressure;  //�趨ˮѹ*10
uint8_t ac_inerpressure;  //ʵ�ʽ�ˮѹ
uint8_t ac_outpressure;   //ʵ�ʳ�ˮѹ
uint32_t ERR_INFO_REC;
uint8_t set_mode;
uint8_t water_temperature;
uint8_t ke_temperature;
uint8_t discharge_water;



uint32_t day_ms;
void get_motor_work_day(void)
{
  day_ms = HAL_GetTick();
	data_day = day_ms/1000; ///60/60/24;
}

//�ָ���������
void restore_factory_settings(void)
{
	ERR_INFO_SEND = 0;
	ERR_INFO_REC = 0;
	MC_AcknowledgeFaultMotor1();  //�������
	target_speed = 800;
  target_pressure = 2.0;
}
