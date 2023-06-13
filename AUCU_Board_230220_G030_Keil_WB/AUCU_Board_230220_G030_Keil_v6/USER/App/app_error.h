#ifndef _APP_ERROR_H
#define _APP_ERROR_H

#include "stm32g0xx_hal.h"

typedef enum 
{
 //null_err=0,
 locked_rotor_err,   //��ת����
 voltage_err,      //��ѹ����
 pressure_sensor_err,    //ѹ������������
 water_flow_sensor_err,  //ˮ������������
 water_shortage_err,   //ȱˮ
 water_leakage_err,     //��©
 motor_phase_loss_err,   //���ȱ��
 controller_overheat_err,  //���������ȹ���
 AD_bias_voltage_err,   //ADƫ�õ�ѹ�쳣
 IPM_voltage_err,    //IPM��ƽ����
 communication_rec_err,   //��巢��ͨ�Ź���
 parameter_saving_err,  //�����������
 parameter_reading_err,  //������ȡ����
 foc_durition_err,
 over_votlage_err,
 under_volttage_err,
 over_heat_err,
 startup_err,   //����ʧ��
 speedfbk_err,  //�ٶȷ�������	
 overcurrent_err,  //����
 software_err,
 remain1_err,
 remain2_err,
 communication_send_err,   //�������ͨ�Ź���
}ERROR_TYPE;


//ADƫ�õ�ѹ�쳣 0x8000  32768   5%���(�ݶ�) 34406 31130
#define AD_bias_lowvoltage  (32768-32768*0.10)  //ADƫ���������
#define AD_bias_upvoltage   (32768+32768*0.10)  //ADƫ���������

extern ERROR_TYPE MOTOR_ERROR;
extern uint8_t IPM_ErrFlag;
extern uint8_t EeSave_Flag;

void error_mode_init(void);
void error_mode_tick(void);
void error_check(void);
#endif