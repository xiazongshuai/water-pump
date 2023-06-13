#ifndef _APP_ERROR_H
#define _APP_ERROR_H

#include "stm32g0xx_hal.h"

typedef enum 
{
 //null_err=0,
 locked_rotor_err,   //堵转故障
 voltage_err,      //电压故障
 pressure_sensor_err,    //压力传感器故障
 water_flow_sensor_err,  //水流传感器故障
 water_shortage_err,   //缺水
 water_leakage_err,     //渗漏
 motor_phase_loss_err,   //电机缺相
 controller_overheat_err,  //控制器过热故障
 AD_bias_voltage_err,   //AD偏置电压异常
 IPM_voltage_err,    //IPM电平故障
 communication_rec_err,   //面板发来通信故障
 parameter_saving_err,  //参数保存故障
 parameter_reading_err,  //参数读取故障
 foc_durition_err,
 over_votlage_err,
 under_volttage_err,
 over_heat_err,
 startup_err,   //启动失败
 speedfbk_err,  //速度反馈故障	
 overcurrent_err,  //过流
 software_err,
 remain1_err,
 remain2_err,
 communication_send_err,   //发给面板通信故障
}ERROR_TYPE;


//AD偏置电压异常 0x8000  32768   5%误差(暂定) 34406 31130
#define AD_bias_lowvoltage  (32768-32768*0.10)  //AD偏置误差下限
#define AD_bias_upvoltage   (32768+32768*0.10)  //AD偏置误差上限

extern ERROR_TYPE MOTOR_ERROR;
extern uint8_t IPM_ErrFlag;
extern uint8_t EeSave_Flag;

void error_mode_init(void);
void error_mode_tick(void);
void error_check(void);
#endif