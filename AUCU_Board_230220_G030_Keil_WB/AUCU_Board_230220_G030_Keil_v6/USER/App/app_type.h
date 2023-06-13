#ifndef _APP_TYPE_H
#define _APP_TYPE_H

#include "stm32g0xx_hal.h"
#include <stdio.h>

#define  power_limit    700     //功率限制700W

#define  display_panel_id  0XAA    //显示板代号
#define  main_panle_id     0X55    //主控板代号
#define  control_cmd       0X10    //控制指令

//主板---->面板
extern uint16_t data_power;     //功率
extern uint16_t data_speed;     //转速
extern uint16_t data_pressure;  //水压
extern uint8_t data_temperature; //模块温度
extern uint16_t data_voltage;  //输出电压
extern uint16_t data_version;  //版本
extern uint16_t data_day;      //通电天数
extern uint16_t actual_motor_speed;  //实际转速
extern uint32_t ERR_INFO_SEND;   //故障信息


//面板---->主板
extern uint16_t target_speed;    //设定转速/100
extern uint8_t target_pressure;  //设定水压*10
extern uint8_t ac_inerpressure;  //实际进水压
extern uint8_t ac_outpressure;   //实际出水压
extern uint32_t ERR_INFO_REC;    //故障信息
extern uint8_t set_mode;          //模式设置
extern uint8_t water_temperature;  //水温
extern uint8_t ke_temperature;     //壳温
extern uint8_t discharge_water;   //水流量


#endif