#include "app_type.h"
#include "main.h"


//主板---->面板
uint16_t data_power;     //功率
uint16_t data_speed;     //转速
uint16_t data_pressure;  //水压
uint8_t  data_temperature; //模块温度
uint16_t data_voltage;  //输出电压
uint16_t data_version = 100;   //版本
uint16_t data_day;      //通电天数

uint16_t actual_motor_speed;  //实际转速
uint32_t ERR_INFO_SEND;   //故障信息


//面板---->主板
uint16_t target_speed;    //设定转速/100
uint8_t target_pressure;  //设定水压*10
uint8_t ac_inerpressure;  //实际进水压
uint8_t ac_outpressure;   //实际出水压
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

//恢复出厂设置
void restore_factory_settings(void)
{
	ERR_INFO_SEND = 0;
	ERR_INFO_REC = 0;
	MC_AcknowledgeFaultMotor1();  //清除故障
	target_speed = 800;
  target_pressure = 2.0;
}
