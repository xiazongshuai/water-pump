#include "app_auto.h"
#include "app_type.h"
#include "usart_handle.h"

//自动调节转速使水压稳定在设定值

float actual_water_pre = 0.0;
int motor_pid_speed = 0;
int auto_work_time;

static int tmp_set_speed = 0;
static int tmp_act_speed = 0;
static int time_3s_flag = 0;

int tmp_hydraulic_pressure;

struct motor_pid_struct
{
  float setpoint;
	float p;
	float i;
	float d;
	int error;
	float increment; 
	int lasterror; //上一次误差
	int preverror; //上上次误差
	int uk;
};
struct motor_pid_struct motor_pid;


void wheel_pid_struct_init(struct motor_pid_struct *P, float p, float i, float d)
{
	P->setpoint = 0;
	P->p = p;
	P->i = i;
	P->d = d;
	P->error = 0;
	P->increment = 0; 
	P->lasterror = 0; //上一次误差
	P->preverror = 0; //上上次误差
	P->uk = 0;
}

int speed_pid_control(struct motor_pid_struct *p, float measual_speed)
{
	p->error = p->setpoint - measual_speed;
	p->increment =  p->p*(p->error - p->lasterror)
					+ p->i*p->error
					+ p->d*(p->error- 2*p->lasterror + p->preverror);
	p->preverror = p->lasterror;
	p->lasterror = p->error;
	p->uk += p->increment;
	return p->uk;
}

void auto_mode_init(void)
{
//	printf("auto_mode\r\n");
}

void auto_mode_tick(void)
{
	motor_pid.setpoint = target_pressure;  //设定的水压
	 
  actual_water_pre = ac_outpressure; //获取实际的水压
  motor_pid_speed = speed_pid_control(&motor_pid,actual_water_pre);  //由水压设置转速
	
	if(tmp_set_speed != motor_pid_speed)
	{
		tmp_set_speed = motor_pid_speed;
	  MC_ProgramSpeedRampMotor1(motor_pid_speed/6,1000);
	  MC_StartMotor1();
	}
	
}


