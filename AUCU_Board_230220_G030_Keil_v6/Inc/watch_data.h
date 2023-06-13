/**
  ******************************************************************************
  * @file    watch_data.h
  ******************************************************************************
  * @ingroup motor control
  */
	
#ifndef WATCH_DATA_H
#define WATCH_DATA_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"


typedef struct
{
  int16_t Vq;
  int16_t Vd;
} Vqd_t;

typedef struct
{
  int16_t Iq_ref_digital;
  int16_t Id_ref_digital;
} Iqd_ref_t;

typedef struct
{
  int16_t Iq_act_digital;
  int16_t Id_act_digital;
} Iqd_act_t;

typedef struct
{
  int16_t I_u_digital;
  int16_t I_v_digital;
} I_uv_t;

typedef struct
{
  float I_u_physical;
  float I_v_physical;
} I_uv_f_t;

typedef struct
{
  float Iq_ref_physical;
  float Id_ref_physical;
} Iqd_ref_phy;

typedef struct
{
  float Iq_act_physical;
  float Id_act_physical;
} Iqd_act_phy;

typedef struct
{
	I_uv_t			w_I_uv_digital;	//digital value from ADC sample
	I_uv_f_t 		w_I_uv_physical;	//unit:A
	Iqd_act_t		w_Iqd_act;
	Vqd_t				w_Vqd;
	Iqd_ref_t 	w_Iqd_ref;
	Iqd_ref_phy	w_Iqd_ref_phy;
	Iqd_act_phy	w_Iqd_act_phy;	
	
	int16_t			w_speed;							//unit:rpm
	int16_t		 	w_hMeasuredSpeed_rpm;	//unit:rpm
  int16_t 		w_hTargetSpeed_rpm;		//unit:rpm
	
	int16_t 		w_hElAngle;		

  uint16_t  CntPhA;                                    /**< PWM Duty cycle for phase A. */
  uint16_t  CntPhB;                                    /**< PWM Duty cycle for phase B. */
  uint16_t  CntPhC;                                    /**< PWM Duty cycle for phase C. */
	
  uint8_t   ctrl_mode; 
	int16_t			w_max_current_multi_1000;
	uint16_t		w_hPWMFrequency;		//Hz
	
	uint8_t			w_flag_circle_limit;
} S_WATCH_DATA;


extern	S_WATCH_DATA		watch_data;

#endif
