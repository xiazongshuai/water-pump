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
  int16_t Iq_ref;
  int16_t Id_ref;
} Iqd_ref_t;

typedef struct
{
  int16_t Iq_act;
  int16_t Id_act;
} Iqd_act_t;

typedef struct
{
  int16_t I_a;
  int16_t I_b;
} Iab_t;


typedef struct
{
  int16_t W_CNT_A;
  int16_t W_CNT_B;
	int16_t W_CNT_C;
}MY_W;

typedef struct
{
	Iab_t				w_Iab_digital;	//digital value from ADC sample
	ab_f_t 			w_Iab_physical;	//unit:A
	Iqd_act_t		w_Iqd_act;
	Vqd_t				w_Vqd;
	Iqd_ref_t 	w_Iqd_ref;
	
	int16_t			w_speed;							//unit:rpm
	int16_t		 	w_hMeasuredSpeed_rpm;	//unit:rpm
  int16_t 		w_hTargetSpeed_rpm;		//unit:rpm
	
	int16_t 		w_hElAngle;	
	
	MY_W        my_w;
} S_WATCH_DATA;





extern	S_WATCH_DATA		watch_data;

#endif