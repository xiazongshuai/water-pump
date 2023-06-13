
///**
//  ******************************************************************************
//  * @file    drive_parameters.h
//  * @author  Motor Control SDK Team, ST Microelectronics
//  * @brief   This file contains the parameters needed for the Motor Control SDK
//  *          in order to configure a motor drive.
//  *
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under Ultimate Liberty license
//  * SLA0044, the "License"; You may not use this file except in compliance with
//  * the License. You may obtain a copy of the License at:
//  *                             www.st.com/SLA0044
//  *
//  ******************************************************************************
//  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/
#include "pmsm_motor_parameters.h"
#include "main.h"
/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/***** user defined motort parameters*****/
#define  USER_CURRENT_1A        19859//19859  
#define  USER_IQMAX             (USER_CURRENT_1A*1.35)
#define  USER_NOMINAL_CURRENT   (USER_CURRENT_1A*0.5)
#define  USER_ID_DEMAG          (-USER_CURRENT_1A*0.5)
#define  USER_MAX_SPEED         4500
#define  USER_RSHUNT            0.1   //180W��Ϊ100��ŷ--19859--0.1      10��ŷ--1985--0.01
#define  USER_AMPLIFICATION_GAIN   10

#define  USER_RS         		 16.5 //32
#define  USER_LS         		 0.07 //0.138
#define  USER_LD          	 0.07 //0.138
#define  USER_KE          		42.39
#define  USER_POLE_PAIR_NUM   2


//#define weak_flux

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       MOTOR_MAX_SPEED_RPM	//3600//3200 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS 3 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD             0.25 /*!<Maximum accepted
                                                            variance on speed
                                                            estimates (percentage) */
																														
																													
//10��ŷ F1:16384   F2:4096  G1:-5257   G2:24185
//100��ŷ F1:16384   F2:16384  G1:-5257   G2:9678

/* State observer scaling factors F1 */
#define F1                               16384
#define F2                               16384//4096
#define F1_LOG                           LOG2((F1))
#define F2_LOG                           LOG2((F2))

/* State observer constants */
#define GAIN1                          -5257
#define GAIN2                          4839 //9678//24185
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                  299   //213
#define PLL_KI_GAIN                  13   //9
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2((PLL_KPDIV))
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP               64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2((64))

#define STO_FIFO_DEPTH_UNIT              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in the unit defined by #SPEED_UNIT */
#define BEMF_CONSISTENCY_TOL             64   /* Parameter for B-emf
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN            64   /* Parameter for B-emf
                                                           amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR 0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   16000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   1500/*1100*/ /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                       number of PWM cycles */


/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT        2997/20  //2954/20 
#define PID_TORQUE_KI_DEFAULT        2826/20  //2740/20 
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           PID_TORQUE_KP_DEFAULT//2954/20
#define PID_FLUX_KI_DEFAULT           PID_TORQUE_KI_DEFAULT//2740/20 
#define PID_FLUX_KD_DEFAULT           PID_TORQUE_KD_DEFAULT//100


/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                     32   //16 
#define TF_KIDIV                     2048 //1024 
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                LOG2((32))     //LOG2((16)) 
#define TF_KIDIV_LOG                LOG2((2048))   //LOG2((1024)) 
#define TF_KDDIV_LOG                  LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE




/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       ( uint16_t )1000 /*!<Execution rate of speed
                                                     regulation loop (Hz) */
#if 0
#define PID_SPEED_KP_DEFAULT         2898//12000//2898/(SPEED_UNIT/10)  /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT         37/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      4096//1024	//512//4096 //16384
#define SP_KIDIV                      16384//4096//2048//1024	//256//16384
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2((SP_KPDIV)) //LOG2((16384))
#define SP_KIDIV_LOG                  LOG2((SP_KIDIV))
#define SP_KDDIV_LOG                  LOG2((16))
#endif

#define PID_SPEED_KP_DEFAULT         96000//12000//2898/(SPEED_UNIT/10)  /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT         296//37/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      4096//512//4096//1024	//512//4096 //16384
#define SP_KIDIV                      16384//2048//16384//2048 //4096//2048//1024	//256//16384
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2((SP_KPDIV)) //LOG2((16384))
#define SP_KIDIV_LOG                  LOG2((SP_KIDIV))
#define SP_KDDIV_LOG                  LOG2((16))


/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          USER_IQMAX//USER_CURRENT_1A*1.35 

/* Default settings */
#define DEFAULT_CONTROL_MODE           MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM       600//400//524//1152
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V          390//18 /*!< Over-voltage threshold */

#define UD_VOLTAGE_THRESHOLD_V          60//10 /*!< Under-voltage threshold */

#ifdef NOT_IMPLEMENTED

#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */

#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */

#define OVP_INVERTINGINPUT_MODE         INT_MODE
#define OVP_INVERTINGINPUT_MODE2        INT_MODE
#define OVP_SELECTION                   COMP_Selection_COMP1
#define OVP_SELECTION2                  COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                1000 /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT         (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT           USER_CURRENT_1A*0.1
/* Phase 2 */
#define PHASE2_DURATION                5000//2560 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT        (800*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT           USER_CURRENT_1A*0.1	
/* Phase 3 */
#define PHASE3_DURATION                0 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT        (800*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT           USER_CURRENT_1A*0.1
/* Phase 4 */
#define PHASE4_DURATION                0 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT        (800*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT           USER_CURRENT_1A*0.1	
/* Phase 5 */
#define PHASE5_DURATION                0 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT        (800*SPEED_UNIT/U_RPM)// 
#define PHASE5_FINAL_CURRENT           USER_CURRENT_1A*0.1

#define ENABLE_SL_ALGO_FROM_PHASE      2
/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG             0  /*!< degrees [0...359] */
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          600//400//524//1152

#define NB_CONSECUTIVE_TESTS           2 /* corresponding to
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         17 /*!< It expresses how much
                                                            estimated speed can exceed
                                                            forced stator electrical
                                                            without being considered wrong.
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  /*!< It expresses how much
                                                             estimated speed can be below
                                                             forced stator electrical
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */

#define TRANSITION_DURATION            25  /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(79)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (7 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/
//flux
#define FW_VOLTAGE_REF                985 /*!<Vs reference, tenth
                                                        of a percent */
#define FW_KP_GAIN                    1  // 3000/20 /*!< Default Kp gain */
#define FW_KI_GAIN                    65 // 5000/20 /*!< Default Ki gain */
#define FW_KPDIV                      32768
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KIDIV                      32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG                  LOG2((32768))
#define FW_KIDIV_LOG                  LOG2((32768))

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/