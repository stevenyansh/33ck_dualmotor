// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc2_service.c
 *
 * @brief This module implements motor control.
 *
 * Component: MOTOR CONTROL APPLICATION
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
*
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <libq.h>

#include "board_service.h"
#include "mc2_init.h"
#include "mc_app_types.h"
#include "mc2_service.h"
#include "tricycle_control.h"
#include "mc1_init.h"   //panjunjun add
#include "mc2_user_params.h"
#include "MC1_AutoHold.h"
#include "MC2_AutoHold.h"
#include "hal/qei2.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="VARIABLES ">

#define POS_SHORT 1
#define POS_LONG  2 

#define SPEED_ITERM_PAR_1       30000
#define SPEED_ITERM_PAR_2       20000
#define SPEED_ITERM_PAR_3       14000       
#define SPEED_ITERM_PAR_4       14000
#define SPEED_ITERM_PAR_5       8000
#define SPEED_ITERM_PAR_6       5000
#define SPEED_ITERM_PAR_7       0
#define SPEED_ITERM_PAR_8       0


#define SPEED_PTERM_PAR_1       10000
#define SPEED_PTERM_PAR_2       4200
#define SPEED_PTERM_PAR_3       3500
#define SPEED_PTERM_PAR_4       3500
#define SPEED_PTERM_PAR_5       900
#define SPEED_PTERM_PAR_6       900
#define SPEED_PTERM_PAR_7       0
#define SPEED_PTERM_PAR_8       0


MC2APP_DATA_T mc2;
MC2APP_DATA_T *pMC2Data = &mc2;

signed int ol_speed_rec_mc2;
signed int ol_current_rec_mc2;
signed char dir_cmd_mc2 = 1;
signed int qRotorAngleMB = 0; //need interface
unsigned char hallb_b;
signed int hallb_cap_last_b;
signed int hallb_cap_b;
signed int hallb_cap_value_b;
signed int angle_cap_360_b;
signed int angle_cap_360_inc_b;
unsigned char cap_360_on_flag_b;
unsigned int cap_360_on_count_b;
unsigned char hall_temp_b;
unsigned char hall_final_b;
unsigned char hall_pre_b;
unsigned char hall_last_b;
unsigned int hall_pwm_count_b;
unsigned char hall_error_count_b;
unsigned int msp_pwm_b;
unsigned int msp_pwm_4x_b;
unsigned char hall_change_flag_b;
unsigned int msp6_delta_b;
unsigned int msp6_last_b;
unsigned char speed_stab_count_b;
unsigned char speed_stab_flag_b;
unsigned int stab_hall_step_b;
unsigned int msp6_pwm_b = 9000;
unsigned int hall_pwm_count6_b = 8999;

/*******mc2 control************/
int MC2_q_currnet_cmd;  //Betty add 20210331
int motor_b_rpm;
int x2c_mc2_iq_cmd;
int x2c_angle_inc_val_mc2 = 10;

unsigned char x2c_motor_command_mc2 = 0;    //0 stop 1 fwd 2 rev 3 shake
int x2c_speed_mode_on_mc2 = 0;
signed int x2c_mc2_id_cmd_man = 12000;
signed int x2c_motor_speed_cmd_mc2;
uint8_t spd_update_flag_b;
signed int command_b_rpm;
signed int command_b_rpm_final;
signed int x2c_iq_close_b = 0;
signed int x2c_id_close_b = 0;
signed long x2c_iq_close_32_b = 0;
signed long x2c_iq_limit_32_b;
signed int x2c_iq_limit_b = 3000;

signed int motor_b_speed_delta;
signed long motor_b_speed_calc_temp;
signed long motor_b_speed_calc_temp_i;
signed int motor_b_command_fix;
signed int motor_b_command_sup;
signed int x2c_motor_b_rpm_delta;
signed int motor_b_rpm_old;
signed int x2c_wkp_mc2 = 30;
int16_t x2c_wki_mc2 = 0;
signed int x2c_wkd_mc2 = 0;
signed int x2c_is_scale_b = 20000;//300; //1000;
signed int x2c_ds_scale_b = 8000;//1000;
uint32_t still_count_b;
uint32_t still_update_mod_b = 35;
signed int x2c_delta_enl_factor_b = 2;
signed int lock_theta_angle_mc2;
unsigned int qei2_updang;
unsigned int qei2_counter;
unsigned int qei2_counter_old;
signed int delta2_qei_counter;
signed int qei2_angle_s16;
unsigned int qei2_counter_mod;
unsigned long qei2_angle_temp;
unsigned int qei2_angle_u16;
signed int qei2_angle_camp_a;
int8_t x2c_spi2_lock_flag;
uint32_t qei_lock_count2 = 0;
unsigned long qei2_velo_val_shrk = 0;
unsigned long x2c_rpm_vel_scale2 = 1500000;
unsigned long x2c_rpm_vel_scale_shrk2 = 150000;
signed int speed_rpm_mc2;
signed long speed_rpm_fltred_long_mc2;
signed int speed_rpm_fltred_mc2;
signed int speed_lpf_coff_mc2 = 500;
signed int speed_rpm_fltred_temp_mc2; 
uint32_t pwup_lock_count2;
int x2c_pos_mode_on_mc2 = 2;
unsigned int x2c_pos_cmd_mc2 = 13800;
unsigned int x2c_pos_cmd_old_mc2;
unsigned int pos_loop_counter_b; 
unsigned int x2c_pos_loop_mod_b = 1;
signed int x2c_pos_outmax_mc2 = 10000;
signed int x2c_max_delta_position_mc2 = 10000;
signed long delta_position_mc2;
signed int pos_outmax_final_mc2;
unsigned int x2c_pos_inc_val_mc2 = 10;


int16_t x2c_wki_mc2_k = 2;
int16_t x2c_wkp_mc2_k = 2;

int16_t x2c_wki_mc2_b = 900;
int16_t x2c_wkp_mc2_b = 5000;

uint8_t pos_state_mc2 = 0;
unsigned long x2c_spi2_pos_target;
unsigned long spi2_angle_read_u32;
signed int spi2_angle_s16;
unsigned int spi2_counter_mod; 
unsigned long spi2_angle_temp;
unsigned long spi2_angle_u17;
signed int spi2_angle_camp_a;
unsigned long spi2_angle_offset_u17;
signed int spi2_angle_offset_s16;
unsigned long spi2_angle_u32_old;
signed long spi2_angle_delta;

unsigned long qei2_vel_l =0;
unsigned long qei2_vel_h =0;
unsigned long qei2_velo_val = 0;
unsigned long qei2_vel_temp = 0;

unsigned long qei2_vel_temp2 = 0;
unsigned long qei2_vel_temp_old = 0;
unsigned int qei2_vel_count = 0;
unsigned int qei2_vel_count_num = 50;
unsigned int qei2_vel_zero = 1;


int x2c_wkp_dec_arr_mc2[20] = {120, 110, 100,  80,  80, 65,  52,  40, 35 , 30, 28, 26,20, 20, 6, 35 };
int x2c_wkd_dec_arr_mc2[20] = {4000,4000,3500,3000,2500,2300,1900,1600,1300,1000,900,600,450,350,40,1450};

int x2c_wkp_sht_arr_mc2[20] = {50, 50,  40,  35, 30,  28,  26,  24,  22,  20, 19, 17,  15, 6 };
int x2c_wkd_sht_arr_mc2[20] = {3000,2500,2300,2200 ,2000,1800,1600,1400,1200,1000,800,600,400,40};
//tiaoyou
//int x2c_speed_wkp_mc2[20] = {27000,26500, 26500,26500,
//26500, 14500, 14500, 14500, 14500, 14500, 14500, 14500, 8500, 100, 1000, 1000, 0, 0, 0};
//int x2c_speed_wki_mc2[20] = {10000,9900, 9500,9000,
//9000, 8000, 8000, 8000, 8000, 7000, 6000, 5000, 4000, 10, 4000, 500, 500, 0, 0};
//chuban:
//int x2c_speed_wkp_mc2[20] = {27000,26500, 26500,26500,
//26500, 26500, 26500, 26500, 26500, 26500, 26500, 26500, 26500, 300, 1000, 1000, 0, 0, 0};
//int x2c_speed_wki_mc2[20] = {10000,9900, 9500,9000,
//9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 100, 4000, 500, 500, 0, 0};

//int x2c_speed_wkp_mc2[20] = {27000,26500, 26500,26500,
//26500, 24500, 22500, 20500, 18500, 16500, 14500, 14500, 14500, 5000, 1000, 1000, 0, 0, 0};
//int x2c_speed_wki_mc2[20] = {10000,9900, 9500,9000,
//9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 1000, 4000, 500, 500, 0, 0};
//wanzhida 12x
int x2c_speed_wkp_mc2[20] = {2500,2500, 2500,2500,
2500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 100, 100, 0, 0, 0};
int x2c_speed_wki_mc2[20] = {2200,2200, 2200,2200,
2200, 2200, 2200, 2200, 1200, 1200, 1200, 1200, 1200, 1200, 400, 50, 50, 0, 0};

int x2c_speed_count = 0;
int x2c_speed_count_num = 100;
int x2c_motor_speed_cmd_mc2_old;
extern signed int x2c_wkp_hsp;
extern signed int x2c_wkd_hsp;
unsigned char x2c_run_motor_mc2;
extern int x2c_current_ramp;
extern uint8_t x2c_angle_open_loop;

extern uint8_t SPI1_read_lock;
extern uint8_t SPI2_read_lock;

uint16_t spi2_loop_count = 5;
uint16_t spi2_lock_count = 5;

void pos_control_mc2(void);
void QEI2_ANGLE_CALC(void);
void SPI2_ANGLE_CALC(void);
void speed_control_mc2(void);
void speed_control_test(void);
void speed_control_foc(void);
void pos_contol_short_mc2_test(void);
void pos_control_mc2_test(void);
// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MC2APP_StateMachine(MC2APP_DATA_T *);
static void MCAPP_MC2ReceivedDataProcess(MC2APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: void MC2APP_StateMachine (MC2APP_DATA_T *)  </B>
*
* @brief Application state machine.
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* @example
* <CODE> MC2APP_StateMachine(&mc); </CODE>
*
*/
static void MC2APP_StateMachine(MC2APP_DATA_T *pMCData)
{
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_LOAD_T *pLoad = pMCData->pLoad;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;
    
    switch(pMCData->appState)
    {
    case MCAPP_INIT:

        pMCData->HAL_PWMDisableOutputs();

        /* Stop the motor */
        pMCData->runCmd = 0;
        
//         if(MC2_dynamic_start==0)
        pMCData->MCAPP_ControlSchemeInit(pControlScheme);
        
        pMCData->MCAPP_InputsInit(pMotorInputs);
        pMCData->MCAPP_LoadInit(pLoad);       
        
        pMCData->appState = MCAPP_CMD_WAIT;

        break;
        
    case MCAPP_CMD_WAIT:
        if(pMCData->runCmd == 1)
        {
            pMCData->appState = MCAPP_OFFSET;
            if(!dir_cmd_mc2)
            {
                pMCData->pControlScheme->ctrlParam.OLCurrentMax = -ol_current_rec_mc2;
                pMCData->pMotor->qMaxOLSpeed = -ol_speed_rec_mc2;
            }
            else
            {
                pMCData->pControlScheme->ctrlParam.OLCurrentMax = ol_current_rec_mc2;
                pMCData->pMotor->qMaxOLSpeed = ol_speed_rec_mc2;
            }
        }
       break;
       
    case MCAPP_OFFSET:

        /* Measure Initial Offsets */
        pMCData->MCAPP_MeasureOffset(pMotorInputs);

        if(pMCData->MCAPP_IsOffsetMeasurementComplete(pMotorInputs))
        {
            pMCData->appState = MCAPP_LOAD_START_READY_CHECK;
        }

        break;

    case MCAPP_LOAD_START_READY_CHECK:
        
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);
        pMCData->MCAPP_LoadStateMachine(pLoad);
        
        if(pMCData->MCAPP_IsLoadReadyToStart(pLoad))
        {
            /* Load is ready, start the motor */
            pMCData->HAL_PWMEnableOutputs();

            pMCData->MCAPP_LoadStartTransition(pControlScheme, pLoad);

            pMCData->appState = MCAPP_RUN;
        }
        break;
            
    case MCAPP_RUN:
        
        /* Compensate motor current offsets */
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);

        pMCData->MCAPP_ControlStateMachine(pControlScheme);
        
        pMCData->MCAPP_LoadStateMachine(pLoad);
      
#if 0
        if (pMCData->runCmd == 0)
        {
            /* Set Target velocity to Open Loop Electrical Speed */
            pControlScheme->ctrlParam.qTargetVelocity = 
                                pMotor->qMaxOLSpeed;

            /* exit loop if motor not run */
            pMCData->appState = MCAPP_LOAD_STOP_READY_CHECK;
        }
#endif
        
        break;

    case MCAPP_LOAD_STOP_READY_CHECK:
        
        pMCData->MCAPP_LoadStateMachine(pLoad);
        
        /* Load is ready, stop the motor */
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);
        pMCData->MCAPP_ControlStateMachine(pControlScheme);
        
        if(pMCData->MCAPP_IsLoadReadyToStop(pLoad))
        {    
            pMCData->MCAPP_LoadStopTransition(pControlScheme, pLoad);
            pMCData->appState = MCAPP_STOP;
        }

        break;

    case MCAPP_STOP:
        pMCData->HAL_PWMDisableOutputs();
        pMCData->appState = MCAPP_INIT;

        break;
        
    case MCAPP_FAULT:
            
        break;
        
    default:
        pMCData->appState = MCAPP_FAULT;
        break;

    } /* end of switch-case */
    

    if(pControlScheme->faultStatus)
    {
        pMCData->HAL_PWMDisableOutputs();
        pMCData->appState = MCAPP_FAULT;
    }
}

/**
* <B> Function: MC2_ADC_INTERRUPT()  </B>
*
* @brief ADC interrupt vector ,and it performs following actions:
*        (1) Reads DC BUS voltage and updates appropriate variables.
*        (2) Reads motor 2 phase currents,bus current and phase voltage
*            feedbacks from ADC data buffers.
*        (3) Executes Field Oriented Control based on the current,voltage
*            feedbacks.
*        (4) Loads duty cycle values generated by FOC to the registers
*            of PWM Generators controlling motor 2.
*/
extern MC1APP_DATA_T *pMC1Data;
int16_t mc2_bus_current_flg = 0;
uint8_t test_speed = 0;
void __attribute__((__interrupt__,no_auto_psv)) MC2_ADC_INTERRUPT()
{
    int16_t adcBuffer;
    static uint16_t spi_loop,count;
    if(pwup_lock_count2 < 16000)
        pwup_lock_count2 ++;
    else if(pwup_lock_count2 == 16000)
    {
        x2c_spi2_lock_flag = 1;
        pwup_lock_count2 ++;
    }
    
    
//            spi2_angle_read_u32 =  SPI2_read_17bit();     
//            spi2_angle_u17 = spi2_angle_read_u32 >> 1;       
    
//    if(SPI1_read_lock)
//        SPI1_read_lock = 0;
//    
//    if(SPI1_read_lock == 0)
//    {
//        SPI2_read_lock = 1;
//        spi2_angle_read_u32 =  SPI2_read_17bit();     
//        spi2_angle_u17 = spi2_angle_read_u32 >> 1;      
//        
//        SPI2_read_lock = 0;
//    }
    
    if(SPI1_read_lock == 0)
    {
        SPI2_read_lock = 1;

        if(spi_loop > spi2_loop_count)
        {
            spi_loop = 0;
            spi2_angle_read_u32 =  SPI2_read_17bit();     
            spi2_angle_u17 = spi2_angle_read_u32 >> 1;          
        }            
        else
        {
            spi_loop++;            
        }  

            
        if(count >spi2_lock_count)
        {
            count = 0;
             SPI2_read_lock = 0;       
        }
        else
            count++;                
    }    

//    if(SPI2_read_lock)
//        SPI2_read_lock = 0;
//    else
//        SPI2_read_lock=1;
    //cpu_temperature = ADCBUF24;
    pMC2Data->HAL_MotorInputsRead(pMC2Data->pMotorInputs);
    
    qei2_counter = QEI2_PositionCountRead();
    while(qei2_counter >= 16384)
    {
        qei2_counter -= 16384;
    }
    
    if(qei2_counter > qei2_counter_old)
    {
        if((qei2_counter > 16000)&&(qei2_counter_old < 100))
        {
            qei2_updang = 1;
        }
        else
        {
            qei2_updang = 0;
        }
    }
    else if(qei2_counter < qei2_counter_old)
    {
        if((qei2_counter_old > 16000)&&(qei2_counter < 100))
        {
            qei2_updang = 0;
        }
        else
        {
            qei2_updang = 1;
        }
    }
    qei2_counter_old = qei2_counter;

    qei2_vel_l = INT2HLDL;
    qei2_vel_h = INT2HLDH;
    qei2_velo_val = (unsigned long)qei2_vel_h << 16;
    qei2_velo_val += qei2_vel_l;


     if(qei2_velo_val < 50)
        qei2_velo_val = 50;
    if(qei2_velo_val >= 65535)
    {
        qei2_velo_val_shrk = __builtin_divud(qei2_velo_val , 10);
        qei2_vel_temp = __builtin_divud(x2c_rpm_vel_scale_shrk2 , qei2_velo_val_shrk);
    }
    else
    {
        if(qei2_velo_val != 0)
            qei2_vel_temp = __builtin_divud(x2c_rpm_vel_scale2 ,(unsigned int)qei2_velo_val);
        else
            qei2_vel_temp = 0;        
    }
//        qei_velo_val = 65535;
    if(qei2_vel_zero)
    {
        if(qei2_vel_temp == qei2_vel_temp_old)
            qei2_vel_count++;
        else
            qei2_vel_count = 0;

        if(qei2_vel_count > qei2_vel_count_num)
        {
            qei2_vel_temp2 = qei2_vel_temp;
            qei2_vel_temp = 0;
            qei2_vel_count = 0;
        }

        if(qei2_vel_temp2 == qei2_vel_temp)
        {
            qei2_vel_temp = 0;
        }

        if(qei2_vel_temp != 0)
            qei2_vel_temp_old = qei2_vel_temp;        
    }
    

//    if(qei_velo_val != 0)
//        qei_vel_temp = __builtin_divud(x2c_rpm_vel_scale , qei_velo_val);
//    else
//        qei_vel_temp = 0;
    
    if(qei2_updang == 1)
    {
        qei2_vel_temp = 0 - qei2_vel_temp;
    }
    
    speed_rpm_fltred_temp_mc2 = (signed long)speed_rpm_fltred_long_mc2 >> 16;
    speed_rpm_fltred_long_mc2 = speed_rpm_fltred_long_mc2 + __builtin_mulss((qei2_vel_temp - speed_rpm_fltred_temp_mc2), speed_lpf_coff_mc2);
    speed_rpm_fltred_mc2 = (signed long)speed_rpm_fltred_long_mc2 >> 16;
    
    
    
    if(still_count_b >= still_update_mod_b)
    {

        spd_update_flag_b = 1;
        still_count_b = 0;
        
        //spi2_angle_delta = spi2_angle_read_u32 - spi2_angle_u32_old;
        motor_b_rpm = speed_rpm_fltred_mc2;
    }
    else
        still_count_b ++;

    

    
    if(x2c_pos_mode_on_mc2 == 1)
    {
        if(pos_loop_counter_b >= x2c_pos_loop_mod_b)
        {
            pos_loop_counter_b = 0;
            pos_control_mc2();           
            //speed_control_mc2();
        }
        else
            pos_loop_counter_b ++;        
    }
    else if(x2c_pos_mode_on_mc2 == 2)
    {
        if(pos_loop_counter_b >= x2c_pos_loop_mod_b)
        {
            pos_loop_counter_b = 0;
            pos_control_mc2_test();
            if(x2c_run_motor_mc2)
                speed_control_foc();            
            //speed_control_mc2();
        }
        else
            pos_loop_counter_b ++;        
    }
    else if(x2c_pos_mode_on_mc2 == 3)
    {
        if(pos_loop_counter_b >= x2c_pos_loop_mod_b)
        {
            pos_loop_counter_b = 0;
            pos_control_mc2();         
            speed_control_mc2();
        }
        else
            pos_loop_counter_b ++;        
    }       
    else if(x2c_speed_mode_on_mc2 == 1)
    {
        if(x2c_motor_speed_cmd_mc2 > 0)
            speed_control_mc2();
        else
            x2c_mc2_iq_cmd = 0;
    }
    else if(x2c_speed_mode_on_mc2 == 2)
    {
        if(x2c_run_motor_mc2)
            speed_control_foc();
    }
    
    if(x2c_angle_open_loop == 1)
    {
        QEI2_ANGLE_CALC();
        if(x2c_motor_command_mc2 == 4)
            pMC2Data->controlScheme.ctrlParam.OLTheta = lock_theta_angle_mc2;
        else
            pMC2Data->controlScheme.ctrlParam.OLTheta += x2c_angle_inc_val_mc2;
//        if(x2c_spi2_lock_flag)
//        {
//            pMC2Data->controlScheme.ctrlParam.OLTheta = 0;
//            spi2_angle_offset_u17 = spi2_angle_u17;
//            while(spi2_angle_offset_u17 >= 18724)
//            {
//                spi2_angle_offset_u17 -= 18724;
//            }
//            spi2_angle_temp = __builtin_muluu(spi2_angle_offset_u17 , 32768);
//            spi2_angle_offset_s16 = __builtin_divud(spi2_angle_temp , 18724);
//        }
    }
    else if(x2c_angle_open_loop == 2)
    {
//        if(x2c_spi2_lock_flag)
//        {
//            pMC2Data->controlScheme.ctrlParam.OLTheta = 0;
//            spi2_angle_offset_u17 = spi2_angle_u17;
//            while(spi2_angle_offset_u17 >= 18724)
//            {
//                spi2_angle_offset_u17 -= 18724;
//            }
//            spi2_angle_temp = __builtin_muluu(spi2_angle_offset_u17 , 32768);
//            spi2_angle_offset_s16 = __builtin_divud(spi2_angle_temp , 18724);
////            QEI1_PositionCount16bitWrite(0);
//        }
//        else
//        {
//            SPI2_ANGLE_CALC();
//            pMC2Data->controlScheme.ctrlParam.OLTheta = spi2_angle_s16 - spi2_angle_offset_s16;
//        }
 
        if(x2c_spi2_lock_flag)
        {
            pMC2Data->controlScheme.ctrlParam.OLTheta = 0;
            QEI2_PositionCount16bitWrite(0);
        }
        else
        {
            QEI2_ANGLE_CALC();
            pMC2Data->controlScheme.ctrlParam.OLTheta = qei2_angle_s16 + qei2_angle_camp_a;
        }        
    }
    
    MC2APP_StateMachine(pMC2Data); 
    pMC2Data->HAL_PWMSetDutyCycles(pMC2Data->pPWMDuty);
    
    adcBuffer = MC2_ClearADCIF_ReadADCBUF();

    MC2_ClearADCIF();
}

void MCAPP_MC2ServiceInit(void)
{
    MCAPP_MC2ParamsInit(pMC2Data);
}
int32_t Velocity_value = 0;    //panjunjun add for test piIq.qInRef
void MCAPP_MC2InputBufferSet(int16_t runCmd, 
                                    int16_t qTargetVelocity)
{
    MC2APP_DATA_T   *pMCData = pMC2Data;
    MCAPP_MOTOR_T   *pMotor = pMC2Data->pMotor;
    signed long  q_current_cmd_tmp;
    int16_t      q_Target_Velocity_temp;
    int16_t      MC2_q_currnet_cmd_temp;

    command_b_rpm = x2c_motor_speed_cmd_mc2;
    if(x2c_spi2_lock_flag)
    {
        //flag_value = 13;         
        if(qei_lock_count2 <= 16000)
        {
            qei_lock_count2 ++;
            if(runCmdMC2 == 0)
            {
                runCmdMC2 = 1;
                pMCData->controlScheme.piId.qOut=0;
                pMCData->controlScheme.piId.qdSum=0;
                pMCData->controlScheme.piIq.qOut=0;
                pMCData->controlScheme.piIq.qdSum=0; 
                HAL_MC2PWMEnableOutputs();   
            }   
            else
            {
                pMCData->controlScheme.piId.qInRef = -x2c_mc2_id_cmd_man;
            }
        }
        else
        {
            x2c_spi2_lock_flag = 0;
            qei_lock_count2 = 0;
            pMCData->controlScheme.piId.qInRef = 0;
            runCmdMC2 = 0;
            HAL_MC2PWMDisableOutputs();
        }
    }
    else if(x2c_run_motor_mc2 != 0)
    {
        if(runCmdMC2 == 0)
        {
            runCmdMC2 = 1;
            pMCData->controlScheme.piId.qOut=0;
            pMCData->controlScheme.piId.qdSum=0;
            pMCData->controlScheme.piIq.qOut=0;
            pMCData->controlScheme.piIq.qdSum=0; 
            x2c_iq_close_32_b = 0;
            x2c_iq_close_b = 0;
            HAL_MC2PWMEnableOutputs();   
        }
        else 
        {
            MC2_q_currnet_cmd = x2c_mc2_iq_cmd;
            if(x2c_speed_mode_on_mc2)
            {
                pMCData->controlScheme.piIq.qInRef = x2c_mc2_iq_cmd;
            }
            else
            {         
               pMCData->controlScheme.piIq.qInRef = x2c_mc2_iq_cmd; 
//            if(pMCData->controlScheme.piIq.qInRef < MC2_q_currnet_cmd)     //Betty add
//                pMCData->controlScheme.piIq.qInRef += x2c_current_ramp;
//            else if (pMCData->controlScheme.piIq.qInRef > MC2_q_currnet_cmd)
//                pMCData->controlScheme.piIq.qInRef -= x2c_current_ramp; 
//            else
//                pMCData->controlScheme.piIq.qInRef = x2c_mc2_iq_cmd;
            }
        }
    }
    else if(x2c_run_motor_mc2 == 0)
    {
        MC2_q_currnet_cmd = 0;
        if(runCmdMC2 == 1)
        {
            if(pMCData->controlScheme.piIq.qInRef > x2c_current_ramp)//CURENTRAMP)
                pMCData->controlScheme.piIq.qInRef -= x2c_current_ramp;//CURENTRAMP; 
            else if(pMCData->controlScheme.piIq.qInRef <  -x2c_current_ramp)
                pMCData->controlScheme.piIq.qInRef += x2c_current_ramp;//CURENTRAMP; 
            else
            {
                runCmdMC2 = 0;
                HAL_MC2PWMDisableOutputs();
            }
        }
    }

   
    pMCData->runCmdBuffer = runCmd; //Betty change 20210407
    MCAPP_MC2ReceivedDataProcess(pMCData);
}

int16_t MCAPP_MC2GetTargetVelocity(void)
{
    return pMC2Data->motorInputs.measurePot;
}

static void MCAPP_MC2ReceivedDataProcess(MC2APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;

    pMCData->runCmd = pMCData->runCmdBuffer;
}








void SPI2_ANGLE_CALC(void)
{
    spi2_counter_mod = __builtin_modud(spi2_angle_u17 , 18724); //131072/7 = 18724
    spi2_angle_temp = __builtin_muluu(spi2_counter_mod , 32768);
    spi2_angle_temp = __builtin_divud(spi2_angle_temp , 9362);//18724/2 = 9362
    spi2_angle_s16 = (signed long)(spi2_angle_temp - Q15(0.9999));
}

void QEI2_ANGLE_CALC(void)
{
    qei2_counter_mod = __builtin_modud(qei2_counter , 2341);
    qei2_angle_temp = __builtin_muluu(qei2_counter_mod , 32768);
    qei2_angle_temp = __builtin_divud(qei2_angle_temp , 1170);//16384/7/2 ;// 4096/7/2
    qei2_angle_s16 = (signed long)(qei2_angle_temp - Q15(0.9999));
}

void pos_to_speed_mc2(void)
{
    if(spi2_angle_read_u32 > x2c_spi2_pos_target)
//    if(qei2_counter > x2c_pos_cmd_mc2)
    {  
        if(x2c_motor_speed_cmd_mc2 > -pos_outmax_final_mc2)
           x2c_motor_speed_cmd_mc2 -= x2c_pos_inc_val_mc2;
        else
            x2c_motor_speed_cmd_mc2 = -pos_outmax_final_mc2;    
    }
    else if(spi2_angle_read_u32 < x2c_spi2_pos_target)
//    else if(qei2_counter < x2c_pos_cmd_mc2) 
    {
        if(x2c_motor_speed_cmd_mc2 < pos_outmax_final_mc2)
            x2c_motor_speed_cmd_mc2 += x2c_pos_inc_val_mc2;
        else
            x2c_motor_speed_cmd_mc2 = pos_outmax_final_mc2;
        
    } 
}

void pos_to_speed_dec_mc2(void)
{
    if(qei2_counter > x2c_pos_cmd_mc2)
    {  
        if(x2c_motor_speed_cmd_mc2 < -pos_outmax_final_mc2)
           x2c_motor_speed_cmd_mc2 += x2c_pos_inc_val_mc2;
        else
            x2c_motor_speed_cmd_mc2 = -pos_outmax_final_mc2;    
    }
    else if(qei2_counter < x2c_pos_cmd_mc2) 
    {
        if(x2c_motor_speed_cmd_mc2 > pos_outmax_final_mc2)
            x2c_motor_speed_cmd_mc2 -= x2c_pos_inc_val_mc2;
        else
            x2c_motor_speed_cmd_mc2 = pos_outmax_final_mc2;
    }     
}

void pos_contol_short_mc2(void)
{
    signed long t32_outmax;
  
    if((delta_position_mc2 > 300) && (delta_position_mc2 <= 500))//???*5=motor_speed_cmd:1000-2000
    { 
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[0];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[0];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 100) && (delta_position_mc2 <= 300))//???*5=motor_speed_cmd:500-1000
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[1];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[1]; 
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }
    else if((delta_position_mc2 > 80) && (delta_position_mc2 <= 100))//???*5=motor_speed_cmd:400-500
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[2];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[2];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }
    else if((delta_position_mc2 > 70) && (delta_position_mc2 <= 80))//???*5=motor_speed_cmd:350-400
    { 
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[3];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[3];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    } 
    else if((delta_position_mc2 > 60) && (delta_position_mc2 <= 70))//???*5=motor_speed_cmd:300-350
    {   
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[4];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[4];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }  
    else if((delta_position_mc2 > 50) && (delta_position_mc2 <= 60))//???*5=motor_speed_cmd:200-300
    {   
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[5];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[5];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }  
    else if((delta_position_mc2 > 40) && (delta_position_mc2 <= 50))//???*5=motor_speed_cmd:200-300
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[6];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[6];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }  
    else if((delta_position_mc2 > 30) && (delta_position_mc2 <= 40))//???*5=motor_speed_cmd:50-100
    {    
         x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[7];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[7];
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }     
    else if((delta_position_mc2 > 20) && (delta_position_mc2 <= 30))//???*5=motor_speed_cmd:50-100
    {   
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[8];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[8]; //700  
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 10) && (delta_position_mc2 <= 20))
    {      
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[9];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[9];//550
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    } 
    else if((delta_position_mc2 > 7) && (delta_position_mc2 <= 10))
    {   
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[10];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[10];//550
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }     
    else if((delta_position_mc2 > 5) && (delta_position_mc2 <= 7))
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[11];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[11];//550
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }             
    else if((delta_position_mc2 > 3) && (delta_position_mc2 <= 5))
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[12];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[12]; //50        
            t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }   
    else
    {
        x2c_wkp_mc2 =  x2c_wkp_sht_arr_mc2[13];
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[13]; //50  
        t32_outmax = __builtin_mulss(delta_position_mc2 , 2000);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);   
    }
//    t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
//    pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);       

    
}


/*
int x2c_speed_wki_mc2[20] = {30000, 20000,14000,
14000, 8000, 5000};
int x2c_speed_wkp_mc2[20] = {10000, 4200,3500,
3500, 900, 900};
 */
int x2c_wkd_count =10;
void pos_contol_short_mc2_test(void)
{

    signed long t32_outmax;
    int x2c_motor_speed_cmd_mc2_tmp = 0;
    
    if(delta_position_mc2 > 10000)
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[0];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[0];
    }    
    else if((delta_position_mc2 > 5000) && (delta_position_mc2 <= 10000))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[1];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[1];  
    }
    else if((delta_position_mc2 > 4000) && (delta_position_mc2 <= 5000))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[2];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[2];  

    }    
    else if((delta_position_mc2 > 3000) && (delta_position_mc2 <= 4000))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[3];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[3]; 
    }
    else if((delta_position_mc2 > 2000) && (delta_position_mc2 <= 3000))
    { 
        x2c_wki_mc2 =  x2c_speed_wki_mc2[4];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[4];
    } 
    else if((delta_position_mc2 > 1000) && (delta_position_mc2 <= 2000))
    {   
        x2c_wki_mc2 =  x2c_speed_wki_mc2[5];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[5];
    }
    else if((delta_position_mc2 > 800) && (delta_position_mc2 <= 1000))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[6];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[6]; 
    }
    else if((delta_position_mc2 > 500) && (delta_position_mc2 <= 800))
    { 
        x2c_wki_mc2 =  x2c_speed_wki_mc2[7];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[7];
    } 
    else if((delta_position_mc2 > 300) && (delta_position_mc2 <= 500))
    {   
        x2c_wki_mc2 =  x2c_speed_wki_mc2[8];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[8];
    }
    else if((delta_position_mc2 > 200) && (delta_position_mc2 <= 300))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[9];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[9]; 
        
    }  
   else if((delta_position_mc2 > 100) && (delta_position_mc2 <= 200))
    {
        x2c_wki_mc2 =  x2c_speed_wki_mc2[10];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[10]; 
        
    }  
    
    else if((delta_position_mc2 > 50) && (delta_position_mc2 <= 100))
    {   
        x2c_wki_mc2 =  x2c_speed_wki_mc2[11];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[11];
    }   
    else if((delta_position_mc2 > 30) && (delta_position_mc2 <= 50))
    {   
        x2c_wki_mc2 =  x2c_speed_wki_mc2[12];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[12];        
    }       
    else 
    {   
        x2c_wki_mc2 =  x2c_speed_wki_mc2[13];
        x2c_wkp_mc2 = x2c_speed_wkp_mc2[13];        
    }       
    
    
    if(delta_position_mc2 > 10000)
    {
        pos_outmax_final_mc2 = 10000;
        //pos_to_speed_mc2();        
    }
    else 
    {
        t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
    }
//  else if (delta_position_mc2 > 20)
//    {
//        t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
//        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    }   
//  else if (delta_position_mc2 > 10)
//    {
//        t32_outmax = __builtin_mulss(delta_position_mc2 , 8000);
//        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    }   
//  else if (delta_position_mc2 > 5)
//    {
//        t32_outmax = __builtin_mulss(delta_position_mc2 , 6000);
//        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    }   
//  else if (delta_position_mc2 > 3)
//    {
//        t32_outmax = __builtin_mulss(delta_position_mc2 , 5000);
//        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    }   
//  else if (delta_position_mc2 > 0)
//    {
//        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    } 
//  else
//    {
//        t32_outmax = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
//        pos_outmax_final_mc2 = 1;//__builtin_divsd(t32_outmax , x2c_max_delta_position_mc2);           
//    } 
    
}

char count_flag = 0; 
void pos_contol_long_mc2(void)
{
    signed long t32_outmax2;
    signed long wkd_temp2;
    signed long wkp_temp2;
    
    if(delta_position_mc2 > 2000)
    {
        pos_outmax_final_mc2 = 10000;   
        x2c_wkp_mc2 = x2c_wkp_hsp;//80
        x2c_wkd_mc2 = x2c_wkd_hsp;//4000;
    }
    else if((delta_position_mc2 > 1500) && (delta_position_mc2 <= 2000))//???*5=motor_speed_cmd:2000-5000
    {
        wkp_temp2 = __builtin_mulss(delta_position_mc2 , x2c_wkp_hsp);
        x2c_wkp_mc2 = __builtin_divsd(wkp_temp2 , 2000) ;       
        wkd_temp2 = __builtin_mulss(delta_position_mc2 , x2c_wkd_hsp);
        x2c_wkd_mc2 = __builtin_divsd(wkd_temp2 , 2000);      
        t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 1000) && (delta_position_mc2 <= 1500))//???*5=motor_speed_cmd:2000-5000
    {
        x2c_wkp_mc2 = 95 + __builtin_divsd(delta_position_mc2 , 100) ;       
        x2c_wkd_mc2 = 3000 + delta_position_mc2;//__builtin_divsd(wkd_temp1 , 1500);    
        t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }   
    else if((delta_position_mc2 > 500) && (delta_position_mc2 <= 1000))//???*5=motor_speed_cmd:2000-5000
    {      
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[1];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[1];    
        t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 300) && (delta_position_mc2 <= 500))//???*5=motor_speed_cmd:1000-2000
    {      
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[2];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[2];    
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 100) && (delta_position_mc2 <= 300))//???*5=motor_speed_cmd:500-1000
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[3];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[3];     
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }
    else if((delta_position_mc2 > 80) && (delta_position_mc2 <= 100))//???*5=motor_speed_cmd:400-500
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[4];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[4];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }
    else if((delta_position_mc2 > 70) && (delta_position_mc2 <= 80))//???*5=motor_speed_cmd:350-400
    { 
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[5];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[5];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    } 
    else if((delta_position_mc2 > 60) && (delta_position_mc2 <= 70))//???*5=motor_speed_cmd:300-350
    {    
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[6];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[6];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }  
    else if((delta_position_mc2 > 50) && (delta_position_mc2 <= 60))//???*5=motor_speed_cmd:200-300
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[7];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[7];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   

    }  
    else if((delta_position_mc2 > 40) && (delta_position_mc2 <= 50))//???*5=motor_speed_cmd:200-300
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[15];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[15];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }  
    else if((delta_position_mc2 > 30) && (delta_position_mc2 <= 40))//???*5=motor_speed_cmd:50-100
    {     
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[8];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[8];
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }     
    else if((delta_position_mc2 > 20) && (delta_position_mc2 <= 30))//???*5=motor_speed_cmd:50-100
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[9];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[9]; //700  
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }    
    else if((delta_position_mc2 > 10) && (delta_position_mc2 <= 20))
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[10];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[10];//550
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    } 
    else if((delta_position_mc2 > 7) && (delta_position_mc2 <= 10))
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[11];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[11];//550
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }     
    else if((delta_position_mc2 > 5) && (delta_position_mc2 <= 7))
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[12];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[12];//550
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }             
    else if((delta_position_mc2 > 3) && (delta_position_mc2 <= 5))
    {
        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[13];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[13]; //50    
                t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = __builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }   
    else if((delta_position_mc2 > 0) && (delta_position_mc2 <= 3))
    {

        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[14];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[14];           
        t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = 0;//__builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }   
    else 
    {

        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[15];
        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[15];           
       // t32_outmax2 = __builtin_mulss(delta_position_mc2 , x2c_pos_outmax_mc2);
        pos_outmax_final_mc2 = 0;//__builtin_divsd(t32_outmax2 , x2c_max_delta_position_mc2);   
    }       
//    if(x2c_speed_count > x2c_speed_count_num)
//    {
//        count_flag = 1;
//        x2c_speed_count = 0;
//        
//    }
//    if(count_flag)
//    {
//        x2c_wkp_mc2 =  x2c_wkp_dec_arr_mc2[15];
//        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[15];  
//    }

       
    
}

void pos_control_mc2(void)
{
    delta_position_mc2 = abs((signed int)qei2_counter - (signed int)x2c_pos_cmd_mc2);

    if(x2c_pos_cmd_mc2 != x2c_pos_cmd_old_mc2)
    {
        count_flag =0;
        x2c_pos_cmd_old_mc2 = x2c_pos_cmd_mc2;
        pos_state_mc2 = (delta_position_mc2 <= 500) ? POS_SHORT :POS_LONG;
    }
    switch(pos_state_mc2)
    {
        case POS_SHORT:
            pos_contol_short_mc2();
            break;
        case POS_LONG:
            pos_contol_long_mc2();
            break;
    }
    pos_to_speed_mc2();
}

void pos_control_mc2_test(void)
{
//    delta_position_mc2 = labs((signed int)qei2_counter - (signed int)x2c_pos_cmd_mc2);
    delta_position_mc2 = labs((signed long)x2c_spi2_pos_target - (signed long)spi2_angle_read_u32);  
    if(x2c_pos_cmd_mc2 != x2c_pos_cmd_old_mc2)
    {
        count_flag =0;
        x2c_pos_cmd_old_mc2 = x2c_pos_cmd_mc2;
        pos_state_mc2 = (delta_position_mc2 <= 500) ? POS_SHORT :POS_LONG;
    }
    switch(pos_state_mc2)
    {
        case POS_SHORT:
            pos_contol_short_mc2_test();
            break;
        case POS_LONG:
            pos_contol_short_mc2_test();
            break;
    }
    pos_to_speed_mc2();
}

int pos_outmax_flag = 0;
void speed_control_mc2(void)
{
//    if(pos_outmax_flag == 0)
//    {
//        if(x2c_motor_speed_cmd_mc2 < pos_outmax_final_mc2)
//        {
//            x2c_motor_speed_cmd_mc2 += x2c_pos_inc_val_mc2;  
//        }    
//        else
//        {
//            x2c_motor_speed_cmd_mc2 = pos_outmax_final_mc2;        
//        }        
//    }
//
//
//    speed_control_test();
    //x2c_motor_speed_cmd_mc2 +=10;
    x2c_iq_limit_32_b = __builtin_mulss(x2c_iq_limit_b , x2c_is_scale_b);
    if(spd_update_flag_b)
    {
        test_speed = 1;
        spd_update_flag_b = 0;
        
        if(command_b_rpm > 15000)
            command_b_rpm_final = 15000;
        else if(command_b_rpm < -15000)
            command_b_rpm_final = -15000;
        else
            command_b_rpm_final = command_b_rpm;
        
        motor_b_speed_delta = command_b_rpm_final - motor_b_rpm;
        
        if(motor_b_speed_delta > 1500)
            motor_b_speed_delta = 1500;
        else if(motor_b_speed_delta < -1500)
            motor_b_speed_delta = -1500;
        
        motor_b_speed_calc_temp = __builtin_mulss(motor_b_speed_delta , x2c_wkp_mc2);
//              motor_a_command_fix = __builtin_divsd(motor_a_speed_calc_temp ,x2c_is_scale);        
        x2c_iq_close_32_b += motor_b_speed_calc_temp;
        
        if(motor_b_speed_delta > 100)
            motor_b_speed_delta = 100;
        else if(motor_b_speed_delta < -100)
            motor_b_speed_delta = -100;
        if(x2c_run_motor_mc2)
        {
            motor_b_speed_calc_temp = __builtin_mulss(motor_b_speed_delta , x2c_wki_mc2);
            motor_b_speed_calc_temp_i +=motor_b_speed_calc_temp;        
            x2c_iq_close_32_b += motor_b_speed_calc_temp_i;            
        }
        else
        {
            motor_b_speed_calc_temp_i = 0;
        }
        
        x2c_motor_b_rpm_delta = motor_b_rpm - motor_b_rpm_old;
        
        if(x2c_motor_b_rpm_delta > 100)
            x2c_motor_b_rpm_delta = 100;
        else if(x2c_motor_b_rpm_delta < -100)
            x2c_motor_b_rpm_delta = -100;
        
        motor_b_rpm_old = motor_b_rpm;

        motor_b_speed_calc_temp = __builtin_mulss(x2c_motor_b_rpm_delta, x2c_delta_enl_factor_b);
        if(motor_b_speed_calc_temp > 32767)
            x2c_motor_b_rpm_delta = 32767;
        else if(motor_b_speed_calc_temp < -32767)
            x2c_motor_b_rpm_delta = -32767;
        else
            x2c_motor_b_rpm_delta = motor_b_speed_calc_temp;

        motor_b_speed_calc_temp = __builtin_mulss(x2c_motor_b_rpm_delta, x2c_wkd_mc2);
        //          motor_a_command_sup = __builtin_divsd(motor_a_speed_calc_temp ,x2c_ds_scale);??
        x2c_iq_close_32_b -= motor_b_speed_calc_temp;

        if(x2c_iq_close_32_b > x2c_iq_limit_32_b)
            x2c_iq_close_32_b = x2c_iq_limit_32_b;
        if(x2c_iq_close_32_b < (0 - x2c_iq_limit_32_b))
            x2c_iq_close_32_b = (0 - x2c_iq_limit_32_b);

        x2c_iq_close_b = __builtin_divsd(x2c_iq_close_32_b ,x2c_is_scale_b);

        x2c_mc2_iq_cmd = x2c_iq_close_b;
    }
}
int speed_test_count;
uint32_t speed_test_count_num = 50000;
int speed_test_pd_count;
int speed_test_pd_count_num = 10000;
int motor_b_rpm_diff;
int motor_b_rpm_diff_flag;
int motor_b_rpm_diff_k_num = 100;

int motor_b_rpm_diff_k = Q15(-0.0005);
int motor_b_rpm_diff_k2 = Q15(-0.0005);

int motor_b_rpm_diff_count = 0;
int motor_b_rpm_diff_count_num = 100;
int x2c_wkd_dec_value = 40;
int x2c_wkp_dec_value = 1;
int motor_b_rpm_value = 100;
void speed_control_test(void)
{
    static char speed_test_flag = 0;  
    static char speed_test_flag2 = 0;  
    int  x2c_wkd_mc2_tmp = 0;   
    int  x2c_wkp_mc2_tmp = 0; 
    int x2c_wkpd_period = 0;
    if(pos_outmax_final_mc2 != x2c_motor_speed_cmd_mc2_old)
    {
        speed_test_flag = 1;
        motor_b_rpm_diff_flag = 0;
        pos_outmax_flag = 0;
        x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[5];
        x2c_wkp_mc2 = x2c_wkp_sht_arr_mc2[5];
        x2c_motor_speed_cmd_mc2_old = pos_outmax_final_mc2;
    }
    
    if(speed_test_flag)
            speed_test_count++;
    
    if(speed_test_count>speed_test_count_num)
    {
        pos_outmax_flag = 1;
        speed_test_count = 0;
        x2c_motor_speed_cmd_mc2 = 1;
        speed_test_flag = 0;
        speed_test_flag2 = 1;
    }
    
    if(speed_test_flag2)
    {
        if(motor_b_rpm < motor_b_rpm_value)
        {
            motor_b_rpm_diff_flag = 1;
        }
        speed_test_pd_count++;
    }
    
    if(motor_b_rpm_diff_flag)
    {
        motor_b_rpm_diff_count++;
        if(motor_b_rpm_diff_count > motor_b_rpm_diff_count_num)
        {
            motor_b_rpm_diff_count =0;
            if(x2c_wkd_mc2 > 50)
            {
                //x2c_wkd_mc2_tmp = x2c_wkd_dec_arr_mc2[14] - x2c_wkd_sht_arr_mc2[5]; 
                //x2c_wkpd_period = __builtin_divsd(speed_test_pd_count_num ,motor_b_rpm_diff_count_num);
                x2c_wkd_mc2 -= x2c_wkd_dec_value;
                
            }
            else
            {
                x2c_wkd_mc2 = 50;
            }

            if(x2c_wkp_mc2 > 10)        
            {
//                x2c_wkp_mc2_tmp = x2c_wkp_dec_arr_mc2[14] - x2c_wkp_sht_arr_mc2[5];      
//                x2c_wkpd_period = __builtin_divsd(speed_test_pd_count_num ,motor_b_rpm_diff_count_num);            
                x2c_wkp_mc2 -= x2c_wkp_dec_value;
            }
            else
            {               
                x2c_wkp_mc2 = 10;            
            } 
//            if((x2c_wkp_mc2 == x2c_wkp_sht_arr_mc2[14]) && (x2c_wkd_mc2 = x2c_wkd_sht_arr_mc2[14]))
//                motor_b_rpm_diff_flag = 0;
        }
    }
    if(speed_test_pd_count > speed_test_pd_count_num)
    {
        speed_test_flag2 = 0;
        speed_test_pd_count = 0;
        //motor_b_rpm_diff_flag = 0;
//        x2c_wkd_mc2 = x2c_wkd_dec_arr_mc2[14];
//        x2c_wkp_mc2 = x2c_wkp_dec_arr_mc2[14];        
    }
        
    
}

int16_t speed_qTargetVelocity;
int16_t speed_qDiff;
int16_t speed_qVelRef;
int16_t speed_qTargetVelocity;       
int16_t speedRampSkipCnt;
int16_t speedRampIncLimit = 10;
int16_t speed_CLSpeedRampRate = 10;
int16_t speedRampDecLimit = 10;

int64_t speed_qKpOut;
int16_t speed_qKpScaleInt = 2;
int32_t speed_qdSumUnsat;
int32_t speed_qdSum;
int32_t speed_qdUnSatOutSum;
int16_t speed_qOutMax = 20000;
int16_t speed_qOutMin = -20000;
int16_t speed_qOut;
int16_t speed_delta;
int32_t speed_calc_temp;
int16_t speed_d_rpm;
int32_t speed_qdSumUnsat_max = 20000000;
int32_t speed_qdSumUnsat_min = -20000000;

int16_t speed_enl = 8;

float x2c_wki_mc2_value_float = 0.02;

void speed_PIController(void)
{  
    int32_t currentError;
    int32_t speed_qOutMax_32bit, speed_qOutMin_32bit ,speed_qdSumUnsat_tmp,speed_tmp;
    
    currentError =  (int32_t)speed_qVelRef - (int32_t)motor_b_rpm;  
    if(currentError > 32767)
        currentError = 32767;
    else if(currentError < -32767)
        currentError = -32767;    
    
    speed_qKpOut  = __builtin_mulss(currentError, x2c_wkp_mc2);
    if (speed_qKpScaleInt > 0)
    {
        speed_qKpOut  = speed_qKpOut  << (int64_t)speed_qKpScaleInt;
    }
    
    if(speed_qKpOut > 2147483647)
        speed_qKpOut = 2147483647;
    else if(speed_qKpOut < -2147483647)
        speed_qKpOut = -2147483647;
    
    speed_qdSumUnsat_tmp = __builtin_mulss(currentError, x2c_wki_mc2);    
    speed_qdSumUnsat = (int32_t)speed_qdSumUnsat_tmp >> (int32_t)speed_enl;    
    speed_qdSumUnsat = speed_qdSum + speed_qdSumUnsat;
    
    
//    speed_tmp = __builtin_mulss(motor_b_rpm, motor_b_rpm_old); 
//    if(speed_tmp < 0)
//    {
//        speed_qdSumUnsat = 0;
//    }
//     

    speed_delta = motor_b_rpm - motor_b_rpm_old;       
    motor_b_rpm_old = motor_b_rpm;
    speed_calc_temp = __builtin_mulss(speed_delta, x2c_wkd_mc2);    

    speed_calc_temp = speed_calc_temp << 10;
    if(speed_calc_temp >= speed_qdSumUnsat_max)
    {
        speed_calc_temp = speed_qdSumUnsat_max;
    }
    else if(speed_calc_temp <= speed_qdSumUnsat_min)
    {
        speed_calc_temp = speed_qdSumUnsat_min;        
    }              

    if(speed_qKpOut + speed_qdSumUnsat > 2147483647)
        speed_qdUnSatOutSum = 2147483647;
    else if(speed_qKpOut + speed_qdSumUnsat < -2147483647)
        speed_qdUnSatOutSum = -2147483647;        
    else
        speed_qdUnSatOutSum = speed_qKpOut + speed_qdSumUnsat - speed_calc_temp;

//    speed_qdUnSatOutSum = speed_qKpOut + speed_qdSumUnsat - speed_calc_temp;
    
  
    speed_qOutMax_32bit = ((int32_t)speed_qOutMax) << (int32_t)12;
    speed_qOutMin_32bit = ((int32_t)speed_qOutMin) << (int32_t)12;
    
	if(speed_qdUnSatOutSum >=  speed_qOutMax_32bit)
    {        
		speed_qOut =  speed_qOutMax;
    }
	else if(speed_qdUnSatOutSum <= speed_qOutMin_32bit)
    {
		speed_qOut =  speed_qOutMin;
    }
	else
    {
		speed_qOut = (int16_t)(speed_qdUnSatOutSum >> 12);
        speed_qdSum = speed_qdSumUnsat;
    }
}



void speed_control_foc(void)
{
    speed_qVelRef = x2c_motor_speed_cmd_mc2;
     /*speed_qTargetVelocity = x2c_motor_speed_cmd_mc2;
    
   speed_qDiff = speed_qVelRef - (int16_t)speed_qTargetVelocity;

    if(speed_qDiff < 0)
    {
        if(speedRampSkipCnt >= speedRampIncLimit)
        {
            speedRampSkipCnt = 0;
            speed_qVelRef = speed_qVelRef + speed_CLSpeedRampRate;
        }
    }
    else if(speed_qDiff > 0)
    {
        if(speedRampSkipCnt >= speedRampDecLimit)
        {
            speedRampSkipCnt = 0;
            speed_qVelRef = speed_qVelRef - speed_CLSpeedRampRate;
        }
    }
    else
    {
        speedRampSkipCnt = 0;
    }
    speedRampSkipCnt++;*/

    /* Execute Outer Speed Loop - Generate Iq Reference Generation */

    speed_PIController();
    x2c_mc2_iq_cmd = speed_qOut;    
}