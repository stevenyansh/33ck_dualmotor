// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_service.c
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
#include "mc1_init.h"
#include "mc_app_types.h"
#include "mc1_service.h"
#include "tricycle_control.h"
#include "mc2_init.h"
#include "mc1_user_params.h" 
#include "MC1_AutoHold.h"
#include "qei1.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="VARIABLES ">

#define POS_SHORT 1
#define POS_LONG  2 


MC1APP_DATA_T mc1;
MC1APP_DATA_T *pMC1Data = &mc1;
unsigned char led_on_flag;


signed int hall_angle_old_mc1;

signed int ol_speed_rec_mc1;
signed int ol_current_rec_mc1;
signed char dir_cmd_mc1 = 0;

unsigned char hall_f_tab[8] = {0,3,6,2,5,1,4,0};
unsigned char hall_r_tab[8] = {0,5,3,1,6,4,2,0};
//154623
signed int hall_theta_tab_st[8] = {0,Q15(0.1666),Q15(-0.5),Q15(-0.1666),Q15(0.8333),Q15(0.5),Q15(-0.8333),0}; //315462
signed int hall_theta_tab_ed[8] = {0,Q15(0.5),Q15(-0.1666),Q15(0.1666),Q15(-0.8333),Q15(0.8333),Q15(-0.5),0};

signed int hall_theta_tab_st_r[8] = {0,Q15(0.5),Q15(-0.1666),Q15(0.1666),Q15(-0.8333),Q15(0.8333),Q15(-0.5),0}; //326451
signed int hall_theta_tab_ed_r[8] = {0,Q15(0.1666),Q15(-0.5),Q15(-0.1666),Q15(0.8333),Q15(0.5),Q15(-0.8333),0};

signed int hall_theta_tab_md[8] = {0,Q15(0.3333),Q15(-0.3333),Q15(0),Q15(0.9999),Q15(0.6666),Q15(-0.6666),0};

/////////////////////////////////////////////////////////////////////////////////////
signed int qRotorAngleMA = 0;  //need interface 
signed int motor_a_rpm;   //need interface 
unsigned char hallb_a;


signed int hallb_cap_last_a;
signed int hallb_cap_a;
signed int hallb_cap_value_a;

signed int angle_cap_360_a;
signed int angle_cap_360_inc_a;

unsigned char cap_360_on_flag_a;
unsigned int cap_360_on_count_a;

unsigned char hall_temp_a;
unsigned char hall_final_a;
unsigned char hall_pre_a;
unsigned char hall_last_a;
unsigned int hall_pwm_count_a;
unsigned char hall_error_count_a;
unsigned int msp_pwm_a;
unsigned int msp_pwm_4x_a;
unsigned char hall_change_flag_a;
unsigned int msp_pwm_hl_last_a[8] = {0,1500,1500,1500,1500,1500,1500,0};

//unsigned char hall_sw_tab_a[8] = {0,1,2,3,4,5,6,0}; //Jiecang Motor
unsigned char hall_sw_tab_a[8] = {0,2,1,3,4,6,5,0}; //Jabil Motor

unsigned int msp6_delta_a;
unsigned int msp6_last_a;
unsigned char speed_stab_count_a;
unsigned char speed_stab_flag_a;
unsigned int stab_hall_step_a;
unsigned int msp6_pwm_a = 9000;
unsigned int hall_pwm_count6_a = 8999;

signed int hall6s_tab_a[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
signed int msp6s_pwm_a = 9000;
signed int msp6s_omega_a;

unsigned int hall6_pwm_count_rec_a[256];
unsigned char add_count_hall6_a;

unsigned char high_speed_mode_a;
unsigned char high_speed_count_a;

signed int hall_angle_value_a;
unsigned int theta_inc_pwm_a;
unsigned int theta_inc_pwm6_a;

unsigned char hall_error_flag_a;
unsigned char motor_forward_flag_a = 1;
unsigned char motor_forward_flag_last_a = 1;

unsigned char motor_forward_count_a;
unsigned char motor_reverse_count_a; 
unsigned char motor_ws_flag_a;
unsigned char motor_unws_count_a;

signed int hall_angle_360_delta_a;
signed int hall_angle_360_comp_a;

signed int hall_angle_delta_a;
//signed int hall_angle_comp_a = -10000;//-1500;  //JieCang Motor
signed int hall_angle_comp_a = 0;//-1500;  //Jabil Motor

unsigned char hall_seq_tab_a[21];   // = {0,0,0,0,0,0,0,0};
unsigned char hall_seq_count_a;


signed int MC1_q_currnet_cmd;  //Betty add 20210331

unsigned char MC1_dynamic_start = 0; //Betty Add
unsigned char MC1_dynamic_start_count = 200;
//signed int MC2_offset_meas_threshold; //Betty add 20210407
signed long  MC1_Vs_init;

signed int x2c_mc1_iq_cmd;

unsigned char x2c_angle_open_loop = 2;
signed int x2c_angle_inc_val = 0;
extern void HALL_READ_MC1(void);

unsigned int cpu_temperature;
signed int x2c_current_ramp = 5;
// </editor-fold>

unsigned char x2c_motor_command_mc1 = 0;    //0 stop 1 fwd 2 rev 3 shake 4 micro step
unsigned char x2c_auto_mode = 0;//1;

unsigned int shake_state;
unsigned int shake_count;
unsigned int x2c_shake_mod = 5000;

unsigned int lock_count;
unsigned int x2c_lock_mod = 1000;
unsigned int x2c_lock_angle_inc = 20;
signed int x2c_lock_current_cmd = 800;


signed long x2c_angle_cap_scale_mod = 6400000;
signed long x2c_cap_rpm_scale = 9437184; //90*65536/0.0625
unsigned char x2c_speed_mode_on = 1;//1;
signed int x2c_id_cmd_man = 2000;
signed int x2c_motor_speed_cmd;

void speed_control_mc1(void);

unsigned char spd_update_flag_a;

signed int command_a_rpm;
signed int command_a_rpm_final;

signed int x2c_iq_close_a = 0;
signed int x2c_id_close_a = 0;

signed long x2c_iq_close_32_a = 0;
signed long x2c_iq_limit_32_a;
signed int x2c_iq_limit_a = 3000;

signed int motor_a_speed_delta;
signed long motor_a_speed_calc_temp;
signed long motor_a_speed_calc_temp_i;
signed int motor_a_command_fix;
signed int motor_a_command_sup;

signed int x2c_motor_a_rpm_delta;
signed int motor_a_rpm_old;

signed int x2c_current_speed_scale = 200;
signed int x2c_wkp = 30;
signed int x2c_wki = 0;
signed int x2c_wkd = 0;

signed int x2c_wkp_dec_arr[20] = {120, 110, 100,  80,  80, 65,  52,  40,  35,  30, 28, 26,20, 20, 6,35 };
signed int x2c_wkd_dec_arr[20] = {4000,4000,3500,3000,2500,2300,1900,1600,1300,1000,900,600,450,350,40,1450};

signed int x2c_wkp_sht_arr[20] = {50, 50,  40,  35, 30,  28,  26,  24,  22,  20, 19, 17,  15, 6 };
signed int x2c_wkd_sht_arr[20] = {3000,2500,2300,2200 ,2000,1800,1600,1400,1200,1000,800,600,400,40};

#if 1
int x2c_speed_wkp[20] = {25000,25000, 25000,25000,
25000, 15500, 15500, 15500, 15000, 12000, 8000, 6000, 4000, 2000, 5000, 5000, 5000, 5000};
int x2c_speed_wki[20] = {22000,22000, 22000,22000,
22000, 22000, 22000, 22000, 12000, 8000, 4000, 3500, 3000, 1000, 900, 900, 900, 900};
#endif
#if 0
int x2c_speed_wkp[20] = {27000,26500, 24500,23400,
20500, 19500, 18500, 17000, 16500, 14000, 12000, 11000, 10000, 10, 5000, 5000, 5000, 5000, 5000};
int x2c_speed_wki[20] = {10000,9900, 9500,9000,
8000, 7000, 6500, 6000, 5500, 4000, 3500, 2500, 2000, 50, 500, 900, 900, 900, 900};
#endif

signed int x2c_is_scale = 20000;//300; //1000;
signed int x2c_ds_scale = 8000;//1000;
unsigned int still_count_a;
unsigned int still_update_mod = 20;//150; 
signed int x2c_delta_enl_factor = 2;

signed int x2c_wkp_hsp = 200;
signed int x2c_wkd_hsp = 6000;
signed int lock_theta_angle_mc1;

unsigned int qei_updang;
unsigned int qei_counter;
unsigned int qei_counter_old;
signed int delta_qei_counter;

signed int qei_angle_s16;
unsigned int qei_counter_mod;
unsigned long qei_angle_temp;
unsigned int qei_angle_u16;
signed int qei_angle_camp_a;
void QEI_ANGLE_CALC(void);

unsigned char x2c_qei_lock_flag = 0;
unsigned int qei_lock_count = 0;

signed int qei_vel_temp;
signed int qei_vel_zero = 1;
unsigned long qei_vel_temp2 = 0;
unsigned long qei_vel_temp_old = 0;
unsigned int qei_vel_count = 0;
unsigned int qei_vel_count_num = 50;

unsigned long qei_velo_val = 0;
unsigned long qei_velo_val_shrk = 0;
unsigned long x2c_rpm_vel_scale = 1500000;
unsigned long x2c_rpm_vel_scale_shrk = 150000;

signed int hall_angle_diff_mc1;
signed int speed_rpm_mc1;
signed long speed_rpm_fltred_long;
signed int speed_rpm_fltred;
signed int speed_lpf_coff = 500;
signed int speed_rpm_fltred_temp; 

unsigned int pwup_lock_count;

unsigned char x2c_pos_mode_on = 1;
void pos_control_mc1(void);

unsigned int x2c_pos_cmd = 13800;
unsigned int x2c_pos_cmd_old;
unsigned char x2c_run_motor = 0;

unsigned int pos_loop_counter; 
unsigned int x2c_pos_loop_mod = 5;

signed int x2c_pos_outmax = 10000;
signed int x2c_max_delta_position = 1000;
signed int delta_position;
signed int pos_outmax_final;
unsigned int x2c_pos_inc_val = 10;
extern signed int x2c_vd_open;
extern unsigned char x2c_vs_openloop;
extern signed int x2c_vq_open ;
unsigned char open_flag = 0;

uint8_t pos_state = 0;
// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MC1APP_StateMachine(MC1APP_DATA_T *);
static void MCAPP_MC1ReceivedDataProcess(MC1APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: void MC1APP_StateMachine (MC1APP_DATA_T *)  </B>
*
* @brief Application state machine.
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* @example
* <CODE> MC1APP_StateMachine(&mc); </CODE>
*
*/
static void MC1APP_StateMachine(MC1APP_DATA_T *pMCData)
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
//        if(MC1_dynamic_start==0)
        pMCData->MCAPP_ControlSchemeInit(pControlScheme);
        
        //pMC1Data->controlScheme.ctrlParam.OLTheta = hall_angle_value_a + hall_angle_comp_a; //Betty Add
        
        
        pMCData->MCAPP_InputsInit(pMotorInputs);
        pMCData->MCAPP_LoadInit(pLoad);       
        
        pMCData->appState = MCAPP_CMD_WAIT;

        break;
        
    case MCAPP_CMD_WAIT:
        if(pMCData->runCmd == 1)
        {
            pMCData->appState = MCAPP_OFFSET;
            if(!dir_cmd_mc1)
            {
                pMCData->pControlScheme->ctrlParam.OLCurrentMax = -ol_current_rec_mc1;
                pMCData->pMotor->qMaxOLSpeed = -ol_speed_rec_mc1;
            }
            else
            {
                pMCData->pControlScheme->ctrlParam.OLCurrentMax = ol_current_rec_mc1;
                pMCData->pMotor->qMaxOLSpeed = ol_speed_rec_mc1;
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
* <B> Function: MC1_ADC_INTERRUPT()  </B>
*
* @brief ADC interrupt vector ,and it performs following actions:
*        (1) Reads DC BUS voltage and updates appropriate variables.
*        (2) Reads motor 1 phase currents,bus current and phase voltage
*            feedbacks from ADC data buffers.
*        (3) Executes Field Oriented Control based on the current,voltage
*            feedbacks.
*        (4) Loads duty cycle values generated by FOC to the registers
*            of PWM Generators controlling motor 1.
*/
extern MC2APP_DATA_T *pMC2Data;

int16_t mc1_bus_current_flg = 0;

    unsigned int qei_vel_l;
    unsigned int qei_vel_h;

void __attribute__((__interrupt__,no_auto_psv)) MC1_ADC_INTERRUPT()
{
    int16_t adcBuffer;
    static unsigned count=0;
//    unsigned int qei_vel_l;
//    unsigned int qei_vel_h;


    if(pwup_lock_count < 16000)
        pwup_lock_count ++;
    else if(pwup_lock_count == 16000)
    {
        x2c_qei_lock_flag = 1;
        pwup_lock_count ++;
    }
    
        
    
//    TEST1 = 1;
    cpu_temperature = ADCBUF24;
    pMC1Data->HAL_MotorInputsRead(pMC1Data->pMotorInputs);
    
    qei_counter = QEI1_PositionCountRead();
    while(qei_counter >= 16384)
    {
        qei_counter -= 16384;
    }
    
    if(qei_counter > qei_counter_old)
    {
        if((qei_counter > 16000)&&(qei_counter_old < 100))
        {
            qei_updang = 1;
        }
        else
        {
            qei_updang = 0;
        }
    }
    else if(qei_counter < qei_counter_old)
    {
        if((qei_counter_old > 16000)&&(qei_counter < 100))
        {
            qei_updang = 0;
        }
        else
        {
            qei_updang = 1;
        }
    }
    qei_counter_old = qei_counter;
    
    qei_vel_l = INT1HLDL;//
    qei_vel_h = INT1HLDH;
    qei_velo_val = (unsigned long)qei_vel_h << 16;
    qei_velo_val += qei_vel_l;
    
//    if(qei_velo_val >= 65535)
//        qei_velo_val = 65535;
    if(qei_velo_val < 50)
        qei_velo_val = 50;
    
    if(qei_velo_val >= 65535)
    {
        qei_velo_val_shrk = __builtin_divud(qei_velo_val , 10);
        qei_vel_temp = __builtin_divud(x2c_rpm_vel_scale_shrk , qei_velo_val_shrk);
    }
    else
    { 
    if(qei_velo_val != 0)
        qei_vel_temp = __builtin_divud(x2c_rpm_vel_scale , (unsigned int)qei_velo_val);
    else
        qei_vel_temp = 0;
    }
    if(qei_updang == 1)
    {
        qei_vel_temp = 0 - qei_vel_temp;
    }
    if(qei_vel_zero)
    {
        if(qei_vel_temp == qei_vel_temp_old)
            qei_vel_count++;
        else
            qei_vel_count = 0;

        if(qei_vel_count > qei_vel_count_num)
        {
            qei_vel_temp2 = qei_vel_temp;
            qei_vel_temp = 0;
            qei_vel_count = 0;
        }

        if(qei_vel_temp2 == qei_vel_temp)
        {
            qei_vel_temp = 0;
        }

        if(qei_vel_temp != 0)
            qei_vel_temp_old = qei_vel_temp;            
    }

    
    speed_rpm_fltred_temp = (signed long)speed_rpm_fltred_long >> 16;
    speed_rpm_fltred_long = speed_rpm_fltred_long + __builtin_mulss((qei_vel_temp - speed_rpm_fltred_temp), speed_lpf_coff);
    speed_rpm_fltred = (signed long)speed_rpm_fltred_long >> 16;

    
    if(still_count_a >= still_update_mod)
    {
        spd_update_flag_a = 1;
        still_count_a = 0;
//        delta_qei_counter = qei_counter - qei_counter_old;
//        if(delta_qei_counter > 0)
//            delta_qei_counter -= 4096;
//        delta_qei_counter = 0 - delta_qei_counter;
//        qei_counter_old = qei_counter;
//        motor_a_rpm = delta_qei_counter;
        motor_a_rpm = speed_rpm_fltred;
    }
    else
        still_count_a ++;
    
    
    if(x2c_motor_command_mc1 == 4)
    {
        x2c_mc1_iq_cmd = x2c_lock_current_cmd;
    }
    else if(x2c_pos_mode_on)
    {
        if(pos_loop_counter >= x2c_pos_loop_mod)
        {
            pos_loop_counter = 0;
            pos_control_mc1();
            speed_control_mc1();
        }
        else
            pos_loop_counter ++;
    }
    if(x2c_pos_mode_on == 2)
    {
        if(pos_loop_counter >= x2c_pos_loop_mod)
        {
            pos_loop_counter = 0;
            pos_control_mc1();
            if(x2c_run_motor)
                speed1_control_foc();            
            //speed_control_mc2();
        }
        else
            pos_loop_counter ++;        
    }    
    else if(x2c_speed_mode_on == 2)
    {
        if(x2c_run_motor)
            speed1_control_foc();
    }    
    else if(x2c_speed_mode_on)
    {
        if(x2c_motor_speed_cmd > 0)
            speed_control_mc1();
        else
            x2c_mc1_iq_cmd = 0;
    }


    if(x2c_angle_open_loop == 1)
    {
//        count++;
//        if(count >100000)
//        {
//            QEI_ANGLE_CALC();
//            if(x2c_motor_command_mc1 == 4)
//                pMC1Data->controlScheme.ctrlParam.OLTheta = lock_theta_angle_mc1;
//            else
//                pMC1Data->controlScheme.ctrlParam.OLTheta += x2c_angle_inc_val;            
//        }
            //if(x2c_spi_cali_cmd)
            {
                //if(count>20000)
                {
                    count=0;
                    if(x2c_motor_command_mc1 == 4)
                       pMC1Data->controlScheme.ctrlParam.OLTheta = lock_theta_angle_mc1;
                   else
                       pMC1Data->controlScheme.ctrlParam.OLTheta += x2c_angle_inc_val;                       
                    
//                    if(i<400)
//                    {
//                       spi_calib_offset_data_u17[i] = angle_test;    
//                    }
//                    else
//                    {
//                       i=0;
//                       x2c_spi_cali_cmd = 0;
//                    }
//                    i++;                    
                }
//                else
//                {
//                    count++;
//                }

            }        

    }
    else if(x2c_angle_open_loop == 0)
    {
        HALL_READ_MC1();   //Betty add 20210226
        pMC1Data->controlScheme.ctrlParam.OLTheta = hall_angle_value_a + hall_angle_comp_a; //- 1500; //+ 2730 ;   //+ x2c_angle_comp_man_a;  //Betty change 20210226
    }
    else if(x2c_angle_open_loop == 2)
    {
        if(x2c_qei_lock_flag)
        {
            pMC1Data->controlScheme.ctrlParam.OLTheta = 0;
            QEI1_PositionCount16bitWrite(0);
        }
        else
        {
            QEI_ANGLE_CALC();
            pMC1Data->controlScheme.ctrlParam.OLTheta = qei_angle_s16 + qei_angle_camp_a;
        }
    }
    
    
    MC1APP_StateMachine(pMC1Data);

    pMC1Data->HAL_PWMSetDutyCycles(pMC1Data->pPWMDuty);
    
    adcBuffer = MC1_ClearADCIF_ReadADCBUF();
 
	MC1_ClearADCIF();
}

void MCAPP_MC1ServiceInit(void)
{
    MCAPP_MC1ParamsInit(pMC1Data);
}

void MCAPP_MC1InputBufferSet(int16_t runCmd, 
                                    int16_t qTargetVelocity)
{
    MC1APP_DATA_T   *pMCData = pMC1Data;
    MCAPP_MOTOR_T   *pMotor = pMC1Data->pMotor;
    signed long  q_current_cmd_tmp;
    int16_t      q_Target_Velocity_temp;
    int16_t      MC1_q_currnet_cmd_temp;
    

    command_a_rpm = x2c_motor_speed_cmd;

    
    if(x2c_qei_lock_flag)
    {
        if(qei_lock_count <= 16000)
        {
            qei_lock_count ++;
            if(runCmdMC1 == 0)
            {
                runCmdMC1 = 1;
                pMCData->controlScheme.piId.qOut=0;
                pMCData->controlScheme.piId.qdSum=0;
                pMCData->controlScheme.piIq.qOut=0;
                pMCData->controlScheme.piIq.qdSum=0; 
                HAL_MC1PWMEnableOutputs();   
            }   
            else
            {
                pMCData->controlScheme.piId.qInRef = -x2c_id_cmd_man;
            }
        }
        else
        {
            x2c_qei_lock_flag = 0;
            qei_lock_count = 0;
            pMCData->controlScheme.piId.qInRef = 0;
            runCmdMC1 = 0;
            HAL_MC1PWMDisableOutputs();
//            x2c_motor_speed_cmd = 1000;
        }
    }
    else if(x2c_run_motor != 0)
    {
        if(runCmdMC1 == 0)
        {
            runCmdMC1 = 1;
            pMCData->controlScheme.piId.qOut=0;
            pMCData->controlScheme.piId.qdSum=0;
            pMCData->controlScheme.piIq.qOut=0;
            pMCData->controlScheme.piIq.qdSum=0; 
            x2c_iq_close_32_a = 0;
            x2c_iq_close_a = 0;
            HAL_MC1PWMEnableOutputs();   
        }
        else 
        {
            MC1_q_currnet_cmd = x2c_mc1_iq_cmd;
            if(x2c_speed_mode_on)
            {
                pMCData->controlScheme.piIq.qInRef = x2c_mc1_iq_cmd;
            }
            else
            {
                if(pMCData->controlScheme.piIq.qInRef < MC1_q_currnet_cmd)     //Betty add
                     pMCData->controlScheme.piIq.qInRef += x2c_current_ramp;//CURENTRAMP;
                else if (pMCData->controlScheme.piIq.qInRef > MC1_q_currnet_cmd)
                     pMCData->controlScheme.piIq.qInRef -= x2c_current_ramp;//CURENTRAMP; 
                else
                    pMCData->controlScheme.piIq.qInRef = x2c_mc1_iq_cmd;
            }
        }
    }
    else if(x2c_run_motor == 0)
    {
        MC1_q_currnet_cmd = 0;
        if(runCmdMC1 == 1)
        {
            if(pMCData->controlScheme.piIq.qInRef > x2c_current_ramp)//CURENTRAMP)
                pMCData->controlScheme.piIq.qInRef -= x2c_current_ramp;//CURENTRAMP; 
            else if(pMCData->controlScheme.piIq.qInRef <  -x2c_current_ramp)
                pMCData->controlScheme.piIq.qInRef += x2c_current_ramp;//CURENTRAMP; 
            else
            {
                runCmdMC1 = 0;
                HAL_MC1PWMDisableOutputs();
            }
        }
    }
    

//    pMCData->controlScheme.piId.qInRef = x2c_id_cmd_man;

    pMCData->runCmdBuffer = runCmd; //Betty change 20210407
    MCAPP_MC1ReceivedDataProcess(pMCData);
}

void speed_control_mc1(void)
{

    
    x2c_iq_limit_32_a = __builtin_mulss(x2c_iq_limit_a , x2c_is_scale);
    if(spd_update_flag_a)
    {
        spd_update_flag_a = 0;
//        command_a_rpm_final = __builtin_divsd(command_a_rpm , 3);// - __builtin_divsd(x2c_iq_close_a , x2c_current_speed_scale);
//        if(command_a_rpm > 5000)
//            command_a_rpm_final = 5000;
//        else if(command_a_rpm < -5000)
//            command_a_rpm_final = -5000;
//        else
//            command_a_rpm_final = command_a_rpm;
        
        if(command_a_rpm > 15000)
            command_a_rpm_final = 15000;
        else if(command_a_rpm < -15000)
            command_a_rpm_final = -15000;
        else
            command_a_rpm_final = command_a_rpm;
        
        motor_a_speed_delta = command_a_rpm_final - motor_a_rpm;
        
        if(motor_a_speed_delta > 1500)
            motor_a_speed_delta = 1500;
        else if(motor_a_speed_delta < -1500)
            motor_a_speed_delta = -1500;
        
        motor_a_speed_calc_temp = __builtin_mulss(motor_a_speed_delta , x2c_wkp);
//              motor_a_command_fix = __builtin_divsd(motor_a_speed_calc_temp ,x2c_is_scale);        
        x2c_iq_close_32_a += motor_a_speed_calc_temp;
        
        if(motor_a_speed_delta > 100)
            motor_a_speed_delta = 100;
        else if(motor_a_speed_delta < -100)
            motor_a_speed_delta = -100;
        if(x2c_run_motor)
        {
            motor_a_speed_calc_temp = __builtin_mulss(motor_a_speed_delta , x2c_wki);
            motor_a_speed_calc_temp_i +=motor_a_speed_calc_temp;        
            x2c_iq_close_32_a += motor_a_speed_calc_temp_i;            
        }
        else
        {
            motor_a_speed_calc_temp_i = 0;
        }



//                if((motor_a_command_fix == 0)&&(command_a_rpm != motor_a_rpm))
//                {
//                    if(command_a_rpm > motor_a_rpm)
//                        x2c_iq_close_a += 1;
//                    else
//                        x2c_iq_close_a -= 1;
//                }



        x2c_motor_a_rpm_delta = motor_a_rpm - motor_a_rpm_old;
        
        if(x2c_motor_a_rpm_delta > 100)
            x2c_motor_a_rpm_delta = 100;
        else if(x2c_motor_a_rpm_delta < -100)
            x2c_motor_a_rpm_delta = -100;
        
        motor_a_rpm_old = motor_a_rpm;

        motor_a_speed_calc_temp = __builtin_mulss(x2c_motor_a_rpm_delta, x2c_delta_enl_factor);
        if(motor_a_speed_calc_temp > 32767)
            x2c_motor_a_rpm_delta = 32767;
        else if(motor_a_speed_calc_temp < -32767)
            x2c_motor_a_rpm_delta = -32767;
        else
            x2c_motor_a_rpm_delta = motor_a_speed_calc_temp;

        motor_a_speed_calc_temp = __builtin_mulss(x2c_motor_a_rpm_delta, x2c_wkd);
        //          motor_a_command_sup = __builtin_divsd(motor_a_speed_calc_temp ,x2c_ds_scale);??
        x2c_iq_close_32_a -= motor_a_speed_calc_temp;

        if(x2c_iq_close_32_a > x2c_iq_limit_32_a)
            x2c_iq_close_32_a = x2c_iq_limit_32_a;
        if(x2c_iq_close_32_a < (0 - x2c_iq_limit_32_a))
            x2c_iq_close_32_a = (0 - x2c_iq_limit_32_a);

        x2c_iq_close_a = __builtin_divsd(x2c_iq_close_32_a ,x2c_is_scale);

//        if(command_a_rpm > 0)
//        {
//            if(x2c_iq_close_a < 0)
//            {
//                x2c_iq_close_a = 0;
//                x2c_iq_close_32_a = 0;
//            }
//        }
//                else if(command_a_rpm < 0)
//                {
//                    if(x2c_iq_close_a > 0)
//                        x2c_iq_close_a = 0;  
//                }
//                if(x2c_iq_close_a > x2c_iq_limit_a)
//                    x2c_iq_close_a = x2c_iq_limit_a;
//                if(x2c_iq_close_a < (0 - x2c_iq_limit_a))
//                    x2c_iq_close_a = (0 - x2c_iq_limit_a);

        x2c_mc1_iq_cmd = x2c_iq_close_a;
    }
}


int16_t MCAPP_MC1GetTargetVelocity(void)
{
    return pMC1Data->motorInputs.measurePot;
}

static void MCAPP_MC1ReceivedDataProcess(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;

//    if(pMCData->runCmd == 1)
//    {
//        if(pMCData->qTargetVelocity > pMotor->qMaxSpeed)
//        {
//            pMCData->qTargetVelocity = pMotor->qMaxSpeed;
//        }
//        else
//        {
//            if(pMCData->qTargetVelocity < pMotor->qMinSpeed)
//            {
//                pMCData->qTargetVelocity = pMotor->qMinSpeed;
//            }
//        }
//
//        pControlScheme->ctrlParam.qTargetVelocity = pMCData->qTargetVelocity;
//    }
//    
//    if((pMCData->runCmdBuffer == 1) || (pMCData->runCmdBuffer == 0))
//    {
//        if((pMotorInputs->measureVdc.value >= 
//                pMotorInputs->measureVdc.dcMinRun) && 
//                (pControlScheme->faultStatus == 0))
//        {
            pMCData->runCmd = pMCData->runCmdBuffer;
//        }        
//    }
//    if(pMotorInputs->measureVdc.value < pMotorInputs->measureVdc.dcMaxStop)
//    {
//        pMCData->runCmd = 0;
//        pMCData->controlScheme.faultStatus = 0;
//    }
}

#if 1
void HALL_READ_MC1(void)
{
    hall_pwm_count6_a ++;    
    if(hall_pwm_count6_a > 15000)
        hall_pwm_count6_a = 15000;
         
	hall_temp_a = hall_sw_tab_a[HALLPORT_A];
	if((hall_temp_a == 0)||(hall_temp_a == 7))
	{
		if(hall_error_count_a >= 5)
		{
			hall_pre_a = 0;
			hall_last_a = 0;
			hall_error_flag_a = 1;
		}
		else
			hall_error_count_a ++;
	}
	else
	{
		hall_error_count_a = 0;

		if((hall_temp_a != hall_last_a)&&(hall_temp_a != hall_pre_a))
		{
            if(hall_last_a == hall_f_tab[hall_temp_a])
            {
                motor_forward_flag_a = 1;
            }
            else if(hall_last_a == hall_r_tab[hall_temp_a])
            {
                motor_forward_flag_a = 0;
            }
            
            msp_pwm_4x_a = (msp_pwm_a + hall_pwm_count_a + hall_pwm_count_a + hall_pwm_count_a) >> 2;
			msp_pwm_a = hall_pwm_count_a;
            msp_pwm_hl_last_a[hall_last_a] = msp_pwm_a;
//            add_count_hall6 ++;
//            hall6_pwm_count_rec[add_count_hall6] = msp_pwm;
            
            if(motor_forward_flag_last_a != motor_forward_flag_a)
            {
                high_speed_mode_a = 0;
                high_speed_count_a = 0;
                speed_stab_flag_a = 0;
                speed_stab_count_a = 0;
            }
            else
            {
                if(motor_ws_flag_a)
                {
                    if(motor_unws_count_a >= 1)
                        motor_ws_flag_a = 0;
                    else
                        motor_unws_count_a ++;
                }
            }
            motor_forward_flag_last_a = motor_forward_flag_a;
            
            
            if(!high_speed_mode_a)
            {
                if(msp_pwm_a < 180)
                {
                    if(high_speed_count_a > 3)
                    {
                        high_speed_mode_a = 1;
                    }
                    else
                        high_speed_count_a ++;
                }
                else
                    high_speed_count_a = 0;
            }
            else
            {
                if(msp_pwm_a > 250)
                {
                    high_speed_mode_a = 0;
                    high_speed_count_a = 0;
                    speed_stab_flag_a = 0;
                    speed_stab_count_a = 0;
                }
            }
            
			hall_pre_a = hall_last_a;
			hall_last_a = hall_temp_a;
			hall_pwm_count_a = 1;
			hall_change_flag_a = 1;
            
#ifndef HALL_ON_A
            hall_seq_count_a ++;
            if(hall_seq_count_a == 20)
                hall_seq_count_a = 0;
            
            hall_seq_tab_a[hall_seq_count_a] = hall_last_a;
            hall_theta_tab_st[hall_last_a] = qRotorAngleMA;
#endif        
            
            hall_seq_count_a ++;
            if(hall_seq_count_a == 20)       
                hall_seq_count_a = 0;
            
            hall_seq_tab_a[hall_seq_count_a] = hall_last_a;

//          
            if(motor_ws_flag_a)
            {
                theta_inc_pwm_a = 0;
                theta_inc_pwm6_a = 0;
                hall_angle_360_comp_a = 0;
                hall_angle_value_a = hall_theta_tab_md[hall_last_a];
            }
            else
            {
                if(hall_temp_a == 1)   //current HALL port value = 1 
                {
                    msp6_pwm_a = hall_pwm_count6_a;  

                    if(msp6_pwm_a < msp6_last_a)
                        msp6_delta_a = msp6_last_a - msp6_pwm_a;
                    else
                        msp6_delta_a = msp6_pwm_a - msp6_last_a;
                    
                    if(msp6_pwm_a < 200)
                    {
                        if(cap_360_on_count_a >= 3)    
                        {
                            cap_360_on_flag_a = 1;   //Capture HALL edge every 360?flag
//                            cap_360_on_flag_a = 0;   //Betty close it for debug
                        }
                        else
                            cap_360_on_count_a ++;
                    }
                    else if(msp6_pwm_a > 250)
                    {
                        cap_360_on_flag_a = 0;
                        cap_360_on_count_a = 0;
                    }
                    
                    if((msp6_delta_a < (msp6_last_a >> 3))&&(high_speed_mode_a))  
                    {
                        if(speed_stab_count_a >= 5)
                            speed_stab_flag_a = 1;      //Calculate delta angle every 360?lag
                        else
                            speed_stab_count_a ++;    
                    }
                    else
                    {
                        speed_stab_flag_a = 0;
                        speed_stab_count_a = 0;
                    }

                    hall_pwm_count6_a = 0;
                    msp6_last_a = msp6_pwm_a;
                    if(msp6_pwm_a)
                    {
                        if(motor_forward_flag_a)
                        {
                            if(!cap_360_on_flag_a)
                                hall_angle_360_delta_a = Q15(0.1666) - hall_angle_value_a - theta_inc_pwm6_a - hall_angle_360_comp_a;
                            else
                                hall_angle_360_delta_a = Q15(0.1666) - hall_angle_value_a - angle_cap_360_inc_a - hall_angle_360_comp_a;
                            hall_angle_360_comp_a = __builtin_divsd(hall_angle_360_delta_a, msp6_pwm_a);    
                    
                        }
                        else
                        {
                            if(!cap_360_on_flag_a)
                                hall_angle_360_delta_a = Q15(0.5) - (hall_angle_value_a - theta_inc_pwm6_a + hall_angle_360_comp_a);  //reverse hall_value=1,table st from Q(0.5)=90? ed wirth Q(0.1666)=30?
                            else
                                hall_angle_360_delta_a = Q15(0.5) - (hall_angle_value_a - angle_cap_360_inc_a + hall_angle_360_comp_a);
                            hall_angle_360_comp_a = __builtin_divsd(hall_angle_360_delta_a, msp6_pwm_a);      
                        }
                             
                    }
                    else
                        hall_angle_360_comp_a = 0;
                         
                    
                    //cap_inc = 65536 * 4375 / 64 / hall_cap
                    if(hallb_cap_value_a)
                        angle_cap_360_inc_a = __builtin_divsd(x2c_angle_cap_scale_mod , hallb_cap_value_a); //4375/64x(TMR3) = 68.36
                    else
                        angle_cap_360_inc_a = 3;
                }        
                
                msp6s_pwm_a -= hall6s_tab_a[hall_pre_a];
                hall6s_tab_a[hall_pre_a] = msp_pwm_a;
                msp6s_pwm_a += msp_pwm_a;
                
                if(!cap_360_on_flag_a)
                {
                    if(msp6s_pwm_a)
                    {
                        msp6s_omega_a = __builtin_divsd(96000,msp6s_pwm_a);//1 pp 60000/0.0625
                    }
                    else
                        msp6s_omega_a = 0;
                }
                else
                {
                    if(hallb_cap_value_a)
                    {
                        msp6s_omega_a = __builtin_divsd(x2c_cap_rpm_scale,hallb_cap_value_a);
                    }
                    else
                        msp6s_omega_a = 0;
                }

                if(!motor_forward_flag_a)
                    motor_a_rpm = 0 - msp6s_omega_a;
                else
                    motor_a_rpm = msp6s_omega_a;
                
//              spd_update_flag_a = 1;
                
                if(msp_pwm_4x_a)        
                    theta_inc_pwm_a = __builtin_divud(Q15(0.3333),msp_pwm_4x_a);
                else
                    theta_inc_pwm_a = 0;
          
                if(!speed_stab_flag_a)
                {
                    if(motor_forward_flag_a)
                        hall_angle_value_a = hall_theta_tab_st[hall_last_a];
                    else
                        hall_angle_value_a = hall_theta_tab_st_r[hall_last_a];
                    theta_inc_pwm6_a = theta_inc_pwm_a;
                }
                else
                {
                    if(msp6_pwm_a)
                        theta_inc_pwm6_a = __builtin_divud(65535 , msp6_pwm_a);
                    else
                        theta_inc_pwm6_a = 0;
                    if(motor_forward_flag_a)
                    {
                        if(!cap_360_on_flag_a)
                            hall_angle_value_a += theta_inc_pwm6_a;
                        else
                            hall_angle_value_a += angle_cap_360_inc_a;
                        hall_angle_value_a += hall_angle_360_comp_a;
                    }
                    else
                    {
                        if(!cap_360_on_flag_a)
                            hall_angle_value_a -= theta_inc_pwm6_a;
                        else
                            hall_angle_value_a -= angle_cap_360_inc_a;
                        hall_angle_value_a += hall_angle_360_comp_a;
                    }
                }
            }
		}
		else
		{
            if(hall_temp_a == hall_pre_a)
            {
                motor_ws_flag_a = 1;
                motor_unws_count_a = 0;
            }
			hall_pwm_count_a ++;
            
            if(motor_ws_flag_a)
            {
                hall_angle_value_a = hall_theta_tab_md[hall_last_a];
            }
            else
            {
                if(speed_stab_flag_a)
                {
//                    msp_pwm_mem = msp_pwm_hl_last[hall_last];
                    if(motor_forward_flag_a)
                    {
                        if(hall_pwm_count_a > (msp_pwm_hl_last_a[hall_last_a] + msp_pwm_hl_last_a[hall_last_a]))
                        {
                            speed_stab_flag_a = 0;
                            speed_stab_count_a = 0;
                            cap_360_on_flag_a = 0;
                            cap_360_on_count_a = 0;
                            if((hall_angle_value_a > 0)&&(hall_theta_tab_md[hall_last_a] < 0))
                            {
                                hall_angle_value_a += theta_inc_pwm_a;
                            }
                            else if(hall_angle_value_a < hall_theta_tab_md[hall_last_a])
                            {
                                hall_angle_value_a += theta_inc_pwm_a;
                            }
                            else
                                hall_angle_value_a -= theta_inc_pwm_a; 
                        }
                        else
                        {
                            if(!cap_360_on_flag_a)
                                hall_angle_value_a += theta_inc_pwm6_a;
                            else
                                hall_angle_value_a += angle_cap_360_inc_a;
                            hall_angle_value_a += hall_angle_360_comp_a;
                        }
                    }
                    else
                    {
                        if(hall_pwm_count_a > (msp_pwm_hl_last_a[hall_last_a] + msp_pwm_hl_last_a[hall_last_a]))
                        {
                            speed_stab_flag_a = 0;
                            speed_stab_count_a = 0;
                            cap_360_on_flag_a = 0;
                            cap_360_on_count_a = 0;
                            if((hall_angle_value_a < 0)&&(hall_theta_tab_md[hall_last_a] < 0))
                            {
                                hall_angle_value_a -= theta_inc_pwm_a;
                            }
                            else if(hall_angle_value_a < hall_theta_tab_md[hall_last_a])
                            {
                                hall_angle_value_a -= theta_inc_pwm_a;
                            }
                            else
                                hall_angle_value_a += theta_inc_pwm_a; 
                        }
                        else
                        {
                            if(!cap_360_on_flag_a)
                                hall_angle_value_a -= theta_inc_pwm6_a;
                            else
                                hall_angle_value_a -= angle_cap_360_inc_a;
                            hall_angle_value_a += hall_angle_360_comp_a;
                        }
                    }
                }
                else
                {
                    if(motor_forward_flag_a)
                    {
                        hall_angle_value_a += theta_inc_pwm_a;
                        if((hall_theta_tab_ed[hall_last_a] < 0)&&(hall_theta_tab_st[hall_last_a] > 0))
                        {
                            if((hall_angle_value_a < 0)&&(hall_angle_value_a >= hall_theta_tab_ed[hall_last_a]))
                                hall_angle_value_a = hall_theta_tab_ed[hall_last_a];
                        }
                        else
                        {
                            if(hall_angle_value_a >= hall_theta_tab_ed[hall_last_a])
                                hall_angle_value_a = hall_theta_tab_ed[hall_last_a];
                        }
                    }
                    else
                    {
                        hall_angle_value_a -= theta_inc_pwm_a;
                        if((hall_theta_tab_ed_r[hall_last_a] > 0)&&(hall_theta_tab_st_r[hall_last_a] < 0))
                        {
                            if((hall_angle_value_a <= hall_theta_tab_ed_r[hall_last_a])&&(hall_angle_value_a > 0))
                                hall_angle_value_a = hall_theta_tab_ed_r[hall_last_a];
                        }
                        else
                        {
                            if(hall_angle_value_a <= hall_theta_tab_ed_r[hall_last_a])
                                hall_angle_value_a = hall_theta_tab_ed_r[hall_last_a];
                        }
                    }
                }
                
                
//                if(msp_pwm_4x >= 400)
//                {
//                    hall_angle_value = hall_theta_tab_md[hall_last];
//                }
            }
         
			if(hall_pwm_count_a >= 1499)
			{
				hall_pwm_count_a = 1499;
				msp_pwm_a = 1500;
                msp_pwm_4x_a = 1500;

                if(hall_last_a == hall_f_tab[hall_temp_a])
                {
                    motor_forward_flag_a = 1;
                }
                else if(hall_last_a == hall_r_tab[hall_temp_a])
                {
                    motor_forward_flag_a = 0;
                }
                
                if(motor_forward_flag_last_a != motor_forward_flag_a)
                {
                    cap_360_on_flag_a = 0;
                    cap_360_on_count_a = 0;
                }
                
                motor_forward_flag_last_a = motor_forward_flag_a;
                msp6s_pwm_a -= hall6s_tab_a[hall_pre_a];
                hall6s_tab_a[hall_pre_a] = msp_pwm_a;
                msp6s_pwm_a += msp_pwm_a;
                
                msp6s_omega_a = 0;
                motor_a_rpm = 0;
				hall_pre_a = 0;
                hall_last_a = hall_temp_a;
				hall_change_flag_a = 1;
                speed_stab_flag_a = 0;
                speed_stab_count_a = 0;
                high_speed_mode_a = 0;
                high_speed_count_a = 0;
                cap_360_on_flag_a = 0;
                cap_360_on_count_a = 0;
//                theta_inc_pwm = 2;
			}
		}
//        hall_angle_value = hall_theta_tab_md[hall_last]; // force md
	}				
}
#endif
unsigned char stop_flag = 0;
unsigned int stop_count = 0;
void QEI_ANGLE_CALC(void)
{
    qei_counter_mod = __builtin_modud(qei_counter , 2341);
    qei_angle_temp = __builtin_muluu(qei_counter_mod , 32768);
    qei_angle_temp = __builtin_divud(qei_angle_temp , 1170);//16384/7/2 ;// 4096/7/2
    qei_angle_s16 = (signed long)(qei_angle_temp - Q15(0.9999));
}
#if 1
void pSpeed_control(int target_speed,unsigned char inc_val)
{
    if(qei_counter > x2c_pos_cmd)
    {
        if(inc_val==0)
        {
            x2c_motor_speed_cmd = -target_speed;
        }
        else
        {
            if(x2c_motor_speed_cmd >  -target_speed)
                x2c_motor_speed_cmd -= inc_val;
            else
                x2c_motor_speed_cmd += inc_val;              
        }
    }
    else if(qei_counter < x2c_pos_cmd) 
    {
        if(inc_val==0)
        {
             x2c_motor_speed_cmd = target_speed;
        }
        else
        {
            if(x2c_motor_speed_cmd >  target_speed)
                x2c_motor_speed_cmd -= inc_val;
            else
                x2c_motor_speed_cmd += inc_val;            
        }

    }
    if(x2c_motor_speed_cmd < 8000)
    {
        x2c_iq_limit_a = 2000;
    }
    else
    {
        x2c_iq_limit_a = 3000;        
    }
}
#endif




void open_loop_parm(void)
{
       x2c_angle_open_loop = 1;
       x2c_vd_open = 1;
       open_flag = 1;
       x2c_vs_openloop = 1;    
}

void close_loop_parm(void)
{
       x2c_angle_open_loop = 2;
       x2c_vd_open = 0;
       open_flag = 0;
       x2c_vs_openloop = 0;    
}    

void pos_contol_short_mc1(void)
{
    signed long t32_outmax;

    if(delta_position > 1000)
    {
        x2c_wki =  x2c_speed_wki[0];
        x2c_wkp = x2c_speed_wkp[0];
    }    
    else if((delta_position > 500) && (delta_position <= 1000))
    {
        x2c_wki =  x2c_speed_wki[1];
        x2c_wkp = x2c_speed_wkp[1];  
    }
    else if((delta_position > 400) && (delta_position <= 500))
    {
        x2c_wki =  x2c_speed_wki[2];
        x2c_wkp = x2c_speed_wkp[2];  
    }    
    else if((delta_position > 300) && (delta_position <= 400))
    {
        x2c_wki =  x2c_speed_wki[3];
        x2c_wkp = x2c_speed_wkp[3]; 
    }
    else if((delta_position > 200) && (delta_position <= 300))
    { 
        x2c_wki =  x2c_speed_wki[4];
        x2c_wkp = x2c_speed_wkp[4];
    } 
    else if((delta_position > 100) && (delta_position <= 200))
    {   
        x2c_wki =  x2c_speed_wki[5];
        x2c_wkp = x2c_speed_wkp[5];
    }
    else if((delta_position > 80) && (delta_position <= 100))
    {
        x2c_wki =  x2c_speed_wki[6];
        x2c_wkp = x2c_speed_wkp[6]; 
    }
    else if((delta_position > 50) && (delta_position <= 80))
    { 
        x2c_wki =  x2c_speed_wki[7];
        x2c_wkp = x2c_speed_wkp[7];
    } 
    else if((delta_position > 30) && (delta_position <= 50))
    {   
        x2c_wki =  x2c_speed_wki[8];
        x2c_wkp = x2c_speed_wkp[8];
    }
    else if((delta_position > 20) && (delta_position <= 30))
    {
        x2c_wki =  x2c_speed_wki[9];
        x2c_wkp = x2c_speed_wkp[9]; 
    }  
    else if((delta_position > 10) && (delta_position <= 20))
    {   
        x2c_wki =  x2c_speed_wki[10];
        x2c_wkp = x2c_speed_wkp[10];
    } 
    else if((delta_position > 5) && (delta_position <= 10))
    {   
        x2c_wki =  x2c_speed_wki[11];
        x2c_wkp = x2c_speed_wkp[11];
    } 
    else if((delta_position > 3) && (delta_position <= 5))
    {   
        x2c_wki =  x2c_speed_wki[12];
        x2c_wkp = x2c_speed_wkp[12];
    }     
    else 
    {   
        x2c_wki =  x2c_speed_wki[13];
        x2c_wkp = x2c_speed_wkp[13];        
    }       
    
    
    
    if(delta_position > 1000)
    {
        pos_outmax_final = 10000;
        //pos_to_speed_mc2();        
    }
    else
    {
        t32_outmax = __builtin_mulss(delta_position , x2c_pos_outmax);
        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
    }
//    if(delta_position > 1000)
//    {
//        pos_outmax_final = 10000;     
//    }
//    else if (delta_position > 30)
//    {
//        t32_outmax = __builtin_mulss(delta_position , x2c_pos_outmax);
//        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
//    }
//    else if (delta_position > 20)
//    {
//        t32_outmax = __builtin_mulss(delta_position , 6000);
//        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
//    }   
//    else if (delta_position > 10)
//    {
//        t32_outmax = __builtin_mulss(delta_position , 3000);
//        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
//    }   
//    else if (delta_position > 5)
//    {
//        t32_outmax = __builtin_mulss(delta_position , 2000);
//        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
//    }   
//    else if (delta_position > 3)
//    {
//        t32_outmax = __builtin_mulss(delta_position , 1000);
//        pos_outmax_final = __builtin_divsd(t32_outmax , x2c_max_delta_position);           
//    }       
}


void pos_to_speed(void)
{
    if(qei_counter > x2c_pos_cmd)
    {  
        if(x2c_motor_speed_cmd > -pos_outmax_final)
           x2c_motor_speed_cmd -= x2c_pos_inc_val;
        else
            x2c_motor_speed_cmd = -pos_outmax_final;    
    }
    else if(qei_counter < x2c_pos_cmd) 
    {
        if(x2c_motor_speed_cmd < pos_outmax_final)
            x2c_motor_speed_cmd += x2c_pos_inc_val;
        else
            x2c_motor_speed_cmd = pos_outmax_final;
    } 
}

void pos_control_mc1(void)
{
    delta_position = abs((signed int)qei_counter - (signed int)x2c_pos_cmd);
    
    if(x2c_pos_cmd != x2c_pos_cmd_old)
    {
        x2c_pos_cmd_old = x2c_pos_cmd;
        pos_state = (delta_position <= 500) ? POS_SHORT :POS_LONG;
    }
    switch(pos_state)
    {
        case POS_SHORT:
            pos_contol_short_mc1();
            break;
        case POS_LONG:
            pos_contol_short_mc1();
            break;
    }
    pos_to_speed();
}



int16_t speed1_qTargetVelocity;
int16_t speed1_qDiff;
int16_t speed1_qVelRef;
int16_t speed1_qTargetVelocity;       
int16_t speedRampSkipCnt_a;
int16_t speedRampIncLimit_a = 10;
int16_t speed_CLSpeedRampRate_a = 10;
int16_t speedRampDecLimit_a = 10;

int32_t speed1_qKpOut;
int16_t speed1_qKpScaleInt;
int32_t speed1_qdSumUnsat;
int32_t speed1_qdSum;
int32_t speed1_qdUnSatOutSum;
int16_t speed1_qOutMax = 5000;
int16_t speed1_qOutMin = -5000;
int16_t speed1_qOut;
int16_t speed1_delta;
int32_t speed1_calc_temp;
int16_t speed1_d_rpm;
int32_t speed1_qdSumUnsat_max = 20000000;
int32_t speed1_qdSumUnsat_min = -20000000;


void speed1_PIController(void)
{  
    int16_t currentError;
    int32_t speed_qOutMax_32bit, speed_qOutMin_32bit ,speed_qdSumUnsat_tmp,speed_tmp;
    
    currentError =  speed1_qVelRef - motor_a_rpm;    
    speed1_qKpOut  = __builtin_mulss(currentError, x2c_wkp);
    if (speed1_qKpScaleInt > 0)
    {
        speed1_qKpOut  = speed1_qKpOut  << (int32_t)speed1_qKpScaleInt;
    }
    speed_qdSumUnsat_tmp = __builtin_mulss(currentError, x2c_wki);    
    speed1_qdSumUnsat = (int32_t)speed_qdSumUnsat_tmp >> (int32_t)10;    
    speed1_qdSumUnsat = speed1_qdSum + speed1_qdSumUnsat;
    
    
//    speed_tmp = __builtin_mulss(motor_b_rpm, motor_b_rpm_old); 
//    if(speed_tmp < 0)
//    {
//        speed_qdSumUnsat = 0;
//    }
//     

    speed1_delta = motor_a_rpm - motor_a_rpm_old;       
    motor_a_rpm_old = motor_a_rpm;
    speed1_calc_temp = __builtin_mulss(speed1_delta, x2c_wkd);    

    speed1_calc_temp = speed1_calc_temp << 10;
    if(speed1_calc_temp >= speed1_qdSumUnsat_max)
    {
        speed1_calc_temp = speed1_qdSumUnsat_max;
    }
    else if(speed1_calc_temp <= speed1_qdSumUnsat_min)
    {
        speed1_calc_temp = speed1_qdSumUnsat_min;        
    }              



    speed1_qdUnSatOutSum = speed1_qKpOut + speed1_qdSumUnsat - speed1_calc_temp;
    
  
    speed_qOutMax_32bit = ((int32_t)speed1_qOutMax) << (int32_t)15;
    speed_qOutMin_32bit = ((int32_t)speed1_qOutMin) << (int32_t)15;
    
	if(speed1_qdUnSatOutSum >=  speed_qOutMax_32bit)
    {        
		speed1_qOut =  speed1_qOutMax;
    }
	else if(speed1_qdUnSatOutSum <= speed_qOutMin_32bit)
    {
		speed1_qOut =  speed1_qOutMin;
    }
	else
    {
		speed1_qOut = (int16_t)(speed1_qdUnSatOutSum >> 15);
        speed1_qdSum = speed1_qdSumUnsat;
    }
}



void speed1_control_foc(void)
{
    speed1_qVelRef = x2c_motor_speed_cmd;
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

    speed1_PIController();
    x2c_mc1_iq_cmd = speed1_qOut;    
}