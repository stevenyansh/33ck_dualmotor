// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file tricycle_control.c
 *
 * @brief This is the sys control application.
 *
 * Component: APPLICATION
 *
 */
// </editor-fold>
#include <xc.h>
#include "stdbool.h"
#include "board_service.h"
#include "tricycle_control.h"
/*
 * Global varible 
 */
//sys moto rotation
uint8_t TriCycle_Direction = 0;     //0: forward    1:backward
uint8_t MOTORA_Rotation = 0;        //0: positive    1:negative
uint8_t MOTORB_Rotation = 0;        //0: positive    1:negative
//sys speed
uint16_t TriCycle_Speed = 0;        //TriCycle_Speed = Number of Turns(every minute) * MOTOR_PERIMETER
uint16_t TriCycle_Speed_bak = 0;
uint16_t TriCycle_Speed_Sum = 0;
//three speed flag
uint8_t Sys_Speed_Lev1  = 1;    //high speed
uint8_t Sys_Speed_Lev2  = 0;    //middle speed  
uint8_t Sys_Speed_Lev3  = 0;    //low speed
//hand break process
uint8_t Sys_HandBreak_Status = 0;
//Sys_Rotation_Angle
uint16_t Sys_Rotation_Angle = 0;

uint8_t Motor_Run_Status = 0;

/**
* <B> Function: Sys_Direction_Set (void)  </B>
*
* @brief Sys_Direction_Set() function,set tricycle forward or backward.
*
*/
uint8_t Sys_Direction_Get(void)
{
    if(PORTDbits.RD5)   //R gear status of GPIO
    {
        TriCycle_Direction = TRICYCLE_FORWARD;
        MOTORA_Rotation = FORWARD_DIRECTION;
        MOTORB_Rotation = FORWARD_DIRECTION;    //??2??????1?????
    }
    else
    {
        TriCycle_Direction = TRICYCLE_BACKWARD;
        MOTORA_Rotation = BACKWARD_DIRECTION;
        MOTORB_Rotation = BACKWARD_DIRECTION;
    }
    return TriCycle_Direction;
}

uint8_t Sys_ParkingStus_Get(void)
{
    uint8_t tricycle_status = 0;    //0:stop, 1:run
    if(PORTDbits.RD6)   //RPARK gear status of GPIO
    {
        runCmdMC1 = 0;
        runCmdMC2 = 0;
        tricycle_status = 0;
    }
    else
    {
        runCmdMC1 = 1;
        runCmdMC2 = 1;
        tricycle_status = 1;
    }
    return tricycle_status;
}

void Sys_HandBreak_Process(void)
{
    if(PORTDbits.RD7 == 0)  //hand break is active
    {
        PORTDbits.RD8 = true;   //TAIL light 
//        PORTBbits.RB6 = true;   //add beep for test
        
        PORTBbits.RB3 = true;  //right electric break
        PORTBbits.RB4 = true;  //rleft electric break
        
        Sys_HandBreak_Status = true;
    }
    else
    {
        PORTDbits.RD8 = false;
//        PORTBbits.RB6 = false;   //add beep for test
        
        PORTBbits.RB3 = false; 
        PORTBbits.RB4 = false;
        
        Sys_HandBreak_Status = false;
    }
}


void mc1_PWM_Fault_Process(void)
{
    if(PG1STATbits.FLTEVT)
    {
        x2c_fault_clear_mc1 = 1;//add for X2C test
        PG1STATbits.FLTEVT = 0;
        PG1FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG1IOCONLbits.FLTDAT = 0x00;    //config PWM up and lower brige stat
    }
    if(PG2STATbits.FLTEVT)
    {
        x2c_fault_clear_mc1 = 1;//add for X2C test
        PG2STATbits.FLTEVT = 0;
        PG2FPCILbits.SWTERM = 1; //Clear Fault PCI 
        PG2IOCONLbits.FLTDAT = 0x00;
    }
    if(PG3STATbits.FLTEVT)
    {
        x2c_fault_clear_mc1 = 1;//add for X2C test
        PG3STATbits.FLTEVT = 0;
        PG3FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG3IOCONLbits.FLTDAT = 0x00;
    }
}

void mc2_PWM_Fault_Process(void)
{
    if(PG6STATbits.FLTEVT)
    {
        x2c_fault_clear_mc2 = 1;//add for X2C test
        PG6STATbits.FLTEVT = 0;
        PG6FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG6IOCONLbits.FLTDAT = 0x00;
    }
    if(PG7STATbits.FLTEVT)
    {
        x2c_fault_clear_mc2 = 1;//add for X2C test
        PG7STATbits.FLTEVT = 0;
        PG7FPCILbits.SWTERM = 1; //Clear Fault PCI 
        PG7IOCONLbits.FLTDAT = 0x00;
    }
    if(PG8STATbits.FLTEVT)
    {
        x2c_fault_clear_mc2 = 1;//add for X2C test
        PG8STATbits.FLTEVT = 0;
        PG8FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG8IOCONLbits.FLTDAT = 0x00;
    }
}
/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         16000
/* Specify dead time in micro seconds */
#define DEADTIME_MICROSEC       4.0
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC            0.0000625
/* Specify PWM Period in micro seconds */
#define LOOPTIME_MICROSEC       62.5
/*
 *function:
 * description:
 * 
 */
extern unsigned char cap_360_on_flag_a;
extern unsigned int msp6_pwm_a;
int16_t fl_Speed_FB;
void Calculate_Sys_Speed(void)
{
//    if(cap_360_on_flag_a)
//    TriCycle_Speed = MOTOR_PERIMETER * 3600/(msp6_pwm_a * MOTOR_POLEPAIRS * LOOPTIME_SEC*1000000);
    TriCycle_Speed = MOTOR_PERIMETER * 3600/(msp6_pwm_a * MOTOR_POLEPAIRS * LOOPTIME_MICROSEC);

    TriCycle_Speed_Sum += TriCycle_Speed - fl_Speed_FB ;	
    fl_Speed_FB = TriCycle_Speed_Sum >> 6;
    if(TriCycle_Speed_bak != TriCycle_Speed) 
    {
        TriCycle_Speed_bak = TriCycle_Speed;
    }
}

void Sys_Param_Get(void)
{
    if(PORTDbits.RD13 == 0) //three speed
    {
        Sys_Speed_Lev1 = 1;
        Sys_Speed_Lev2 = 0;
        Sys_Speed_Lev3 = 0;
    }
}


void Sys_Rotation_Angle_Process(void)
{
    uint16_t angel_value = 0;
    angel_value = SYS_ROTATION_ANGEL;
    
    if(angel_value > 16384)
        angel_value = 16384;
    else if(angel_value < 0)
        angel_value = 0;
    else
    {
        Sys_Rotation_Angle = 10;
    }
}

void SysMC_Temp_Process(void)
{
    uint16_t temp1 = 0;
    uint16_t temp2 = 0;
    
    temp1 = ADCBUF19;
    temp2 = ADCBUF18;

}