/* 
 * File:   tricycle_control.h
 * Author: SZ-P881
 *
 * Created on March 23, 2021, 10:39 AM
 */

#ifndef TRICYCLE_CONTROL_H
#define	TRICYCLE_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#define     TRICYCLE_FORWARD    0   
#define     TRICYCLE_BACKWARD   1 

#define     FORWARD_DIRECTION   0
#define     BACKWARD_DIRECTION  1   
    
#define     Sys_Alarm_ON()      PORTBbits.RB6 = true    
#define     Sys_Alarm_OFF()     PORTBbits.RB6 = false     
extern int16_t runCmdMC1;
extern int16_t runCmdMC2;

extern uint8_t TriCycle_Direction;     //0: forward    1:backward
extern uint8_t MOTORA_Rotation;        //0: positive    1:negative
extern uint8_t MOTORB_Rotation;        //0: positive    1:negative

extern uint16_t TriCycle_Speed;
extern uint16_t TriCycle_Speed_bak;

extern unsigned char x2c_fault_clear_mc1;
extern unsigned char x2c_fault_clear_mc2;

extern uint8_t Sys_Speed_Lev1;
extern uint8_t Sys_Speed_Lev2;
extern uint8_t Sys_Speed_Lev3;

extern uint8_t Sys_HandBreak_Status;
extern uint8_t Motor_Run_Status;

uint8_t Sys_Direction_Get(void);
uint8_t Sys_ParkingStus_Get(void);
void Sys_HandBreak_Process(void);
void mc1_PWM_Fault_Process(void);
void mc2_PWM_Fault_Process(void);
void Calculate_Sys_Speed(void);
void Sys_Param_Get(void);
void Sys_Rotation_Angle_Process(void);
void SysMC_Temp_Process(void);
#ifdef	__cplusplus
}
#endif

#endif	/* TRICYCLE_CONTROL_H */

