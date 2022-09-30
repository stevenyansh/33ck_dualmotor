// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file main.c
 *
 * @brief This is the main entry to the application.
 *
 * Component: APPLICATION
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

#include <xc.h>

#include "board_service.h"

#include "diagnostics/diagnostics.h"

#include "mc1_service.h"
#include "mc2_service.h"
#include "tricycle_control.h"
#include "uart1.h"
#include"pin_manager.h"
#include"spi1.h"
#include"spi2.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">

volatile int16_t sendData[10];
volatile int16_t recData[4];

signed int x2c_run_cmd;
signed long x2c_run_count;
signed long x2c_run_mod = 10000;
signed char x2c_dir_cmd;

unsigned int led_toggle_count;
unsigned int led_toggle_mod = 4000;
unsigned char led_flag;
unsigned int led_flash_count;
unsigned int led_flash_mod;

#define LED_OFF LATAbits.LATA3 = 0;
#define LED_ON LATAbits.LATA3 = 1;

void led_control(void);

#define SW1_IN PORTCbits.RC8
#define SW2_IN PORTCbits.RC9

void sw_read(void);

unsigned int sw1_on_count;
unsigned int sw1_off_count;
unsigned char sw1_on_flag;

unsigned int sw2_on_count;
unsigned int sw2_off_count;
unsigned char sw2_on_flag;

signed int adc_sp_cmd_sample;


// </editor-fold>


// <editor-fold defaultstate="collapsed" desc=" Function Declarations ">

signed char uart_motor_speed_cmd;
signed int uart_motor_speed_final;

unsigned char uart_read_data;


unsigned char uart_send_status;
unsigned char uart_read_seq;
unsigned char uart_send_seq;

unsigned char uart_send_buff[8];
unsigned char uart_read_buff[8];
unsigned char uart_read_data;

unsigned char uart_send_sum;
unsigned char uart_read_sum;

unsigned char comm_in_flag;

unsigned int uart_time_out_count;
unsigned int uart_time_out_mod;

unsigned int uart_send_delay;
unsigned char uart_send_data;

unsigned int auto_run_delay;

// </editor-fold>
/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
extern unsigned int x2c_pos_cmd_mc2; 
extern unsigned int x2c_pos_cmd; 
extern unsigned char x2c_run_motor;
uint8_t Spi_TxData[4]={0x83,0xff,0xff,0xff};
uint16_t Spi_pRxData[4]={0};
uint32_t AngleIn17bits = 0;
uint32_t AngleIn17bits_tmp = 0;

void Delay_ms(unsigned int time)
{
    unsigned char n;
    while(time > 0)
    {
        for(n = 0; n< 187;++n )
        {
            asm("nop");
        }
        time--;
    }
}
uint8_t SPI1_read_lock = 0;
uint8_t SPI2_read_lock = 0;
uint16_t SPI1_lock_count;
uint16_t SPI2_lock_count;
uint32_t AngleIn17bits_test2;
uint32_t SPI2_read_17bit(void)
{
    uint32_t read_value;
    
//    IO_RD7_SetDigitalOutput();   
    
//    if(SPI1_read_lock == 0)
    {
//        SPI2_read_lock = 1;
        SS2OUT_SetLow();
        Spi_pRxData[0] = SPI2_Exchange16bit(((0x03|0x80)<<8)|0x55);
        //IO_RD7_SetHigh();
        //Delay_ms(1);
        //IO_RD7_SetLow();
        Spi_pRxData[1] = SPI2_Exchange16bit(((0x04|0x80)<<8)|0x55);
        //IO_RD7_SetHigh();
        //Delay_ms(1);
        //IO_RD7_SetLow();
        Spi_pRxData[2] = SPI2_Exchange16bit(((0x05|0x80)<<8)|0x55);
        SS2OUT_SetHigh();
        AngleIn17bits_test2=((((uint32_t)Spi_pRxData[0]&0x00ff)<<10)|(((uint32_t)Spi_pRxData[1]&0x00fc)<<2)|(((uint32_t)Spi_pRxData[2]&0x00f0)>>4))&0x3ffff;        
//    SPI1_lock_count++;
//    if(SPI1_lock_count > 1000)
//    {
//        SPI2_read_lock = 0;
//        SPI1_lock_count = 0;
//    }    
    }

//    Delay_ms(5);
//    SPI2_read_lock = 0;  
    //AngleIn17bits_test=(AngleIn17bits_test *360+1)/262144;
    return (AngleIn17bits_test2);	    
}
uint32_t AngleIn17bits_test;

#if 1
uint32_t ReadAngle_16bit(void)
{
    
//    if(SPI2_read_lock == 0)
    {
//        SPI1_read_lock = 1;
	    SS1OUT_SetLow();
        Spi_pRxData[0] = SPI1_Exchange16bit(((0x03|0x80)<<8)|0x55);
	    //SS1OUT_SetHigh();
        //Delay_ms(1);
	    //SS1OUT_SetLow();
        Spi_pRxData[1] = SPI1_Exchange16bit(((0x04|0x80)<<8)|0x55);
	    //SS1OUT_SetHigh();
        //Delay_ms(1);
	    //SS1OUT_SetLow();
        Spi_pRxData[2] = SPI1_Exchange16bit(((0x05|0x80)<<8)|0x55);
	    SS1OUT_SetHigh();   
        
        AngleIn17bits_test=((((uint32_t)Spi_pRxData[0]&0x00ff)<<10)|(((uint32_t)Spi_pRxData[1]&0x00fc)<<2)|(((uint32_t)Spi_pRxData[2]&0x00f0)>>4))&0x3ffff;
        SPI2_lock_count++;
//        if(SPI2_lock_count > 1000)
//        {
//            SPI1_read_lock = 0;
//            SPI2_lock_count = 0;
//        }         
    }
   
//    Delay_ms(5);    
//    SPI1_read_lock = 0;
	    //AngleIn17bits_test=(AngleIn17bits_test *360+1)/262144;
    return (AngleIn17bits_test);	    
}
#endif

int main (void)
{
    InitOscillator();
    SetupGPIOPorts();  
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsInit();
#endif 
        
    HAL_InitPeripherals();

#ifdef MOTO_CONTROL_A
    MCAPP_MC1ServiceInit();
//    SCCP1_CAPTURE_Start();  //Betty Add
#endif 
    
#ifdef MOTO_CONTROL_B
    MCAPP_MC2ServiceInit();
//    SCCP2_CAPTURE_Start();  //Betty Add
#endif 
    
    HAL_ResetPeripherals();
    SS1OUT_SetHigh();
    IO_RD7_SetHigh();
    //Delay_ms(5000);
    //x2c_run_motor = 1;
    while(1)
    { 
#ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepMain();
#endif
    }
}

int16_t runCmdMC1 = 0, qTargetVelocityMC1, btnPressed;
int16_t runCmdMC2 = 0, qTargetVelocityMC2, btnPressed;
unsigned char x2c_fault_clear_mc1;
unsigned char x2c_fault_clear_mc2;
int angle_value = 0;
int x2c_auto_test = 0;
int x2c_auto_test_val = 1055;

int x2c_auto_test_offset = 2000;

void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt (void)
{
    static int count=0;
    static unsigned long count2=0;    
    count++;
    if(count>100)
    {
//        angle_value = ReadAngle_16bit();
        count = 0;
    }
    if(x2c_auto_test)
    {
        count2++;
        if(count2>100000)
        {
            if(x2c_pos_cmd_mc2<12000 || x2c_pos_cmd < 12000)
            {
                x2c_pos_cmd += x2c_auto_test_val;    
                x2c_pos_cmd_mc2 += x2c_auto_test_val;  
            }
            else
            {
                x2c_pos_cmd = x2c_auto_test_offset;  
                x2c_pos_cmd_mc2 = x2c_auto_test_offset;  
            }
            count2=0;
        }          
    }

    #ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepIsr();
    #endif
    
    led_control();
    
//    sw_read();
    uart_speed_cmd_read();
   
    if(x2c_fault_clear_mc1)
    {
        x2c_fault_clear_mc1 = 0;
        PG1FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG2FPCILbits.SWTERM = 1; //Clear Fault PCI   
        PG3FPCILbits.SWTERM = 1; //Clear Fault PCI
    }
   
    if(x2c_fault_clear_mc2)
    {
        x2c_fault_clear_mc2 = 0;
        PG6FPCILbits.SWTERM = 1; //Clear Fault PCI
        PG7FPCILbits.SWTERM = 1; //Clear Fault PCI   
        PG8FPCILbits.SWTERM = 1; //Clear Fault PCI        
    }
#ifdef MOTO_CONTROL_A        
    qTargetVelocityMC1 = MCAPP_MC1GetTargetVelocity();   //Betty add
    MCAPP_MC1InputBufferSet(runCmdMC1, qTargetVelocityMC1);
#endif
#ifdef MOTO_CONTROL_B     
    qTargetVelocityMC2 = MCAPP_MC2GetTargetVelocity();   //Betty add
    MCAPP_MC2InputBufferSet(runCmdMC2, qTargetVelocityMC2);
#endif 
    uart_data_send();
    
    TIMER1_InterruptFlagClear();
    

}


void led_control(void)
{
    if(x2c_motor_command_mc1 == 0)
    {
        LED_OFF;
//        if(led_flag)
//        {
//            LED_ON;
//        }
//        else
//        {
//            LED_OFF;
//        }
    }
    else
    {
        led_flash_mod = x2c_motor_command_mc1;
        if(led_flash_count < led_flash_mod)
        {
            if(led_toggle_count < led_toggle_mod)
            {
                led_toggle_count ++;
            }
            else
            {
                led_toggle_count = 0;
                if(led_flag)
                {
                    led_flag = 0;
                    LED_OFF;
                    led_flash_count ++;
                }
                else
                {
                    led_flag = 1;
                    LED_ON;
                }
            }
        }
        else
        {
            if(led_toggle_count < __builtin_muluu(led_toggle_mod,3))
            {
                led_toggle_count ++;
            }
            else
            {
                led_flash_count = 0;
                led_toggle_count = 0;
            }
        }
    }
}

void sw_read(void)
{
    adc_sp_cmd_sample = (int16_t)(ADCBUF11 >> 1);
    if(!SW1_IN)
    {
        if(!sw1_on_flag)
        {
            if(sw1_on_count >= 200)
            {
                sw1_on_flag = 1;
                if(x2c_motor_command_mc1 < 3)
                {
                    x2c_motor_command_mc1 ++;
                }
                else
                    x2c_motor_command_mc1 = 0;
            }
            else
                sw1_on_count ++;
        }
        else
        {
            sw1_off_count = 0;
        }
    }
    else
    {
        if(sw1_on_flag)
        {
            if(sw1_off_count >= 200)
            {
                sw1_on_flag = 0;
            }
            else
                sw1_off_count ++;
        }
        else
        {
            sw1_on_count = 0;
        }
    }
    
    if(!SW2_IN)
    {
        if(!sw2_on_flag)
        {
            if(sw2_on_count >= 200)
            {
                sw2_on_flag = 1;
                if(x2c_motor_command_mc1 == 0)
                    x2c_motor_command_mc1 = 5;
            }
            else
                sw2_on_count ++;
        }
        else
        {
            sw2_off_count = 0;
        }
    }
    else
    {
        if(sw2_on_flag)
        {
            if(sw2_off_count >= 200)
            {
                sw2_on_flag = 0;
            }
            else
                sw2_off_count ++;
        }
        else
        {
            sw2_on_count = 0;
        }
    }    
}


void uart_speed_cmd_read(void)
{
//    uart_time_out_count
//    uart_time_out_mod

    unsigned int t_motor_cmd_abs;
//        uart_motor_speed_cmd = (signed char)uart_read_data;
//        uart_motor_speed_final = uart_motor_speed_cmd;
//        uart_motor_speed_final = __builtin_mulss(uart_motor_speed_final , 10);
//
//        command_a_rpm = uart_motor_speed_final;//      
//        uart_time_out_count = 0;


    if(UART2_IsReceiveBufferDataReady())
    {
        uart_read_data = UART2_DataRead();
        if(uart_read_seq == 0)
        {
            uart_read_buff[0] = uart_read_data;
            if(uart_read_data == 0x55)
            {
                uart_read_seq ++;
            }
            else
                uart_read_seq = 0;
            uart_read_sum = uart_read_data;
        }
        else if(uart_read_seq == 1)
        {
            uart_read_buff[1] = uart_read_data;
            if(uart_read_data == 0x02)
            {
                uart_read_seq ++;
                uart_read_sum = uart_read_data;
            }
            else
                uart_read_seq = 0;
        }
        else if(uart_read_seq == 2)
        {
            uart_read_buff[2] = uart_read_data;
            if(uart_read_data == 0x03) //length
            {
                uart_read_seq ++;
                uart_read_sum += uart_read_data;
            }
            else
                uart_read_seq = 0;
        }
        else if(uart_read_seq == 3) //dir
        {
            uart_read_buff[3] = uart_read_data;
            uart_read_seq ++;
            uart_read_sum += uart_read_data;
        }
        else if(uart_read_seq == 4) //rpm H
        {
            uart_read_buff[4] = uart_read_data;
            uart_read_seq ++;
            uart_read_sum += uart_read_data;
        }
        else if(uart_read_seq == 5) //rpm L
        {
            uart_read_buff[5] = uart_read_data;
            uart_read_seq ++;
            uart_read_sum += uart_read_data;
        }
        else if(uart_read_seq == 6) //check sum
        {
            uart_read_buff[6] = uart_read_data;
            if(uart_read_buff[6] == uart_read_sum)
            {
                uart_read_seq ++;
            }
            else
                uart_read_seq = 0;
        }
        else if(uart_read_seq == 7)
        {
            uart_read_buff[7] = uart_read_data;
            if(uart_read_buff[7] == 0xAA)
            {
//                comm_in_flag = 1;
                uart_time_out_count = 0;
                t_motor_cmd_abs = uart_read_buff[4];
                t_motor_cmd_abs = t_motor_cmd_abs << 8;
                t_motor_cmd_abs += uart_read_buff[5];
                x2c_motor_speed_cmd = __builtin_divud(t_motor_cmd_abs , 10);
       
                x2c_motor_command_mc1 = uart_read_buff[3];
//                if(uart_read_buff[3] == 1)
//                    command_a_rpm  = 0 - t_motor_cmd_abs;
//                else
//                    command_a_rpm = t_motor_cmd_abs;
            }
            uart_read_seq = 0;;
        }
    }
    else
    {
//        if(uart_time_out_count < 16000)
//            uart_time_out_count ++;
//        else
//            command_a_rpm = 0;
    }
}

void uart_data_send(void)
{
    unsigned int t_motor_a_rpm_abs;
    if(uart_send_delay >= 4000)
    {
        if(UART2_IsTransmissionComplete())
        {
            UART2_DataWrite(uart_send_buff[uart_send_seq]);

            uart_send_seq ++;
            if(uart_send_seq > 7)
            {
                uart_send_seq = 0;
                uart_send_delay = 0;
            }
        }
    }
    else
    {
        uart_send_buff[0] = 0x55;
        uart_send_buff[1] = 0x01;
        uart_send_buff[2] = 0x03;
        uart_send_buff[3] = x2c_motor_command_mc1;

        t_motor_a_rpm_abs = __builtin_muluu(abs(motor_a_rpm) , 10);
        uart_send_buff[4] = t_motor_a_rpm_abs >> 8;
        uart_send_buff[5] = t_motor_a_rpm_abs & 0xFF;
        uart_send_buff[6] = uart_send_buff[1];
        uart_send_buff[6] += uart_send_buff[2];
        uart_send_buff[6] += uart_send_buff[3];
        uart_send_buff[6] += uart_send_buff[4];
        uart_send_buff[6] += uart_send_buff[5];
        uart_send_buff[7] = 0xAA;
        uart_send_delay ++;
    }
}