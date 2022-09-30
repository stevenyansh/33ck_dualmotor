/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.169.0
        Device            :  dsPIC33CK256MP206
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.50
        MPLAB 	          :  MPLAB X v5.40
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/


/**
    Section: Includes
*/

#include <xc.h>
#include <stdio.h>
#include "pin_manager.h"

/**
 Section: Driver Interface Function Definitions
*/
void PIN_MANAGER_Initialize (void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0020;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x001F;
    TRISB = 0xF9FD;
    TRISC = 0xFFFF;
    TRISD = 0xF67B;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x001F;
    ANSELB = 0x019F;
    ANSELC = 0x00CE;
    ANSELD = 0x2400;
    
    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_RPCON(0x0000); // unlock PPS

//    RPOR4bits.RP41R = 0x001B;    //RB9->UART3:U3TX
//    RPOR17bits.RP66R = 0x0005;    //RD2->SPI1:SDO1
//    RPINR27bits.U3RXR = 0x0030;    //RC0->UART3:U3RX
//    RPINR20bits.SDI1R = 0x0041;    //RD1->SPI1:SDI1
//    RPOR5bits.RP42R = 0x000E;    //RB10->INTERNAL OSCILLATOR:REFO1
//    RPINR20bits.SCK1R = 0x0040;    //RD0->SPI1:SCK1OUT
//    RPOR16bits.RP64R = 0x0006;    //RD0->SPI1:SCK1OUT
//    RPOR20bits.RP72R = 0x0007;    //RD8->SPI1:SS1OUT

//    RPINR20bits.SDI1R = 0x004F;    //RD15->SPI1:SDI1
    RPINR20bits.SDI1R = 0x0028;    //RB8->SPI1:SDI1    
    RPOR22bits.RP76R = 0x0005;    //RD12->SPI1:SDO1 
    
    RPOR20bits.RP72R = 0x0007;    //RD8->SPI1:SS1OUT
//    RPOR22bits.RP77R = 0x0006;    //RD13->SPI1:SCK1OUT
    RPOR16bits.RP64R = 0x0009;    //RD0->SPI2:SCK2OUT
//    RPINR20bits.SCK1R = 0x004D;    //RD13->SPI1:SCK1OUT
    RPOR17bits.RP66R = 0x0008;    //RD2->SPI2:SDO2
    RPINR22bits.SCK2R = 0x0040;    //RD0->SPI2:SCK2OUT
    RPOR19bits.RP71R = 0x000A;    //RD7->SPI2:SS2OUT
    RPINR22bits.SDI2R = 0x0041;    //RD1->SPI2:SDI2
//    RPOR23bits.RP78R = 0x0005;    //RD14->SPI1:SDO1
   
//    RPOR16bits.RP64R = 0x0000;    //RD0->SPI2:SCK2OUT

    RPOR22bits.RP77R = 0x0006;    //RD13->SPI1:SCK1OUT
//    RPINR22bits.SDI2R = 0x0041;    //RD1->SPI2:SDI2

    RPINR20bits.SCK1R = 0x004D;    //RD13->SPI1:SCK1OUT
//    RPINR22bits.SCK2R = 0x0040;    //RD0->SPI2:SCK2OUT
    
    
    __builtin_write_RPCON(0x0800); // lock PPS
}

