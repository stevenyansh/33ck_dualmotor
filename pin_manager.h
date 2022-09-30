/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.169.0
        Device            :  dsPIC33CK128MP206
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

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>

/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RD0, high using LATD0.

  @Description
    Sets the GPIO pin, RD0, high using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 high (1)
    SCK1OUT_SetHigh();
    </code>

*/
#define SCK1OUT_SetHigh()          (_LATD0 = 1)
/**
  @Summary
    Sets the GPIO pin, RD0, low using LATD0.

  @Description
    Sets the GPIO pin, RD0, low using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 low (0)
    SCK1OUT_SetLow();
    </code>

*/
#define SCK1OUT_SetLow()           (_LATD0 = 0)
/**
  @Summary
    Toggles the GPIO pin, RD0, using LATD0.

  @Description
    Toggles the GPIO pin, RD0, using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD0
    SCK1OUT_Toggle();
    </code>

*/
#define SCK1OUT_Toggle()           (_LATD0 ^= 1)
/**
  @Summary
    Reads the value of the GPIO pin, RD0.

  @Description
    Reads the value of the GPIO pin, RD0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD0
    postValue = SCK1OUT_GetValue();
    </code>

*/
#define SCK1OUT_GetValue()         _RD0
/**
  @Summary
    Configures the GPIO pin, RD0, as an input.

  @Description
    Configures the GPIO pin, RD0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an input
    SCK1OUT_SetDigitalInput();
    </code>

*/
#define SCK1OUT_SetDigitalInput()  (_TRISD0 = 1)
/**
  @Summary
    Configures the GPIO pin, RD0, as an output.

  @Description
    Configures the GPIO pin, RD0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an output
    SCK1OUT_SetDigitalOutput();
    </code>

*/
#define SCK1OUT_SetDigitalOutput() (_TRISD0 = 0)
/**
  @Summary
    Sets the GPIO pin, RD1, high using LATD1.

  @Description
    Sets the GPIO pin, RD1, high using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD1 high (1)
    SDI1_SetHigh();
    </code>

*/
#define SDI1_SetHigh()          (_LATD1 = 1)
/**
  @Summary
    Sets the GPIO pin, RD1, low using LATD1.

  @Description
    Sets the GPIO pin, RD1, low using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD1 low (0)
    SDI1_SetLow();
    </code>

*/
#define SDI1_SetLow()           (_LATD1 = 0)
/**
  @Summary
    Toggles the GPIO pin, RD1, using LATD1.

  @Description
    Toggles the GPIO pin, RD1, using LATD1.

  @Preconditions
    The RD1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD1
    SDI1_Toggle();
    </code>

*/
#define SDI1_Toggle()           (_LATD1 ^= 1)
/**
  @Summary
    Reads the value of the GPIO pin, RD1.

  @Description
    Reads the value of the GPIO pin, RD1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD1
    postValue = SDI1_GetValue();
    </code>

*/
#define SDI1_GetValue()         _RD1
/**
  @Summary
    Configures the GPIO pin, RD1, as an input.

  @Description
    Configures the GPIO pin, RD1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD1 as an input
    SDI1_SetDigitalInput();
    </code>

*/
#define SDI1_SetDigitalInput()  (_TRISD1 = 1)
/**
  @Summary
    Configures the GPIO pin, RD1, as an output.

  @Description
    Configures the GPIO pin, RD1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD1 as an output
    SDI1_SetDigitalOutput();
    </code>

*/
#define SDI1_SetDigitalOutput() (_TRISD1 = 0)
/**
  @Summary
    Sets the GPIO pin, RD2, high using LATD2.

  @Description
    Sets the GPIO pin, RD2, high using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 high (1)
    SDO1_SetHigh();
    </code>

*/
#define SDO1_SetHigh()          (_LATD2 = 1)
/**
  @Summary
    Sets the GPIO pin, RD2, low using LATD2.

  @Description
    Sets the GPIO pin, RD2, low using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 low (0)
    SDO1_SetLow();
    </code>

*/
#define SDO1_SetLow()           (_LATD2 = 0)
/**
  @Summary
    Toggles the GPIO pin, RD2, using LATD2.

  @Description
    Toggles the GPIO pin, RD2, using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD2
    SDO1_Toggle();
    </code>

*/
#define SDO1_Toggle()           (_LATD2 ^= 1)
/**
  @Summary
    Reads the value of the GPIO pin, RD2.

  @Description
    Reads the value of the GPIO pin, RD2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD2
    postValue = SDO1_GetValue();
    </code>

*/
#define SDO1_GetValue()         _RD2
/**
  @Summary
    Configures the GPIO pin, RD2, as an input.

  @Description
    Configures the GPIO pin, RD2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an input
    SDO1_SetDigitalInput();
    </code>

*/
#define SDO1_SetDigitalInput()  (_TRISD2 = 1)
/**
  @Summary
    Configures the GPIO pin, RD2, as an output.

  @Description
    Configures the GPIO pin, RD2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an output
    SDO1_SetDigitalOutput();
    </code>

*/
#define SDO1_SetDigitalOutput() (_TRISD2 = 0)
/**
  @Summary
    Sets the GPIO pin, RD8, high using LATD8.

  @Description
    Sets the GPIO pin, RD8, high using LATD8.

  @Preconditions
    The RD8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD8 high (1)
    SS1OUT_SetHigh();
    </code>

*/
#define SS1OUT_SetHigh()          (_LATD8 = 1)
/**
  @Summary
    Sets the GPIO pin, RD8, low using LATD8.

  @Description
    Sets the GPIO pin, RD8, low using LATD8.

  @Preconditions
    The RD8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD8 low (0)
    SS1OUT_SetLow();
    </code>

*/
#define SS1OUT_SetLow()           (_LATD8 = 0)
/**
  @Summary
    Toggles the GPIO pin, RD8, using LATD8.

  @Description
    Toggles the GPIO pin, RD8, using LATD8.

  @Preconditions
    The RD8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD8
    SS1OUT_Toggle();
    </code>

*/
#define SS1OUT_Toggle()           (_LATD8 ^= 1)
/**
  @Summary
    Reads the value of the GPIO pin, RD8.

  @Description
    Reads the value of the GPIO pin, RD8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD8
    postValue = SS1OUT_GetValue();
    </code>

*/
#define SS1OUT_GetValue()         _RD8
/**
  @Summary
    Configures the GPIO pin, RD8, as an input.

  @Description
    Configures the GPIO pin, RD8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD8 as an input
    SS1OUT_SetDigitalInput();
    </code>

*/
#define SS1OUT_SetDigitalInput()  (_TRISD8 = 1)
/**
  @Summary
    Configures the GPIO pin, RD8, as an output.

  @Description
    Configures the GPIO pin, RD8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD8 as an output
    SS1OUT_SetDigitalOutput();
    </code>

*/
#define SS1OUT_SetDigitalOutput() (_TRISD8 = 0)

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the dsPIC33CK128MP206
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
#define IO_RD7_SetHigh()          (_LATD7 = 1)
/**
  @Summary
    Sets the GPIO pin, RD7, low using LATD7.

  @Description
    Sets the GPIO pin, RD7, low using LATD7.

  @Preconditions
    The RD7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD7 low (0)
    IO_RD7_SetLow();
    </code>

*/
#define IO_RD7_SetLow()           (_LATD7 = 0)
/**
  @Summary
    Toggles the GPIO pin, RD7, using LATD7.

  @Description
    Toggles the GPIO pin, RD7, using LATD7.

  @Preconditions
    The RD7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD7
    IO_RD7_Toggle();
    </code>

*/
#define IO_RD7_Toggle()           (_LATD7 ^= 1)
/**
  @Summary
    Reads the value of the GPIO pin, RD7.

  @Description
    Reads the value of the GPIO pin, RD7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD7
    postValue = IO_RD7_GetValue();
    </code>

*/
#define IO_RD7_GetValue()         _RD7
/**
  @Summary
    Configures the GPIO pin, RD7, as an input.

  @Description
    Configures the GPIO pin, RD7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD7 as an input
    IO_RD7_SetDigitalInput();
    </code>

*/
#define IO_RD7_SetDigitalInput()  (_TRISD7 = 1)
/**
  @Summary
    Configures the GPIO pin, RD7, as an output.

  @Description
    Configures the GPIO pin, RD7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD7 as an output
    IO_RD7_SetDigitalOutput();
    </code>

*/
#define IO_RD7_SetDigitalOutput() (_TRISD7 = 0)

/**
  @Summary
    Sets the GPIO pin, RD7, high using LATD7.

  @Description
    Sets the GPIO pin, RD7, high using LATD7.

  @Preconditions
    The RD7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD7 high (1)
    SS2OUT_SetHigh();
    </code>

*/
#define SS2OUT_SetHigh()          (_LATD7 = 1)
/**
  @Summary
    Sets the GPIO pin, RD7, low using LATD7.

  @Description
    Sets the GPIO pin, RD7, low using LATD7.

  @Preconditions
    The RD7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD7 low (0)
    SS2OUT_SetLow();
    </code>

*/
#define SS2OUT_SetLow()           (_LATD7 = 0)
void PIN_MANAGER_Initialize (void);



#endif
