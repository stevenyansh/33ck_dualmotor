// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc2_init.c
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 2 using field oriented control.
 * In this application to initialize variable required to run Fan.
 *
 * Component: APPLICATION (Motor Control 2 - mc2)
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
#include <string.h>

#include "mc2_init.h"
#include "mc2_calc_params.h"
#include "mc_app_types.h"

#include "board_service.h"
#include "generic_load.h"
#include "generic_load_types.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_MC2ControlSchemeConfig(MC2APP_DATA_T *);
static void MCAPP_MC2LoadConfig(MC2APP_DATA_T *);
static void MCAPP_MC2FeedbackConfig(MC2APP_DATA_T *);
static void MCAPP_MC2LoadStartTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC2LoadStopTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC2OutputConfig(MC2APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: MCAPP_MC2ParamsInit (MC2APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the Application data structure required for 
* controlling motor 2.
* @return none.
* @example
* <CODE> MCAPP_MC2ParamsInit(&mc2); </CODE>
*
*/
void MCAPP_MC2ParamsInit(MC2APP_DATA_T *pMCData)
{    
    /* Reset all variables in the data structure to '0' */
    memset(pMCData,0,sizeof(MC2APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    pMCData->pLoad = &pMCData->load;
    pMCData->pMotor = &pMCData->motor;
    pMCData->pPWMDuty = &pMCData->PWMDuty;
    
    /* Configure Feedbacks */
    MCAPP_MC2FeedbackConfig(pMCData);
    
    /* Configure Control Scheme */
    MCAPP_MC2ControlSchemeConfig(pMCData);
    
    /* Configure Load */
    MCAPP_MC2LoadConfig(pMCData);
    
    /* Configure Outputs */
    MCAPP_MC2OutputConfig(pMCData);

    /* Set motor control state as 'MTR_INIT' */
    pMCData->appState = MCAPP_INIT;
}

void MCAPP_MC2FeedbackConfig(MC2APP_DATA_T *pMCData)
{
    pMCData->HAL_MotorInputsRead = HAL_MC2MotorInputsRead;
    
    pMCData->motorInputs.measureVdc.dcMinRun = 
            NORM_VALUE(MC2_MOTOR_START_DC_VOLT, MC2_PEAK_VOLTAGE);
    
    if(MC2_MOTOR_STOP_DC_VOLT < MC2_MOTOR_START_DC_VOLT)
    {
        pMCData->motorInputs.measureVdc.dcMaxStop = 
            NORM_VALUE(MC2_MOTOR_STOP_DC_VOLT, MC2_PEAK_VOLTAGE);
    }
    else
    {
        pMCData->motorInputs.measureVdc.dcMaxStop = 
                pMCData->motorInputs.measureVdc.dcMinRun;
    }

}

void MCAPP_MC2ControlSchemeConfig(MC2APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    
    float tmp;
    
    pControlScheme = pMCData->pControlScheme;
    pMotorInputs = pMCData->pMotorInputs;
    pMotor = pMCData->pMotor;
    
    /* Configure Inputs */  
    pControlScheme->pIa = &pMotorInputs->measureCurrent.Ia;
    pControlScheme->pIb = &pMotorInputs->measureCurrent.Ib;
    pControlScheme->pVdc = &pMotorInputs->measureVdc.value;  
    pControlScheme->pMotor = pMCData->pMotor;
    
    /* Initialize motor parameters */
    pMotor->polePairs = POLEPAIRS;
    pMotor->qRs = NORM_RS;
    pMotor->RsScale = 15 - NORM_RS_SCALE;
    pMotor->qL0Dt = NORM_L0DTBASE;
    pMotor->qL1Dt = NORM_L1DTBASE;
    pMotor->LsDtBaseScale = 15 - NORM_LSDTBASE_SCALE;
    pMotor->qInvKFIBase = 
            __builtin_divsd(__builtin_mulss(NORM_INVKFIBASE, 
                                                MAXIMUM_SPEED_RPM),
                                MC2_PEAK_SPEED_RPM);
    
    pMotor->invKFIBaseScale = 15 - NORM_INVKFIBASE_SCALE;
    pMotor->qLdDt = NORM_LDDTBASE;
    pMotor->qLqDt = NORM_LQDTBASE;
    pMotor->qLdDtThreshold = NORM_LDDT_THRESHOLD;

    pMotor->qMaxSpeed = 
                NORM_VALUE(MAXIMUM_SPEED_RPM, MC2_PEAK_SPEED_RPM);
    
    pMotor->qMaxOLSpeed = 
                NORM_VALUE(END_SPEED_RPM, MC2_PEAK_SPEED_RPM);
    ol_speed_rec_mc2 = pMotor->qMaxOLSpeed;
            
    if(MINIMUM_SPEED_RPM < END_SPEED_RPM)
    {
        pMotor->qMinSpeed = 
                NORM_VALUE(END_SPEED_RPM, MC2_PEAK_SPEED_RPM);
    }
    else
    {
        pMotor->qMinSpeed = 
                NORM_VALUE(MINIMUM_SPEED_RPM, MC2_PEAK_SPEED_RPM);
    }

    pMotor->qRatedCurrent = NORM_VALUE(RATED_CURRENT, MC2_PEAK_CURRENT);

    tmp = (32768.0 * 32768.0)/
            (((float)pMotor->qInvKFIBase) * 
                ((float)(1 << NORM_INVKFIBASE_SCALE)));

    pMotor->KFIBaseScale = 0;
    while(tmp >= 32767.0)
    {
        tmp = tmp/2.0;
        pMotor->KFIBaseScale++;
    }
    pMotor->qKFIBase = (int16_t)tmp;
    pMotor->KFIBaseScale = 15 - pMotor->KFIBaseScale;
    
    /* Initialize FOC control parameters */
#ifdef  OPEN_LOOP_FUNCTIONING
    pControlScheme->ctrlParam.openLoop = 1;
#else
    pControlScheme->ctrlParam.openLoop = 0;
#endif            

    pControlScheme->ctrlParam.lockTimeLimit = LOCK_TIME;
    pControlScheme->ctrlParam.lockCurrent = 
                    NORM_VALUE(LOCK_CURRENT, MC2_PEAK_CURRENT);

    pControlScheme->ctrlParam.OLCurrent = 
                    NORM_VALUE(MIN_OPENLOOP_CURRENT, MC2_PEAK_CURRENT);
    pControlScheme->ctrlParam.OLCurrentMax = 
                    NORM_VALUE(MAX_OPENLOOP_CURRENT, MC2_PEAK_CURRENT);
    ol_current_rec_mc2 = pControlScheme->ctrlParam.OLCurrentMax;
    pControlScheme->ctrlParam.OLCurrentRampRate = OPENLOOP_CURRENT_RATE;
    pControlScheme->ctrlParam.OLCurrentRampSkipCntLimit = 
                OPENLOOP_CURRENT_COUNT_LIMIT;

    pControlScheme->ctrlParam.speedRampSkipCntLimit = OPENLOOP_SPD_COUNT_LIMIT;
    pControlScheme->ctrlParam.OLSpeedRampRate = OPENLOOP_SPD_RATE;

    pControlScheme->ctrlParam.speedRampIncLimit = SPD_RAMP_UP_COUNT_LIMIT;
    pControlScheme->ctrlParam.speedRampDecLimit = SPD_RAMP_DN_COUNT_LIMIT;

    pControlScheme->ctrlParam.qTargetVelocity = pMotor->qMaxOLSpeed;
    
    pControlScheme->ctrlParam.CLSpeedRampRate = SPEEDREFRAMP;
    
    pControlScheme->ctrlParam.normDeltaT = MC2_NORM_DELTAT;
    pControlScheme->ctrlParam.OmegaDtGain = MC2_WBASE_DT_LSGAIN;

    /* Initialize PI controller used for D axis current control */
    pControlScheme->piId.qKp = D_CURRCNTR_PTERM;
    pControlScheme->piId.qKi = D_CURRCNTR_ITERM;
    pControlScheme->piId.qOutMax = D_CURRCNTR_OUTMAX;
    pControlScheme->piId.qOutMin = (-pControlScheme->piId.qOutMax);
    pControlScheme->piId.qKpScaleInt = D_CURRCNTR_KPSCALE;

    /* Initialize PI controller used for Q axis current control */
    pControlScheme->piIq.qKp = Q_CURRCNTR_PTERM;
    pControlScheme->piIq.qKi = Q_CURRCNTR_ITERM;
    pControlScheme->piIq.qOutMax = Q_CURRCNTR_OUTMAX;
    pControlScheme->piIq.qOutMin = (-pControlScheme->piIq.qOutMax);
    pControlScheme->piIq.qKpScaleInt = Q_CURRCNTR_KPSCALE;

    /* Initialize PI controller used for speed control */
    pControlScheme->piSpeed.qKp = SPEEDCNTR_PTERM;
    pControlScheme->piSpeed.qKi = SPEEDCNTR_ITERM;
    pControlScheme->piSpeed.qOutMax = 
                    NORM_VALUE(SPEEDCNTR_OUTMAX, MC2_PEAK_CURRENT);
    pControlScheme->piSpeed.qOutMin = (-pControlScheme->piSpeed.qOutMax);
    pControlScheme->piSpeed.qKpScaleInt = SPEEDCNTR_KPSCALE;

    /* Initialize ATPLL Estimator */
    pControlScheme->estimATPLL.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->estimATPLL.pVdc = pControlScheme->pVdc;
    pControlScheme->estimATPLL.pIAlphaBeta = &pControlScheme->ialphabeta;
    pControlScheme->estimATPLL.pVAlphaBeta = &pControlScheme->valphabeta;
    pControlScheme->estimATPLL.pMotor = pMCData->pMotor;

    pControlScheme->estimATPLL.qKpGain = ESTIMATOR_KP_GAIN;
    pControlScheme->estimATPLL.qEmagFiltConst = EMAG_FILT_CONST;
    pControlScheme->estimATPLL.qOmegaFiltConst = OMEGA_FILT_CONST;
    pControlScheme->estimATPLL.qOmegaPIFiltConst = OMEGA_PI_FILT_CONST;

    /* Initialize field weakening controller */
    
    pControlScheme->idRefGen.pFdWeak = &pControlScheme->idRefGen.fdWeak;
    
    pControlScheme->idRefGen.fdWeak.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.fdWeak.pVdc = pControlScheme->pVdc;
    pControlScheme->idRefGen.fdWeak.pEstim = &pControlScheme->estimInterface;
    pControlScheme->idRefGen.fdWeak.pMotor = pMCData->pMotor;
    pControlScheme->idRefGen.fdWeak.pVdq = &pControlScheme->vdq;

    pControlScheme->idRefGen.fdWeak.IdRefFiltConst = IDREF_FILT_CONST;
    pControlScheme->idRefGen.fdWeak.LqsDIqsFiltConst = LQSIQS_FILT_CONST;
    pControlScheme->idRefGen.fdWeak.IdRefMin = 
                    NORM_VALUE(FD_WEAK_IDREF_LT, MC2_PEAK_CURRENT);

    /* Initialize MTPA controller */
    pControlScheme->idRefGen.pmtpa = &pControlScheme->idRefGen.mtpa;
    
    pControlScheme->idRefGen.mtpa.idRefMax = 0;
    pControlScheme->idRefGen.mtpa.idRefMin = 
                    NORM_VALUE(MTPA_IDREF_LT, MC2_PEAK_CURRENT);
    pControlScheme->idRefGen.mtpa.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.mtpa.pEstim = &pControlScheme->estimInterface;
    pControlScheme->idRefGen.mtpa.pMotor = pMCData->pMotor;

    if(pMotor->qLdDt <= pMotor->qLdDtThreshold)
    {
        /* Enable Saliency Observer & MTPA if motor has significant saliency */
        pControlScheme->idRefGen.mtpa.enable = 1;
        pControlScheme->estimATPLL.SAL_OBS = 1;
    }
    else
    {
        /* Disable Saliency Observer and MTPA if motor does not have significant
           saliency */
        pControlScheme->idRefGen.mtpa.enable = 0;
        pControlScheme->estimATPLL.SAL_OBS = 0;
    }

    /* Output Initializations */
    pControlScheme->pwmPeriod = MC2_LOOPTIME_TCY;
    pControlScheme->pPWMDuty = pMCData->pPWMDuty;
    
    /* Initialize Stall Detection parameters */
#ifdef  STALL_DETECT_ENABLE
        pControlScheme->stallDetect.enable = 1;
#else
        pControlScheme->stallDetect.enable = 0;
#endif 

    pControlScheme->stallDetect.pEAlphaBeta = &pControlScheme->
                                                estimATPLL.qEAlphaBeta;
    pControlScheme->stallDetect.pEmagExp = &pControlScheme->estimATPLL.qEmagExp;
    pControlScheme->stallDetect.pidq = &pControlScheme->idq;
    pControlScheme->stallDetect.pCtrlParam = &pControlScheme->ctrlParam;
    
    pControlScheme->stallDetect.qEmagFiltConst = STALL_DETECT_EMAG_FILT_CONST;
    pControlScheme->stallDetect.EmagStallCountLimit = 
                                                STALL_DETECT_EMAG_COUNT_LIMIT;
    pControlScheme->stallDetect.qEmagMargin = STALL_DETECT_EMAG_LIMIT;
    pControlScheme->stallDetect.currLimit = 
                    NORM_VALUE(STALL_DETECT_CURRENT_LIMIT, MC2_PEAK_CURRENT);
    pControlScheme->stallDetect.currStallCountLimit = 
                                            STALL_DETECT_CURRENT_COUNT_LIMIT;
    /* Initialize application structure */
    pMCData->MCAPP_ControlSchemeInit = MCAPP_FOCInit;
    pMCData->MCAPP_ControlStateMachine = MCAPP_FOCStateMachine;
    
    pMCData->MCAPP_InputsInit = MCAPP_MeasureCurrentInit;
    pMCData->MCAPP_MeasureOffset = MCAPP_MeasureCurrentOffset;
    pMCData->MCAPP_GetProcessedInputs = MCAPP_MeasureCurrentCalibrate;
    pMCData->MCAPP_IsOffsetMeasurementComplete = 
            MCAPP_MeasureCurrentOffsetStatus;
    
    pMCData->qMaxSpeedFactor = MC2_QMAX_SPEED_FACTOR;
}

/**
* <B> Function: MCAPP_MC2LoadConfig (MC2APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the FOC data structure required for controlling motor 2.
* @return none.
* @example
* <CODE> MCAPP_MC2LoadConfig(&mcData); </CODE>
*
*/

void MCAPP_MC2LoadConfig(MC2APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T  *pControlScheme;
    
    pControlScheme = pMCData->pControlScheme;
    
    pControlScheme->estimInterface.pOmega = 
            &pControlScheme->estimATPLL.qOmegaFilt;

    pControlScheme->estimInterface.pEmag = 
            &pControlScheme->estimATPLL.qEmagFilt;
    //panjunjun add (void *)for project warning
    pMCData->MCAPP_LoadInit = MCAPP_GenericLoadInit;
    pMCData->MCAPP_LoadStateMachine = MCAPP_GenericLoadStateMachine;
    pMCData->MCAPP_IsLoadReadyToStart = MCAPP_IsGenericLoadReadyToStart;
    pMCData->MCAPP_IsLoadReadyToStop = MCAPP_IsGenericLoadReadyToStop;
    
    pMCData->MCAPP_LoadStartTransition = MCAPP_MC2LoadStartTransition;
    pMCData->MCAPP_LoadStopTransition = MCAPP_MC2LoadStopTransition;
}


void MCAPP_MC2LoadStartTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pControlScheme->focState = FOC_CLOSE_LOOP;  //FOC_RTR_LOCK;
    pLoad->state = GENERIC_LOAD_RUN;
}

void MCAPP_MC2LoadStopTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pLoad->state = GENERIC_LOAD_STOP;
}

void MCAPP_MC2OutputConfig(MC2APP_DATA_T *pMCData)
{
    pMCData->HAL_PWMSetDutyCycles = HAL_MC2PWMSetDutyCycles;
    pMCData->HAL_PWMEnableOutputs = HAL_MC2PWMEnableOutputs;
    pMCData->HAL_PWMDisableOutputs = HAL_MC2PWMDisableOutputs;
    pMCData->MCAPP_HALSetVoltageVector = HAL_MC2SetVoltageVector;
}