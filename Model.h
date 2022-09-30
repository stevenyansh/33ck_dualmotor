/****************************************************************************
Copyright (C), 2015, Yealink Inc, All rights reserved
FileName: /E/VCC20/Software/VCC20_20150707/Utilities/Common/Model.h
Description:  
Version:  
Changelog:
Author: 
Email: 
Mobile: 
*****************************************************************************/

#ifndef _Model_H_
#define _Model_H_

/*For common use*/
/*Macro Definition*/
/************************************************************************/
/*1.0.0.0       first version 2021/12/23   1.solve qei INTXTMRL can't update data in low speed;
 *                                         2.add Id and Iq filter function;   
 *                                         3.speed run in pid mode                                                                  
 * 
 *   */
/************************************************************************/
#define CONFIG_SOFTWARE_VERSION     "1.0.0.0"

#define SW_LOADERVER                1
/*Data Type Specification*/
/************************************************************************/
/*                                                                      */
/************************************************************************/
#define  MIC_SOFT_VERSION        5  /*��Ƭ������汾��*/
#define  MIC_PROTO_VERSION       2  /*ͨ��Э��汾��20*/
#define  PHONE_TYPE              140 /*�豸����*/
#define  PHONE_EXTID             0  /*�豸��չID*/
#define  PCB_VERSION             1  /*PCB�汾��*/
#define  RESERVE                 0  /*Ԥ��*/
#define  TSC_TYPE                0  /*�������ͺ�[0-3] Byte3[5:4]*/
#define  LCD_TYPE                0  /*Һ���ͺ�[0-15] Byte[3:0]*/
#define  LCD_ROTATE              0  /*Byte3[7] Һ����ת180��*/
#define  FUNCTION_BYTE2          2  /*����λ2*/
#define  FUNCTION_BYTE1          0  /*����λ1*/
#define  PHY_TYPE                0  /*PHY����[0-3] Byte0[7-6]*/
#define  SENSOR_TYPE             1  /*sensor type Byte0[4:2] 0:IMX383 1:IMX283*/
#define  CODEC_TYPE              1  /*codec??[0-3] Byte0[7-5] 0:PCM1865 1:ES7210*/

/*************************************************************
 �豸Ӳ���汾�Ź���8�ֽ����
 B7[7:0]=��̩����汾��
 B6[7:0]=��̩ͨѶЭ��
 B5[7:0]=����
 B4[7:4]=PCB�汾��
 B4[3:0]=Ԥ��λ
 B3[7:0]=����λ3
 B3[7]    ==>  LCD��ת180�ȱ�־ 0:����ת
 B3[6]    ==>  ����
 B3[5:4]  ==>  TSC�ͺ�
 B3[3:0]  ==>  LCD�ͺ�
 B2[7:0]=����λ2  
 B1[7:0]=����λ1  
 B0[7:0]=����λ0    
*************************************************************/
#define HD_VERSION_BYTE7        (MIC_SOFT_VERSION%16)
#define HD_VERSION_BYTE6        ((MIC_PROTO_VERSION%4) + (PHONE_EXTID%16)*16)
#define HD_VERSION_BYTE5        (PHONE_TYPE%256)
#define HD_VERSION_BYTE4        (PCB_VERSION%16)*16
#define HD_VERSION_BYTE3        (LCD_ROTATE*128 +(TSC_TYPE%4)*16 + (LCD_TYPE%16))
#define HD_VERSION_BYTE2        (FUNCTION_BYTE2%256)
#define HD_VERSION_BYTE1        (FUNCTION_BYTE1%256)
#define HD_VERSION_BYTE0        ((CODEC_TYPE%8)*32+(SENSOR_TYPE%8)*4)

#endif
