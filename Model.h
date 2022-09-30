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
#define  MIC_SOFT_VERSION        5  /*单片机软件版本号*/
#define  MIC_PROTO_VERSION       2  /*通信协议版本号20*/
#define  PHONE_TYPE              140 /*设备类型*/
#define  PHONE_EXTID             0  /*设备扩展ID*/
#define  PCB_VERSION             1  /*PCB版本号*/
#define  RESERVE                 0  /*预留*/
#define  TSC_TYPE                0  /*触摸屏型号[0-3] Byte3[5:4]*/
#define  LCD_TYPE                0  /*液晶型号[0-15] Byte[3:0]*/
#define  LCD_ROTATE              0  /*Byte3[7] 液晶旋转180度*/
#define  FUNCTION_BYTE2          2  /*功能位2*/
#define  FUNCTION_BYTE1          0  /*功能位1*/
#define  PHY_TYPE                0  /*PHY类型[0-3] Byte0[7-6]*/
#define  SENSOR_TYPE             1  /*sensor type Byte0[4:2] 0:IMX383 1:IMX283*/
#define  CODEC_TYPE              1  /*codec??[0-3] Byte0[7-5] 0:PCM1865 1:ES7210*/

/*************************************************************
 设备硬件版本号共由8字节组成
 B7[7:0]=合泰软件版本号
 B6[7:0]=合泰通讯协议
 B5[7:0]=机型
 B4[7:4]=PCB版本号
 B4[3:0]=预留位
 B3[7:0]=功能位3
 B3[7]    ==>  LCD旋转180度标志 0:不旋转
 B3[6]    ==>  保留
 B3[5:4]  ==>  TSC型号
 B3[3:0]  ==>  LCD型号
 B2[7:0]=功能位2  
 B1[7:0]=功能位1  
 B0[7:0]=功能位0    
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
