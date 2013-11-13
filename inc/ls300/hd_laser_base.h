/*!
 * \file hd_laser_base.h
 * \brief	控制板通讯协议
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser base
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */
#ifndef HD_LASER_BASE_H
#define HD_LASER_BASE_H

#include <arch/hd_plat_base.h>
#include <math.h>

#define DEFAULT_COM_PORT "com1"
#define DEFAULT_COM_BAUDRATE 38400
#define DEFAULT_COM_TIMEOUT  (1E6) //1秒

//------------------------------------------------------------------------------
//量纲转换
//1度 = 100步,每转180度
#define ANGLE_TO_STEP(_angle_) ((int)((_angle_)*100))
#define STEP_TO_ANGLE(_step_) ((((float)(_step_))/100))
#define PULSE_SPEED_TO_STEP_TIME(_speed_) ((_speed_)*2*40)   //in us
//通过电压值获得倾角
#define VOLTATE_TO_DIP(_value_) (asin(((double)_value_ - 1024.0) / 1638.0) * 180.0 / M_PI)

//------------------------------------------------------------------------------
#define STX 0x23   /*every PC->Control packet is started by STX*/
#define ACKSTX 0x06 /*every Control->PC packet is started by ACKSTX*/
#define ALLSTEPS ANGLE_TO_STEP(180) //对应180度
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))

#define FAST_SPEED 20 //水平转台最快转速
#define REVERSE_PULSE 500 //对应5度
#define TAKEPHOTO_ANGLE 30 //相机拍照间隔角度
//const const e_uint8
static const e_uint8 START_WORK[] = { 0x23, 0x4d, 0x4f, 0x54, 0x4f, 0x2c,
		0x46, 0x2c, 0x33, 0x36, 0x30, 0x30, 0x30, 0x2c, 0x30, 0x30, 0x32, 0x30,
		0x40, 0x0 };

#define TIMEOUT_TURNTABLE (1E6)//转盘 100MS
#define TIMEOUT_SEARCH_ZERO (1E6*30)//转盘到0点 30秒
#define TIMEOUT_ANGLE 	(1E6)  	//倾角
#define TIMEOUT_STEP 	(1E6)  	//水平角度
#define TIMEOUT_TEMPERATURE (1E6)		//温度
#define TIMEOUT_CAMERA (1E6*4)	//相机
#define TIMEOUT_LED (1E6)			//指示灯
#define TIMEOUT_INFO (1E6)		//取控制板信息,Battery
//-----------------------------------------------------------------------------

//开始工作回应码
#define MOTO_MSG_F "MOTO,F,%05d,%04d"
#define MOTO_MSG_R "MOTO,R,%05d,%04d"
static const e_uint8 MOTO_SUCCESS[] = { 'M', 'O', 'T', 'O', 'O', 'K', '!', 0 }; //success
//不使用 static const e_uint8 MOTO_ERROR[] = { 'M', 'O', 'T', 'O', 'N', 'O', '!', 0 }; //error

//停止水平台转动
static const e_uint8 STOP_WORK[] = { 'S', 'T', 'O', 'P', 0 };
static const e_uint8 HALT_MSG[] = { 'H', 'A', 'L', 'T', '!', 0 };

//测试控制板状态
static const e_uint8 CHECK_STATUS[] = { 'S', 'T', 'S', 0 };
static const e_uint8 CHECK_SUCCESS[] = { 'G','O',  ',',  'O',  'N',  '!', 0 };

//搜索零点
static const e_uint8 SEARCH_ZERO[] = { 'S', 'E', 'Z', 'E', 0 };
static const e_uint8 SEARCH_SUCCESS[] = { 'S', 'E', 'Z', 'E', 'O', 'K', '!', 0 }; //success
//不使用 static const e_uint8 SEARCH_ERROR[] = { 'S', 'E', 'Z', 'E', 'N', 'O', '!', 0 }; //error

//照相
static const e_uint8 TAKEPHOTO[] = { 'T', 'A', 'P', 'H', 0 };
static const e_uint8 PHOTO_SUCCESS[] = { 'P', 'H', 'O', 'T', 'O', 'O','K', '!', 0 };
//不使用 static const e_uint8 PHOTO_ERROR[] = { 'T', 'A', 'P', 'H', 'N', 'O', '!', 0 };

//LED指示灯
static const e_uint8 SET_LEDRED[] = { 'L', 'E', 'D', 'R', 0 };
static const e_uint8 SET_LEDGREEN[] = { 'L', 'E', 'D', 'G', 0 };
static const e_uint8 SET_LEDOFF[] = { 'L', 'E', 'D', 'X', 0 };
//static const e_uint8 LED_SUCCESS[] = {'L,'E','D','R','O','K',0}; #!@
static const e_uint8 LED_ERROR[] = { 'l', 'E', 'D', 'N', 'O', '!', 0 };

// 关机提示
#define SEND_MSG_SHUTDOWN "#FM@"

//-----------------------------------------------------------------------------

//返回当前状态
static const e_uint8 GET_TEMPERATURE[] = { 'R', 'E', 'T', 'E', 0 };
#define MSG_RET_TEMPERATURE "%f!"
static const e_uint8 GET_ANGLE[] = { 'R', 'E', 'A', 'N', 0 };
#define MSG_RET_ANGLE "%5d!%5d!"
static const e_uint8 GET_BATTERY[] = { 'R', 'E', 'B', 'A', 0 };
#define MSG_RET_BATTERY "%f!"
static const e_uint8 GET_ERROR[] = { 'R', 'S', 'T', 'A', 'N', 'O', '!', 0 };

//获取硬件信息
static const e_uint8 GET_INFO[][7] = {
		{ 'R', 'E', 'M', 'F', 0 }, { 'R', 'E', 'P', 'N', 0 }, { 'R', 'E', 'M',
				'O', 0 },
		{ 'R', 'E', 'S', 'N', 0 }, { 'R', 'E', 'P', 'D', 0 }, { 'R', 'E', 'I',
				'P', 0 },
		{ 0 } };
#define MSG_RET_INFO "%s"

//返回当前水平转台走了多少步
static const e_uint8 GET_STEP[] = { 'R', 'E', 'S', 'T', 0 };
static const e_uint8 STEP_ERROR[] = { 'R', 'E', 'S', 'T', 'N', 'O', '!', 0 };
#define MSG_RET_STEP "%05d!"

//
//// 停止水平转台
//#define SENDMSGSTOP #STOP@
//#define RECVMSGSTOP #HALT!@
//
//// 返回步数
//#define SENDMSGREST #REST@
//#define RECVMSGREST #%05d!@
//
//// 返回温度
//#define SENDMSGRETE #RETE@
//#define RECVMSGRETE #%02.1f@
//
//// 返回水平倾角
//#define SENDMSGREAN #REAN@
//#define RECVMSGREAN #%5d!%5d!@
//
//// 相机拍照
//#define SENDMSGTAPH #TAPH@
//#define RECVMSGTAPH #PHOTOOK!@
//
//// 亮绿灯
//#define SENDMSGLEDR #LEDR@
//#define RECVMSGLEDR #LEDROK!@
//
//// 亮红灯
//#define SENDMSGLEDG #LEDG@
//#define RECVMSGLEDG #LEDROK!@
//
//// 熄灯
//#define SENDMSGLEDX #LEDX@
//#define RECVMSGLEDX #LEDXOK!@
//
//// 搜索水平转台零点
//#define SENDMSGSEZE #SEZE@
//#define RECVMSGSEZE #SEZEOK!@
//
//// 返回公司名称
//#define SENDMSGREMF #REMF@
//#define RECVMSGREPN #B9E3D6DDD6D0BAA3B4EFCEC0D0C7B5BCBABDBCBCCAF5B9C9B7DDD3D0CFDEB9ABCBBE@
//
//// 返回产品型号
//#define SENDMSGREMO #REMO@
//#define RECVMSGREMO #LS300@
//
//// 返回产品编号
//#define SENDMSGRESN #RESN@
//#define RECVMSGRESN #LS300-%03d@
//
//// 返回生成日期
//#define SENDMSGREPD #REPD@
//#define RECVMSGREPD #%4d.%02d.%02d@
//
//// 返回硬件IP地址
//#define SENDMSGREIP #REIP@
//#define RECVMSGREIP #192.168.1.10@
//
//// 关机提示
//#define SENDMSGFM #FM@
//
//// 水平转台转动(#MOTO,正转/反转,步数,速度@)
//#define SENDMSGMOTO #MOTO,R/F,%05d,%04d@
//#define RECVMSGMOTO #MOTOOK!@

#endif /*HD_LASER_BASE_H*/

