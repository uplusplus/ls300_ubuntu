/*!
 * \file hd_laser_control.h
 * \brief	控制板通讯
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser base
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */
#ifndef HD_LASER_CONTROL_H
#define HD_LASER_CONTROL_H

/* Dependencies */
#include "hd_connect.h"
#include "hd_laser_base.h"
#include "hd_message_monitor.h"
/*结构体定义*/

typedef struct angle_t //倾角
{
	e_float64 dX; //X倾角值
	e_float64 dY; //Y倾角值
} angle_t;

typedef struct laser_control_t {
//private
	e_uint32 start_steps; //起始角度步数
	e_uint32 end_steps; //终止角度步数
	e_uint32 real_steps; //实际整个范围走过的步数
	e_uint16 plus_delay; //每一步的延迟时间
	e_float64 angle_pre;
	e_float64 current_angle; //当前转台角度

	//通讯相关
	hd_connect_t serial_port; // 通过serial连接开发板
	//信号监听器
	msg_monitor_t monitor;
	//设备监听接口
	fsocket_t *fs_turntable; //转盘
	fsocket_t *fs_dip; //倾角
	fsocket_t *fs_temperature; //温度
	fsocket_t *fs_camera; //相机
	fsocket_t *fs_led; //指示灯
	fsocket_t *fs_info; //取控制板信息

	volatile e_int32 state; //如果在工作，是不可以设置参数的，要设置的话，必须先停止工作
} laser_control_t;

/*接口定义*/
#ifdef __cplusplus
extern "C" {
#endif

//初始化
e_int32 DEV_EXPORT hl_open(laser_control_t *lc, char *com_name,
		e_uint32 baudrate);

e_int32 DEV_EXPORT hl_open_socket(laser_control_t *lc, e_uint8 *ip,
		e_uint32 port);

//关闭串口
e_int32 DEV_EXPORT hl_close(laser_control_t *lc);

//准备工作
e_int32 DEV_EXPORT hl_turntable_prepare(laser_control_t *lc, e_float64 pre_start_angle);
//开始工作
e_int32 DEV_EXPORT hl_turntable_start(laser_control_t *lc);

//停止工作
e_int32 DEV_EXPORT hl_turntable_stop(laser_control_t *lc);

//获取当前水平转台的角度
e_float64 DEV_EXPORT hl_turntable_get_angle(laser_control_t *lc);
//获取当前水平转台的步数
e_int32 DEV_EXPORT hl_turntable_get_step(laser_control_t *lc);

//转台工作
e_int32 DEV_EXPORT hl_turntable_config(laser_control_t *lc, e_uint32 plus_delay,
		e_float64 start, e_float64 end);
//根据实际传过来的水平旋转角度，调整水平台，以较快速度转到实际水平台的起始角度/转台回到起始原点
e_int32 DEV_EXPORT hl_turntable_turn(laser_control_t *lc, e_float64 angle);
e_int32 DEV_EXPORT hl_turntable_turn_async(laser_control_t *lc, e_float64 angle);
e_int32 DEV_EXPORT hl_turntable_fast_turn(laser_control_t *lc, e_float64 angle);

e_int32 DEV_EXPORT hl_turntable_check(laser_control_t *lc);


//相机拍照
e_int32 DEV_EXPORT hl_camera_take_photo(laser_control_t *lc);

//获取温度
e_float64 DEV_EXPORT hl_get_temperature(laser_control_t *lc);

//获取倾斜度
e_int32 DEV_EXPORT hl_get_tilt(laser_control_t *lc, angle_t* angle);

//获取状态
e_float64 DEV_EXPORT hl_get_battery(laser_control_t *lc);

//亮红灯
e_int32 DEV_EXPORT hl_led_red(laser_control_t *lc);

//亮绿灯
e_int32 DEV_EXPORT hl_led_green(laser_control_t *lc);

//LED熄灭
e_int32 DEV_EXPORT hl_led_off(laser_control_t *lc);

//搜索零点
e_int32 DEV_EXPORT hl_search_zero(laser_control_t *lc);

//硬件信息
e_int32 DEV_EXPORT hl_get_info(laser_control_t *lc, e_uint32 idx,
		e_uint8* buffer, e_int32 blen);

#ifdef __cplusplus
}
#endif

#endif /*HD_LASER_CONTROL_H*/

