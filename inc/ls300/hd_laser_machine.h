/*!
 * \file hd_lm_server.h
 * \brief 定义了mseeage bus服务
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_LASER_MACHINE_H
#define HD_LASER_MACHINE_H

/* Dependencies */
#include <arch/hd_plat_base.h>
#include <ls300/hd_data_manager.h>
#include <ls300/hd_laser_control.h>
#include <arch/hd_thread_api.h>

#define USE_LASER_MACHINE 1

/* data structure*/
typedef struct laser_machine_t {
	e_float64 battery;
	e_float64 temperature;
	angle_t tilt;
	e_float64 angle;
	e_uint32 usec_timestamp;
	e_int8 LED;

	data_manager_t *dm;
	laser_control_t *lc;
	semaphore_t wakeup;
	ethread_t* main_thread;
	int is_paused;
	int state;
} laser_machine_t;

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

laser_machine_t* DEV_EXPORT lm_get_instance();

e_int32 DEV_EXPORT lm_init(laser_control_t *lc);
e_int32 DEV_EXPORT lm_uninit();

e_int32 DEV_EXPORT lm_start_record_angle(data_manager_t* dm);
e_int32 DEV_EXPORT lm_stop_record_angle();

e_int32 DEV_EXPORT lm_start_status_monitor();
e_int32 DEV_EXPORT lm_stop_status_monitor();

//

//准备工作
e_int32 DEV_EXPORT lm_turntable_prepare(e_float64 pre_start_angle);
//开始工作
e_int32 DEV_EXPORT lm_turntable_start();

//停止工作
e_int32 DEV_EXPORT lm_turntable_stop();

//获取当前水平转台的角度
e_float64 DEV_EXPORT lm_turntable_get_angle();
//获取当前水平转台的步数
e_int32 DEV_EXPORT lm_turntable_get_step();

//转台工作
e_int32 DEV_EXPORT lm_turntable_config(e_uint32 plus_delay, e_float64 start,
		e_float64 end);
//根据实际传过来的水平旋转角度，调整水平台，以较快速度转到实际水平台的起始角度/转台回到起始原点
e_int32 DEV_EXPORT lm_turntable_turn(e_float64 angle);
e_int32 DEV_EXPORT lm_turntable_turn_async(e_float64 angle);
e_int32 DEV_EXPORT lm_turntable_fast_turn(e_float64 angle);
e_int32 DEV_EXPORT lm_turntable_searchzero();

e_int32 DEV_EXPORT lm_turntable_check();

//相机拍照
e_int32 DEV_EXPORT lm_camera_take_photo();

//获取温度
e_float64 DEV_EXPORT lm_get_temperature();

//获取倾斜度
e_int32 DEV_EXPORT lm_get_tilt(angle_t* angle);

//获取状态
e_float64 DEV_EXPORT lm_get_battery();

//亮红灯
e_int32 DEV_EXPORT lm_led_red();

//亮绿灯
e_int32 DEV_EXPORT lm_led_green();

//LED熄灭
e_int32 DEV_EXPORT lm_led_off();

//LED
char* DEV_EXPORT lm_led_state();

//搜索零点
e_int32 DEV_EXPORT lm_search_zero();

//硬件信息
e_int32 DEV_EXPORT lm_get_info(e_uint32 idx, e_uint8* buffer, e_int32 blen);

#ifdef __cplusplus
}
#endif

#endif /*HD_LASER_MACHINE_H*/
