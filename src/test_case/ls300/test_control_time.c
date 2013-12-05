/*!
 * \file test_laser_control.c
 * \brief laser control test
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include "../hd_test_config.h"
#if TEST_CONTROL_TIME
#include <ls300/hd_laser_control.h>
#include <arch/hd_timer_api.h>
#include <signal.h>

int loop = 1;
laser_control_t control;

void sig_handler(int sig) {
	if (sig == SIGINT) {
		loop = 0;
	}
}

int main() {
	int ret;
	angle_t angle;
	e_uint32 start_time;
	e_float64 fvalue;

	signal(SIGINT, sig_handler);
	DMSG((STDOUT,"LASER CONTROL TEST start.\r\n"));

//ret = hl_open_socket(&control,"192.168.1.10",49152);
	ret = hl_open(&control, "/dev/ttyUSB0", 38400);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST turn.\r\n"));

	DMSG((STDOUT,"hl_turntable_turn \n"));
	hl_turntable_config(&control, 400, 0, 360);
	ret = hl_turntable_turn_async(&control, 100);
	e_assert(ret>0, ret);

	while (loop) {
//		ret = hl_turntable_check(&control);
//		DMSG(( STDOUT, "[%d]\t hl_turntable_check.\t%s\n",idx,strret(ret)));
//
//		//相机拍照
//		ret = hl_camera_take_photo(&control);
//		DMSG(( STDOUT, "[%d]\t hl_camera_take_photo.\t%s\n",idx,strret(ret)));
//
//		//亮红灯
//		ret = hl_led_red(&control);
//		DMSG(( STDOUT, "[%d]\t hl_led_red.\t%s\n",idx,strret(ret)));
//
//		//板子有bug，暂不测
//		//亮绿灯
//		ret = hl_led_green(&control);
//		e_check(ret>0);
//
//		//LED熄灭
//		ret = hl_led_off(&control);
//		DMSG(( STDOUT, "[%d]\t hl_led_off.\t%s\n",idx,strret(ret)));

        //获取温度
//		start_time = GetTickCount();
//		fvalue = hl_get_temperature(&control);
//		DMSG(( STDOUT, "%d\t", GetTickCount()-start_time));

		//获取状态
		start_time = GetTickCount();
		fvalue = hl_get_battery(&control);
		DMSG(( STDOUT, "%d\t", GetTickCount()-start_time));
//
//		//获取倾斜度
//		start_time = GetTickCount();
//		ret = hl_get_tilt(&control, &angle);
//		e_check(ret<=0);
//		DMSG(( STDOUT, "%d\t", GetTickCount()-start_time));
//
//		//获取当前水平转台的角度
//		start_time = GetTickCount();
//		fvalue = hl_turntable_get_angle(&control);
//		DMSG(( STDOUT, "%d\n", GetTickCount()-start_time));
	}

	hl_turntable_stop(&control);
	hl_close(&control);
	return 0;
}
#endif
