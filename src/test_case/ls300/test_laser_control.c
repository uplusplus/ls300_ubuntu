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
#if TEST_CONTROL
#include <ls300/hd_laser_control.h>
#include <arch/hd_timer_api.h>

static volatile int loop = 1;

laser_control_t control;

char *strret(int ret){
	if(ret>0) return "OK";
	else return "FAILED";
}


static void test_loop(void* data) {
	int ret, idx;
	e_float64 fvalue;
	angle_t angle;
	idx = (int) data;

	DMSG((STDOUT,"Thread start~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%d\n",idx));
	while (loop) {

		ret = hl_turntable_check(&control);
		DMSG(( STDOUT, "[%d]\t hl_turntable_check.\t%s\n",idx,strret(ret)));

		//相机拍照
		ret = hl_camera_take_photo(&control);
		DMSG(( STDOUT, "[%d]\t hl_camera_take_photo.\t%s\n",idx,strret(ret)));

		//获取温度
		fvalue = hl_get_temperature(&control);
		DMSG(( STDOUT, "[%d]\thl_get_temperature :\t %.3f\n",idx,fvalue));

		//获取倾斜度
		ret = hl_get_tilt(&control, &angle);
		e_check(ret<=0);
		DMSG(( STDOUT, "[%d]\thl_get_tilt :\t %.3f %.3f\n",idx, angle.dX, angle.dY));
//
//		//获取状态
		fvalue = hl_get_battery(&control);
		DMSG(( STDOUT, "[%d]\thl_get_battery :\t %.3f\n",idx,fvalue));
		//亮红灯
		ret = hl_led_red(&control);
		DMSG(( STDOUT, "[%d]\t hl_led_red.\t%s\n",idx,strret(ret)));

//板子有bug，暂不测
//		//亮绿灯
//		ret = hl_led_green(&control);
//		e_check(ret>0);

		//LED熄灭
		ret = hl_led_off(&control);
		DMSG(( STDOUT, "[%d]\t hl_led_off.\t%s\n",idx,strret(ret)));
//
		//获取当前水平转台的角度
		fvalue = hl_turntable_get_angle(&control);
		DMSG(( STDOUT, "[%d]\t hl_turntable_get_angle :\t %.3f\n",idx,fvalue));
	}
}

#define TN 2

int main() {
	int ret, i;

	ethread_t* threads[TN] = { 0 };

	DMSG((STDOUT,"LASER CONTROL TEST start.\r\n"));

//ret = hl_open_socket(&control,"192.168.1.10",49152);
	ret = hl_open(&control, "/dev/ttyUSB0", 38400);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST turn.\r\n"));

	DMSG((STDOUT,"hl_turntable_turn \n"));
	hl_turntable_config(&control, 400, 0, 360);
	ret = hl_turntable_turn_async(&control, 100);
	e_assert(ret>0, ret);

	for (i = 0; i < TN; i++) {
		ret = createthread("LASER_CONTROL", (thread_func) &test_loop, (void*) i,
				NULL, &threads[i]);
		if (ret <= 0) {
			DMSG((STDOUT, "createhread failed!\r\n"));
			break;
		}
		ret = resumethread(threads[i]);
		if (ret <= 0) {
			DMSG((STDOUT, "write resumethread failed!\r\n"));
			break;
		}
	}

	getchar();

	loop = 0;

	for (i = 0; i < TN; i++) {
		killthread(threads[i]);
	}

	ret = hl_turntable_stop(&control);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST PASSED.\r\n"));
	hl_close(&control);

	Delay(100);
	return 0;
}
#endif
