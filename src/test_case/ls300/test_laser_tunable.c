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
#if TEST_TUNABLE
#include <ls300/hd_laser_control.h>
#include <arch/hd_timer_api.h>
#include <signal.h>

laser_control_t control;

void sig_handler(int sig) {
	if (sig == SIGINT) {
		hl_turntable_stop(&control);

		DMSG((STDOUT,"LASER CONTROL TEST PASSED.\r\n"));
		hl_close(&control);
		exit(0);
	}
}
/*
 * 5000
 2500
 1250
 850
 700
 400
 250
 200
 150
 100
 50
 *
 * */
int main(int argc, char **argv) {
	int ret, step;
	e_float64 angle, speed_in, angle_in, speed;
	int time_last, time;

	signal(SIGINT, sig_handler);

	DMSG((STDOUT,"Trunale test start.\r\n"));

	if (argc == 3) {
		speed_in = atoi(argv[1]);
		angle_in = atof(argv[2]);
	} else {
		speed_in = 200;
		angle_in = 180;
	}

//ret = hl_open_socket(&control,"192.168.1.10",49152);
	ret = hl_open(&control, "/dev/ttyUSB0", 38400);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST turn.\r\n"));

	DMSG((STDOUT,"hl_turntable_turn \n"));
	hl_turntable_config(&control, speed_in, 0, angle_in);
//
//	ret = hl_turntable_start(&control);
//	e_assert(ret>0, ret);

	ret = hl_turntable_turn_async(&control, angle_in);
	e_assert(ret>0, ret);

	time_last = GetTickCount();
#if 1
	do {
		Delay(500);
		time = GetTickCount() - time_last;
		step = hl_turntable_get_step(&control);
		speed = (double) time / (step * 80);
		DMSG((STDOUT,"%d\t%d\t%f\n",time,step,speed));
	} while (step > 0);
#else
	do {
		Delay(500);
		time = GetTickCount() - time_last;
		angle = hl_turntable_get_angle(&control);
		speed = (double) time / (ANGLE_TO_STEP(angle) * 80);
		DMSG((STDOUT,"%d\t%f\t%f\n",time,angle,speed));
	}while (angle > 0);
#endif

	hl_turntable_stop(&control);
	hl_close(&control);
	return 0;
}
#endif
