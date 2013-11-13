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

int main() {
	int ret;
	e_float64 angle, langle;
	laser_control_t control;

	DMSG((STDOUT,"LASER CONTROL TEST start.\r\n"));

//ret = hl_open_socket(&control,"192.168.1.10",49152);
	ret = hl_open(&control, "/dev/ttyUSB0", 38400);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST turn.\r\n"));

	DMSG((STDOUT,"hl_turntable_turn \n"));
	hl_turntable_config(&control, 50, 0, 180);

	hl_turntable_prepare(&control, 20);

	ret = hl_turntable_start(&control);
	e_assert(ret>0, ret);

	do {
		langle = angle;
		angle = hl_turntable_get_angle(&control);
	} while (angle <= 0);

	Delay(1000);

	DMSG((STDOUT,"START ANGLE:%f\n",langle));

	do {
		langle = angle;
		angle = hl_turntable_get_angle(&control);
	} while (angle > 0);

	DMSG((STDOUT,"EXIT ANGLE:%f\n",langle));

	ret = hl_turntable_stop(&control);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER CONTROL TEST PASSED.\r\n"));
	hl_close(&control);

	Delay(100);
	return 0;
}
#endif
