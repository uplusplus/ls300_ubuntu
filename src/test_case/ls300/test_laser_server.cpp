/*!
 * \file test_laser_server.c
 * \brief laser scan test
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
#include <stdio.h>
#include <stdlib.h>
#include <comm/hd_utils.h>
#include <server/hd_webserver.h>

#if TEST_LASER_SERVER

int main(int argc, char **argv) {
	int ret;
	char ban[] =
			"\n"
			"	 _     ____ _____  ___   ___   __     ______    ___  \n"
			"	| |   / ___|___ / / _ \\ / _ \\  \\ \\   / /___ \\  / _ \\ \n"
			"	| |   \\___ \\ |_ \\| | | | | | |  \\ \\ / /  __) || | | |\n"
			"	| |___ ___) |__) | |_| | |_| |   \\ V /  / __/ | |_| |\n"
			"	|_____|____/____/ \\___/ \\___/     \\_/  |_____(_)___/ \n";
	printf("%s", ban);

	if (argc > 1) {
		ret = webserver_start(argv[1]);
	} else
		ret = webserver_start("/sdcard/ls300/html/http/");
	e_assert(ret > 0, -1);
//	while (getchar() != 'q')
//		;
	webserver_loop();
	webserver_stop();
	DMSG((STDOUT,"Laser Server Test stop.\r\n"));
}

#endif
