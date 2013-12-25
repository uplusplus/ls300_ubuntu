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

int main() {
	char ban[] = "	 _     ____ _____  ___   ___   __     ______    ___  \n"
			"	| |   / ___|___ / / _ \\ / _ \\  \\ \\   / /___ \\  / _ \\ \n"
			"	| |   \\___ \\ |_ \\| | | | | | |  \\ \\ / /  __) || | | |\n"
			"	| |___ ___) |__) | |_| | |_| |   \\ V /  / __/ | |_| |\n"
			"	|_____|____/____/ \\___/ \\___/     \\_/  |_____(_)___/ \n";
	printf("%s", ban);
	webserver_start("/sdcard/ls300/html/http/");
//	while (getchar() != 'q')
//		;
	webserver_loop();
	webserver_stop();
	DMSG((STDOUT,"Laser Server Test stop.\r\n"));
}

#endif
