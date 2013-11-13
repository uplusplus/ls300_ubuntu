/*!
 * \file test_laser_scan.c
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

#if TEST_WEB

extern "C" int websocket_start(char *root_dir);
extern "C" int webhttp_start(char *root_dir);

int main(){
char ban[]=
"	 _     ____ _____  ___   ___   __     ______    ___  \n"
"	| |   / ___|___ / / _ \\ / _ \\  \\ \\   / /___ \\  / _ \\ \n"
"	| |   \\___ \\ |_ \\| | | | | | |  \\ \\ / /  __) || | | |\n"
"	| |___ ___) |__) | |_| | |_| |   \\ V /  / __/ | |_| |\n"
"	|_____|____/____/ \\___/ \\___/     \\_/  |_____(_)___/ \n";
	printf("%s",ban);

	display.w = 400;
	display.h = 400;
	display.buf = (e_uint8*)malloc(display.w*display.h);
	display.hash = 1;

	websocket_start("./html/websocket/");
	webhttp_start("./html/");

	getchar();

	return 0;
}

#endif
