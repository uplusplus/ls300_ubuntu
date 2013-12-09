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
#if TEST_SCAN
#include <ls300/hd_laser_scan.h>
#include <signal.h>

scan_job_t* job;

void sig_handler(int sig) {
	if (sig == SIGINT) {
		sj_cancel(job);
		getchar();
		sj_destroy(job);
		exit(0);
	}
}
extern "C" int webserver_start(char *root_dir);

int main() {
	signal(SIGINT, sig_handler);
	int ret;
	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_create(&job, "/dev/ttyUSB0", 38400, "192.168.1.10", 49152);
	e_assert(ret>0, ret);

	ret = sj_check_devices(job);
	e_assert(ret>0, ret);

	webserver_start("./html/http/");

	DMSG((STDOUT,"LASER scan TEST config.\r\n"));

#if LINUX
	sj_set_data_dir(job, ".", ".");
#else
	sj_set_data_dir(job, "/sdcard/ls300/data/point_cloud",
			"/sdcard/ls300/data/image", "/data/data/com.hdsy.ls300/files");
#endif
	/*
	 time(s)		resolotion		speed_h		speed_v		width		height
	 1800				0.0625		1250			7			2884		2164
	 1224							850				10

	 288			0.125			200				5				1441		1081
	 144			0.25			100				5				721			541
	 72			0.375			50				7				505			361
	 72			0.5				50				5				361			271
	 */

//	ret =  sj_config(job, 50, 0, 360, 5, 0.5, -45, 90);
//	ret =  sj_config(job, 50, 0, 360, 7, 0.375, -45, 90);
//	ret =  sj_config(job, 100, 0, 360, 5, 0.25, -45, 90);
//	ret =  sj_config(job, 850, 0, 360, 10, 0.0625, -45, 90);
	ret = sj_config(job, 1250, 0, 360, 7, 0.0625, -45, 90);

//	ret =  sj_config(job, 850, 160, 200, 7, 0.0625, -45, 90);
//	ret =  sj_config(job, 200, 0, 360, 7, 0.0625, -45, 90);

	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER scan TEST scan point.\r\n"));
	ret = sj_scan_point(job);
	e_assert(ret>0, ret);

	getchar();
	DMSG((STDOUT,"LASER scan TEST scan photo.\r\n"));
	ret = sj_scan_photo(job);
	e_assert(ret>0, ret);

	getchar();

	DMSG((STDOUT,"LASER scan TEST PASSED.\r\n"));
	sj_destroy(job);
}

#if 0

create_job
set_callbacks(on_done,on_cancel);
start_job
cancel_job
stop_job

int main()
{
	signal(SIGINT, sig_handler);
	int ret;
	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_create(&job); //创建，初始化变量，申请内存
	e_assert(ret>0, ret);

	//配置全局参数
	//数据存取目录
	//连接参数
	ret = sj_set_globe_config();
	e_assert(ret>0, ret);

	ret = sj_connect(&job);//连接到控制板，连接到扫描仪
	e_assert(ret>0, ret);

	ret = sj_make_job_sick(job, 50, 0, 360, 5, 0.25, -45, 90);
	e_assert(ret>0, ret);
	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_start_job_sick(job);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_wait(job);
	e_assert(ret>0, ret);

	ret = sj_make_job_photo(job, 50, 0, 360, 5, 0.25, -45, 90);
	e_assert(ret>0, ret);
	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_start_job_photo(job);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER scan TEST start.\r\n"));
	ret = sj_wait(job);
	e_assert(ret>0, ret);

	DMSG((STDOUT,"LASER scan TEST PASSED.\r\n"));
	sj_destroy(job);
}
#endif
#endif
