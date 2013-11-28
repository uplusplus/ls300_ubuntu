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
/**/

#include "../hd_test_config.h"
#if TEST_SCAN_CONFIG
#include <ls300/hd_laser_sick.h>

int main() {
	int ret;
	laser_control_t control;
	sickld_t sick;
	laser_sick_t *ls;
	DMSG((STDOUT,"scan test config.\r\n"));

//	ret = hl_open(&control, "/dev/ttyUSB0", 38400); //(char*) "/dev/ttyUSB0", 38400);
//	if (e_failed(ret)) {
//		return 1;
//	}
//	ret = sld_create(&sick, "192.168.1.10", 49152);
//	if (e_failed(ret)) {
//		return 1;
//	}

	/*ret.add(new ScanConfig("Panoramic-Low", -45, 90, 0, 360,
	 Precision.PRECISION_LEVEL_LOW));
	 ret.add(new ScanConfig("Panoramic-Normal", -45, 90, 0, 360,
	 Precision.PRECISION_LEVEL_NORMAL));
	 ret.add(new ScanConfig("Panoramic-Middle", -45, 90, 0, 360,
	 Precision.PRECISION_LEVEL_MIDDLE));
	 ret.add(new ScanConfig("Panoramic-High", -45, 90, 0, 360,
	 Precision.PRECISION_LEVEL_HIGH));
	 new precision(10f, 0.5f, 50, 720, 720, 72),
	 new precision(7f, 0.375f, 100, 1008, 960, 144),
	 new precision(10f, 0.25f, 100, 1440, 1440, 144),
	 new precision(5f, 0.125f, 400, 2880, 2880, 576),
	 new precision(2.5f, 0.0625f, 2500, 9000, 5760, 3600), };
	 */
	ret = ls_init(&ls, (void*)1, (void*)1, NULL, NULL);

	ret = ls_phrase_config(ls, 50, 0, 360, 5, 0.5, 1, -45, 90);
	ret = ls_phrase_config(ls, 50, 0, 360, 7, 0.375, 1, -45, 90);
	ret = ls_phrase_config(ls, 100, 0, 360, 5, 0.25, 1, -45, 90);
	ret = ls_phrase_config(ls, 200, 0, 360, 5, 0.125, 1, -45, 90);
	ret = ls_phrase_config(ls, 850, 0, 360, 10, 0.0625, 4, -45, 90);

	e_assert(ret>0, ret);

	ls_uninit(ls);
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
