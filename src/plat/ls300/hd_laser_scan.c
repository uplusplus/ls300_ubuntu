/*!
 * \file hd_laser_scan.c
 * \brief	扫描总控
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser base
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include <arch/hd_timer_api.h>
#include <sickld/sickld.h>
#include <ls300/hd_laser_scan.h>
#include <ls300/hd_laser_control.h>
#include <ls300/hd_laser_photo.h>
#include <ls300/hd_laser_sick.h>

struct scan_job_t {
	//给子模块用的，统一管理
	sickld_t *sick;
	laser_control_t* control;

	//子模块
	laser_photo_t* photo_work;
	laser_sick_t* sick_work;

	//配置参数记录
	//垂直方向
	e_uint32 speed_v; // hz
	e_float64 resolution_v;
	e_float64 start_angle_v; //垂直起始角度，这里暂时固定分为两个扇区
	e_float64 end_angle_v; //垂直终止角度
	//水平方向
	e_uint32 speed_h; // s
	e_float64 start_angle_h; //水平起始角度
	e_float64 end_angle_h; //水平终止角度

	//文件读写
	char data_dir[MAX_PATH_LEN];
	char gray_dir[MAX_PATH_LEN];
	char files_dir[MAX_PATH_LEN];

	e_int32 state;
};

enum {
	STATE_NONE = 0, STATE_IDLE = 1, STATE_WORK = 2, STATE_CANCEL = 3
};

//State change
// 			create 		work 		done
// NONE		idle	  	work 		idle

/**
 *\brief 创建扫描控制模块
 *\param scan_job_t* 定义了扫描任务。
 *\retval E_OK 表示成功。
 */
e_int32 sj_create(scan_job_t** sj_ret, char*dev, int baudrate, char* ip,
		int port) {
	e_int32 ret;
	scan_job_t* sj;
	sj = (scan_job_t*) malloc(sizeof(struct scan_job_t));
	e_assert(sj, E_ERROR_BAD_ALLOCATE);
	memset(sj, 0, sizeof(scan_job_t));
	strcpy(sj->data_dir, "./");
	strcpy(sj->gray_dir, "./");

	sj->control = (laser_control_t*) malloc(sizeof(struct laser_control_t));
	e_assert(sj->control, E_ERROR_BAD_ALLOCATE);
	memset(sj->control, 0, sizeof(laser_control_t));
	if (dev == NULL) {
		dev = (char*) "/dev/ttyUSB0";
		baudrate = 38400;
	}
	ret = hl_open(sj->control, dev, baudrate); //(char*) "/dev/ttyUSB0", 38400);
	if (e_failed(ret))
	{
		hl_close(sj->control);
		free(sj->control);
		goto FAILED;
	}

	ret = sld_create(&sj->sick, ip, port);
	if (e_failed(ret)) {
		DMSG((STDOUT,"connect to sick scanner failed!\nplease check that you'v set the correct IP address.\n"));
		hl_close(sj->control);
		free(sj->control);
		sld_release(&sj->sick);
		goto FAILED;
	}

	sj->state = STATE_IDLE;
	(*sj_ret) = sj;
	return E_OK;

	FAILED: (*sj_ret) = NULL;
	free(sj);
	return ret;
}

e_int32 sj_check_devices(scan_job_t* sj) {
	int ret;
	e_assert(sj&&sj->state==STATE_IDLE, E_ERROR_INVALID_HANDLER);
	ret = hl_turntable_check(sj->control);
	e_assert(ret>0, ret);
	ret = sld_initialize(sj->sick);
	e_assert(ret>0, ret);
	ret = sld_get_status(sj->sick);
	e_assert(ret>0, ret);
	return E_OK;
}

/**
 *\brief 与扫描仪断开连接   
 *\param scan_job_t* 定义了扫描任务。
 *\retval E_OK 表示成功。
 */
e_int32 sj_destroy(scan_job_t* sj) {
	e_assert(sj&&sj->state==STATE_IDLE, E_ERROR_INVALID_HANDLER);
	if (sj->photo_work)
		lp_uninit(sj->photo_work);
	if (sj->sick_work)
		ls_uninit(sj->sick_work);

	hl_close(sj->control);
	sld_release(&sj->sick);
	sj->state = 0;
	free(sj->control);
	free(sj);
	return E_OK;
}

static e_int32 sj_assign_boundaries(scan_job_t* sj,
		const e_float64 start_angle_v, const e_float64 end_angle_v,
		const e_float64 resolution_v) {

	sj->start_angle_v = start_angle_v;
	sj->end_angle_v = end_angle_v;

	while (fmod(90 + sj->start_angle_v, resolution_v) != 0) {
		sj->start_angle_v += 0.125;
	}
	while (fmod(90 + sj->end_angle_v, resolution_v) != 0) {
		sj->end_angle_v += 0.125;
	}

	e_assert(
			sj->start_angle_v<=90 && sj->start_angle_v>=-45 &&sj->end_angle_v<=90 && sj->end_angle_v>=-45,
			E_ERROR);

	return E_OK;
}

e_int32 sj_config(scan_job_t* sj, e_uint32 speed_h,
		const e_float64 start_angle_h, const e_float64 end_angle_h,
		e_uint32 speed_v, e_float64 resolution_v,
		const e_float64 active_sector_start_angle,
		const e_float64 active_sector_stop_angle) {
	int ret;
	e_assert(sj&&sj->state==STATE_IDLE, E_ERROR_INVALID_HANDLER);
	e_assert(
			active_sector_start_angle<=90 && active_sector_start_angle>=-45 && active_sector_stop_angle<=90 && active_sector_stop_angle>=-45 && active_sector_stop_angle > active_sector_start_angle,
			E_ERROR_INVALID_PARAMETER);
	//记录配置
	sj->start_angle_h = start_angle_h;
	sj->end_angle_h = end_angle_h;
	sj->speed_h = speed_h;

	ret = sj_assign_boundaries(sj, active_sector_start_angle,
			active_sector_stop_angle, resolution_v);
	e_assert(ret>0, ret);
	sj->speed_v = speed_v;
	sj->resolution_v = resolution_v;

	return E_OK;
}

static e_int32 on_status_change(void* thiz, int state) {
	scan_job_t* sj = (scan_job_t*) thiz;
	if (state == STATE_IDLE) {
		if (sj->photo_work)
			lp_uninit(sj->photo_work);
		if (sj->sick_work)
			ls_uninit(sj->sick_work);
		sj->photo_work = NULL;
		sj->sick_work = NULL;
		sj->state = STATE_IDLE;
		DMSG((STDOUT, "on_status_change:scan job routine stopped.\r\n"));
	} else if (state == STATE_WORK) {
//		sj->state = STATE_WORK;
		DMSG((STDOUT, "on_status_change:scan job routine started.\r\n"));
	}
}

e_int32 sj_scan_photo(scan_job_t* sj) {
	int ret;
	e_assert(sj&&sj->state==STATE_IDLE, E_ERROR_INVALID_HANDLER);
	DMSG((STDOUT, "scan job do photo scan...\r\n"));
	sj->state = STATE_WORK;

	ret = lp_init(&sj->photo_work, sj->control, on_status_change, sj);
	e_assert(ret>0, ret);
	ret = lp_scan(sj->photo_work, sj->start_angle_h, sj->end_angle_h);
	e_assert(ret>0, ret);
	return E_OK;
}

e_int32 sj_scan_point(scan_job_t* sj) {
	int ret;
	e_assert(sj&&sj->state==STATE_IDLE, E_ERROR_INVALID_HANDLER);
	DMSG((STDOUT, "scan job do point scan...\r\n"));
	sj->state = STATE_WORK;

	ret = ls_init(&sj->sick_work, sj->control, sj->sick, on_status_change, sj);
	e_assert(ret>0, ret);
	ret = ls_scan(sj->sick_work, sj->data_dir, sj->gray_dir, sj->files_dir,
			sj->speed_h, sj->start_angle_h, sj->end_angle_h, sj->speed_v,
			sj->resolution_v, sj->start_angle_v, sj->end_angle_v);
	e_assert(ret>0, ret);
	return E_OK;
}

/**
 *\brief 停止扫描 
 *\param scan_job_t* 定义了扫描任务。
 *\retval E_OK 表示成功。
 */
e_int32 sj_cancel(scan_job_t* sj) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	int i = 0;
	DMSG((STDOUT, "scan job routine try stop...\r\n"));
	//check
	if (sj->photo_work)
		lp_cancel(sj->photo_work);
	if (sj->sick_work)
		ls_cancel(sj->sick_work);
	return E_OK;
}

/**
 *\brief 设置点云数据存储目录,灰度图存储目录。  
 *\param scan_job_t* 定义了扫描任务。
 *\param ptDir 定义了点云数据存储目录。 
 *\param grayDir 定义了灰度图存储目录。
 *\retval E_OK 表示成功。
 */
e_int32 sj_set_data_dir(scan_job_t* sj, char* ptDir, char *grayDir,
		char *filesDir) {
	e_assert(sj&&ptDir&&grayDir, E_ERROR_INVALID_HANDLER);
	strncpy(sj->data_dir, ptDir, sizeof(sj->data_dir));
	strncpy(sj->gray_dir, grayDir, sizeof(sj->gray_dir));
	strncpy(sj->files_dir, filesDir, sizeof(sj->files_dir));
	return E_OK;
}

e_int32 sj_get_state(scan_job_t* sj) {
	e_assert(sj, E_ERROR_INVALID_HANDLER);
	return sj->state;
}

//获取当前水平转台的角度
e_float64 sj_get_angle(scan_job_t* sj) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_turntable_get_angle(sj->control);
}

//获取温度
e_float64 sj_get_temperature(scan_job_t* sj) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_get_temperature(sj->control);
}

//获取倾斜度
e_int32 sj_get_tilt(scan_job_t* sj, angle_t* angle) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_get_tilt(sj->control, angle);
}

//获取状态
e_float64 sj_get_battery(scan_job_t* sj) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_get_battery(sj->control);
}

e_int32 sj_led(scan_job_t* sj, int status) {
	int ret;
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	switch (status) {
	case 0: //LED熄灭
		ret = hl_led_off(sj->control);
		break;
	case 1: //亮绿灯
		ret = hl_led_green(sj->control);
		break;
	case 2: //亮红灯
		ret = hl_led_red(sj->control);
		break;
	default:
		ret = E_ERROR_INVALID_PARAMETER;
	}
	return ret;
}

//搜索零点
e_int32 sj_search_zero(scan_job_t* sj) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_search_zero(sj->control);
}

//硬件信息
e_int32 sj_get_info(scan_job_t* sj, e_uint32 idx, e_uint8* buffer,
		e_int32 blen) {
	e_assert(sj&&sj->state, E_ERROR_INVALID_HANDLER);
	return hl_get_info(sj->control, idx, buffer, blen);
}
