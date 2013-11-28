/*!
 * \file hd_laser_sick.c
 * \brief	sick工作者
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser base
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */
#include <pthread.h>
#include <ls300/hd_laser_sick.h>
#include <arch/hd_timer_api.h>
#include <ls300/hd_laser_scan.h>
#include <ls300/hd_scan_data_pool.h>
#include <ls300/hd_data_manager.h>

/*结构体定义*/
struct laser_sick_t {
	sickld_t *sick;
	laser_control_t* control;
	data_manager_t* writer;
	scan_pool_t pool;
	ethread_t *thread_read;
	ethread_t *thread_write;

	//配置参数记录
	//垂直方向
	e_uint32 height;
	e_uint32 speed_v; // hz
	e_uint32 interlace_v;
	e_float64 resolution_v;
	e_float64 start_angle_v[2]; //垂直起始角度，这里暂时固定分为两个扇区
	e_float64 end_angle_v[2]; //垂直终止角度
	//水平方向
	e_uint32 width;
	e_uint32 speed_h; // s
	e_float64 start_angle_h; //水平起始角度
	e_float64 end_angle_h; //水平终止角度
	struct {
		e_uint8 left;
		e_uint8 right;
	} active_sectors;

	//外部指针，引用的。
	point_t *points_polar;
	point_t *points_gray;
	scan_data_t * scan_data_buf;

	//临时记录
	e_float64 angle_dif_per_cloumn;
	e_float64 angle_dif_per_degree;
	e_uint32 slip_idx;
	e_uint32 slip_tick;
	e_float64 pre_scan_angle;

	e_float32 h_w; //图像真实高宽比

	e_int32 (*on_status_change)(void*, int);
	void* ctx;
	ethread_t *thread_work;
	int state;
};

//不考虑上层的错误调用
static enum {
	STATE_NONE = 0,
	STATE_IDLE = 1,
	STATE_WORK = 2,
	STATE_DONE = 3,
	STATE_CANCEL = 4
};

////////////////////////////////////////////////////////////////////////////////
//local definations
#define PRE_SCAN_WAIT_TIME  5000000 //us
//#define HACK_INVALID_DATA 1
//#define HACK_PNT_ANGLE_H 1
//#define HACK_SLIP_ANGLE 1
//#define HACK_SLIP_IDX 1
#define HACK_SICK_BUGS 1

////////////////////////////////////////////////////////////////////////////////
//local functions
#define should_continue (ls->state==STATE_WORK)

static e_int32 inter_ls_scan(laser_sick_t *ls);
static e_int32 thread_scan_func(laser_sick_t *ls);
static e_int32 filter_data(laser_sick_t *ls, scan_data_t * pdata);
static void write_pool_data_routine(laser_sick_t *ls);
static void read_pool_data_routine(laser_sick_t *ls);
static void print_config(laser_sick_t* ls);

static int f2r(int speed);
static int r2f(int speed);

e_int32 ls_init(laser_sick_t **lsp, laser_control_t *lc, sickld_t *sick,
		e_int32 (*on_status_change)(void*, int), void* ctx) {
	laser_sick_t *ls;
	e_assert(lc&&sick, E_ERROR_INVALID_HANDLER);
	ls = (laser_sick_t *) calloc(1, sizeof(struct laser_sick_t));
	e_assert(ls, E_ERROR_BAD_ALLOCATE);
	pool_init(&ls->pool);
	ls->control = lc;
	ls->sick = sick;
	ls->on_status_change = on_status_change;
	ls->ctx = ctx;
	ls->state = STATE_IDLE;
	(*lsp) = ls;
	return E_OK;
}

e_int32 ls_uninit(laser_sick_t *ls) {
	e_assert(ls&&ls->state, E_ERROR_INVALID_HANDLER);
	if (ls->state == STATE_WORK) {
		ls_cancel(ls);
		Delay(500);
	}
	if (ls->thread_work)
		killthread(ls->thread_work);
	if (ls->thread_read)
		killthread(ls->thread_read);
	if (ls->thread_write)
		killthread(ls->thread_write);
	pool_destroy(&ls->pool);
	memset(ls, 0, sizeof(laser_sick_t));
	free(ls);
	return E_OK;
}

e_int32 ls_scan(laser_sick_t *ls, char* ptDir, char *grayDir, char *files_dir,
		e_uint32 speed_h_delay, const e_float64 start_angle_h,
		const e_float64 end_angle_h, e_uint32 speed_v_hz,
		e_float64 resolution_v, const e_uint32 interlace_v,
		const e_float64 start_angle_v, const e_float64 end_angle_v) {
	e_int32 ret;
	e_assert(ls && ls->state == STATE_IDLE, E_ERROR_INVALID_STATUS);

	ls->state = STATE_WORK;

	ls->on_status_change(ls->ctx, ls->state);

	ret = ls_phrase_config(ls, speed_h_delay, start_angle_h, end_angle_h,
			speed_v_hz, resolution_v, interlace_v, start_angle_v, end_angle_v);
	if (ret <= 0) {
		DMSG((STDOUT, "ls_phrase_config failed!\r\n"));
		ls->state = STATE_IDLE;
		return E_ERROR;
	}

	//创建输出端子
	ls->writer = dm_alloc(ptDir, grayDir, files_dir, ls->width, ls->height,
			ls->h_w,
			ls->active_sectors.right && ls->active_sectors.left ?
					E_DWRITE : E_WRITE);
	if (ls->writer == 0) {
		DMSG((STDOUT, "dm_alloc failed!\r\n"));
		ls->state = STATE_IDLE;
		return E_ERROR;
	}
	ret = dm_alloc_buffer(ls->writer, DATA_BLOCK_TYPE_COLUMN, &ls->points_polar,
			&ls->points_gray);
	if (e_failed( ret )) {
		DMSG((STDOUT, "dm_alloc_buffer failed!\r\n"));
		ls->state = STATE_IDLE;
		return E_ERROR;
	}
	ls->slip_idx = 0;
	ls->slip_tick = 0;

	DMSG((STDOUT, "scan job routine starting read routine...\r\n"));
	//check priv
	if (ls->thread_work)
		killthread(ls->thread_work);
	ret = createthread("point scan  scan thread",
			(thread_func) &thread_scan_func, ls, NULL, &ls->thread_work);
	if (ret <= 0) {
		DMSG((STDOUT, "create point scan thread failed!\r\n"));
		ls->state = STATE_IDLE;
		return E_ERROR;
	}
	ret = resumethread(ls->thread_work);
	if (ret <= 0) {
		DMSG((STDOUT, "resume point scan  thread failed!\r\n"));
		if (ls->thread_work)
			killthread(ls->thread_work);
		ls->state = STATE_IDLE;
		return E_ERROR;
	}
	DMSG((STDOUT, "point scan job routine start successful.\r\n"));
	return E_OK;
}

e_int32 ls_cancel(laser_sick_t *ls) {
	e_int32 ret;
	ls->state = STATE_CANCEL;
//	//停止转台
//	ret = hl_turntable_stop(ls->control);
//	e_check(ret<=0);
	pool_cancle(&ls->pool);
//	//停止扫描仪
//	ret = sld_uninitialize(ls->sick);
//	e_assert(ret>0, ret);
	return E_OK;
}

static e_int32 thread_scan_func(laser_sick_t *ls) {
	int ret;
	void* result;
	ret = inter_ls_scan(ls);

	if (ret > 0) {
		DMSG((STDOUT,"sick main thread try wait sub thread....\n"));
		//等待子线程
		pthread_join(ls->thread_write->thread_id, &result);
		pthread_join(ls->thread_read->thread_id, &result);
	}

	DMSG(
			(STDOUT,"sick main thread all sub thread quit,try stop devices....\n"));

	//停止转台
	ret = hl_turntable_stop(ls->control);
	e_check(ret<=0);
	//停止扫描仪
	ret = sld_uninitialize(ls->sick);
	e_check(ret<=0);

	//等待数据刷新完毕
	Delay(1000);

	DMSG((STDOUT,"sick main thread save files.... state:%d\n",ls->state));
//	dm_free(ls->writer, ls->state == STATE_DONE); //写入文件
	dm_free(ls->writer, true); //写入文件
	ls->state = STATE_IDLE;
	DMSG((STDOUT,"sick main thread sounds on_status_change....\n"));
	ls->on_status_change(ls->ctx, ls->state);
	return ret;
}

static e_int32 inter_ls_scan(laser_sick_t *ls) {
	e_int32 ret;

	//提交配置到控制板
//	ret = hl_turntable_config(ls->control, ls->speed_h, ls->start_angle_h,
//			ls->end_angle_h + ls->pre_scan_angle);
	ret = hl_turntable_config(ls->control, r2f(ls->speed_h), ls->start_angle_h,
			ls->end_angle_h + ls->pre_scan_angle);
	e_assert(ret>0, ret);

	//开始真正数据采集前的准备工作,快速转动转台到0点
//	ret = hl_search_zero(ls->control);
//	e_assert(ret>0 && should_continue, ret);
//	Delay(100);

	//开始真正数据采集前的准备工作,快速转动转台到需要的启始位置
	ret = hl_turntable_prepare(ls->control, ls->pre_scan_angle);
	e_assert(ret>0 && should_continue, ret);

	//初始化sick扫描设备
	ret = sld_initialize(ls->sick);
	e_assert(ret>0 && should_continue, ret);

	ret = sld_set_global_params_and_scan_areas_interlace(ls->sick, ls->speed_v,
			ls->resolution_v, ls->interlace_v, ls->start_angle_v,
			ls->end_angle_v,
			ls->active_sectors.left + ls->active_sectors.right);
	e_assert(ret>0 && should_continue, ret);

	/*打印当前扫描参数配置（经优化后的，生成的有效配置）*/
	ret = sld_print_sector_config(ls->sick);
	e_assert(ret>0 && should_continue, ret);

	//让转台开始正常转动
	ret = hl_turntable_start(ls->control);
	e_assert(ret>0 && should_continue, ret);

	DMSG((STDOUT, "scan job routine starting write routine...\r\n"));
	//启动sick数据读取线程
	ret = createthread("hd_scan_job_write",
			(thread_func) &write_pool_data_routine, ls, NULL,
			&ls->thread_write);
	e_assert(ret>0 && should_continue, ret);

	ret = resumethread(ls->thread_write);
	e_assert(ret>0 && should_continue, ret);

	DMSG((STDOUT, "scan job routine starting read routine...\r\n"));
	//启动数据回写线程
	ret = createthread("hd_scan_job_read",
			(thread_func) &read_pool_data_routine, ls, NULL, &ls->thread_read);
	e_assert(ret>0 && should_continue, ret);

	ret = resumethread(ls->thread_read);
	e_assert(ret>0 && should_continue, ret);
	DMSG((STDOUT, "scan job routine start done.\r\n"));

	return ret;
}

/**
 *\brief 设置扫描区域
 *\param scan_job 定义了扫描任务。
 *\param speed_h 定义了水平速度。
 *\param start_angle_h 定义了水平开始角度。
 *\param end_angle_h 定义了水平结束角度。
 *\param speed_v 定义了垂直速度。
 *\param resolution_v 定义了垂直分辨率。
 *\param active_sector_start_angles 定义了扫描区域开始角度。
 *\param active_sector_stop_angles 定义了扫描区域结束角度。
 *\retval E_OK 表示成功。
 */
e_int32 ls_phrase_config(laser_sick_t *ls, e_uint32 speed_h,
		const e_float64 start_angle_h, const e_float64 end_angle_h,
		e_uint32 speed_v, e_float64 resolution_v, e_uint32 interlace_v,
		const e_float64 active_sector_start_angle,
		const e_float64 active_sector_stop_angle) {

	e_assert(ls, E_ERROR_INVALID_HANDLER);

	e_assert(
			active_sector_start_angle<=90 && active_sector_start_angle>=-45 && active_sector_stop_angle<=90 && active_sector_stop_angle>=-45 && active_sector_stop_angle > active_sector_start_angle,
			E_ERROR_INVALID_PARAMETER);

	ls->start_angle_v[0] = active_sector_start_angle + 90;
	ls->end_angle_v[0] = active_sector_stop_angle + 90;

	//记录配置
	ls->start_angle_h = start_angle_h;
	ls->end_angle_h = end_angle_h;
	ls->speed_v = speed_v;
	ls->resolution_v = resolution_v;
	ls->interlace_v = interlace_v;
	ls->speed_h = f2r(speed_h);

	//挖个坑,先把第二个角度记下
	ls->start_angle_v[1] = 360 - ls->end_angle_v[0];
	ls->end_angle_v[1] = 360 - ls->start_angle_v[0];

	if (ls->end_angle_v[0] >= 180) { //限制了,垂直角度为 -45~+90 度之间
		ls->end_angle_v[0] = 180; //中间点放到前面
		ls->start_angle_v[1] = 180 + ls->resolution_v * ls->interlace_v;
	}

	//只会用到0号
	if (ls->start_angle_h < 180 && ls->end_angle_h <= 180) //都在左边
			{
		//都在左边时，此种情况下，控制板的水平角度不用做换算，保持不变
		ls->active_sectors.left = 1;
		ls->active_sectors.right = 0;
	}
	//只用到1号
	else if (ls->start_angle_h >= 180 && ls->end_angle_h > 180) //都在右边
			{
		ls->start_angle_v[0] = ls->start_angle_v[1];
		ls->end_angle_v[0] = ls->end_angle_v[1];
		ls->start_angle_h = ls->start_angle_h - 180;
		ls->end_angle_h = ls->end_angle_h - 180;
		ls->active_sectors.left = 0;
		ls->active_sectors.right = 1;
	}
	//对于超过180，直接全景
	else if (ls->end_angle_h - ls->start_angle_h > 180) {
		ls->end_angle_h = ls->start_angle_h + 180.0;
		ls->active_sectors.left = 1;
		ls->active_sectors.right = 1;
	}
	//对于小于180，可以在一边
	else if (ls->start_angle_h < 180 && ls->end_angle_h > 180) {
//		ls->end_angle_h = ls->start_angle_h + 180.0;
		ls->active_sectors.left = 1;
//		ls->active_sectors.right = 1;
	} else {
		DMSG((STDOUT,"cal_h_angle EXTRAM FAILT!\r\n"));
		return E_ERROR;
	}

	ls->height = (ls->end_angle_v[0] - ls->start_angle_v[0]) / ls->resolution_v
			+ ls->interlace_v;
	ls->width = ANGLE_TO_STEP( ls->end_angle_h - ls->start_angle_h )
			* PULSE_SPEED_TO_STEP_TIME(ls->speed_h) * ls->speed_v / 1E6
			/ ls->interlace_v + 1;
	ls->pre_scan_angle =
			STEP_TO_ANGLE(PRE_SCAN_WAIT_TIME / PULSE_SPEED_TO_STEP_TIME(ls->speed_h));
	ls->angle_dif_per_cloumn =
			STEP_TO_ANGLE( (1E6/ls->speed_v) / PULSE_SPEED_TO_STEP_TIME(ls->speed_h));
	ls->angle_dif_per_degree = ls->angle_dif_per_cloumn / 360;

	ls->h_w = (ls->end_angle_v[0] - ls->start_angle_v[0])
			/ ((ls->end_angle_h - ls->start_angle_h)
					* (ls->active_sectors.right + ls->active_sectors.left));

	print_config(ls);

	return E_OK;
}

static void print_sdata(laser_sick_t *ls, scan_data_t *sdata) {
	int i, idx = 0, dof;

	DMSG(
			(STDOUT,"{profile_number=%u, layer_num=%u}\n",sdata->profile_number,sdata->layer_num));
	if (ls->active_sectors.left) {
		DMSG((STDOUT,"[sector=%d]\n",idx));
		dof = sdata->sector_data_offsets[idx];
		for (i = 0; i < sdata->num_measurements[idx]; i++) {
			DMSG(
					(STDOUT,"\t[%d] %f, %f, %u]\n",i, sdata->range_measurements[dof+i],sdata->angle_measurements[dof+i],(unsigned int)sdata->echo_measurements[dof+i]));
		}
		idx++;
	}
	if (ls->active_sectors.right) {
		DMSG((STDOUT,"[sector=%d]\n",idx));
		dof = sdata->sector_data_offsets[idx];
		for (i = 0; i < sdata->num_measurements[idx]; i++) {
			DMSG(
					(STDOUT,"\t[%d] %f, %f, %u]\n",i, sdata->range_measurements[dof+i],sdata->angle_measurements[dof+i],(unsigned int)sdata->echo_measurements[dof+i]));
		}
		idx++;
	}
}

static void write_pool_data_routine(laser_sick_t *ls) {
	e_int32 ret, first_time = 1, delay, layer_num, profile_number;
	e_float64 angle_dif, started_angle, before_angle; //记录起始位置，保证180度是对齐的
	scan_data_t sdata = { 0 };

	DMSG((STDOUT,"scan job:write_data_routine start.\r\n"));
	angle_dif = ls->angle_dif_per_cloumn;
	delay = ANGLE_TO_STEP(angle_dif) * PULSE_SPEED_TO_STEP_TIME(ls->speed_h)
			/ 1e3;
	before_angle =
			STEP_TO_ANGLE((630000.0/PULSE_SPEED_TO_STEP_TIME(ls->speed_h)));

	ret = sld_set_sensor_mode_to_rotate(ls->sick);
	e_assert(ret>0);

	DMSG((STDOUT,"control trun to start angle...\n"));
//等待扫描仪就位,会有准备工作开始
	do {
//		DMSG(
//				(STDOUT,"sld_get_measurements sick is not in target position: delay and retry. angle_dif_per_cloumn=%f\r\n",angle_dif));
		sdata.h_angle = hl_turntable_get_angle(ls->control) - ls->pre_scan_angle
				+ ls->start_angle_h;
		DMSG(
				(STDOUT,"control trun to start angle:  %f %f\n",sdata.h_angle,before_angle));
//		ret = sld_flush(ls->sick);
		if (sdata.h_angle > ls->start_angle_h - before_angle) {
//			if (ret > 0) {
//				DMSG((STDOUT,"sld_get_measurements sick is in target position!\r\n Start to get data.\r\n"));
//				DMSG((STDOUT,"\rWRITE TO FILE: %f end: %f\n",sdata.h_angle,ls->end_angle_h));
//				pool_write(&ls->pool, &sdata);
//			}
//			started_angle = sdata.h_angle;
			break;
		}
		Delay(delay);
	} while (ls->state == STATE_WORK);

	DMSG((STDOUT,"control now on start angle.start to get data...\n"));

	ret = sld_set_sensor_mode_to_measure_ex(ls->sick);
	e_assert(ret>0);

	while (ls->state == STATE_WORK) {
		//1先读取水平方向角度
		sdata.h_angle = hl_turntable_get_angle(ls->control) - ls->pre_scan_angle
				+ ls->start_angle_h;
		if (first_time) {
			started_angle = sdata.h_angle;
		}

//		if (sdata.h_angle >= ls->end_angle_h + started_angle + angle_dif) {
//			DMSG((STDOUT,"sld_get_measurements sick is out of target position again,@%f!\r\n stop get data.\r\n",sdata.h_angle));
//			//ls->state = STATE_CANCEL;
//			break;
//		}

		if (sdata.h_angle < -1e-6) {
			DMSG((STDOUT,"angle = %f, leave.\n",sdata.h_angle));
//			pool_disconnect(&ls->pool);
//			break;
		}

		ret = sld_get_measurements_ex(ls->sick, &sdata);
//		DMSG((STDOUT,"profile_counter=%d profile_number=%d\n",sdata.profile_counter,sdata.profile_number));
		if (!first_time) {
			if (layer_num != sdata.layer_num
					|| profile_number != sdata.profile_number)
				DMSG(
						(STDOUT,"lost a data layer_num=%d profile_number=%d\n",layer_num,profile_number));
		}
		layer_num = sdata.layer_num + 1;
		if (layer_num == ls->interlace_v)
			layer_num = 0;
		profile_number = sdata.profile_number + 1;

		if (ret == E_ERROR_TIME_OUT) {
			DMSG((STDOUT,"sld_get_measurements ERROR: RETRY.\r\n"));
			Delay(10);
			continue;
		} else if (ret <= 0) {
			DMSG(
					(STDOUT,"sld_get_measurements sickld is down?ret=%d Out.\r\n",(int)ret));
			break;
		}

//		print_sdata(ls,&sdata);
		ret = pool_write(&ls->pool, &sdata);
		if (e_failed(ret))
			break;
		DMSG(
				(STDOUT,"\r[%d] %f end: %f\n",sdata.profile_number, sdata.h_angle,ls->end_angle_h));
		first_time = 0;
	}
	sld_set_sensor_mode_to_idle(ls->sick);
	DMSG((STDOUT,"scan job:write_data_routine done.\n"));
}

static void read_pool_data_routine(laser_sick_t *ls) {
	e_int32 ret;
	scan_data_t sdata;
	DMSG((STDOUT,"scan job:read_data_routine start.\r\n"));

	//创建数据交错缓冲区
	if (ls->interlace_v > 1)
		ls->scan_data_buf = (scan_data_t*) malloc(
				sizeof(scan_data_t) * ls->interlace_v);

	//TODO:更详细的异常处理
	while (ls->state == STATE_WORK) {
		ret = pool_read(&ls->pool, &sdata);
		if (ret <= 0) {
			DMSG((STDOUT,"pool has been cleared, leave!\r\n"));
			break;
		}
//		DMSG((STDOUT,"read_data_routine read data at angle: %f.\r\n", sdata.h_angle));
		filter_data(ls, &sdata);
	}

	free(ls->scan_data_buf);

	if (e_check(ls->slip_idx != ls->width, "#ERROR# 列数与要求的不一致!\n")) {
		DMSG(
				(STDOUT, "ls->slip_idx = %u, ls->width = %u \n", (unsigned int)ls->slip_idx,(unsigned int)ls->width));
		ls->width = ls->slip_idx; //TODO:修复这里,不允许简单丢掉！!
	}
	pool_cancle(&ls->pool);
	DMSG((STDOUT,"scan job:read_data_routine done.\r\n"));
}

static e_uint16 intensity_filter(e_uint32 echo) {
//	if (echo < 70 || echo > 700) //反射强度过滤
//	{
//				echo = 0;
//	}
	return echo;
}

static e_uint8 gray_filter(e_uint32 echo) {
	if (echo <= 70)
		return 0;
	else if (echo > 370)
		return 255;
	return (echo - 70) * 255.0 / 300.0;
}

//tick_idx 0...3
static e_int32 one_slip(laser_sick_t* ls, e_int32 tick_idx, scan_data_t * pdata,
		e_int32 data_offset, e_uint32 data_num, e_int32 type) {

	point_gray_t* pgray;
	point_polar_t *ppoint;
	int k, cheight;

	ppoint = ((point_polar_t*) ls->points_polar->mem) + tick_idx;
	pgray = ((point_gray_t*) ls->points_gray->mem) + tick_idx;

	cheight = ls->height / ls->interlace_v;
	if (e_check(data_num != cheight && data_num+1 != cheight, "#ERROR# 点数个数与要求的不一致!\n")) {
		DMSG(
				(STDOUT,"#ERROR# 点数个数多一个?不可能 data_num[%u] ls->height[%u]\n", (unsigned int)data_num,(unsigned int)ls->height/ls->interlace_v));
		return E_ERROR;
	}

	if (!type) {
		for (k = data_num - 1; k >= 0; --k) {
			ppoint->distance = pdata->range_measurements[data_offset + k];
			ppoint->angle_v = pdata->angle_measurements[data_offset + k];
			ppoint->angle_h = pdata->h_angle;

#if HACK_PNT_ANGLE_H
			ppoint->angle_h += ppoint->angle_v * ls->angle_dif_per_degree;
#endif

			ppoint->intensity = intensity_filter(
					pdata->echo_measurements[data_offset + k]);
			pgray->gray = gray_filter(
					pdata->echo_measurements[data_offset + k]);

			if (ppoint->distance <= 0.0000005) {
//				DMSG((STDOUT,"I"));
#ifdef HACK_INVALID_DATA
				(*ppoint) = *(ppoint - ls->interlace_v);
				(*pgray) = *(pgray - ls->interlace_v);
#endif
			}

			if (ppoint->distance > 20 || ppoint->intensity > 700)
				DMSG(
						(STDOUT,"[%d]\t%f\t%f\t%f\t%u\n",k, ppoint->distance,ppoint->angle_h,ppoint->angle_v,(unsigned int)ppoint->intensity));

			ppoint += ls->interlace_v;
			pgray += ls->interlace_v;
		} // end for
	} else {
#define SHIFT_NUM 0
		//补一行
		if (ls->active_sectors.left && ls->active_sectors.right
				&& cheight == data_num + 1) {
			ppoint += SHIFT_NUM + 1;
			pgray += SHIFT_NUM + 1;
		}

		for (k = 0; k < data_num - SHIFT_NUM; ++k) {
			ppoint->distance = pdata->range_measurements[data_offset + k];
			ppoint->angle_v = pdata->angle_measurements[data_offset + k];
			ppoint->angle_h = pdata->h_angle;
#if HACK_PNT_ANGLE_H
			ppoint->angle_h += ppoint->angle_v * ls->angle_dif_per_degree;
#endif
			ppoint->intensity = intensity_filter(
					pdata->echo_measurements[data_offset + k]);
			pgray->gray = gray_filter(
					pdata->echo_measurements[data_offset + k]);
			if (ppoint->distance <= 0.0000005) {
//				DMSG((STDOUT,"|"));
#ifdef HACK_INVALID_DATA
				(*ppoint) = *(ppoint - ls->interlace_v);
				(*pgray) = *(pgray - ls->interlace_v);
#endif
			}
			if (ppoint->distance > 20 || ppoint->intensity > 700)
				DMSG(
						(STDOUT,"[%d]\t%f\t%f\t%f\t%u\n",k, ppoint->distance,ppoint->angle_h,ppoint->angle_v,(unsigned int)ppoint->intensity));
			ppoint += ls->interlace_v;
			pgray += ls->interlace_v;
		} // end for
	}

	return E_OK;
}

static scan_data_t* add_slip_tick(laser_sick_t* ls, scan_data_t * pdata_in) {
	int idx = 0, dof, i;
	e_float64 dif;
#if HACK_SLIP_ANGLE
	pdata_in->h_angle = ls->slip_tick * ls->angle_dif_per_cloumn
	+ ls->start_angle_h;
#endif

	ls->slip_tick++; //1..4
	if (ls->interlace_v == 1)
		return pdata_in;

#if HACK_SICK_BUGS
	//偶数列的最后一个距离数据是错的。把最后一个数据移到0点
	//过渡方案，修正这个问题！！！！！！！！！！！！！！！！！！！！！！！！！
	/*
	 SICK profile_counter=908,profile_number=1,layer_num=2
	 SICK profile_counter=909,profile_number=2,layer_num=3
	 SICK profile_counter=910,profile_number=3,layer_num=0
	 SICK profile_counter=911,profile_number=4,layer_num=1
	 *
	 SICK profile_counter=912,profile_number=5,layer_num=2
	 SICK profile_counter=913,profile_number=6,layer_num=3
	 SICK profile_counter=914,profile_number=7,layer_num=0
	 SICK profile_counter=915,profile_number=8,layer_num=1
	 *
	 * */
	if (0 == pdata_in->layer_num % 2) {
		if (ls->active_sectors.left) {
			pdata_in->range_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;
			pdata_in->echo_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;
			pdata_in->angle_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;
			idx++;
		}
		if (ls->active_sectors.right) {
			pdata_in->range_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;
			pdata_in->echo_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;
			pdata_in->angle_measurements[pdata_in->sector_data_offsets[idx]
					+ pdata_in->num_measurements[idx] - 1] = 0;

			dof = pdata_in->sector_data_offsets[idx];
			dif = ls->start_angle_v[idx]
					+ ls->resolution_v * pdata_in->layer_num
					- pdata_in->angle_measurements[dof + 0];
			for (i = 0; i < pdata_in->num_measurements[idx]; i++) {
				pdata_in->angle_measurements[dof + i] += dif;
//				DMSG((STDOUT,"\t[%d] %f, %f, %u]\n",i, pdata_in->range_measurements[dof+i],
//						pdata_in->angle_measurements[dof+i],(unsigned int)pdata_in->echo_measurements[dof+i]));
			}

		}
	}

//	if (ls->active_sectors.left)
//		DMSG((STDOUT,"[%d,0] %f\n",pdata_in->layer_num, pdata_in->angle_measurements[pdata_in->sector_data_offsets[0]]));
//	if (ls->active_sectors.right)
//		DMSG((STDOUT,"[%d,1] %f\n",pdata_in->layer_num, pdata_in->angle_measurements[pdata_in->sector_data_offsets[1]]));
#endif
	memcpy(ls->scan_data_buf + pdata_in->layer_num, pdata_in,
			sizeof(scan_data_t));

	if (ls->slip_tick % ls->interlace_v == 0)
		return ls->scan_data_buf;

	return 0;
}

/************************************************************************/
/*         根据当前水平角度范围和垂直角度范围，得到每一圈实际扫描数据和灰度图        */
/************************************************************************/

static e_int32 filter_data(laser_sick_t* ls, scan_data_t * pdata_in) {
	int didx = 0, tick_idx;
	scan_data_t * pdata_buf, *pdata;
	e_assert(ls&&ls->state, E_ERROR_INVALID_HANDLER);

	if (!(pdata_buf = add_slip_tick(ls, pdata_in)))
		return E_OK;

//	DMSG((STDOUT,"ls->slip_idx == %d/%d pdata->h_angle=%f \n", ls->slip_idx+1, ls->width, pdata->h_angle ));
	if (ls->active_sectors.left) {
		for (tick_idx = 0, pdata = pdata_buf; tick_idx < ls->interlace_v;
				tick_idx++, pdata++) {
			one_slip(ls, tick_idx, pdata, pdata->sector_data_offsets[didx],
					pdata->num_measurements[didx], 0);
		}
		dm_update(ls->writer, ls->slip_idx, 0);
		didx++;
	}

	if (ls->active_sectors.right) {
		for (tick_idx = 0, pdata = pdata_buf; tick_idx < ls->interlace_v;
				tick_idx++, pdata++) {
			one_slip(ls, tick_idx, pdata, pdata->sector_data_offsets[didx],
					pdata->num_measurements[didx], 1);
		}
		dm_update(ls->writer, ls->slip_idx, 1);
		didx++;
	}

	ls->slip_idx++;
	if (ls->slip_idx >= ls->width) {
		ls->state = STATE_DONE;
		DMSG(
				(STDOUT,"ls_filter_data get enough data,last angle h = %f\n",pdata_buf->h_angle));
	}
	return E_OK;
}

static void print_config(laser_sick_t* ls) {
	e_assert(ls&&ls->state);
	DMSG(
			(STDOUT,"\t======================================================\n"));
	DMSG((STDOUT,"\tLS300 V2.0 Config\n"));
	DMSG((STDOUT,"\tTurntable:\n"));
	DMSG((STDOUT,"\t\tSpeed:%u μs / plus\n",(unsigned int)ls->speed_h));
	DMSG((STDOUT,"\t\tAngle:%f - %f\n",ls->start_angle_h,ls->end_angle_h));
	DMSG((STDOUT,"\tLaser Machine:\n"));
	DMSG((STDOUT,"\t\tSpeed:%u Hz\n",(unsigned int)ls->speed_v));
	DMSG((STDOUT,"\t\tResolution:%f\n",ls->resolution_v));
	DMSG((STDOUT,"\t\tInterlace:%d\n",ls->interlace_v));
	if (ls->active_sectors.left)
		DMSG(
				(STDOUT,"\t\tAngle:%f - %f\n",ls->start_angle_v[0],ls->end_angle_v[0]));
	if (ls->active_sectors.right)
		DMSG(
				(STDOUT,"\t\tAngle:%f - %f\n",ls->start_angle_v[1],ls->end_angle_v[1]));
	DMSG((STDOUT,"\tOutput:\n"));
	DMSG((STDOUT,"\t\tWidth:%d Height:%d\n",ls->width,ls->height));
	DMSG(
			(STDOUT,"\t\tangle_dif_per_column:%f angle_dif_per_degree:%f\n", ls->angle_dif_per_cloumn,ls->angle_dif_per_degree));
	DMSG(
			(STDOUT,"\t======================================================\n"));
}
/*50	49.6
 100	99.8
 150	150.2
 200	199.7
 250	250.4
 400	406.9
 700	800.3
 850	965.4
 1250	1374.5
 2500	2836.4*/
static int f2r(int speed) {
	switch (speed) {
//	case 50:
//		return 49.6;
//	case 100:
//		return 99.8;
//	case 150:
//		return 150.2;
//	case 200:
//		return 199.7;
//	case 250:
//		return 250.4;
	case 400:
		return 407;
	case 700:
		return 800;
	case 850:
		return 965;
	case 1250:
		return 1375;
	case 2500:
		return 2836;
	}
	return speed;
}

static int r2f(int speed) {
	switch (speed) {
//	case 50:
//		return 49.6;
//	case 100:
//		return 99.8;
//	case 150:
//		return 150.2;
//	case 200:
//		return 199.7;
//	case 250:
//		return 250.4;
	case 407:
		return 400;
	case 800:
		return 700;
	case 965:
		return 850;
	case 1375:
		return 1250;
	case 2836:
		return 2500;
	}
	return speed;
}
