/*!
 * \file hd_laser_control.c
 * \brief	控制板通讯
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser base
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include <ls300/hd_laser_photo.h>
#include <arch/hd_timer_api.h>

/*结构体定义*/
struct laser_photo_t {
	laser_control_t *lc;
	int picture_num;
	e_float64 camera_angle_start;
	e_float64 camera_angle_end;

	ethread_t *thread_work;

	e_int32 (*on_status_change)(void*,int);
	void*ctx;

	int state;
};

//不考虑上层的错误调用
static enum {
	STATE_NONE = 0,
	STATE_IDLE = 1,
	STATE_WORK = 2,
	STATE_CANCEL = 3
};

#define should_continue (lp->state==STATE_WORK)
static e_int32 inter_lp_scan(laser_photo_t *lp);
static e_int32 thread_scan_func(laser_photo_t *lp);

e_int32 lp_init(laser_photo_t **lpp, laser_control_t *lc,e_int32 (*on_status_change)(void*,int),void*ctx) {
	laser_photo_t *lp;
	e_assert(lc, E_ERROR_INVALID_HANDLER);
	lp = (laser_photo_t *)calloc(1,sizeof(struct laser_photo_t));
	e_assert(lp, E_ERROR_BAD_ALLOCATE);
	lp->lc = lc;
	lp->on_status_change = on_status_change;
	lp->ctx = ctx;
	lp->state = STATE_IDLE;
	(*lpp) = lp;
	return E_OK;
}

e_int32 lp_uninit(laser_photo_t *lp) {
	e_assert(lp&&lp->state, E_ERROR_INVALID_HANDLER);
	if(lp->state == STATE_WORK){
		lp_cancel(lp);
		Delay(500);
	}
	if(lp->thread_work) killthread(lp->thread_work);
	memset(lp, 0, sizeof(laser_photo_t));
	free(lp);
	return E_OK;
}

e_int32 lp_scan(laser_photo_t *lp,
		e_float64 camera_angle_start, e_float64 camera_angle_end) {
	e_int32 ret;
	e_assert(lp && lp->state == STATE_IDLE, E_ERROR_INVALID_STATUS);

	lp->state = STATE_WORK;
	lp->on_status_change(lp->ctx,lp->state);

	lp->camera_angle_start = camera_angle_start;
	lp->camera_angle_end = camera_angle_end;

	DMSG((STDOUT, "scan job routine starting read routine...\r\n"));

	//check priv
	if(lp->thread_work) killthread(lp->thread_work);
	ret = createthread("photo scan thread", (thread_func) &thread_scan_func,
						lp, NULL, &lp->thread_work);
	if ( ret <= 0 ) {
		DMSG((STDOUT, "create photo thread failed!\r\n"));
		lp->state = STATE_IDLE;
		return E_ERROR;
	}
	ret = resumethread(lp->thread_work);
	if ( ret <= 0 ) {
		DMSG((STDOUT, "resume photo thread failed!\r\n"));
		if(lp->thread_work) killthread(lp->thread_work);
		lp->state = STATE_IDLE;
		return E_ERROR;
	}
	DMSG((STDOUT, "photo scan job routine start successful.\r\n"));
}

e_int32 lp_cancel(laser_photo_t *lp) {
	e_int32 ret;
	lp->state = STATE_CANCEL;
	ret = hl_turntable_stop(lp->lc);
	e_assert(ret>0, ret);
	return E_OK;
}

static e_int32 thread_scan_func(laser_photo_t *lp) {
	e_int32 ret;
	ret = inter_lp_scan(lp);
	lp->state = STATE_IDLE;
	lp->on_status_change(lp->ctx,lp->state);
	return ret;
}

static e_int32 inter_lp_scan(laser_photo_t *lp)
{
	e_float64 start_angle;
	e_float64 camera_angle_start, camera_angle_end;
	e_int32 ret;
	laser_control_t *lc = lp->lc;
	camera_angle_start = lp->camera_angle_start;
	camera_angle_end = lp->camera_angle_end;

	e_assert(lc && camera_angle_start < camera_angle_end, E_ERROR_INVALID_PARAMETER);

//打印配置信息
	DMSG((STDOUT,"hl_camera_scan picture ["));
	for (start_angle = camera_angle_start; start_angle < camera_angle_end;
			start_angle += TAKEPHOTO_ANGLE)
			{
		DMSG((STDOUT,"%f° - ",start_angle));
	}
	DMSG((STDOUT,"]\r\n"));

//开始照相
	DMSG((STDOUT,"hl_camera_scan starting....\r\n"));
	DMSG((STDOUT,"hl_camera_scan goto start positon [%f]°\r\n",camera_angle_start));
	ret = hl_turntable_fast_turn(lc, camera_angle_start); //转至起始角
	e_assert(ret>0 && should_continue, ret);
	Delay(300); //拍照时间停留增加（由0.1秒改为0.3秒），保证充分聚焦

	ret = hl_camera_take_photo(lc);
	e_assert(ret>0 && should_continue, ret);
	lp->picture_num = 1;
	DMSG((STDOUT,"hl_camera_scan take [%d] photos.\r\n",(int)lp->picture_num));
//每隔一定角度拍照一次
	for (start_angle = camera_angle_start;
			should_continue && start_angle < camera_angle_end;
			start_angle += TAKEPHOTO_ANGLE)
			{
		ret = hl_turntable_fast_turn(lc, TAKEPHOTO_ANGLE);
		e_assert(ret>0 && should_continue, ret);
		Delay(300); //聚焦时间
		ret = hl_camera_take_photo(lc);
		lp->picture_num++;
		DMSG((STDOUT,"hl_camera_scan take [%d] photos.\r\n",(int)lp->picture_num));
	}

	return ret;
}

