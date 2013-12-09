/*
 * =====================================================================================
 *
 *       Filename:  hd_laser_machine.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013年12月01日
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  uplusplus
 *        Company:  zhd-hdsy
 *
 * =====================================================================================
 */
#include <ls300/hd_laser_machine.h>

enum {
	STATE_NONE = 0,
	STATE_PAUSE = 1,
	STATE_WORK = 2,
	STATE_CANCEL = 3,
};

static laser_machine_t laser_machine = { 0 };
static laser_machine_t* lm = &laser_machine;
static void main_loop(void* vs);

laser_machine_t* lm_get_instance() {
	return lm;
}

static int pause_loop() {
	int ret = lm->state;
	if (lm->state != STATE_WORK)
		return ret;
	lm->state = STATE_PAUSE;
	while (!lm->is_paused) {
		Delay(10);
	}
	return ret;
}

static void resume_loop(int last_state) {
	if (lm->state != STATE_PAUSE
			&& last_state != STATE_PAUSE)
		return;

	lm->state = STATE_WORK;
	lm->is_paused = 0;
	semaphore_post(&lm->wakeup);
}

static int exit_loop() {
	e_assert(
			lm->state==STATE_WORK||lm->state==STATE_PAUSE,
			E_ERROR_INVALID_STATUS);
	resume_loop(lm->state);
	lm->state = STATE_CANCEL;
	while (lm->state) {
		Delay(100);
	}
	return E_OK;
}

e_int32 lm_init(laser_control_t *lc) {
	int ret;
	e_assert(!lm->state, E_ERROR_INVALID_STATUS);
	lm->state = STATE_PAUSE;
	lm->is_paused = 0;
	semaphore_init(&lm->wakeup, 0);

	lm->lc = lc;

	DMSG(
			(STDOUT, "laser machine starting main routine...\r\n"));
	//启动sick数据读取线程
	ret = createthread("laser_machine",
			(thread_func) &main_loop, lm, NULL,
			&lm->main_thread);
	e_assert(ret>0, ret);

	ret = resumethread(lm->main_thread);
	e_assert(ret>0, ret);

	return E_OK;
}

e_int32 lm_uninit() {
	exit_loop();
	killthread(lm->main_thread);
	semaphore_destroy(&lm->wakeup);
	memset(lm, 0, sizeof(laser_machine_t));
}

e_int32 lm_start_record_angle(data_manager_t* dm) {
	e_assert(lm->state&&dm,
			E_ERROR_INVALID_PARAMETER);
	lm->dm = dm;
	resume_loop(lm->state);
	return E_OK;
}
e_int32 lm_stop_record_angle() {
	e_assert(lm->state,
			E_ERROR_INVALID_PARAMETER);
	lm->dm = NULL;
	return E_OK;
}

e_int32 lm_start_status_monitor() {
	e_assert(lm->state,
			E_ERROR_INVALID_PARAMETER);
	resume_loop(lm->state);
	return E_OK;
}

e_int32 lm_stop_status_monitor() {
	e_assert(lm->state,
			E_ERROR_INVALID_PARAMETER);
	pause_loop();
	lm->lc = NULL;
	return E_OK;
}

static int getBattery();
static int getTemperature();
static int getTilt();
static int getAngle();

#define ts_battery 			{getBattery, 1}
#define ts_temperature   	{getTemperature, 1}
#define ts_getTilt  		{getTilt, 10}
#define ts_getAngle  		{getAngle, 1}
static const struct {
	int (*work)();
	int seq_len;
} system_timeslice[32] = { ts_getAngle,
		ts_getAngle, ts_battery, ts_getAngle,
		ts_getAngle, ts_temperature, ts_getAngle,
		ts_getAngle, ts_battery, ts_getAngle,
		ts_getAngle, ts_temperature, ts_getAngle,
		ts_getAngle, ts_battery, ts_getAngle,
		ts_getAngle, ts_temperature, ts_getAngle,
		ts_getAngle, ts_getAngle, ts_getTilt, { },
		{ }, { }, { }, { }, { }, { }, { }, { },
		ts_getAngle };

static void main_loop(void* vs) {
	static int seq = 0;
	while (lm->state) {
		while (lm->state == STATE_WORK) {
			system_timeslice[seq].work();
			seq += system_timeslice[seq].seq_len;
			seq &= 31;
//			DMSG((STDOUT,"_%d_",seq));
		}
		if (lm->state == STATE_PAUSE) {
			lm->is_paused = 1;
			semaphore_wait(&lm->wakeup);
		} else if (lm->state == STATE_CANCEL)
			break;
	}

	lm->state = STATE_NONE;
}

int getBattery() {
	lm->battery = hl_get_battery(lm->lc);
//	DMSG((STDOUT,"battery %8.4f\n",lm->battery));
}
int getTemperature() {
	//获取温度
	lm->temperature = hl_get_temperature(lm->lc);
//	DMSG((STDOUT,"temperature %8.4f\n",lm->temperature));
}
int getTilt() {
	hl_get_tilt(lm->lc, &lm->angle);
//	DMSG((STDOUT,"tilt %8.4f %8.4f\n",lm->tilt.dX,lm->tilt.dY));
}
int getAngle() {
	lm->angle_usec_timestamp = GetTickCount();
	lm->angle = hl_turntable_get_angle(lm->lc);
	if (lm->dm)
		dm_write_tunable(lm->dm,
				lm->angle_usec_timestamp,
				lm->angle);
//	DMSG((STDOUT,"angle %8.4f\n",lm->angle));
}

//开始工作
e_int32 lm_turntable_prepare(
		e_float64 pre_start_angle) {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_prepare(lm->lc,
			pre_start_angle);
	resume_loop(state);
	return ret;
}

//开始工作
e_int32 lm_turntable_start() {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_start(lm->lc);
	resume_loop(state);
	return ret;
}

//停止工作
e_int32 lm_turntable_stop() {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_stop(lm->lc);
	resume_loop(state);
	return ret;
}

//设置转台参数//设置旋转速度,区域
e_int32 lm_turntable_config(e_uint32 plus_delay,
		e_float64 start, e_float64 end) {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_config(lm->lc, plus_delay,
			start, end);
	resume_loop(state);
	return ret;
}

//根据实际传过来的水平旋转角度，调整水平台，以较快速度转到实际水平台的起始角度/转台回到起始原点
e_int32 lm_turntable_turn(e_float64 angle) {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_turn(lm->lc, angle);
	resume_loop(state);
	return ret;
}

//异步转动
e_int32 lm_turntable_turn_async(e_float64 angle) {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_turn_async(lm->lc, angle);
	resume_loop(state);
	return ret;
}

e_int32 lm_turntable_fast_turn(e_float64 angle) {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_fast_turn(lm->lc, angle);
	resume_loop(state);
	return ret;
}

e_int32 lm_turntable_check() {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_check(lm->lc);
	resume_loop(state);
	return ret;
}

//相机拍照
e_int32 lm_camera_take_photo() {
	int ret, state;
	state = pause_loop();
	ret = hl_camera_take_photo(lm->lc);
	resume_loop(state);
	return ret;
}

//获取当前水平转台的角度
e_int32 lm_turntable_get_step() {
	int ret, state;
	state = pause_loop();
	ret = hl_turntable_get_step(lm->lc);
	resume_loop(state);
	return ret;
}

e_float64 lm_turntable_get_angle() {
	e_float64 ret;
	int state = pause_loop();
	ret = hl_turntable_get_angle(lm->lc);
	resume_loop(state);
	return ret;
}

//获取温度
e_float64 lm_get_temperature() {
	e_float64 ret;
	int state = pause_loop();
	ret = hl_get_temperature(lm->lc);
	resume_loop(state);
	return ret;
}

//获取倾斜度
e_int32 lm_get_tilt(angle_t* angle) {
	int ret, state;
	state = pause_loop();
	ret = hl_get_tilt(lm->lc, angle);
	resume_loop(state);
	return ret;
}

//获取状态
e_float64 lm_get_battery() {
	e_float64 ret;
	int state = pause_loop();
	ret = hl_get_battery(lm->lc);
	resume_loop(state);
	return ret;
}

//亮红灯
e_int32 lm_led_red() {
	int ret, state;
	state = pause_loop();
	ret = hl_led_red(lm->lc);
	resume_loop(state);
	return ret;
}

//亮绿灯
e_int32 lm_led_green() {
	int ret, state;
	state = pause_loop();
	ret = hl_led_green(lm->lc);
	resume_loop(state);
	return ret;
}

//LED熄灭
e_int32 lm_led_off() {
	int ret, state;
	state = pause_loop();
	ret = hl_led_off(lm->lc);
	resume_loop(state);
	return ret;
}

//以最快速度调整到指定水平范围起始角
e_int32 lm_search_zero() {
	int ret, state;
	state = pause_loop();
	ret = hl_search_zero(lm->lc);
	resume_loop(state);
	return ret;
}

//硬件信息
e_int32 lm_get_info(e_uint32 idx, e_uint8* buffer,
		e_int32 blen) {
	int ret, state;
	state = pause_loop();
	ret = hl_get_info(lm->lc, idx, buffer, blen);
	resume_loop(state);
	return ret;
}
