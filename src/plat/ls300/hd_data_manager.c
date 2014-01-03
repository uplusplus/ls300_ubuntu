/*!
 * \file hd_data_manager.h
 * \brief 数据存取
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include <ls300/hd_data_manager.h>
#include <arch/hd_timer_api.h>
#include <arch/hd_file_api.h>
#include <jpg/hd_jpeg.h>
#include <comm/hd_utils.h>

char EGL_NODE[100] = "test.sprite";
display_t display = { 0 };

//#ifdef DMSG
//#undef DMSG
//#define DMSG
//#endif
#define START_VIDEO_SERVER 1
#define DANUM 2
//2个输出点：一个点云输出点，一个灰度图输出点

struct data_manager_t {
	e_int32 state;
	//文件读写
	char data_file[MAX_PATH_LEN];
	char gray_file[MAX_PATH_LEN];
	char tunable_file[MAX_PATH_LEN];

	//点云宽度
	e_uint32 width, height;

	//数据缓冲
	int buf_type;
	//点云缓冲区
	point_t *points_xyz;
	//灰度图缓冲区
	point_t *points_gray;

	data_adapter_t adapters_point_cloud;
	data_adapter_t adapters_gray;
	file_t f_tunable;
};

static e_int32 init_display(int width, int height, float h_w, int mode);
static void uninit_display();

data_manager_t*
dm_alloc(char* ptDir, char *grayDir, int width, int height, float h_w, int mode) {
	int i, ret;
	data_adapter_t* pda;
	system_time_t sys_time;

	e_assert(ptDir&&grayDir&&width&&height&& (mode==E_DWRITE||mode==E_WRITE),
			E_ERROR_INVALID_PARAMETER);

	ret = hd_creatdir(ptDir);
//	e_assert(ret>0, E_ERROR);
	ret = hd_creatdir(grayDir);
//	e_assert(ret>0, E_ERROR);

	data_manager_t*dm = calloc(1, sizeof(data_manager_t));
	e_assert(dm, E_ERROR_BAD_ALLOCATE);

	ret = init_display(width, height, h_w, mode);
	e_assert(ret>0, ret);

	dm->width = width;
	dm->height = height;

	GetLocalTime(&sys_time);
	sprintf(dm->data_file, "%s/%d-%d-%d-%d-%d-%d.pcd", ptDir, sys_time.year,
			sys_time.month, sys_time.day, sys_time.hour, sys_time.minute,
			sys_time.second);
	sprintf(dm->gray_file, "%s/%d-%d-%d-%d-%d-%d.jpg", grayDir, sys_time.year,
			sys_time.month, sys_time.day, sys_time.hour, sys_time.minute,
			sys_time.second);
	sprintf(dm->tunable_file, "%s/%d-%d-%d-%d-%d-%d.tun", ptDir, sys_time.year,
			sys_time.month, sys_time.day, sys_time.hour, sys_time.minute,
			sys_time.second);

	ret = da_open(&dm->adapters_point_cloud, dm->data_file, width, height, h_w,
			mode);
	e_check(ret<=0);
	ret = da_open(&dm->adapters_gray, "test.memgray", width, height, h_w, mode);
	e_check(ret<=0);

	ret = fi_open(dm->tunable_file, F_WRITE | F_CREATE, &dm->f_tunable);
	e_check(!ret);

#if START_VIDEO_SERVER
	socket_video_server_start("127.0.0.1", 9090, E_SOCKET_TCP);
#endif
	dm->state = 1;
	return dm;
}

e_int32 dm_free(data_manager_t *dm, int save) {
	int i, ret;
	e_assert(dm, E_ERROR_INVALID_HANDLER);

	DMSG((STDOUT,"dm_free close file save?%d\n",save));

#if START_VIDEO_SERVER
	socket_video_server_stop();
#endif

	if (display.buf)
		gray_to_jpeg_file(dm->gray_file, display.buf, display.w, display.h);

	ret = da_close(&dm->adapters_point_cloud, save);
	e_check(ret<=0);

	ret = da_close(&dm->adapters_gray, save);
	e_check(ret<=0);

	fi_close(&dm->f_tunable);

	if (dm->points_xyz)
		free(dm->points_xyz);
	if (dm->points_gray)
		free(dm->points_gray);
	free(dm);

	uninit_display();

	return E_OK;
}

e_int32 dm_alloc_buffer(data_manager_t *dm, int buf_type, point_t **pnt_buf,
		point_t **gray_buf) {
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);
	if (DATA_BLOCK_TYPE_COLUMN == buf_type) {
		if (dm->points_xyz)
			free(dm->points_xyz);
		if (dm->points_gray)
			free(dm->points_gray);
		dm->points_xyz = malloc_points(PNT_TYPE_POLAR, dm->height);
		dm->points_gray = malloc_points(PNT_TYPE_GRAY, dm->height);
	} else {
		return E_ERROR_INVALID_PARAMETER;
	}

	dm->buf_type = buf_type;
	(*pnt_buf) = dm->points_xyz;
	(*gray_buf) = dm->points_gray;
	e_assert(dm->points_xyz && dm->points_gray, E_ERROR_BAD_ALLOCATE);
	return E_OK;
}

e_int32 dm_update(data_manager_t *dm, int c, int file_right) {
	int ret;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);
	ret = da_write_column(&dm->adapters_point_cloud, c, dm->points_xyz,
			file_right);
	e_check(ret<=0);
	ret = da_write_column(&dm->adapters_gray, c, dm->points_gray, file_right);
	e_check(ret<=0);
	return E_OK;
}

//不要多线程调用此函数
e_int32 dm_write_meta(data_manager_t *dm, e_uint32 usec_timestamp,
		char *prefix, char *fmt, ...) {
	int ret;
	va_list ap;
	char buf[21];
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	va_start(ap, fmt);

	ret = fi_printf(&dm->f_tunable, "\n%10u %s\t", usec_timestamp, prefix);
	e_assert(ret>0, E_ERROR);
	ret = fi_vprintf(&dm->f_tunable, fmt, ap);
	e_assert(ret>0, E_ERROR);
	return E_OK;
}

static e_int32 init_display(int width, int height, float h_w, int mode) {
	display.w = mode == E_DWRITE ? width * 2 : width;
	display.h = height;
	display.h_w = h_w;
	display.buf = (e_uint8 *) calloc(display.w * display.h, 1);
	e_assert(display.buf, E_ERROR_BAD_ALLOCATE);
	return E_OK;
}

static void uninit_display() {
	free(display.buf);
	display.w = display.h = display.buf = NULL;
}
