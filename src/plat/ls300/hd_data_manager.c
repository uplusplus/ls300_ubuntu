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

char EGL_NODE[100] = "test.sprite";

//#ifdef DMSG
//#undef DMSG
//#define DMSG
//#endif

#define DANUM 4
//三个输出点：一个点云输出点，一个灰度图输出点，一个监控输出点。

struct data_manager_t {
	e_int32 state;
	e_uint32 num;
	//文件读写

	char data_file[MAX_PATH_LEN];
	char gray_file[MAX_PATH_LEN];
	char pipe_file[MAX_PATH_LEN];

	//点云宽度
	e_uint32 width, height;

	//数据缓冲
	int buf_type;
	//点云缓冲区
	point_t *points_xyz;
	//灰度图缓冲区
	point_t *points_gray;

	data_adapter_t adapters[];
};

data_manager_t*
dm_alloc(char* ptDir, char *grayDir, char *files_dir, int width, int height,
		float h_w, int mode) {
	int i, ret;
	data_adapter_t* pda;
	system_time_t sys_time;
	char *files[DANUM];

	e_assert(ptDir&&grayDir&&width&&height&& (mode==E_DWRITE||mode==E_WRITE),
			E_ERROR_INVALID_PARAMETER);

	data_manager_t*dm =
			calloc(1, sizeof(data_manager_t) + sizeof(data_adapter_t) * DANUM);
	e_assert(dm, dm);

	dm->width = width;
	dm->height = height;

	GetLocalTime(&sys_time);
	sprintf(dm->data_file, "%s/%d-%d-%d-%d-%d-%d.pcd", ptDir, sys_time.year,
			sys_time.month, sys_time.day, sys_time.hour, sys_time.minute,
			sys_time.second);
	sprintf(dm->gray_file, "%s/%d-%d-%d-%d-%d-%d.jpg", grayDir, sys_time.year,
			sys_time.month, sys_time.day, sys_time.hour, sys_time.minute,
			sys_time.second);

	files[0] = dm->data_file;
	files[1] = dm->gray_file;
	files[2] = "test.memgray";
#if LINUX
	files[3] = "/tmp/test.sprite";
#else
	snprintf(EGL_NODE, sizeof(EGL_NODE), "%s/egl.sprite", files_dir);
	files[3] = EGL_NODE;
#endif

	pda = dm->adapters;
	dm->num = 0;
	for (i = 0; i < sizeof(files) / sizeof(char*); i++) {
		DMSG((STDOUT,"data_manager try open file[%d]=%s",i,files[i]));
		ret = da_open(pda, files[i], width, height,h_w, mode);
		if (e_failed(ret)) {
			continue;
		}
		dm->num++;
		pda++;
	}

	if (dm->num > 0)
		dm->state = 1;
	return dm;
}

e_int32 dm_free(data_manager_t *dm, int save) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm, E_ERROR_INVALID_HANDLER);

	DMSG((STDOUT,"dm_free close file save?%d",save));

	pda = dm->adapters;
	for (i = 0; i < dm->num; i++) {
		ret = da_close(&pda[i], save);
		e_check(ret<=0);
		DMSG((STDOUT,"Close %d da\n",i+1));
	}

	if (dm->points_xyz)
		free(dm->points_xyz);
	if (dm->points_gray)
		free(dm->points_gray);
	free(dm);
	return E_OK;
}

e_int32 dm_write_point(data_manager_t *dm, int x, int y, point_t* point,
		int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;
	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_write_point(&pda[i], x, y, point, file_right);
		e_check(ret<=0);

	}

	return E_OK;
}

e_int32 dm_write_row(data_manager_t *dm, e_uint32 row_idx, point_t* point,
		int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;
	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_write_row(&pda[i], row_idx, point, file_right);
		e_check(ret<=0);
	}
	return E_OK;
}

e_int32 dm_write_column(data_manager_t *dm, e_uint32 column_idx, point_t* point,
		int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;
	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_write_column(&pda[i], column_idx, point, file_right);
		e_check(ret<=0);
	}
	return E_OK;
}

e_int32 dm_append_points(data_manager_t *dm, point_t* point, int pt_num,
		int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;
	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_append_points(&pda[i], point, pt_num, file_right);
		e_check(ret<=0);
	}
	return E_OK;
}

e_int32 dm_append_row(data_manager_t *dm, point_t* point, int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;

	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_append_row(&pda[i], point, file_right);
		e_check(ret<=0);
	}
	return E_OK;
}

e_int32 dm_append_column(data_manager_t *dm, point_t* point, int file_right) {
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;

	for (i = 0; i < dm->num; i++) {
		if (point->type != pda[i].pnt_type)
			continue;
		ret = da_append_column(&pda[i], point, file_right);
		e_check(ret<=0);
	}
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
	int i, ret;
	data_adapter_t* pda;
	e_assert(dm&&dm->state, E_ERROR_INVALID_HANDLER);

	pda = dm->adapters;

	if (DATA_BLOCK_TYPE_COLUMN == dm->buf_type) {
		for (i = 0; i < dm->num; i++) {
			if (dm->points_gray->type == pda[i].pnt_type) {
				ret = da_write_column(&pda[i], c, dm->points_gray, file_right);
				e_check(ret<=0);
			} else if (dm->points_xyz->type == pda[i].pnt_type) {
				ret = da_append_points(&pda[i], dm->points_xyz, dm->height,
						file_right);
				e_check(ret<=0);
			}
		}
	} else {
		return E_ERROR_INVALID_PARAMETER;
	}

	return E_OK;
}
