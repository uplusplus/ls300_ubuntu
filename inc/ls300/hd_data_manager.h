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

#ifndef HD_DATA_MANAGER_H
#define HD_DATA_MANAGER_H

#include <ls300/hd_data_adapter.h>

typedef struct data_manager_t data_manager_t;

enum {
	DATA_BLOCK_TYPE_NONE = 0, DATA_BLOCK_TYPE_COLUMN = 1,
};

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif
////////////////////////////////////////////////////////////////////////
//
data_manager_t* DEV_EXPORT dm_alloc(char* ptDir, char *grayDir, char *files_dir,
		int width, int height, float h_w, int mode);
e_int32 DEV_EXPORT dm_free(data_manager_t *da, int save);

e_int32 DEV_EXPORT dm_write_point(data_manager_t *da, int x, int y,
		point_t* point, int file_right);
e_int32 DEV_EXPORT dm_write_row(data_manager_t *da, e_uint32 row_idx,
		point_t* point, int file_right);
e_int32 DEV_EXPORT dm_write_column(data_manager_t *da, e_uint32 column_idx,
		point_t* point, int file_right);
e_int32 DEV_EXPORT dm_append_points(data_manager_t *da, point_t* point,
		int pt_num, int file_right);
e_int32 DEV_EXPORT dm_append_row(data_manager_t *da, point_t* point,
		int file_right);
e_int32 DEV_EXPORT dm_append_column(data_manager_t *da, point_t* point,
		int file_right);

//扩展API，用于缓冲区分配管理
e_int32 DEV_EXPORT dm_alloc_buffer(data_manager_t *dm, int buf_type,
		point_t **pnt_buf, point_t **gray_buf);
e_int32 DEV_EXPORT dm_update(data_manager_t *dm, int c, int file_right);
e_int32 DEV_EXPORT dm_write_tunable(data_manager_t *dm, e_uint32 usec_timestamp,
		e_float64 angle);

#ifdef __cplusplus
}
#endif

#endif /*HD_DATA_MANAGER_H*/
