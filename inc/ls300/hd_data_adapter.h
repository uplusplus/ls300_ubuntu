/*!
 * \file hd_data_adapter.h
 * \brief 数据存取适配器
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_DATA_ADAPTER_H
#define HD_DATA_ADAPTER_H

#include <arch/hd_plat_base.h>
#include <comm/hd_point_type.h>

#define POINT_NUM_BUF        (10*1024)
#define FILE_SUFFIX_LENGTH   (16)

typedef struct file_ptr_t {
	size_t handle;
	e_int32 mode;
	char file_name[MAX_PATH_LEN];
	e_uint32 width;
	e_uint32 width_sum;
	e_uint32 height;
	e_float32 h_w;
	e_int32 pnt_type;
	e_uint32 cursor;
	e_uint32 max_file_idx;
} file_ptr_t;

typedef struct {
	e_int32 (*open)(file_ptr_t *file);
	e_int32 (*close)(file_ptr_t *file, int save);
	e_int32 (*write_column)(file_ptr_t *file, e_uint32 column_idx,
			point_t* point, int file_idx);
} file_op_t;

typedef struct data_adapter_t {
	file_ptr_t file;
	e_int32 pnt_type;
	file_op_t op;
	e_int32 state;
} data_adapter_t;

typedef struct {
	char file_suffix[FILE_SUFFIX_LENGTH];
	e_int32 pnt_type;
	file_op_t op;
} file_adapter_t;

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif
////////////////////////////////////////////////////////////////////////
//用户用
e_int32 DEV_EXPORT da_open(data_adapter_t *da, e_uint8 *name, int width,
		int height, e_float32 h_w, int mode);
e_int32 DEV_EXPORT da_close(data_adapter_t *da, int save);
e_int32 DEV_EXPORT da_write_column(data_adapter_t *da, e_uint32 column_idx,
		point_t* point, int file_idx);
#ifdef __cplusplus
}
#endif

#endif /*HD_DATA_ADAPTER_H*/
