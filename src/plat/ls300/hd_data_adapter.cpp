/*
 * =====================================================================================
 *
 *       Filename:  hd_data_adapter.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013年06月19日 15时03分25秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mei Kang (), meikang9527@163.com
 *        Company:  College of Information Engineering of CDUT
 *
 *        modify:
 *        			Joy.you 20120628  	接口重构,PCL格式重构
 *        			Joy.you 20120629  	添加JPG格式支持
 *        			Joy.you 20120629  	添加GIF格式支持
 *
 *
 * =====================================================================================
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string.h>
#include <netinet/in.h>

#include <ls300/hd_data_adapter.h>
#include <arch/hd_file_api.h>
#include <comm/hd_utils.h>
#include <arch/hd_pipe_api.h>
#include <arch/hd_timer_api.h>
#include <ls300/hd_connect.h>

#include <las/HLSWriter.h>
#include <jpg/hd_jpeg.h>
#include <gif/gif_api.h>

////////////////////////////////////////////////////////////////////////
//底层开发

static e_int32 register_file_format(data_adapter_t *da, int i); //注册文件读写接口

//PCL
static e_int32 inter_pcl_open(file_ptr_t *file);
static e_int32 inter_pcl_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_pcl_close(file_ptr_t *file, int save);
static e_int32 inter_pcl_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_pcl_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_pcl_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_pcl_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_pcl_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_pcl_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_pcl_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_pcl_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);

//hls
static e_int32 inter_hls_open(file_ptr_t *file);
static e_int32 inter_hls_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_hls_close(file_ptr_t *file, int save);
static e_int32 inter_hls_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_hls_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_hls_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_hls_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_hls_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_hls_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_hls_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_hls_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_hls_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);

//jpeg
static e_int32 inter_jpg_open(file_ptr_t *file);
static e_int32 inter_jpg_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_jpg_close(file_ptr_t *file, int save);
static e_int32 inter_jpg_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_jpg_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_jpg_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_jpg_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_jpg_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_jpg_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_jpg_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_jpg_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_jpg_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);

//gif
static e_int32 inter_gif_open(file_ptr_t *file);
static e_int32 inter_gif_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_gif_close(file_ptr_t *file, int save);
static e_int32 inter_gif_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_gif_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_gif_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_gif_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_gif_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_gif_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_gif_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_gif_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_gif_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);

//sprite
static e_int32 inter_sprite_open(file_ptr_t *file);
static e_int32 inter_sprite_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_sprite_close(file_ptr_t *file, int save);
static e_int32 inter_sprite_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_sprite_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_sprite_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_sprite_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_sprite_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_sprite_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_sprite_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_sprite_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_sprite_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);
static e_int32 inter_sprite_on_data_change(file_ptr_t *file1,
		file_ptr_t *file2);

//memgray
static e_int32 inter_memgray_open(file_ptr_t *file);
static e_int32 inter_memgray_combine(file_ptr_t *file1, file_ptr_t *file2);
static e_int32 inter_memgray_close(file_ptr_t *file, int save);
static e_int32 inter_memgray_write_point(file_ptr_t *file, int x, int y,
		point_t* point);
static e_int32 inter_memgray_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point);
static e_int32 inter_memgray_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point);
static e_int32 inter_memgray_append_points(file_ptr_t *file, point_t* point,
		int pt_num);
static e_int32 inter_memgray_append_row(file_ptr_t *file, point_t* point);
static e_int32 inter_memgray_append_column(file_ptr_t *file, point_t* point);
static e_int32 inter_memgray_read_points(file_ptr_t *file, point_t* point,
		int buf_len);
static e_int32 inter_memgray_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf);
static e_int32 inter_memgray_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf);
static e_int32 inter_memgray_on_data_change(file_ptr_t *file1,
		file_ptr_t *file2);

//open和close是必须有的接口
file_adapter_t file_adpt[] = {
	{
		file_suffix : "pcd1",
		pnt_type: PNT_TYPE_XYZ,
		op : {
			open:inter_pcl_open,
			combine:inter_pcl_combine,
			on_data_change:NULL,
			close:inter_pcl_close,
			write_point:inter_pcl_write_point,
			write_row:inter_pcl_write_row,
			write_column:inter_pcl_write_column,
			append_points:inter_pcl_append_points,
			append_row:inter_pcl_append_row,
			append_column:inter_pcl_append_column,
			read_points:inter_pcl_read_points,
			read_row:inter_pcl_read_row,
			read_column:inter_pcl_read_column
		}
	},
	{
		file_suffix : "pcd",
		pnt_type: PNT_TYPE_POLAR,
		op : {
			open:inter_pcl_open,
			combine:inter_pcl_combine,
			on_data_change:NULL,
			close:inter_pcl_close,
			write_point:inter_pcl_write_point,
			write_row:inter_pcl_write_row,
			write_column:inter_pcl_write_column,
			append_points:inter_pcl_append_points,
			append_row:inter_pcl_append_row,
			append_column:inter_pcl_append_column,
			read_points:inter_pcl_read_points,
			read_row:inter_pcl_read_row,
			read_column:inter_pcl_read_column
		}
	},
	{
		file_suffix : "hls",
		pnt_type: PNT_TYPE_POLAR,
		op : {
			open:inter_hls_open,
			combine:inter_hls_combine,
			on_data_change:NULL,
			close:inter_hls_close,
			write_point:inter_hls_write_point,
			write_row:inter_hls_write_row,
			write_column:inter_hls_write_column,
			append_points:inter_hls_append_points,
			append_row:inter_hls_append_row,
			append_column:inter_hls_append_column,
			read_points:inter_hls_read_points,
			read_row:inter_hls_read_row,
			read_column:inter_hls_read_column
		}
	},
	{
		file_suffix : "jpg",
		pnt_type: PNT_TYPE_GRAY,
		op : {
			open:inter_jpg_open,
			combine:inter_jpg_combine,
			on_data_change:NULL,
			close:inter_jpg_close,
			write_point:inter_jpg_write_point,
			write_row:inter_jpg_write_row,
			write_column:inter_jpg_write_column,
			append_points:inter_jpg_append_points,
			append_row:inter_jpg_append_row,
			append_column:inter_jpg_append_column,
			read_points:inter_jpg_read_points,
			read_row:inter_jpg_read_row,
			read_column:inter_jpg_read_column
		}
	},
	{
		file_suffix : "gif",
		pnt_type: PNT_TYPE_GRAY,
		op : {
			open:inter_gif_open,
			combine:inter_gif_combine,
			on_data_change:NULL,
			close:inter_gif_close,
			write_point:inter_gif_write_point,
			write_row:inter_gif_write_row,
			write_column:inter_gif_write_column,
			append_points:inter_gif_append_points,
			append_row:inter_gif_append_row,
			append_column:inter_gif_append_column,
			read_points:inter_gif_read_points,
			read_row:inter_gif_read_row,
			read_column:inter_gif_read_column
		}
	},
	{
		file_suffix : "sprite",
		pnt_type: PNT_TYPE_GRAY,
		op : {
			open:inter_sprite_open,
			combine:inter_sprite_combine,
			on_data_change:inter_sprite_on_data_change,
			close:inter_sprite_close,
			write_point:inter_sprite_write_point,
			write_row:inter_sprite_write_row,
			write_column:inter_sprite_write_column,
			append_points:inter_sprite_append_points,
			append_row:inter_sprite_append_row,
			append_column:inter_sprite_append_column,
			read_points:inter_sprite_read_points,
			read_row:inter_sprite_read_row,
			read_column:inter_sprite_read_column
		}
	},
	{
		file_suffix : "memgray",
		pnt_type: PNT_TYPE_GRAY,
		op : {
			open:inter_memgray_open,
			combine:inter_memgray_combine,
			on_data_change:inter_memgray_on_data_change,
			close:inter_memgray_close,
			write_point:inter_memgray_write_point,
			write_row:inter_memgray_write_row,
			write_column:inter_memgray_write_column,
			append_points:inter_memgray_append_points,
			append_row:inter_memgray_append_row,
			append_column:inter_memgray_append_column,
			read_points:inter_memgray_read_points,
			read_row:inter_memgray_read_row,
			read_column:inter_memgray_read_column
		}
	},
};

const static int FILE_TYPE_NUM = sizeof(file_adpt) / sizeof(file_adapter_t);

/////////////////////////////////////////////////////////////////////////////////
//da
static char* get_file_suffix(char* name) {
	int len = strlen(name);
	char *p = name + len - 1;
	for (; p != name; p--) {
		if ((*p) == '.') {
			return p + 1;
		}
	}
	return NULL;
}

e_int32 da_open(data_adapter_t *da, e_uint8 *name, int width, int height,
		float h_w, int mode) {
	e_assert(strlen((char*) name) < MAX_PATH_LEN, E_ERROR);

	memset(da, 0, sizeof(data_adapter_t));

	char* file_type = NULL;
	int num;

	strcpy(da->file_info.file_name, (char*) name);

	da->file_info.width = width;
	da->file_info.height = height;
	da->file_info.h_w = h_w;
	da->file_info.mode = mode;

	file_type = get_file_suffix((char*) name);

	for (num = 0; num < FILE_TYPE_NUM; num++) {
		int ret = strncmp(file_type, file_adpt[num].file_suffix,
				strlen(file_adpt[num].file_suffix));
		if (!ret) {
			register_file_format(da, num);
			da->file_info.pnt_type = file_adpt[num].pnt_type;
			if (mode == E_READ) {
				da->file.read.info = da->file_info;
				ret = da->op.open(&da->file.read);
				e_assert(ret>0, ret);
				da->file_info = da->file.read.info;
			} else if (E_DWRITE == mode) {
				da->file.write.left.info = da->file_info;
				da->file.write.left.is_main = 1;
				da->file.write.right.info = da->file_info;
				strcat(da->file.write.right.info.file_name, ".tmp");
				ret = da->op.open(&da->file.write.left);
				e_assert(ret>0, ret);
				ret = da->op.open(&da->file.write.right);
				e_assert(ret>0, ret);
			} else if (E_WRITE == mode) {
				da->file.write.left.info = da->file_info;
				da->file.write.left.is_main = 1;
				ret = da->op.open(&da->file.write.left);
				e_assert(ret>0, ret);
			}
			break;
		}
	}
	if (num == FILE_TYPE_NUM)
		return E_ERROR;

	da->state = 1;
	return E_OK;
}

e_int32 da_close(data_adapter_t *da, int save) {
	e_int32 ret;
	e_assert(da && da->state, E_ERROR_INVALID_HANDLER);
	if (da->file_info.mode == E_READ) {
		ret = da->op.close(&da->file.read, 0);
	} else if (da->file_info.mode == E_DWRITE) {
		if (save && da->op.combine) {
			ret = da->op.combine(&da->file.write.left, &da->file.write.right);
			e_assert(ret > 0, ret);
		}
		da->file.write.left.info.mode = E_WRITE;
		ret = da->op.close(&da->file.write.right, save);
		e_assert(ret > 0, ret);
		ret = da->op.close(&da->file.write.left, save);
	} else if (da->file_info.mode == E_WRITE) {
		ret = da->op.close(&da->file.write.left, save);
	}
	e_assert(ret > 0, ret);
	da->state = 0;
	return E_OK;
}

e_int32 da_read_points(data_adapter_t *da, point_t* point, int buf_len) {
	e_assert(
			da && da->state && da->pnt_type == point->type && da->op.read_points && da->file_info.mode == E_READ,
			E_ERROR_INVALID_HANDLER);
	da->op.read_points(&da->file.read, point, buf_len);
	return E_OK;
}

e_int32 da_append_points(data_adapter_t *da, point_t* point, int pt_num,
		int file_right) {
	e_int32 ret;
	e_assert(
			da && da->state && da->pnt_type == point->type && da->op.append_points && point,
			E_ERROR_INVALID_HANDLER);
	file_ptr_t * file;

	if (!file_right || da->file_info.mode == E_WRITE) {
		e_assert(da->file_info.mode==E_WRITE || da->file_info.mode==E_DWRITE,
				E_ERROR);
		file = &da->file.write.left;
	} else {
		e_assert(da->file_info.mode==E_DWRITE, E_ERROR);
		file = &da->file.write.right;
	}

	file->modify = 1;
	e_assert(file->cousor + pt_num <= file->info.width * file->info.height,
			E_ERROR);
	ret = da->op.append_points(file, point, pt_num);

	if (da->op.on_data_change)
		da->op.on_data_change(&da->file.write.left, &da->file.write.right);

	return ret;
}

e_int32 da_append_row(data_adapter_t *da, point_t* point, int file_right) {
	e_int32 ret;
	e_assert(
			da && da->state && da->pnt_type == point->type && da->op.append_row && point,
			E_ERROR_INVALID_HANDLER);
	file_ptr_t * file;

	if (!file_right || da->file_info.mode == E_WRITE) {
		e_assert(da->file_info.mode==E_WRITE || da->file_info.mode==E_DWRITE,
				E_ERROR);
		file = &da->file.write.left;
	} else {
		e_assert(da->file_info.mode==E_DWRITE, E_ERROR);
		file = &da->file.write.right;
	}

	file->modify = 1;
	e_assert(
			file->cousor + file->info.width <= file->info.width * file->info.height,
			E_ERROR);
	ret = da->op.append_row(file, point);

	if (da->op.on_data_change)
		da->op.on_data_change(&da->file.write.left, &da->file.write.right);

	return ret;
}

e_int32 da_write_row(data_adapter_t *da, e_uint32 row_idx, point_t* point,
		int file_right) {
	e_int32 ret;
	e_assert(
			da && da->state && da->pnt_type == point->type && da->op.write_row && point,
			E_ERROR_INVALID_HANDLER);
	file_ptr_t * file;

	if (!file_right || da->file_info.mode == E_WRITE) {
		e_assert(da->file_info.mode==E_WRITE || da->file_info.mode==E_DWRITE,
				E_ERROR);
		file = &da->file.write.left;
	} else {
		e_assert(da->file_info.mode==E_DWRITE, E_ERROR);
		file = &da->file.write.right;
	}

	file->modify = 1;
	e_assert(row_idx < file->info.height, E_ERROR);
	ret = da->op.write_row(file, row_idx, point);

	if (da->op.on_data_change)
		da->op.on_data_change(&da->file.write.left, &da->file.write.right);

	return ret;
}

e_int32 da_write_column(data_adapter_t *da, e_uint32 column_idx, point_t* point,
		int file_right) {
	e_int32 ret;
	e_assert(
			da && da->state&& da->pnt_type == point->type && da->op.write_column && point,
			E_ERROR_INVALID_HANDLER);
	file_ptr_t * file;

	if (!file_right || da->file_info.mode == E_WRITE) {
		e_assert(da->file_info.mode==E_WRITE || da->file_info.mode==E_DWRITE,
				E_ERROR);
		file = &da->file.write.left;
	} else {
		e_assert(da->file_info.mode==E_DWRITE, E_ERROR);
		file = &da->file.write.right;
	}

	file->modify = 1;
	e_assert(column_idx < file->info.width, E_ERROR);
	ret = da->op.write_column(file, column_idx, point);

	if (da->op.on_data_change)
		da->op.on_data_change(&da->file.write.left, &da->file.write.right);

	return ret;
}

e_int32 da_write_point(data_adapter_t *da, int x, int y, point_t* point,
		int file_right) {
	return E_ERROR;
}

e_int32 da_read_row(data_adapter_t *da, e_uint32 row_idx, point_t* buf) {
	return E_ERROR;
}

e_int32 da_read_column(data_adapter_t *da, e_uint32 column_idx, point_t* buf) {
	return E_ERROR;
}

e_int32 da_append_column(data_adapter_t *da, point_t* point, int file_right) {
	return E_ERROR;
}
e_int32 da_read_header(data_adapter_t *da, file_info_t *header) {
	return E_ERROR;
}

static e_int32 register_file_format(data_adapter_t *da, int i) {
	e_assert(file_adpt[i].op.open && file_adpt[i].op.close, E_ERROR);
	da->op = file_adpt[i].op;
	da->pnt_type = file_adpt[i].pnt_type;
	return E_OK;
}
//
//////////////////////////////////////////////////////////////////////////////////
//PCL
#if LINUX
#define HAS_LIBPCL 1
#endif

#if HAS_LIBPCL
static e_int32 inter_pcl_open(file_ptr_t *file)
{
	if ( file->info.mode == E_READ ) {
		pcl::PointCloud<pcl::PointXYZ> * cloud = new pcl::PointCloud<pcl::PointXYZ>();
		if ( pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1 ) //打开点云文件
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
			return (-1);
		}
		file->handle = (size_t) cloud;
		file->info.width = cloud->width;
		file->info.height = cloud->height;
	}
	else
	{
		//打开缓存
		pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
		// 创建点云缓存
		cloud->width = file->info.width;
		cloud->height = file->info.height;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		file->handle = (size_t) cloud;
	}

	return E_OK;
}
static e_int32 inter_pcl_combine(file_ptr_t *file1, file_ptr_t *file2)
{
	pcl::PointCloud<pcl::PointXYZ> *cloud1 =
	(pcl::PointCloud<pcl::PointXYZ> *) (pcl::PointCloud<pcl::PointXYZ> *) file1->handle;
	pcl::PointCloud<pcl::PointXYZ> *cloud2 =
	(pcl::PointCloud<pcl::PointXYZ> *) (pcl::PointCloud<pcl::PointXYZ> *) file2->handle;

	(*cloud1) += (*cloud2);

	file2->info.mode = -1; //不需保存
	return E_OK;
}
static e_int32 inter_pcl_close(file_ptr_t *file, int save) {
	pcl::PointCloud<pcl::PointXYZ> *cloud =
	(pcl::PointCloud<pcl::PointXYZ> *) file->handle;
	if ( save && file->info.mode == E_WRITE ) {
		pcl::io::savePCDFileASCII(file->info.file_name, *cloud);
	}
	delete cloud;
	return E_OK;
}
static e_int32 inter_pcl_write_point(file_ptr_t *file, int x, int y, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_write_row(file_ptr_t *file, e_uint32 row_idx, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point) {
	return E_ERROR;
}

static point_xyz_t* transfor2xyz(point_t* pnts, int i, point_xyz_t *xyz) {
	switch (pnts->type) {
		case PNT_TYPE_XYZ:
		return ((point_xyz_t*) pnts->mem) + i;
		case PNT_TYPE_POLAR:
		{
			point_polar_t *polar = (point_polar_t*) pnts->mem;
			hd_polar2xyz(&xyz->x, &xyz->y, &xyz->z,
					polar[i].distance, polar[i].angle_h, polar[i].angle_v);
			return xyz;
		}
		default:
		DMSG((STDOUT,"transfor2xyz ERROR \n"));
		return NULL;
	}
}

static e_int32 inter_pcl_append_points(file_ptr_t *file, point_t* pnts, int pt_num) {
	pcl::PointCloud<pcl::PointXYZ> *cloud =
	(pcl::PointCloud<pcl::PointXYZ> *) (pcl::PointCloud<pcl::PointXYZ> *) file->handle;
	int idx = 0;
	point_xyz_t xyz, *pxyz;

	while (pt_num--)
	{
		pxyz = transfor2xyz(pnts, idx, &xyz);
		cloud->points[file->cousor].x = pxyz->x;
		cloud->points[file->cousor].y = pxyz->y;
		cloud->points[file->cousor].z = pxyz->z;
		file->cousor++;
		idx++;
	}
	return E_OK;

}
static e_int32 inter_pcl_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_pcl_read_points(file_ptr_t *file, point_t* pnts, int buf_len) {
	pcl::PointCloud<pcl::PointXYZ> *cloud =
	(pcl::PointCloud<pcl::PointXYZ> *) file->handle;
	point_xyz_t *point = (point_xyz_t*) pnts->mem;
	while (buf_len--)
	{
		point->x = cloud->points[file->cousor].x;
		point->y = cloud->points[file->cousor].y;
		point->z = cloud->points[file->cousor].z;
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_pcl_read_row(file_ptr_t *file, e_uint32 row_idx, point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_pcl_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}

#else

static e_int32 inter_pcl_open(file_ptr_t *file) {
	e_int32 ret;
	if (file->info.mode == E_READ) {
		file_t *f = (file_t *) malloc(sizeof(file_t));
		ret = fi_open(file->info.file_name, F_READ, f);
		e_assert(ret, E_ERROR_INVALID_HANDLER);
		file->handle = (size_t) f;
		fscanf((FILE*) f->fpid, "%20u,%20u\n", &file->info.width,
				&file->info.height);
	} else {
		file_t *f = (file_t *) malloc(sizeof(file_t));
		ret = fi_open(file->info.file_name, F_READ | F_WRITE | F_CREATE, f);
		e_assert(ret, E_ERROR_INVALID_HANDLER);

		if (setvbuf((FILE*) f->fpid, NULL, _IOFBF, 65536) != 0) {
			fprintf(stderr, "WARNING: setvbuf() failed with buffer size %u\n",
					65536);
		}

		file->handle = (size_t) f;
		fprintf((FILE*) f->fpid, "%20u,%20u\n", file->info.width,
				file->info.height);
	}

	return E_OK;
}
static e_int32 inter_pcl_combine(file_ptr_t *file1, file_ptr_t *file2) {
	FILE* f1, *f2;
	unsigned int count, w, h, done;
	int buf_size = 1024 * 1024;
	char *buf = (char*) malloc(buf_size);
	e_assert(buf, E_ERROR_INVALID_CALL);

	f1 = (FILE*) ((file_t*) file1->handle)->fpid;
	f2 = (FILE*) ((file_t*) file2->handle)->fpid;
	rewind(f1);
	rewind(f2);
	w = file1->info.width + file2->info.width;
	h = file2->info.height + file2->info.height;

	//写头
	fprintf(f1, "%20u,%20u\n", w, h);
	//合并后面部分
	fseek(f2, 40, SEEK_SET);
	fseek(f1, 0, SEEK_END);
	while ((count = fread(buf, 1, buf_size, f2)) != 0) {
		done = fwrite(buf, 1, count, f1);
		e_assert(done == count, E_ERROR_INVALID_CALL);
	}

	free(buf);
	file2->info.mode = -1; //不需保存
	return E_OK;
}
static e_int32 inter_pcl_close(file_ptr_t *file, int save) {
	file_t* fd = (file_t*) file->handle;
	FILE* f1 = (FILE*) fd->fpid;

	DMSG((STDOUT,"inter_pcl_close try close:%s",file->info.file_name));

	switch (file->info.mode) {
	case E_READ:
	case E_WRITE:
	case E_DWRITE:
		fclose(f1);
		DMSG((STDOUT,"inter_pcl_close save file:%s",file->info.file_name));
		break;
	case -1:
		fclose(f1);
		fi_delete(file->info.file_name);
		DMSG(
				(STDOUT,"inter_pcl_close delete tmp file:%s",file->info.file_name));
		break;
	default:
		DMSG((STDOUT,"ERROR file.info.mode"));
	}

	free(fd);
	return E_OK;
}
static e_int32 inter_pcl_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point) {
	return E_ERROR;
}

static point_xyz_t* transfor2xyz(point_t* pnts, int i, point_xyz_t *xyz) {
	point_polar_t *polar = (point_polar_t*) pnts->mem;
	switch (pnts->type) {
	case PNT_TYPE_XYZ:
		return ((point_xyz_t*) pnts->mem) + i;
	case PNT_TYPE_POLAR:
		hd_polar2xyz(&xyz->x, &xyz->y, &xyz->z, polar[i].distance,
				polar[i].angle_h, polar[i].angle_v);
		return xyz;
	default:
		DMSG((STDOUT,"transfor2xyz ERROR \n"));
		return NULL;
	}
}

static e_int32 inter_pcl_append_points(file_ptr_t *file, point_t* pnts,
		int pt_num) {
	int idx = 0;
	point_xyz_t xyz, *pxyz;
	file_t* fd = (file_t*) file->handle;
	FILE* f1 = (FILE*) fd->fpid;

	while (pt_num--) {
		pxyz = transfor2xyz(pnts, idx, &xyz);
		fprintf(f1, "%.10g,%.10g,%.10g\n", pxyz->x, pxyz->y, pxyz->z);
		file->cousor++;
		idx++;
	}
	return E_OK;

}
static e_int32 inter_pcl_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_pcl_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_pcl_read_points(file_ptr_t *file, point_t* pnts,
		int buf_len) {
	point_xyz_t *point = (point_xyz_t*) pnts->mem;
	file_t* fd = (file_t*) file->handle;
	FILE* f1 = (FILE*) fd->fpid;

	while (buf_len--) {
		fscanf(f1, "%0.10g,%0.10g,%0.10g\n", &point->x, &point->y, &point->z);
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_pcl_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_pcl_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}

#endif

//HLS

static void create_guid(int *num1, short *num2, short *num3, long long* num4) {
	srand(GetTickCount());
	(*num1) = rand();
	(*num2) = rand() & 0xFFFF;
	(*num3) = rand() & 0xFFFF;
	(*num4) = (((long long) rand()) & 0xFFFF) << 16 | (rand() & 0xFFFF);
}

static hd::HLSWriter* create_hlswriter(char* fname) {
	hd::HLSWriter * writer = new hd::HLSWriter();
	hd::HLSheader hlsHeader;
	strcpy(hlsHeader.file_signature, "HLSF");
	int num1 = 0;
	short num2 = 0;
	short num3 = 0;
	long long num4 = 0;
	create_guid(&num1, &num2, &num3, &num4); //仍然挂的话就把这提出到线程外部
	hlsHeader.project_ID_GUID_data_1 = num1;
	hlsHeader.project_ID_GUID_data_2 = num2;
	hlsHeader.project_ID_GUID_data_3 = num3;
	memcpy(hlsHeader.project_ID_GUID_data_4, &num4, sizeof(num4));

	hlsHeader.file_source_id = 0;
	hlsHeader.version_major = 1;
	hlsHeader.version_minor = 0;
	strcpy(hlsHeader.system_identifier, "HD 3LS 001");
	strcpy(hlsHeader.generating_software, "HD-3LS-SCAN");
	//在文件头中写入倾角信息
	hlsHeader.yaw = 0.0f; //扫描仪航向
	hlsHeader.pitch = 0.0f; //dY;//扫描仪前后翻滚角
	hlsHeader.roll = 0.0f; //dX;//扫描仪左右翻滚角

	system_time_t sys_time;
	GetLocalTime(&sys_time);
	hlsHeader.file_creation_day = sys_time.day;
	hlsHeader.file_creation_year = sys_time.year;
	if (!writer->create(fname, &hlsHeader)) {
		delete writer;
		return NULL;
	}
	return writer;
}

static e_int32 inter_hls_open(file_ptr_t *file) {
	if (file->info.mode == E_READ) {
//		file->handle = (size_t) NULL;
//		file->info.width = cloud->width;
//		file->info.height = cloud->height;
	} else {
		//打开缓存
		hd::HLSWriter * writer = create_hlswriter(file->info.file_name);
		if (!writer)
			return E_ERROR;
		// 创建点云缓存;
		file->handle = (size_t) writer;
	}

	return E_OK;
}
static e_int32 inter_hls_combine(file_ptr_t *file1, file_ptr_t *file2) {
//	hls::PointCloud<hls::PointXYZ> *cloud1 =
//			(hls::PointCloud<hls::PointXYZ> *) (hls::PointCloud<hls::PointXYZ> *) file1->handle;
//	hls::PointCloud<hls::PointXYZ> *cloud2 =
//			(hls::PointCloud<hls::PointXYZ> *) (hls::PointCloud<hls::PointXYZ> *) file2->handle;
//
//	(*cloud1) += (*cloud2);
//
//	file2->info.mode = -1; //不需保存
	return E_OK;
}
static e_int32 inter_hls_close(file_ptr_t *file, int save) {
	hd::HLSWriter * writer = (hd::HLSWriter *) file->handle;

	if (save && file->info.mode == E_WRITE) {
		writer->close(file->info.width * file->info.height, file->info.width,
				file->info.height);
	}

	delete writer;
	return E_OK;
}
static e_int32 inter_hls_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_hls_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_hls_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point) {
	return E_ERROR;
}

static e_int32 inter_hls_append_points(file_ptr_t *file, point_t* pnts,
		int pt_num) {
	int i, type;
	hd::HLSWriter * writer = (hd::HLSWriter *) file->handle;
	point_polar_t *polar = (point_polar_t*) pnts->mem;

	type = file->is_main ? 0 : 1;
	for (i = 0; i < pt_num; i++) {
		writer->write_point(polar[i].distance, polar[i].angle_h,
				polar[i].angle_v, polar[i].intensity, type);
		file->cousor++;
	}

	return E_OK;
}
static e_int32 inter_hls_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_hls_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_hls_read_points(file_ptr_t *file, point_t* pnts,
		int buf_len) {
	return E_ERROR;
}
static e_int32 inter_hls_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_hls_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}

////////////////////////////////////////////////////////////////////////////////
//JPG
static e_int32 inter_jpg_open(file_ptr_t *file) {
	if (file->info.mode == E_READ) {
//		pcl::PointCloud < pcl::PointXYZ > *cloud = new pcl::PointCloud<pcl::PointXYZ>();
//		if (pcl::io::loadPCDFile < pcl::PointXYZ > ("test_pcd.pcd", *cloud) == -1) //打开点云文件
//				{
//			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
//			return (-1);
//		}
//		file->handle = (size_t) cloud;
//		file->info.width = cloud->width;
//		file->info.height = cloud->height;
		return E_ERROR;
	} else {
		//打开缓存
		e_uint8 *jpg = (e_uint8 *) malloc(file->info.width * file->info.height);
		file->handle = (size_t) jpg;
	}

	return E_OK;
}
static e_int32 inter_jpg_combine(file_ptr_t *file1, file_ptr_t *file2) {
	unsigned int width, height;
	e_uint8 *jpg1 = (e_uint8 *) file1->handle;
	e_uint8 *jpg2 = (e_uint8 *) file2->handle;
#if 0
//TODO:支持多种合并,等高合并,按行合并,按列合并
	e_assert(file1->info.width == file2->info.width, E_ERROR_INVALID_PARAMETER);

	width = file1->info.width;
	height = file1->info.height+file2->info.height;

	e_uint8 *jpg = (e_uint8 *) malloc(width * height);
	e_uint8* output=jpg;
	memcpy(jpg, jpg1, file1->info.width * file1->info.height);
	memcpy(jpg + file1->info.width * file1->info.height, jpg2,
			file2->info.width * file2->info.height);
#else
	e_assert(file1->info.height == file2->info.height,
			E_ERROR_INVALID_PARAMETER);

	width = file1->info.width + file2->info.width;
	height = file1->info.height;
	e_uint8 *jpg = (e_uint8 *) malloc(width * height);
	e_uint8* output = jpg;
	for (unsigned int i = 0; i < height; i++) {
		memcpy(jpg, jpg1, file1->info.width);
		jpg += file1->info.width;
		jpg1 += file1->info.width;
		memcpy(jpg, jpg2, file2->info.width);
		jpg += file2->info.width;
		jpg2 += file2->info.width;
	}
#endif

//调整合并后的高宽
	file1->info.width = width;
	file1->info.height = height;
	free((void*) file1->handle);
	file1->handle = (size_t) output;

	file2->info.mode = -1; //不需保存
	return E_OK;
}


static e_int32 inter_jpg_close(file_ptr_t *file, int save) {
	e_uint8 *jpg = (e_uint8 *) file->handle;
	DMSG((STDOUT,"inter_jpg_close try save file:%s",file->info.file_name));
	if (save && (file->info.mode == E_WRITE || E_DWRITE == file->info.mode)) {
		gray_to_jpeg_file(file->info.file_name, jpg, file->info.width,
				file->info.height);
		DMSG((STDOUT,"inter_jpg_close save file done:%s",file->info.file_name));
	}
	free(jpg);
	return E_OK;
}
static e_int32 inter_jpg_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_jpg_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_jpg_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* pnts) {
	unsigned int i;
	e_uint8 *jpg = (e_uint8 *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	jpg += column_idx;

	for (i = 0; i < file->info.height; i++) {
		(*jpg) = point->gray;
		jpg += file->info.width;
		point++;
	}
	return E_OK;
}
static e_int32 inter_jpg_append_points(file_ptr_t *file, point_t* pnts,
		int pt_num) {
	e_uint8 *jpg = (e_uint8 *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (pt_num--) {
		jpg[file->cousor] = point->gray;
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_jpg_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_jpg_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_jpg_read_points(file_ptr_t *file, point_t* pnts,
		int buf_len) {
	e_uint8 *jpg = (e_uint8 *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (buf_len--) {
		point->gray = jpg[file->cousor];
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_jpg_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_jpg_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}

////////////////////////////////////////////////////////////////////////////////
//JPG
typedef struct jpg_private_t {
	gif_t gif;
	e_uint8 *buf;
} jpg_private_t;
static e_int32 inter_gif_open(file_ptr_t *file) {
	e_int32 ret;
	if (file->info.mode == E_READ) {
//		pcl::PointCloud < pcl::PointXYZ > *cloud = new pcl::PointCloud<pcl::PointXYZ>();
//		if (pcl::io::loadPCDFile < pcl::PointXYZ > ("test_pcd.pcd", *cloud) == -1) //打开点云文件
//				{
//			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
//			return (-1);
//		}
//		file->handle = (size_t) cloud;
//		file->info.width = cloud->width;
//		file->info.height = cloud->height;
		return E_ERROR;
	} else {
		//打开文件
		jpg_private_t *gif = (jpg_private_t *) malloc(sizeof(jpg_private_t));
		gif->buf = (e_uint8*) calloc(file->info.width, file->info.height);
		ret = gif_open(&gif->gif, file->info.file_name, file->info.width,
				file->info.height, E_WRITE);
		e_assert(ret > 0, ret);

		ret = gif_put_header(&gif->gif);
		if (e_failed(ret)) {
			gif_close(&gif->gif);
			return ret;
		}

		file->handle = (size_t) gif;
	}

	return E_OK;
}

static e_int32 inter_gif_combine(file_ptr_t *file1, file_ptr_t *file2) {
	int ret;
	jpg_private_t *gif1 = (jpg_private_t *) file1->handle;
	jpg_private_t *gif2 = (jpg_private_t *) file2->handle;

//TODO:支持多种合并,等高合并,按行合并,按列合并
	e_assert(
			file1->info.width == file2->info.width && file1->info.height == file2->info.height,
			E_ERROR_INVALID_PARAMETER);

#if 0
	//合并
	gif_append(&gif1->gif, &gif2->gif);
	//干掉gif2
	fi_delete((char*) gif2->gif.file_name);

#else
	//打开文件
	jpg_private_t *gif = (jpg_private_t *) malloc(sizeof(jpg_private_t));
	gif->buf = (e_uint8*) calloc(file1->info.width + file2->info.width,
			file1->info.height);

	ret = gif_union(&gif->gif, &gif1->gif, &gif2->gif);
	e_assert(ret > 0, ret);
	gif_close(&gif1->gif);
	fi_delete((char*) gif1->gif.file_name);
	fi_delete((char*) gif2->gif.file_name);
	free(gif1->buf);
	free(gif1);
	file1->handle = (size_t) gif;
#endif

	file2->info.mode = -1; //不需保存
	return E_OK;
}

static e_int32 inter_gif_close(file_ptr_t *file, int save) {
	int ret = E_OK;
	jpg_private_t *gif = (jpg_private_t *) file->handle;
	gif_close(&gif->gif);

	if (strcmp((char*) gif->gif.file_name, (char*) file->info.file_name)) {
		fi_delete((char*) file->info.file_name);
		ret = fi_rename((char*) file->info.file_name,
				(char*) gif->gif.file_name);
		if (!ret)
			ret = E_ERROR;
		else
			ret = E_OK;
	}

	free(gif->buf);
	free(gif);
	return ret;
}
static e_int32 inter_gif_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}

static e_int32 _insert_frame(file_ptr_t *file) {
	int ret;
	e_uint32 j;
	e_uint8 *pbuf;
	jpg_private_t *gif = (jpg_private_t *) file->handle;

	ret = gif_put_image(&gif->gif); //产生帧
	e_assert(ret > 0, ret);

	for (pbuf = gif->buf, j = 0; j < file->info.height;
			j++, pbuf += file->info.width) {
		gif_put_scan_line(&gif->gif, pbuf);
		e_assert(ret > 0, ret);
	}
	return E_OK;
}

static e_int32 inter_gif_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* pnts) {
	int ret;
	jpg_private_t *gif = (jpg_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;

	//add one row
	for (unsigned int i = 0; i < file->info.width; i++) {
		gif->buf[row_idx * file->info.width + i] = point->gray;
		point++;
		file->cousor++;
	}

	ret = _insert_frame(file);
	e_assert(ret > 0, ret);

	return E_OK;
}
static e_int32 inter_gif_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* pnts) {
	e_int32 ret;
	jpg_private_t *gif = (jpg_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	e_uint8* pbuf = gif->buf + column_idx;

	//add one row
	for (unsigned int i = 0; i < file->info.height; i++) {
		(*pbuf) = point->gray;
		pbuf += file->info.width;
		point++;
	}

	ret = _insert_frame(file);
	e_assert(ret > 0, ret);
	return E_OK;
}
static e_int32 inter_gif_append_points(file_ptr_t *file, point_t* point,
		int pt_num) {
	e_uint8 *gif = (e_uint8 *) file->handle;
	return E_ERROR;
}
static e_int32 inter_gif_append_row(file_ptr_t *file, point_t* pnts) {
	int ret;
	jpg_private_t *gif = (jpg_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	//add one row
	for (unsigned int i = 0; i < file->info.width; i++) {
		gif->buf[file->cousor++] = point->gray;
		point++;

	}
	ret = _insert_frame(file);
	e_assert(ret > 0, ret);

	return E_OK;
}
static e_int32 inter_gif_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_gif_read_points(file_ptr_t *file, point_t* point,
		int buf_len) {
//	e_uint8 *gif = (e_uint8 *) file->handle;
	return E_ERROR;
}
static e_int32 inter_gif_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_gif_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}

////////////////////////////////////////////////////////////////////////////////
//SPRITE
// server open -> write -> sendframe
// client readframe

typedef struct sprite_private_t {
	hd_connect_t connect;
	e_uint8 *buf;
	e_uint32 size;
} sprite_private_t;
static e_int32 inter_sprite_open(file_ptr_t *file) {
	int ret, size, h, w;
	float h_w;
	e_uint8 buf[64];
	//打开缓存
	sprite_private_t *sprite = (sprite_private_t *) calloc(
			sizeof(sprite_private_t), 1);
	size = file->info.width * file->info.height;
	if (file->is_main) {
		//ret = sc_open_pipe(&sprite->connect, file->info.file_name, size);
		ret = sc_open_socket(&sprite->connect, file->info.file_name, 0,
				E_SOCKET_NAME);
		if (e_failed(ret)) {
			sc_close(&sprite->connect);
			free(sprite);
			return ret;
		}
		w = file->info.mode == E_DWRITE ?
				file->info.width * 2 : file->info.width;
		h = file->info.height;

		//发送长宽信息
//		h_w = h / (float) w;
		h_w = file->info.h_w;
		sprintf((char*) buf, "ABCD%05d%05d%08.2fEFGH", w, h, h_w);
		ret = sc_try_connect(&sprite->connect, 2);
		if (e_failed(ret)) {
			DMSG((STDOUT,"Name:%s",file->info.file_name));
			sc_close(&sprite->connect);
			free(sprite);
			return ret;
		}

		ret = sc_select(&sprite->connect, E_WRITE, 100000);
		if (e_failed(ret)) {
			sc_close(&sprite->connect);
			free(sprite);
			return ret;
		}

		ret = sc_send(&sprite->connect, buf, 26);
		if (e_failed(ret)) {
			sc_close(&sprite->connect);
			free(sprite);
			return ret;
		}
		DMSG((STDOUT,"Success to create sprite.\n"));
	}

	if (file->is_main && file->info.mode == E_DWRITE) //主文件用于文件合并
			{
		sprite->buf = (e_uint8 *) calloc(size, 2);
	} else {
		sprite->buf = (e_uint8 *) calloc(size, 1);
	}

	sprite->size = size;
	file->handle = (size_t) sprite;
	return E_OK;
}

static void printfv(char* buf, int len) {
	char c, l = '0';
	int count = 0;

	while (len--) {
		c = *buf++;
		if (c == l) {
			count++;
		} else {
			if (count > 0)
				printf(" %c[%d]", l == 0 ? '0' : l, count);
			l = c;
			count = 1;
		}
	}
	if (count > 0)
		printf(" %c[%d]", l == 0 ? '0' : l, count);
}

static void inter_sprite_text(e_uint8 *buf, int w, int h) {
	for (int i = 0; i < h; i++) {
		printfv((char*) buf, w);
		printf("\n");
		buf += w;
	}
}

static e_int32 inter_sprite_combine(file_ptr_t *file1, file_ptr_t *file2) {
	file_ptr_t *main_file = NULL;
	e_uint8 *src, *dst;
	unsigned int width, height;
	sprite_private_t *main_sprite, *sub_sprite;

	if (file1->is_main) {
		main_file = file1;
	} else if (file2->is_main) {
		main_file = file2;
		file2 = file1;
	}

	if (main_file->info.mode != E_DWRITE)
		return E_OK;
	e_assert(
			file1->info.width == file2->info.width && file1->info.height == file1->info.height,
			E_ERROR);

	if (!file2->modify)
		return E_OK;

	main_sprite = (sprite_private_t *) main_file->handle;
	sub_sprite = (sprite_private_t *) file2->handle;
	width = main_file->info.width;
	height = main_file->info.height;

//	DMSG((STDOUT,"\nmain_sprite:\n"));
//	inter_sprite_text(main_sprite->buf, width * 2, height);
//	DMSG((STDOUT,"\nsub_sprite:\n"));
//	inter_sprite_text(sub_sprite->buf, width, height);

	e_assert(main_file, E_ERROR);
	dst = main_sprite->buf + main_file->info.width;
	src = sub_sprite->buf;
	for (unsigned int i = 0; i < height; i++) {
		memcpy(dst, src, width);
		dst += width * 2;
		src += width;
	}

//	DMSG((STDOUT,"\nresult_sprite:\n"));
//	inter_sprite_text(main_sprite->buf, width * 2, height);

	file2->modify = 0;
	return E_OK;
}

static e_int32 inter_sprite_close(file_ptr_t *file, int save) {
	sprite_private_t *sprite = (sprite_private_t *) file->handle;
	if (file->is_main)
		sc_close(&sprite->connect);
	free(sprite->buf);
	free(sprite);
	return E_OK;
}

static int Check_FPS() {
	static int last_time = 0;
	float fps = 0;
	int time = GetTickCount(); //us
	if (last_time)
		fps = 1e6 / (time - last_time);

	if (fps > 1.0f)
		return 0;

	last_time = time;
	return 1;
}

static e_int32 _sprite_insert_frame(file_ptr_t *file) {
	int ret;
	sprite_private_t *sprite = (sprite_private_t *) file->handle;
	e_assert(file->is_main, E_ERROR);

	if (!Check_FPS()) {
//		DMSG((STDOUT,"FPS 过高，放弃更新。!\n"));
		return 1;
	}

	ret = sc_select(&sprite->connect, E_WRITE, 1E6);
	e_assert(ret > 0, ret);

	if (file->info.mode != E_DWRITE)
		ret = sc_send(&sprite->connect, sprite->buf, sprite->size);
	else
		ret = sc_send(&sprite->connect, sprite->buf, sprite->size * 2);
	e_assert(ret > 0, ret);
	return E_OK;
}
static e_int32 inter_sprite_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_sprite_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_sprite_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* pnts) {
	e_uint8 *pbuf;
	sprite_private_t *sprite = (sprite_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	e_assert(file->info.width > column_idx, E_ERROR);
	pbuf = &sprite->buf[column_idx];
	for (unsigned int i = 0; i < file->info.height; i++) {
		(*pbuf) = point->gray;
		pbuf += file->info.width;
		if (file->is_main && file->info.mode == E_DWRITE)
			pbuf += file->info.width;
		point++;
	}
	return E_OK;
}
static e_int32 inter_sprite_append_points(file_ptr_t *file, point_t* pnts,
		int pt_num) {
	sprite_private_t *sprite = (sprite_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (pt_num--) {
		sprite->buf[file->cousor] = point->gray;
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_sprite_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_sprite_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_sprite_read_points(file_ptr_t *file, point_t* pnts,
		int buf_len) {
	e_uint8 *sprite = (e_uint8 *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (buf_len--) {
		point->gray = sprite[file->cousor];
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_sprite_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_sprite_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_sprite_on_data_change(file_ptr_t *file1,
		file_ptr_t *file2) {
	e_int32 ret;
	ret = inter_sprite_combine(file1, file2);
	e_assert(ret>0, ret);

	if (file1->is_main) {
		ret = _sprite_insert_frame(file1);
	} else {
		ret = _sprite_insert_frame(file2);
	}
	e_assert(ret>0, ret);
	return E_OK;
}
///////////////////////////////////////////////////////////////////////////////
//memgray
#include <comm/hd_utils.h>

display_t display = { 0 };

typedef struct memgray_private_t {
	e_uint32 size;
	e_uint8 * buf;
} memgray_private_t;

extern display_t display;

static e_int32 inter_memgray_open(file_ptr_t *file) {
	//打开缓存
	memgray_private_t *memgray = (memgray_private_t *) calloc(
			sizeof(memgray_private_t), 1);
	int size = file->info.width * file->info.height;

	if (file->is_main) {
		display.w =
				file->info.mode == E_DWRITE ?
						file->info.width * 2 : file->info.width;
		display.h = file->info.height;
		DMSG((STDOUT,"Success to create memgray.\n"));
	}

	if (file->is_main && file->info.mode == E_DWRITE) //主文件用于文件合并
			{
		display.buf = (e_uint8 *) calloc(size, 2);
	}

	display.h_w = file->info.h_w;

	memgray->buf = (e_uint8 *) calloc(size, 1);
	memgray->size = size;
	file->handle = (size_t) memgray;
	return E_OK;
}

static e_int32 inter_memgray_combine(file_ptr_t *file1, file_ptr_t *file2) {
	file_ptr_t *main_file = NULL;
	e_uint8 *src1, *src2, *dst;
	unsigned int width, height;
	memgray_private_t *main_memgray, *sub_memgray;

	e_assert(display.buf, E_ERROR);

	if (file1->is_main) {
		main_file = file1;
	} else if (file2->is_main) {
		main_file = file2;
		file2 = file1;
	}

	if (main_file->info.mode != E_DWRITE)
		return E_OK;
	e_assert(
			file1->info.width == file2->info.width && file1->info.height == file1->info.height,
			E_ERROR);

	if (!file2->modify)
		return E_OK;

	main_memgray = (memgray_private_t *) main_file->handle;
	sub_memgray = (memgray_private_t *) file2->handle;
	width = main_file->info.width;
	height = main_file->info.height;

	e_assert(main_file, E_ERROR);
	dst = display.buf;
	src1 = main_memgray->buf;
	src2 = sub_memgray->buf;
	for (unsigned int i = 0; i < height; i++) {
		for (unsigned int j = 0; j < width; j++) {
			(*dst++) = (*src1++);
		}
		for (unsigned int j = 0; j < width; j++) {
			(*dst++) = (*src2++);
		}
	}

	display.hash++;

	file2->modify = 0;
	return E_OK;
}

static e_int32 inter_memgray_close(file_ptr_t *file, int save) {
	memgray_private_t *memgray = (memgray_private_t *) file->handle;

	if (file->is_main) {
		free(display.buf);
		display.buf = NULL;
	}
	free(memgray->buf);
	free(memgray);
	return E_OK;
}

static e_int32 _memgray_insert_frame(file_ptr_t *file) {
	e_assert(file->is_main&&display.buf, E_ERROR);
	return E_OK;
}
static e_int32 inter_memgray_write_point(file_ptr_t *file, int x, int y,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_memgray_write_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* point) {
	return E_ERROR;
}
static e_int32 inter_memgray_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* pnts) {
	e_uint8 *pbuf;
	memgray_private_t *memgray = (memgray_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	e_assert(file->info.width > column_idx, E_ERROR);
	pbuf = &memgray->buf[column_idx];
	for (unsigned int i = 0; i < file->info.height; i++) {
		(*pbuf) = point->gray;
		pbuf += file->info.width;
		point++;
	}
	return E_OK;
}
static e_int32 inter_memgray_append_points(file_ptr_t *file, point_t* pnts,
		int pt_num) {
	memgray_private_t *memgray = (memgray_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (pt_num--) {
		memgray->buf[file->cousor] = point->gray;
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_memgray_append_row(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}
static e_int32 inter_memgray_append_column(file_ptr_t *file, point_t* point) {
	return E_ERROR;
}

static e_int32 inter_memgray_read_points(file_ptr_t *file, point_t* pnts,
		int buf_len) {
	e_uint8 *memgray = (e_uint8 *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	while (buf_len--) {
		point->gray = memgray[file->cousor];
		point++;
		file->cousor++;
	}
	return E_OK;
}
static e_int32 inter_memgray_read_row(file_ptr_t *file, e_uint32 row_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_memgray_read_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* buf) {
	return E_ERROR;
}
static e_int32 inter_memgray_on_data_change(file_ptr_t *file1,
		file_ptr_t *file2) {
	e_int32 ret;
	ret = inter_memgray_combine(file1, file2);
	e_assert(ret>0, ret);
	return E_OK;
}

