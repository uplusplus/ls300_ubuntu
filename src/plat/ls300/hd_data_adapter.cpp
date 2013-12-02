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
static e_int32 inter_pcl_close(file_ptr_t *file, int save);
static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point, int file_idx);

//hls
static e_int32 inter_hls_open(file_ptr_t *file);
static e_int32 inter_hls_close(file_ptr_t *file, int save);
static e_int32 inter_hls_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point, int file_idx);
//memgray
static e_int32 inter_memgray_open(file_ptr_t *file);
static e_int32 inter_memgray_close(file_ptr_t *file, int save);
static e_int32 inter_memgray_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point, int file_idx);

//open和close是必须有的接口
file_adapter_t file_adpt[] = {
	{
		file_suffix : "pcd1",
		pnt_type: PNT_TYPE_XYZ,
		op : {
			open:inter_pcl_open,
			close:inter_pcl_close,
			write_column:inter_pcl_write_column,
		}
	},
	{
		file_suffix : "pcd",
		pnt_type: PNT_TYPE_POLAR,
		op : {
			open:inter_pcl_open,
			close:inter_pcl_close,
			write_column:inter_pcl_write_column,
		}
	},
	{
		file_suffix : "hls",
		pnt_type: PNT_TYPE_POLAR,
		op : {
			open:inter_hls_open,
			close:inter_hls_close,
			write_column:inter_hls_write_column,
		}
	},
	{
		file_suffix : "memgray",
		pnt_type: PNT_TYPE_GRAY,
		op : {
			open:inter_memgray_open,
			close:inter_memgray_close,
			write_column:inter_memgray_write_column,
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
		e_float32 h_w, int mode) {
	e_assert(strlen((char*) name) < MAX_PATH_LEN, E_ERROR);

	memset(da, 0, sizeof(data_adapter_t));

	char* file_type = NULL;
	int num;

	strcpy(da->file.file_name, (char*) name);

	da->file.width = width;
	da->file.width_sum = mode == E_DWRITE ? width * 2 : width;
	da->file.height = height;
	da->file.h_w = h_w;
	da->file.max_file_idx = mode == E_DWRITE ? 1 : 0;
	da->file.mode = mode;

	file_type = get_file_suffix((char*) name);

	for (num = 0; num < FILE_TYPE_NUM; num++) {
		int ret = strncmp(file_type, file_adpt[num].file_suffix,
				strlen(file_adpt[num].file_suffix));
		if (!ret) {
			register_file_format(da, num);
			da->file.pnt_type = file_adpt[num].pnt_type;
			ret = da->op.open(&da->file);
			e_assert(ret>0, ret);
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
	ret = da->op.close(&da->file, save);
	e_assert(ret > 0, ret);
	da->state = 0;
	return E_OK;
}

e_int32 da_write_column(data_adapter_t *da, e_uint32 column_idx, point_t* point,
		int file_idx) {
	e_int32 ret;
	e_assert(
			da && da->state&& da->pnt_type == point->type && da->op.write_column && point,
			E_ERROR_INVALID_HANDLER);

	e_assert(column_idx>=0 && column_idx < da->file.width, E_ERROR);
	ret = da->op.write_column(&da->file, column_idx, point, file_idx);

	return ret;
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

#define PCL_HAS_I 1
#if PCL_HAS_I
#define PCL_PNT_TYPE pcl::PointXYZI
#else
#define PCL_PNT_TYPE pcl::PointXYZ
#endif

static e_int32 inter_pcl_open(file_ptr_t *file) {
	if (file->mode == E_READ) {
		pcl::PointCloud<PCL_PNT_TYPE> * cloud =
				new pcl::PointCloud<PCL_PNT_TYPE>();
		if (pcl::io::loadPCDFile<PCL_PNT_TYPE>("test_pcd.pcd", *cloud) == -1) //打开点云文件
				{
			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
			return (-1);
		}
		file->handle = (size_t) cloud;
		file->width = cloud->width;
		file->height = cloud->height;
	} else {
		//打开缓存
		pcl::PointCloud<PCL_PNT_TYPE>* cloud =
				new pcl::PointCloud<PCL_PNT_TYPE>();
		// 创建点云缓存
		cloud->width = file->width_sum;
		cloud->height = file->height;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		file->handle = (size_t) cloud;
	}

	return E_OK;
}

static e_int32 inter_pcl_close(file_ptr_t *file, int save) {
	pcl::PointCloud<PCL_PNT_TYPE> *cloud =
			(pcl::PointCloud<PCL_PNT_TYPE> *) file->handle;
	if (save) {
		pcl::io::savePCDFileBinary(file->file_name, *cloud);
	}
	delete cloud;
	return E_OK;
}

static point_xyz_t* transfor2xyz(point_t* pnts, int i, point_xyz_t *xyz) {
	switch (pnts->type) {
	case PNT_TYPE_XYZ:
		return ((point_xyz_t*) pnts->mem) + i;
	case PNT_TYPE_POLAR: {
		point_polar_t *polar = (point_polar_t*) pnts->mem;
		hd_polar2xyz(&xyz->x, &xyz->y, &xyz->z, polar[i].distance,
				polar[i].angle_h, polar[i].angle_v);
#if PCL_HAS_I
		xyz->intensity = polar[i].intensity;
#endif
//		if(xyz->x<0 || xyz->y<0 || xyz->intensity>700)
//			DMSG((STDOUT,"[%f,%f,%f]\n",polar[i].distance,
//				polar[i].angle_h, polar[i].angle_v));
		return xyz;
	}
	default:
		DMSG((STDOUT,"transfor2xyz ERROR \n"));
		return NULL;
	}
}
static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point, int file_idx) {
	pcl::PointCloud<PCL_PNT_TYPE> *cloud =
			(pcl::PointCloud<PCL_PNT_TYPE> *) (pcl::PointCloud<PCL_PNT_TYPE> *) file->handle;
	int idx = 0;
	point_xyz_t xyz, *pxyz;
	e_uint32 pt_num = file->height;

	while (pt_num--) {
		pxyz = transfor2xyz(point, idx, &xyz);
		cloud->points[file->cursor].x = pxyz->x;
		cloud->points[file->cursor].y = pxyz->y;
		cloud->points[file->cursor].z = pxyz->z;
#if PCL_HAS_I
		cloud->points[file->cursor].intensity = pxyz->intensity;
#endif
		file->cursor++;
		idx++;
	}
	return E_OK;
}

#else

static e_int32 inter_pcl_open(file_ptr_t *file) {
	e_int32 ret;
	if (file->mode == E_READ) {
		file_t *f = (file_t *) malloc(sizeof(file_t));
		ret = fi_open(file->file_name, F_READ, f);
		e_assert(ret, E_ERROR_INVALID_HANDLER);
		file->handle = (size_t) f;
		fscanf((FILE*) f->fpid, "%20u,%20u\n", &file->width,
				&file->height);
	} else {
		file_t *f = (file_t *) malloc(sizeof(file_t));
		ret = fi_open(file->file_name, F_READ | F_WRITE | F_CREATE, f);
		e_assert(ret, E_ERROR_INVALID_HANDLER);

		if (setvbuf((FILE*) f->fpid, NULL, _IOFBF, 65536) != 0) {
			fprintf(stderr, "WARNING: setvbuf() failed with buffer size %u\n",
					65536);
		}

		file->handle = (size_t) f;
		fprintf((FILE*) f->fpid, "%20u,%20u\n",
				file->width_sum,
				file->height);
	}

	return E_OK;
}

static e_int32 inter_pcl_close(file_ptr_t *file, int save) {
	file_t* fd = (file_t*) file->handle;
	FILE* f1 = (FILE*) fd->fpid;
	DMSG((STDOUT,"inter_pcl_close try close:%s",file->file_name));
	fclose(f1);
	free(fd);
	return E_OK;
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

static e_int32 inter_pcl_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point,int file_idx) {
	int idx = 0;
	point_xyz_t xyz, *pxyz;
	file_t* fd = (file_t*) file->handle;
	FILE* f1 = (FILE*) fd->fpid;
	e_uint32 pt_num = file->height;

	while (pt_num--) {
		pxyz = transfor2xyz(pnts, idx, &xyz);
		fprintf(f1, "%.10g,%.10g,%.10g\n", pxyz->x, pxyz->y, pxyz->z);
		file->cursor++;
		idx++;
	}
	return E_OK;
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
	if (file->mode == E_READ) {
	} else {
		//打开缓存
		hd::HLSWriter * writer = create_hlswriter(file->file_name);
		if (!writer)
			return E_ERROR;
		// 创建点云缓存;
		file->handle = (size_t) writer;
	}

	return E_OK;
}

static e_int32 inter_hls_close(file_ptr_t *file, int save) {
	hd::HLSWriter * writer = (hd::HLSWriter *) file->handle;
	if (save) {
		writer->close(file->width_sum * file->height, file->width_sum,
				file->height);
	}
	delete writer;
	return E_OK;
}

static e_int32 inter_hls_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* point, int file_idx) {
	unsigned int i;
	hd::HLSWriter * writer = (hd::HLSWriter *) file->handle;
	point_polar_t *polar = (point_polar_t*) point->mem;
	e_uint32 pt_num = file->height;

	for (i = 0; i < pt_num; i++) {
		writer->write_point(polar[i].distance, polar[i].angle_h,
				polar[i].angle_v, polar[i].intensity, file_idx);
		file->cursor++;
	}

	return E_OK;
}

///////////////////////////////////////////////////////////////////////////////
//memgray
#include <comm/hd_utils.h>
typedef struct memgray_private_t {
	e_uint32 size;
	e_uint8 * buf;
} memgray_private_t;

static e_int32 inter_memgray_open(file_ptr_t *file) {
	//打开缓存
	memgray_private_t *memgray = (memgray_private_t *) calloc(
			sizeof(memgray_private_t), 1);
	unsigned int size = file->width_sum * file->height;
	memgray->buf = (e_uint8 *) calloc(size, 1);
	e_assert(memgray->buf, E_ERROR_BAD_ALLOCATE);
	memgray->size = size;
	file->handle = (size_t) memgray;
	return E_OK;
}

static e_int32 inter_memgray_close(file_ptr_t *file, int save) {
	memgray_private_t *memgray = (memgray_private_t *) file->handle;
	free(memgray->buf);
	free(memgray);
	return E_OK;
}

static e_int32 _memgray_insert_frame(file_ptr_t *file) {
	memgray_private_t *memgray;
	e_assert(display.buf, E_ERROR);
	memgray = (memgray_private_t *) file->handle;
	e_assert(memgray&&memgray->buf, E_ERROR);
	memcpy(display.buf, memgray->buf, file->width_sum * file->height);
	display.hash++;
	return E_OK;
}

static e_int32 inter_memgray_write_column(file_ptr_t *file, e_uint32 column_idx,
		point_t* pnts, int file_idx) {
	e_uint8 *pbuf;
	memgray_private_t *memgray = (memgray_private_t *) file->handle;
	point_gray_t *point = (point_gray_t*) pnts->mem;
	e_assert(file->width > column_idx, E_ERROR);
	column_idx += file_idx * file->width;
	pbuf = &memgray->buf[column_idx];
	for (unsigned int i = 0; i < file->height; i++) {
		(*pbuf) = point->gray;
		pbuf += file->width_sum;
		point++;
	}

	if (file_idx == file->max_file_idx)
		_memgray_insert_frame(file);

	return E_OK;
}
