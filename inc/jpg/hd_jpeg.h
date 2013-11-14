//--------------------------------------------------------------------------------
// hd_jpeg.h
// uplusplus
// 2013-10-14
//
//
//--------------------------------------------------------------------------------

#ifndef _HD_JPEG_H_
#define _HD_JPEG_H_

#include <arch/hd_plat_base.h>

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

e_int32 DEV_EXPORT gray_to_jpeg_mem(e_uint32 uWidth, e_uint32 uHeight,
		unsigned char* pImg, e_uint32 iQuality, unsigned char **r_buf,
		e_uint32 *r_size);

e_int32 DEV_EXPORT gray_to_jpeg_file(char *filename, unsigned char *bits, int width,
		int height);

e_int32 DEV_EXPORT gray_to_jpeg_file_rotation(char *filename, unsigned char *bits,
		int width, int height);

/* 接口定义 */
#ifdef __cplusplus
}
#endif

#endif
