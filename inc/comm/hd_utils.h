/*!
 * \file hd_utils.h
 * \brief 
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_UTILS_H
#define HD_UTILS_H

/* Dependencies */
#include <arch/hd_plat_base.h>
/*结构体定义*/

typedef struct display_t {
	int w, h;
	float h_w;
	int hash;
	e_uint8* buf;
} display_t;

extern display_t display;
extern char EGL_NODE[100];

/*接口定义*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Compute the message checksum (single-byte XOR).
 * \param data The address of the first data element in a sequence of bytes to be included in the sum
 * \param length The number of byte in the data sequence
 */
e_uint8 DEV_EXPORT hd_compute_xor(const e_uint8 * const data,
		const e_uint32 length);
void DEV_EXPORT hd_polar2xyz(float *x, float *y, float *z, double distance,
		double angle_h, double angle_v);
int DEV_EXPORT hd_soft_stretch(e_uint8* src, int s_x, int s_y, int s_w, int s_h,
		e_uint8* dst, int d_x, int d_y, int d_w, int d_h, int bpp);
e_uint8* DEV_EXPORT hd_image_stretch(int screen_w, int screen_h, int bpp,
		e_uint8* src, int s_w, int s_h, int s_h_w, int *d_w, int *d_h);
void DEV_EXPORT hd_print_image(e_uint8* msg, int w, int h);

int DEV_EXPORT hd_video_set_src(e_uint8 *src, int s_w, int s_h, float s_h_w);
int DEV_EXPORT hd_video_set_screen(int screen_w, int screen_h, int bpp,
		void (*set_image)(unsigned char* pixel, int w, int h));
int DEV_EXPORT hd_video_upate_screen();

#ifdef __cplusplus
}
#endif

#endif /*HD_UTILS_H*/
