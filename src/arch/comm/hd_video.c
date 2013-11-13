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

/* This a stretch blit implementation based on ideas given to me by
 Tomasz Cejner - thanks! :)

 April 27, 2000 - Sam Lantinga
 */

#include <arch/hd_plat_base.h>
#include <comm/hd_utils.h>

#define DEFINE_COPY_ROW(name, type)         \
static void name(type *src, int src_w, type *dst, int dst_w)    \
{                                           \
    int i;                                  \
    int pos, inc;                           \
    type pixel = 0;                         \
                                            \
    pos = 0x10000;                          \
    inc = (src_w << 16) / dst_w;            \
    for ( i=dst_w; i>0; --i ) {             \
        while ( pos >= 0x10000L ) {         \
            pixel = *src++;                 \
            pos -= 0x10000L;                \
        }                                   \
        *dst++ = pixel;                     \
        pos += inc;                         \
    }                                       \
}
/* *INDENT-OFF* */
DEFINE_COPY_ROW(copy_row1, e_uint8)
DEFINE_COPY_ROW(copy_row2, e_uint16)
DEFINE_COPY_ROW(copy_row4, e_uint32)
/* *INDENT-ON* */

static void copy_row3(e_uint8 * src, int src_w, e_uint8 * dst, int dst_w) {
	int i;
	int pos, inc;
	e_uint8 pixel[3] = { 0, 0, 0 };

	pos = 0x10000;
	inc = (src_w << 16) / dst_w;
	for (i = dst_w; i > 0; --i) {
		while (pos >= 0x10000L) {
			pixel[0] = *src++;
			pixel[1] = *src++;
			pixel[2] = *src++;
			pos -= 0x10000L;
		}
		*dst++ = pixel[0];
		*dst++ = pixel[1];
		*dst++ = pixel[2];
		pos += inc;
	}
}

/* Perform a stretch blit between two surfaces of the same format.
 NOTE:  This function is not safe to call from multiple threads!
 */
int hd_soft_stretch(e_uint8* src, int s_x, int s_y, int s_w, int s_h,
		e_uint8* dst, int d_x, int d_y, int d_w, int d_h, int bpp) {
	int src_locked;
	int dst_locked;
	int pos, inc;
	int dst_maxrow;
	int src_row, dst_row;
	e_uint8 *srcp = NULL;
	e_uint8 *dstp;

	/* Set up the data... */
	pos = 0x10000;
	inc = (s_h << 16) / d_h;
	src_row = s_y;
	dst_row = d_y;

	/* Perform the stretch blit */
	for (dst_maxrow = dst_row + d_h; dst_row < dst_maxrow; ++dst_row) {
		dstp = (e_uint8 *) dst + (dst_row * d_w) + (d_x * bpp);
		while (pos >= 0x10000L) {
			srcp = (e_uint8 *) src + (src_row * s_w) + (s_x * bpp);
			++src_row;
			pos -= 0x10000L;
		}
		switch (bpp) {
		case 1:
			copy_row1(srcp, s_w, dstp, d_w);
			break;
		case 2:
			copy_row2((e_uint16 *) srcp, s_w, (e_uint16 *) dstp, d_w);
			break;
		case 3:
			copy_row3(srcp, s_w, dstp, d_w);
			break;
		case 4:
			copy_row4((e_uint32 *) srcp, s_w, (e_uint32 *) dstp, d_w);
			break;
		}
		pos += inc;
	}

	return E_OK;
}

/*
 *  源： 数据的宽，高			实际物体高宽比
 *  的： 屏幕的 宽，高
 *  要求： 显示的结果图像：
 *  		宽为 4 的整数倍
 *  		d_w =  screen_w 转为 4N
 *  		d_h = d_w * h_w比
 *  		s -> d
 *
 *  		数据尺寸(源数据)  实际尺寸（决定显示比例）   纹理尺寸（影响最终的宽，4N）  屏幕尺寸（决定显示的宽度）
 * */

e_uint8* hd_image_stretch(int screen_w, int screen_h, int bpp, e_uint8* src,
		int s_w, int s_h, int s_h_w, int *d_w, int *d_h) {
	e_uint8* dst;

	//占满屏幕算法
	if (screen_h / (float)screen_w >= s_h_w) { //宽度为限制条件
		(*d_w) = screen_w & ~3; //(screen_w + 3) & ~3;
		(*d_h) = (*d_w) * s_h_w;
	} else { //高度为限制条件
		(*d_h) = screen_h;
		(*d_w) = (*d_h) / s_h_w;
		(*d_w) = (*d_w) & ~3; //((*d_w) + 3) & ~3;
	}

	dst = (e_uint8*) malloc((*d_w) * (*d_h) * bpp);
	e_assert(dst, E_ERROR_BAD_ALLOCATE);

	hd_soft_stretch(src, 0, 0, s_w, s_h, dst, 0, 0, (*d_w), (*d_h), bpp);
	return dst;
}

typedef struct hd_video_t {
	int screen_w, screen_h, bpp;

	int s_w, s_h;
	float s_h_w;
	e_uint8 *src;
	void (*set_image)(unsigned char* pixel, int w, int h);

	int d_w, d_h;
	e_uint8 *dst;
} hd_video_t;

hd_video_t video = { 0 };

static int hd_video_state() {
	return video.s_w && video.s_h && video.src && video.s_h_w && video.screen_w
			&& video.screen_h && video.bpp;
}

static int hd_video_upate_context() {
	e_uint8 *dst_old;
	if (!hd_video_state())
		return 0;

	//占满屏幕算法
	if (video.screen_h / video.screen_w >= video.s_h_w) { //宽度为限制条件
		video.d_w = (video.screen_w) & ~3;//(video.screen_w + 3) & ~3;
		video.d_h = video.d_w * video.s_h_w;
	} else { //高度为限制条件
		video.d_h = video.screen_h;
		video.d_w = video.d_h / video.s_h_w;
		video.d_w = (video.d_w) & ~3;//(video.d_w + 3) & ~3;
	}

	dst_old = video.dst;
	video.dst = (e_uint8*) malloc(video.d_w * video.d_h * video.bpp);
	// e_assert(video.dst, E_ERROR_BAD_ALLOCATE);

	video.set_image(video.dst, video.d_w, video.d_h);

	free(dst_old);
	return 1;
}

int hd_video_set_src(e_uint8 *src, int s_w, int s_h, float s_h_w) {
	video.src = src;
	video.s_w = s_w;
	video.s_h = s_h;
	video.s_h_w = s_h_w;
	return hd_video_upate_context();
}

int hd_video_set_screen(int screen_w, int screen_h, int bpp,
		void (*set_image)(unsigned char* pixel, int w, int h)) {
	video.screen_w = screen_w;
	video.screen_h = screen_h;
	video.bpp = bpp;
	video.set_image = set_image;
	return hd_video_upate_context();
}

int hd_video_upate_screen() {
	if (!hd_video_state())
		return 0;

	hd_soft_stretch(video.src, 0, 0, video.s_w, video.s_h, video.dst, 0, 0,
			video.d_w, video.d_h, video.bpp);
	return 1;
}
