/*!
 * \file hd_laser_photo.h
 * \brief	扫描图像
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser work
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_LASER_PHOTO_H
#define HD_LASER_PHOTO_H

/* Dependencies */
#include <arch/hd_plat_base.h>
#include <ls300/hd_laser_control.h>

typedef struct laser_photo_t laser_photo_t;

/*接口定义*/
#ifdef __cplusplus
extern "C"
{
#endif

e_int32 DEV_EXPORT lp_init(laser_photo_t **lpp, laser_control_t *lc,
		e_int32 (*on_status_change)(void*,int),void* ctx);
e_int32 DEV_EXPORT lp_uninit(laser_photo_t *lp);
e_int32 DEV_EXPORT lp_scan(laser_photo_t *lp,
		e_float64 camera_angle_start, e_float64 camera_angle_end);
e_int32 DEV_EXPORT lp_cancel(laser_photo_t *lp);

#ifdef __cplusplus
}
#endif

#endif /*HD_LASER_PHOTO_H*/
