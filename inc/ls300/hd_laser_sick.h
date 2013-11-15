/*!
 * \file hd_laser_work.h
 * \brief	扫描工作者
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd laser work
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_LASER_SICK_H
#define HD_LASER_SICK_H

/* Dependencies */
#include <arch/hd_plat_base.h>
#include <ls300/hd_laser_control.h>
#include <sickld/sickld.h>

/*结构体定义*/
typedef struct laser_sick_t laser_sick_t;

/*接口定义*/
#ifdef __cplusplus
extern "C" {
#endif

e_int32 DEV_EXPORT ls_init(laser_sick_t **ls, laser_control_t *lc,
		sickld_t *sick, e_int32 (*on_status_change)(void*, int), void*ctx);
e_int32 DEV_EXPORT ls_uninit(laser_sick_t *ls);
e_int32 DEV_EXPORT ls_scan(laser_sick_t *ls, char* ptDir, char *grayDir,
		char *files_dir, e_uint32 speed_h_delay, const e_float64 start_angle_h,
		const e_float64 end_angle_h, e_uint32 speed_v_hz,
		e_float64 resolution_v, e_uint32 interlace_v,const e_float64 start_angle_v,
		const e_float64 end_angle_v);
e_int32 DEV_EXPORT ls_cancel(laser_sick_t *ls);
e_int32 DEV_EXPORT ls_phrase_config(laser_sick_t *ls, e_uint32 speed_h,
		const e_float64 start_angle_h, const e_float64 end_angle_h,
		e_uint32 speed_v, e_float64 resolution_v,e_uint32 interlace_v,
		const e_float64 active_sector_start_angles,
		const e_float64 active_sector_stop_angles);

#ifdef __cplusplus
}
#endif

#endif /*HD_LASER_SICK_H*/
