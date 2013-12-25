/*!
 * \file hd_websserver.h
 * \brief 定义了web服务
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_WEBSERVER_H
#define HD_WEBSERVER_H

/* Dependencies */
#include <arch/hd_plat_base.h>

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

e_int32 DEV_EXPORT webserver_start(char *root_dir);
void   DEV_EXPORT webserver_loop();
e_int32 DEV_EXPORT webserver_stop(void);

#ifdef __cplusplus
}
#endif

#endif /*HD_WEBSERVER_H*/
