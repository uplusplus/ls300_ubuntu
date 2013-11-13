/*!
 * \file hd_websocket.h
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

#ifndef HD_WEBSOCKET_H
#define HD_WEBSOCKET_H

/* Dependencies */
#include <arch/hd_plat_base.h>

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

int DEV_EXPORT websocket_start(char *root_dir);
int DEV_EXPORT webhttp_start(char *root_dir);

e_int32 DEV_EXPORT websocket_stop(void);

#ifdef __cplusplus
}
#endif

#endif /*HD_WEBSOCKET_H*/
