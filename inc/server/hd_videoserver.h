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

e_int32 DEV_EXPORT socket_video_server_start(char *address, e_uint32 port,
		int socket_type);
e_int32 DEV_EXPORT socket_video_server_stop();

#ifdef __cplusplus
}
#endif

#endif /*HD_WEBSOCKET_H*/
