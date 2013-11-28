/*!
 * \file hd_scan_data_pool.h
 * \brief data pool
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd data pool
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_SCAN_DATA_POOL_H
#define HD_SCAN_DATA_POOL_H

/* Dependencies */
#include "sickld/sickld_base.h"
#include <comm/hd_list.h>
#include <arch/hd_thread_api.h>

#define DATA_BUFFER_MAX_NUM		500
#define POOL_SLEEP_TIMEOUT 		(1e5)

typedef struct {
	struct list_head list_node;
	scan_data_t data;
} scan_pool_data_t;

typedef struct scan_pool_t {
	struct list_head *read_node, *write_node;
	scan_pool_data_t buffer_mem[DATA_BUFFER_MAX_NUM];
	semaphore_t sem_read;
	semaphore_t sem_write;
	volatile e_int32 state;
} scan_pool_t;

typedef void on_scan_data(scan_data_t* data);

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

void DEV_EXPORT pool_init(scan_pool_t* pool);
void DEV_EXPORT pool_cancle(scan_pool_t* pool);
void DEV_EXPORT pool_disconnect(scan_pool_t* pool);
void DEV_EXPORT pool_destroy(scan_pool_t* pool);
e_int32 DEV_EXPORT pool_read(scan_pool_t* pool, scan_data_t *data);
e_int32 DEV_EXPORT pool_write(scan_pool_t* pool, scan_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /*HD_SCAN_DATA_POOL_H*/
