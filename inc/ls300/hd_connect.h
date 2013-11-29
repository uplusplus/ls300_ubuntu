/*!
 * \file sick_connect.h
 * \brief 定义了到sick扫描仪连接类
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#ifndef HD_CONNECT_H
#define HD_CONNECT_H

/* Dependencies */
#include <arch/hd_socket_api.h>
#include <arch/hd_serial_api.h>
#include <arch/hd_pipe_api.h>

#define CONNECT_DELAY (3000) //ms

/*结构体定义*/
typedef struct hd_connect_t {
	//all field is private,do not access direct
	union {
		socket_t *socket;
		serial_t serial;
		pipe_t pipe;
	};
	int state;
	int mask;
} hd_connect_t;

/* 接口定义 */
#ifdef __cplusplus
extern "C" {
#endif

e_int32 DEV_EXPORT sc_open_socket(hd_connect_t* sc, char* sick_ip_address,
		e_uint16 sick_tcp_port, int socket_type);
e_int32 DEV_EXPORT sc_open_serial(hd_connect_t* sc, char* com_name,
		e_uint32 baudrate);
e_int32 DEV_EXPORT sc_open_pipe(hd_connect_t* sc, char* memory_name,
		e_uint32 size);

e_int32 DEV_EXPORT sc_close(hd_connect_t* sc);
e_int32 DEV_EXPORT sc_state(hd_connect_t *sc);

//	select/bind/listen/connect/accept socket connection
/*timeout,单位usec, 1秒 = 1000000微秒*/
e_int32 DEV_EXPORT sc_select(hd_connect_t *sc, e_int32 type,
		e_int32 timeout_usec);
e_int32 DEV_EXPORT sc_connect(hd_connect_t *sc);
e_int32 DEV_EXPORT sc_try_connect(hd_connect_t *sc, e_uint32 max_times);
e_int32 DEV_EXPORT sc_recv(hd_connect_t *sc, e_uint8 *buffer,
		e_uint32 bytes_to_read);
e_int32 DEV_EXPORT sc_send(hd_connect_t *sc, e_uint8 *buffer,
		e_uint32 bytes_to_send);
e_int32 DEV_EXPORT sc_recv_ex(hd_connect_t *sc, e_uint8 *buffer,
		e_uint32 bytes_to_read, e_uint32 timeout, e_int32 *loop);
e_int32 DEV_EXPORT sc_send_ex(hd_connect_t *sc, e_uint8 *buffer,
		e_uint32 bytes_to_send, e_uint32 timeout, e_int32 *loop);
e_int32 DEV_EXPORT sc_error(hd_connect_t *sc);

e_int32 DEV_EXPORT sc_request(hd_connect_t *sc, e_uint8 *send_buffer,
		e_uint32 slen, e_uint8 *recv_buffer, e_uint32 rlen,
		e_uint32 timeout_usec);

e_int32 DEV_EXPORT sc_request_and_check(hd_connect_t *sc, e_uint8 *send_buffer,
		e_uint32 slen, e_uint8 *recv_buffer, e_uint32 rlen,
		e_uint8 * check_string, e_uint32 timeout_usec);

char * DEV_EXPORT sc_tostring(hd_connect_t *sc);

#ifdef __cplusplus
}
#endif

#endif /*HD_CONNECT_H*/
