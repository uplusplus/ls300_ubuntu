/*!
 * \file test_message_monitor.c
 * \brief message monitor
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include "../hd_test_config.h"
#if TEST_MSG_MONITOR
#include <ls300/hd_message_monitor.h>
#include <arch/hd_thread_api.h>
#include <arch/hd_timer_api.h>
#include <signal.h>

volatile int count = MAX_CLIENT_SIZE;
volatile int run = 1;

void request_thread(void *data) {
	int i = 10;
	e_int32 ret;
	e_uint8 buf[128];
	e_uint8 msg[128];
	fsocket_t* fd = (fsocket_t*) data;
	sprintf(msg, "[msg:REQUEST MSG FROM %d]", fd->id);

	while (i--) {
		DMSG((STDOUT,"[C]\t[%d] client send %s\n",i,msg));
		ret = fsocket_request(fd, msg, strlen(msg), buf, 128, 1000000000); //1 s
		if (ret > 0) {
			DMSG(
					(STDOUT,"[C]\t[%s.%d] client request recv [ %s ]\n", fd->name,fd->id,buf));
		} else {
			DMSG(
					(STDOUT,"[C]\t[%s.%d] client request failed\n",fd->name,fd->id));
		}

		Delay(1000);
	}

	fsocket_close(fd);
	count++;
}

void server_func(void *data) {
	int re_len;
	e_int32 ret;
	e_uint8 c;
	e_uint8 recv_buf[128];
	e_uint8 send_buf[8] = { '#', 0, 0, 0, 0, 'O', 'K', '@' };
	fsocket_t* fd = (fsocket_t*) data;
	hd_connect_t connect;
	socket_t *client;

	DMSG((STDOUT,"[S]\tStart......\n"));

	ret = sc_open_socket(&connect, "127.0.0.1", 6666, E_SOCKET_TCP);
	e_assert(ret>0);
	ret = Socket_Bind(connect.socket);
	e_assert(ret>0);
	ret = Socket_Listen(connect.socket);
	e_assert(ret>0);
	for (; run;) {
		ret = Socket_Accept(connect.socket, &client);
		if (ret == E_ERROR_RETRY) {
			Socket_Select(connect.socket, E_READ, 5e5);
			continue;
		} else if (ret < 0) {
			break;
		}

		re_len = 0;
		for (; run;) {
			ret = Socket_Recv(client, &c, 1);
			if (ret == 0) {
				Socket_Select(client, E_READ, 5e5);
				continue;
			} else if (ret < 0) {
				break;
			}

			recv_buf[re_len++] = c;
			if (c == '#') {
				re_len = 0;
				recv_buf[re_len++] = c;
			} else if (c == '@') {
				recv_buf[re_len++] = 0;
				DMSG((STDOUT,"[S]\tGet:%s\n",recv_buf));
				ret = Socket_Select(client, E_WRITE, 5e5);
				e_assert(ret>0);
				strncpy(send_buf, recv_buf, 5);
				ret = Socket_Send(client, send_buf, 8);
				e_assert(ret>0);
				re_len = 0;
			}
		}
		Socket_Close(&client);
	}
	sc_close(&connect);
}

void sig_handler(int sig) {
	if (sig == SIGINT) {
		run = 0;
		Delay(100);
		exit(0);
	}
}

int main(int argc, char *argv[]) {
	e_int32 ret, i;
	hd_connect_t connect;
	msg_monitor_t monitor = { 0 };
	ethread_t* server_thread;
	ethread_t* threads[MAX_CLIENT_SIZE];
	e_uint8 name[128] = { 't', 'e', 's', 't', 0 };
	signal(SIGINT, sig_handler);
	ret = createthread("request", (thread_func) &server_func, (void*) NULL,
			NULL, &server_thread);
	if (ret <= 0) {
		DMSG((STDOUT,"createhread server failed!\r\n"));
		return E_ERROR;
	}
	ret = resumethread(server_thread);
	if (ret <= 0) {
		DMSG((STDOUT,"resumethread failed!\r\n"));
		return E_ERROR;
	}

	Delay(1000);

	ret = sc_open_socket(&connect, "127.0.0.1", 6666, E_SOCKET_TCP);
	e_assert(ret>0, ret);
	ret = sc_connect(&connect);
	e_assert(ret>0, ret);
	ret = mm_init(&monitor, &connect);
	e_assert(ret>0, ret);
	mm_start(&monitor);

#if 0
	fsocket_t* fd = mm_create_socket(&monitor, name);
	if (!fd) {
		DMSG((STDOUT,"ERROR CREATE FAKE SOCKET\r\n"));
	} else {
		request_thread(fd);
	}
#else
	while (count--) {
		sprintf(name, "Thread[%d]", count);
		fsocket_t* fd = mm_create_socket(&monitor, name);
		DMSG((STDOUT,"create request thread %d\r\n",count));
		ret = createthread("request", (thread_func) &request_thread, (void*) fd,
				NULL, &threads[count]);
		if (ret <= 0) {
			DMSG((STDOUT,"createhread failed!\r\n"));
			return E_ERROR;
		}
		ret = resumethread(threads[count]);
		if (ret <= 0) {
			DMSG((STDOUT,"resumethread failed!\r\n"));
			return E_ERROR;
		}
	}
#endif
	while (count < MAX_CLIENT_SIZE - 1) {
//		DMSG((STDOUT,"COUNT = %d\n",count));
		sleep(1);
	}
	run = 0;
	for (i = 0; i < MAX_CLIENT_SIZE; i++) {
		killthread(threads[i]);
	}
	mm_stop(&monitor);
	sc_close(&connect);
	return 0;
}

#endif
