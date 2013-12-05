/*
 * =====================================================================================
 *
 *       Filename:  hd_socket_video_server.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013年11月19日
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  uplusplus
 *        Company:  zhd-hdsy
 * =====================================================================================
 */
#include <string.h>

#include <arch/hd_file_api.h>
#include <arch/hd_thread_api.h>
#include <comm/hd_utils.h>
#include <arch/hd_pipe_api.h>
#include <arch/hd_timer_api.h>
#include <ls300/hd_connect.h>
#include <server/hd_videoserver.h>

#include <comm/hd_utils.h>
#include <jpg/hd_jpeg.h>

//#ifdef DMSG
//#undef DMSG
//#define DMSG
//#endif

static struct video_server_t {

	ethread_t* thread_loop;
	char address[MAX_PATH_LEN];
	int port;
	int socket_type;
	int last_hash;

	int loop;
} video_server;

#define FPS_DELAY 1000 //ms
static void server_loop(void* vs) {
	int ret, bytes_to_send, err, size;
	video_server.loop = 1;
	hd_connect_t connect;
	char buf[100];

	DMSG((STDOUT, "socket video server loop routine started.\r\n"));

	for (; video_server.loop;) {
		while (!(display.w && display.h && display.buf))
			sleep(1); //首次连接，显示未就绪，等待
		size = display.h * display.w;
		sprintf((char*) buf, "ABCD%05d%05d%08.2fEFGH", display.w, display.h,
				display.h_w);

		ret = sc_open_socket(&connect, video_server.address, video_server.port,
				video_server.socket_type);
		if (ret > 0)
			ret = sc_try_connect(&connect, 65535);

		DMSG((STDOUT, "server_loop connected to server.\n"));

		//发送长宽信息
		ret = sc_send_ex(&connect, buf, 26, 1e6, &video_server.loop);
		DMSG((STDOUT, "server_loop start frame send loop.\n"));
		video_server.last_hash = -1;
		if (ret > 0) {
			for (; video_server.loop;) { //定时发送帧数据
				if (display.hash != video_server.last_hash) {
					ret = sc_send_ex(&connect, display.buf, size, 1e6,
							&video_server.loop);
					if (ret <= 0)
						break;
					video_server.last_hash = display.hash;
				}
				Delay(FPS_DELAY);
//				DMSG((STDOUT,"server_loop send a frame.\n"));
			}
		}

		sc_close(&connect);
		DMSG((STDOUT, "server_loop disconnected,reset.\n"));
	}
	DMSG((STDOUT, "socket video server loop routine stoped...\r\n"));
}

e_int32 socket_video_server_start(char *address, e_uint32 port, int socket_type) {
	int ret;

	e_assert(address, E_ERROR_INVALID_ADDRESS);

	strncpy(video_server.address, address, sizeof(video_server.address));
	video_server.port = port;
	video_server.socket_type = socket_type;

	DMSG((STDOUT, "socket video server start loop routine...\r\n"));
	ret = createthread("socket_video_server", (thread_func) &server_loop,
			&video_server, NULL, &video_server.thread_loop);
	if (ret <= 0) {
		DMSG((STDOUT, "createhread failed!\r\n"));
		return E_ERROR;
	}
	ret = resumethread(video_server.thread_loop);
	if (ret <= 0) {
		DMSG((STDOUT, "resumethread failed!\r\n"));
		return E_ERROR;
	}

	return E_OK;
}

e_int32 socket_video_server_stop() {

	DMSG((STDOUT, "socket video server stop...\r\n"));
	video_server.loop = 0;
	killthread(video_server.thread_loop);

	return E_OK;
}

