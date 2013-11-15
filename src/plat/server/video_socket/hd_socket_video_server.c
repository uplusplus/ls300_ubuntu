/*
 * =====================================================================================
 *
 *       Filename:  hd_socket_video_server.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013年06月19日 15时03分25秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mei Kang (), meikang9527@163.com
 *        Company:  College of Information Engineering of CDUT
 *
 *        modify:
 *        			Joy.you 20120628  	接口重构,PCL格式重构
 *        			Joy.you 20120629  	添加JPG格式支持
 *        			Joy.you 20120629  	添加GIF格式支持
 *
 *
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

static struct video_server_t {

	ethread_t* thread_loop;
	char address[MAX_PATH_LEN];
	int port;
	int socket_type;

	int loop;
} video_server;

#define FPS_DELAY 1000 //ms
static void server_loop(void* vs) {
	int ret;
	video_server.loop = 1;
	hd_connect_t connect;
	char buf[100];

	DMSG((STDOUT, "socket video server loop routine started.\r\n"));

	for (; video_server.loop;) {
		while (!(display.w && display.h && display.buf))
			sleep(1); //首次连接，显示未就绪，等待

		sprintf((char*) buf, "ABCD%05d%05d%08.2fEFGH", display.w, display.h,
				display.h_w);

		ret = sc_open_socket(&connect, video_server.address, video_server.port,
				video_server.socket_type);
		if (ret > 0)
			ret = sc_try_connect(&connect, 65535);

		DMSG((STDOUT,"server_loop connected to server.\n"));

		//发送长宽信息
		if (ret > 0)
			ret = sc_select(&connect, E_WRITE, 100000);
		if (ret > 0)
			ret = sc_send(&connect, buf, 26);

		DMSG((STDOUT,"server_loop start frame send loop.\n"));
		if (ret > 0) {
			for (; video_server.loop;) { //定时发送帧数据
				ret = sc_select(&connect, E_WRITE, 1E6);
//					DMSG((STDOUT,"server_loop sc_select ret %d.\n",ret));
				if (ret == E_ERROR_TIME_OUT)
					continue;
				else if (ret > 0)
					ret = sc_send(&connect, display.buf, display.h * display.w);
				if (ret <= 0)
					break;
				Delay(FPS_DELAY);
//					DMSG((STDOUT,"server_loop send a frame.\n"));
			}
		}

		if (ret <= 0)
			sc_close(&connect);
		DMSG((STDOUT,"server_loop disconnected,reset.\n"));

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

