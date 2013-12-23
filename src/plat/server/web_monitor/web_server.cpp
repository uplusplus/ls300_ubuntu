// Copyright (c) 2004-2012 Sergey Lyubka
// This file is a part of mongoose project, http://github.com/valenok/mongoose
extern "C" {
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <stdarg.h>
#include <pthread.h>
#include "mongoose.h"
#include "session.h"
#include <comm/hd_utils.h>
#include <jpg/hd_jpeg.h>
#include <ls300/hd_laser_scan.h>
#include <ls300/hd_laser_machine.h>
#include <server/hd_webserver.h>
}

static char document_root[1024] = { 0 };

#define SIMPLE_REPLAY_FORMAT "{ \"query\": \"%s\", \"success\": %d, \"message\":[\"%s\"] }"
#define REPLAY_FORMAT(_EXTRA_) "{ \"query\": \"%s\", \"success\": %d, \"message\":[\"%s\"]," _EXTRA_ "}"

#define SIMPLE_REPLAY_FORMAT_AJAX "%s({ \"query\": \"%s\", \"success\": %d, \"message\":[\"%s\"] })"
#define REPLAY_FORMAT_AJAX(_EXTRA_) "%s({ \"query\": \"%s\", \"success\": %d, \"message\":[\"%s\"]," _EXTRA_ "})"

#define COMMANDS "\"authorize\",\"devicestate\",\"gray.jpg\",\"graysize\",\"turntable\"," \
	"\"led\",\"takephoto\",\"angle\",\"tilt\",\"temperature\",\"battery\",\"config\",\"pointscan\"," \
	"\"photoscan\", \"cancel\",\"availablePlusDelay\",\"availableFrequency\",\"availableResolution\"," \
	"\"availablePrecision\",\"help\""

#define AVIABLEPLUSDELAY "20, 50, 100, 150, 200, 250, 400, 700, 850, 1250, 2500, 5000"
#define AVIABLEFREQUENCY "5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20"
#define AVIABLERESOLUTION "0.0625, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1.125, 1.25, 1.375, 1.5, 1.625, 1.75, 1.875, 2.0"
#define AVIABLEPRECISION \
	"{name: \"PRECISION_LEVEL_LOW\",  frequency: 5, resolution:  0.5,plusdelay: 50, widthL:  361, height:  271, time:  72},\
	{name: \"PRECISION_LEVEL_NORMAL\",  frequency: 7, resolution:  0.375, plusdelay: 50, widthL:  505, height:  361, time:  72},\
	{name: \"PRECISION_LEVEL_MIDDLE\",  frequency: 5, resolution:  0.25, plusdelay: 100, widthL:  721, height:  541, time:  144},\
	{name: \"PRECISION_LEVEL_HIGH\",  frequency: 5, resolution:  0.125, plusdelay: 200, widthL:  1441, height:  1081, time:  288},\
	{name: \"PRECISION_LEVEL_EXTRA\",  frequency: 7, resolution:  0.0625, plusdelay: 1250, widthL:  2884, height:  2164, time:  1800}"

static char *commands[] = { "authorize", "devicestate", "gray.jpg", "graysize",
		"turntable", "led", "takephoto", "angle", "tilt", "temperature",
		"battery", "config", "pointscan", "photoscan", "cancel",
		"availablePlusDelay", "availableFrequency", "availableResolution",
		"availablePrecision", "help" };

static struct {
	int hash;
	char* jpg_buf;
	e_uint32 buf_size, w, h;
	scan_job_t* ls300;
	laser_machine_t* ls300m;
} server = { -1, 0, 0, 0, 0 };

static const char *ajax_reply_start = "HTTP/1.1 200 OK\r\n"
		"Cache: no-cache\r\n"
		"Content-Type: application/x-javascript\r\n"
		"\r\n";

static const char *reply_unzutorized = "HTTP/1.1 401 Unauthorized\r\n"
		"Cache: no-cache\r\n"
		"Connection: close\r\n\r\n"
		"\r\n";

static int strcompare(const char** p1, char** p2) {
	char *str1 = *p1, *str2 = *p2;
	int i = 0;
	for (;;) {
		if (!str2[i] && !str1[i])
			return 0;
		if (str1[i] - str2[i])
			return str1[i] - str2[i];
		i++;
	}
}

static void init_server() {
	qsort(commands, sizeof(commands) / sizeof(commands[0]), sizeof(commands[0]),
			strcompare);
}

static int is_command_avaiable(char *cmd) {
	return bsearch(&cmd, commands, sizeof(commands) / sizeof(commands[0]),
			sizeof(commands[0]), strcompare);
}

#if 0
static int urlcompare(const char* url, char* patt) {
	int i = 0;
	for (;;) {
		if (patt[i] == '*')
		return 1;
		if ((!patt[i] && url[i]) || (patt[i] && !url[i]))
		return 0;
		if (!patt[i] && !url[i])
		return 1;
		if (url[i] - patt[i])
		return 0;
		i++;
	}
}
#else
#define urlcompare(_a_,_b_) !strcasecmp(_a_,_b_)
#endif

static int get_qsvar(const struct mg_request_info *request_info,
		const char *name, char *dst, size_t dst_len) {
	const char *qs = request_info->query_string;
	return mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

#define get_request_param_int(request_info, param, name,default_value) do{\
		name = get_qsvar(request_info, #name, param, sizeof(param)) > 0 ? \
				atoi(param) : default_value;}while(0)

#define get_request_param_float(request_info, param, name,default_value) do{\
		name = get_qsvar(request_info, #name, param, sizeof(param)) > 0 ? \
				atof(param) : default_value;}while(0)

// If "callback" param is present in \"query\" string, this is JSONP call.
// Return 1 in this case, or 0 if "callback" is not specified.
// Wrap an output in Javascript function call.
static int handle_jsonp(struct mg_connection *conn,
		const struct mg_request_info *request_info, char cb[64]) {
	return get_qsvar(request_info, "callback", cb, 64);
}

static int send_ok(struct mg_connection *conn, const char* extra, int size) {
	int ret = 0;
	if (size) {
		if (extra)
			ret = mg_printf(conn, "HTTP/1.1 200 OK\r\n"
					"Cache-Control: no-store, no-cache, must-revalidate\r\n"
					"Cache-Control: post-check=0, pre-check=0\r\n"
					"Pragma: no-cache\r\nConnection: close\r\n"
					"Content-Length: %d\r\n%s\r\n\r\n", size, extra);
		else
			ret = mg_printf(conn, "HTTP/1.1 200 OK\r\n"
					"Cache-Control: no-store, no-cache, must-revalidate\r\n"
					"Cache-Control: post-check=0, pre-check=0\r\n"
					"Pragma: no-cache\r\nConnection: close\r\n"
					"Content-Length: %d\r\n\r\n", size);
	} else {
		if (extra)
			ret = mg_printf(conn, "HTTP/1.1 200 OK\r\n"
					"Cache-Control: no-store, no-cache, must-revalidate\r\n"
					"Cache-Control: post-check=0, pre-check=0\r\n"
					"Pragma: no-cache\r\nConnection: close\r\n%s\r\n\r\n",
					extra);
		else
			ret = mg_printf(conn, "HTTP/1.1 200 OK\r\n"
					"Cache-Control: no-store, no-cache, must-revalidate\r\n"
					"Cache-Control: post-check=0, pre-check=0\r\n"
					"Pragma: no-cache\r\nConnection: close\r\n\r\n");
	}

	return ret;
}

// Print message to buffer. If buffer is large enough to hold the message,
// return buffer. If buffer is to small, allocate large enough buffer on heap,
// and return allocated buffer.
static int alloc_vprintf(char **buf, size_t size, const char *fmt, va_list ap) {
	va_list ap_copy;
	int len;

	// Windows is not standard-compliant, and vsnprintf() returns -1 if
	// buffer is too small. Also, older versions of msvcrt.dll do not have
	// _vscprintf().  However, if size is 0, vsnprintf() behaves correctly.
	// Therefore, we make two passes: on first pass, get required message length.
	// On second pass, actually print the message.
	va_copy(ap_copy, ap);
	len = vsnprintf(NULL, 0, fmt, ap_copy);

	if (len > (int) size && (size = len + 1) > 0 &&
			(*buf = (char *) malloc(size)) == NULL)
	{
		len = -1; // Allocation failed, mark failure
	} else {
		va_copy(ap_copy, ap);
		vsnprintf(*buf, size, fmt, ap_copy);
	}

	return len;
}
#define MG_BUF_LEN 8192
//#define send_replay(_conn_,_fmt_,_cmd_,_success_,_msg_,args...) \
//		_send_replay(_conn_,_fmt_,_cmd_,_success_,_msg_,##args)

static int send_replay(struct mg_connection *conn, char *fmt,
		/*char* cmd, int success,char* msg,*/...) {
	char mem[MG_BUF_LEN], *buf = mem;
	int len, ret;
	char cb[64];
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	int json_call = handle_jsonp(conn, request_info, cb);

	va_list ap;
	va_start(ap, fmt);

	if ((len = alloc_vprintf(&buf, sizeof(mem), fmt, ap)) > 0) {
		if (json_call > 0) {
			ret = send_ok(conn, "Content-Type: application/x-javascript",
					len + json_call + 2);
			ret += mg_printf(conn, "%s(", cb);
			ret += mg_write(conn, buf, len);
			ret += mg_printf(conn, "%c", ')');
		} else {
			ret = send_ok(conn, "Content-Type: text/json; charset=UTF-8", len);
			ret += mg_write(conn, buf, len);
		}
	}
	if (buf != mem && buf != NULL) {
		free(buf);
	}

	return ret;
}

static int send_login_ok(struct mg_connection *conn, char *sessionid,
		char *user, char *fmt, ...) {
	char extra[1024], mem[MG_BUF_LEN - 1024], *buf = mem;
	int len, ret;
	va_list ap;
	va_start(ap, fmt);

	if ((len = alloc_vprintf(&buf, sizeof(mem), fmt, ap)) > 0) {
		snprintf(extra, 1024,
				"Set-Cookie: session=%s; max-age=3600; http-only\r\nSet-Cookie: user=%s",
				sessionid, user);
		ret = send_ok(conn, extra, len);
		ret += mg_write(conn, buf, len);
	}
	if (buf != mem && buf != NULL) {
		free(buf);
	}

	return ret;
}

static void send_jpg(struct mg_connection *conn) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	if (!display.buf) {
		mg_printf(conn, "HTTP/1.1 302 Found\r\n"
				"Set-Cookie: original_url=%s\r\n"
				"Location: %s\r\n\r\n", request_info->uri, "/placeholder.jpg");
	} else {
		if (server.w != display.w || server.h != display.h) {
			//原图发生变化，重置缓冲区
			free(server.jpg_buf);
			server.jpg_buf = NULL;
			server.buf_size = NULL;
			server.w = display.w;
			server.h = display.h;
		}
		if (server.hash != display.hash) {
			gray_to_jpeg_mem(display.w, display.h, display.buf, 50,
					&server.jpg_buf, &server.buf_size);
			server.hash = display.hash;
		}
		send_ok(conn,
				"Content-Type: image/jpeg;"
				//"charset=UTF-8\r\n"
				//"Content-Disposition: attachment;filename=gray.jpg"
				, server.buf_size);

		mg_write(conn, server.jpg_buf, server.buf_size);
//		} else {
//			mg_printf(conn, "HTTP/1.1 304 Not Modified\r\n\r\n");
//		}
	}
}

static int do_command(struct mg_connection *conn, const char *cmd) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	int ret = 1;

	if (urlcompare(cmd, "help")) {
		send_replay(conn, REPLAY_FORMAT("\"commands\":[%s]"), cmd, 1, "",
				COMMANDS);
	} else if (urlcompare(cmd, "availablePlusDelay")) {
		send_replay(conn, REPLAY_FORMAT("\"availablePlusDelay\": [%s]"), cmd, 1,
				"", AVIABLEPLUSDELAY);
	} else if (urlcompare(cmd, "availableFrequency")) {
		send_replay(conn, REPLAY_FORMAT("\"availableFrequency\": [%s]"), cmd, 1,
				"", AVIABLEFREQUENCY);
	} else if (urlcompare(cmd, "availableResolution")) {
		send_replay(conn, REPLAY_FORMAT("\"availableResolution\": [%s]"), cmd,
				1, "", AVIABLERESOLUTION);
	} else if (urlcompare(cmd, "availablePrecision")) {
		send_replay(conn, REPLAY_FORMAT("\"availablePrecision\": [%s]"), cmd, 1,
				"", AVIABLEPRECISION);
	} else if (urlcompare(cmd, "authorize")) {
		struct session * session = authorize_ex(conn, request_info);
		if (session)
			send_login_ok(conn, (char*) session->session_id,
					(char*) session->user,
					REPLAY_FORMAT("\"userlevel\": \"%s\""), cmd, 1,
					"login success.", "admin");
		else {
			send_replay(conn, REPLAY_FORMAT("\"userlevel\": \"%s\""), cmd, 0,
					"login failed.", "unauthorize");
		}
	} else {
		ret = 0;
	}

	return ret;
}

static int do_ls300_command(struct mg_connection *conn, const char *cmd) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	int ret = 1;
	char param[32];
	if (urlcompare(cmd, "gray.jpg")) {
		send_jpg(conn);
	} else if (urlcompare(cmd, "graysize")) {
		send_replay(conn,
				REPLAY_FORMAT("\"size\": { \"width\": %d, \"height\": %d}"),
				cmd, 1, "", display.w, display.h);
	} else if (urlcompare(cmd, "angle")) {
		send_replay(conn, REPLAY_FORMAT("\"angle\": %8.4f"), cmd, 1, "",
				server.ls300m->angle);
	} else if (urlcompare(cmd, "tilt")) {
		send_replay(conn, REPLAY_FORMAT("\"dx\": %8.4f,\"dy\":%8.4f"), cmd, 1,
				"", server.ls300m->tilt.dX, server.ls300m->tilt.dY);
	} else if (urlcompare(cmd, "temperature")) {
		send_replay(conn, REPLAY_FORMAT("\"temperature\": %8.4f"), cmd, 1, "",
				server.ls300m->temperature);
	} else if (urlcompare(cmd, "battery")) {
		send_replay(conn, REPLAY_FORMAT("\"battery\": %8.4f"), cmd, 1, "",
				server.ls300m->battery);
	} else if (urlcompare(cmd, "devicestate")) {
		send_replay(conn, REPLAY_FORMAT("\"state\": \"%s\""), cmd, 1, "",
				sj_state(server.ls300));
	} else if (urlcompare(cmd, "config")) {
		e_uint32 plusdelay, frequency;
		e_float64 resolution, start_angle_h, end_angle_h, start_angle_v,
				end_angle_v;
		get_request_param_int(request_info, param, plusdelay, 100);
		get_request_param_int(request_info, param, frequency, 5);
		get_request_param_float(request_info, param, resolution, 0.25);
		get_request_param_float(request_info, param, start_angle_h, 0);
		get_request_param_float(request_info, param, end_angle_h, 360);
		get_request_param_float(request_info, param, start_angle_v, -45);
		get_request_param_float(request_info, param, end_angle_v, 90);

		ret = sj_config(server.ls300, plusdelay, start_angle_h, end_angle_h,
				frequency, resolution, start_angle_v, end_angle_v);
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Laser scan config failed.device busy.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Laser scan config success.");
		}
	} else if (urlcompare(cmd, "pointscan")) {
		ret = sj_scan_point(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Laser scan start scan point cloud failed.device busy.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Laser scan start scan point cloud successfull.");
		}
	} else if (urlcompare(cmd, "photoscan")) {
		ret = sj_scan_photo(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Laser scan start scan photo failed.device busy.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Laser scan start scan photo successfull.");
		}
	} else if (urlcompare(cmd, "cancel")) {
		ret = sj_cancel(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Laser scan stop failed.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Laser scan stopped.");
		}
	} else if (urlcompare(cmd, "takephoto")) {
		ret = lm_camera_take_photo();
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Laser scan take photo failed.device busy.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Laser scan take photo successfull.");
		}
	} else if (urlcompare(cmd, "led")) {
		ret = get_qsvar(request_info, "color", param, sizeof(param));
		if (ret <= 0)
			send_replay(conn, REPLAY_FORMAT("\"led\": \"%s\""), cmd, 1, "",
					lm_led_state());
		else {
			if (!strcmp(param, "red"))
				ret = lm_led_red();
			else if (!strcmp(param, "green"))
				ret = lm_led_green();
			else if (!strcmp(param, "off"))
				ret = lm_led_off();
			else
				ret = E_ERROR;

			if (e_failed(ret)) {
				send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
						"Operate led failed.");
			} else {
				send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
						"Operate led success.");
			}
		}
	} else if (urlcompare(cmd, "turntable")) {
		float angle;
		get_request_param_float(request_info, param, angle, 0);
		if (fabs(angle) < 1e-6)
			ret = lm_turntable_stop();
		else
			ret = lm_turntable_fast_turn(angle);

		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Operate turntable failed.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Operate turntable successful.");
		}
	} else if (urlcompare(cmd, "searchzero")) {
		ret = lm_turntable_searchzero();
		if (e_failed(ret)) {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 0,
					"Operate turntable failed.");
		} else {
			send_replay(conn, SIMPLE_REPLAY_FORMAT, cmd, 1,
					"Operate turntable successful.");
		}
	} else {
		ret = 0;
	}

	return ret;
}
static int enter_ls300_evn() {
	server.ls300 = sj_global_instance();
	server.ls300m = lm_get_instance();
	e_assert(server.ls300 && server.ls300m, 0);
	return 1;
}

#include "web_server.cm"

//#define USE_AUTHORIZE 1

static int begin_request_handler(struct mg_connection *conn) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	char q[32] = { 0 };

	fprintf(stderr, "Access URI:%s rq=%s\n", request_info->uri,
			request_info->query_string);

	char* ua = mg_get_header(conn, "User-Agent");
#ifdef USE_AUTHORIZE
	if (!ua || strcasecmp(ua, "LS300CLIENT")) {
		//此处用于常规页面验证
		if (!is_authorized(conn, request_info)) {
			redirect_to_login(conn, request_info);
		} else if (strcmp(request_info->uri, authorize_url) == 0) {
			authorize(conn, request_info);
		}
	}

	if (authorize_ex(conn, request_info) && ae_handler(conn))
	return 1;
#else
	if (ae_handler(conn))
		return 1;
#endif

	if (get_qsvar(request_info, "q", q, sizeof(q)) <= 0)
		return 0;

#ifdef USE_AUTHORIZE
	if (strcasecmp(q, "authorize") && !authorize_ex(conn, request_info)) {
		mg_printf(conn, "%s", reply_unzutorized);
		goto done;
	}
#endif

	if (!is_command_avaiable(q)) {
		send_replay(conn, SIMPLE_REPLAY_FORMAT, q, 0, "unknown command.");
		goto done;
	}

	if (do_command(conn, q))
		goto done;

	if (!enter_ls300_evn()) {
		if (urlcompare(q, "gray.jpg")) {
			mg_printf(conn, "HTTP/1.1 302 Found\r\n"
					"Set-Cookie: original_url=%s\r\n"
					"Location: %s\r\n\r\n", request_info->uri,
					"/placeholder.jpg");
		}
		mg_printf(conn,
				"{ \"query\": \"%s\", \"success\": 0, \"message\":[\"ls300 is not ready.\"] }",
				q);
		goto done;
	}

	do_ls300_command(conn, q);

//	if (is_jsonp) {
//		mg_printf(conn, "%s", ")");
//	}

	done:

	return 1;
}

static struct mg_context *ctx = NULL;

e_int32 webserver_start(char *root_dir) {
	struct mg_callbacks callbacks;
	const char *options[] = { "listening_ports", "8082", "document_root",
			root_dir, NULL };

	if (ctx)
		return 0;

	init_server();

	hd_strncpy(document_root, root_dir, 1024);

	memset(&callbacks, 0, sizeof(callbacks));
	callbacks.begin_request = begin_request_handler;
	ctx = mg_start(&callbacks, NULL, options);
	DMSG((STDOUT,"web server running...\n"));
	return 1;
}

e_int32 webserver_stop(void) {
	if (!ctx)
		return 0;
	free(server.jpg_buf);
	lm_uninit();
	sj_destroy(server.ls300);
	mg_stop(ctx);
	ctx = NULL;
	return 1;
}
