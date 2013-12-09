// Copyright (c) 2004-2012 Sergey Lyubka
// This file is a part of mongoose project, http://github.com/valenok/mongoose

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

#define HELP "'commands:authorize,devicestatus,gray.jpg, graysize, turntable," \
	"led, angle, tilt, temperature, battery, config, pointscan, photoscan," \
	"cancel, aviablePlusDelay,aviableFrequency,aviableResolution," \
	"aviablePrecises,help.'"
#define AVIABLEPLUSDELAY "20, 50, 100, 150, 200, 250, 400, 700, 850, 1250, 2500, 5000"
#define AVIABLEFREQUENCY "5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20"
#define AVIABLERESOLUTION "20, 50, 100, 150, 200, 250, 400, 700, 850, 1250, 2500, 5000"
#define AVIABLEPRECISION \
	"{name: 'PRECISION_LEVEL_LOW',  frequency: 5, resolution:  0.5,plusdelay: 50, widthL:  361, height:  271, time:  72},\
	{name: 'PRECISION_LEVEL_NORMAL',  frequency: 7, resolution:  0.375, plusdelay: 50, widthL:  505, height:  361, time:  72},\
	{name: 'PRECISION_LEVEL_MIDDLE',  frequency: 5, resolution:  0.25, plusdelay: 100, widthL:  721, height:  541, time:  144},\
	{name: 'PRECISION_LEVEL_HIGH',  frequency: 5, resolution:  0.125, plusdelay: 200, widthL:  1441, height:  1081, time:  288},\
	{name: 'PRECISION_LEVEL_EXTRA',  frequency: 7, resolution:  0.0625, plusdelay: 1250, widthL:  2884, height:  2164, time:  1800}"

static char *commands[] = { "authorize", "devicestatus", "gray.jpg",
		" graysize", "turntable", "led", "angle", "tilt", "temperature",
		" battery", "config", " pointscan", "photoscan", "cancel",
		" aviablePlusDelay", "aviableFrequency", "aviableResolution",
		"aviablePrecises", "help" };

static struct {
	int hash;
	char* jpg_buf;
	int jpg_size;
	scan_job_t* ls300;
	laser_machine_t* ls300m;
} server = { -1, 0, 0, 0 };

static const char *ajax_reply_start = "HTTP/1.1 200 OK\r\n"
		"Cache: no-cache\r\n"
		"Content-Type: application/x-javascript\r\n"
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

// If "callback" param is present in query string, this is JSONP call.
// Return 1 in this case, or 0 if "callback" is not specified.
// Wrap an output in Javascript function call.
static int handle_jsonp(struct mg_connection *conn,
		const struct mg_request_info *request_info) {
	char cb[64];

	get_qsvar(request_info, "callback", cb, sizeof(cb));
	if (cb[0] != '\0') {
		mg_printf(conn, "%s(", cb);
	}

	return cb[0] == '\0' ? 0 : 1;
}

static int send_replay(struct mg_connection *conn, int success, char *q,
		char *msg) {
	int ret;

	ret = mg_printf(conn, "{ query: '%s', success: %d, message:['%s'] }", q,
			success, msg);

	return ret;
}

static int send_size_info(struct mg_connection *conn) {
	int ret;
	if (display.buf == NULL)
		ret =
				mg_printf(conn,
						"{ query: 'size', success: 0, message:['display buf is null.'] }");
	else
		ret =
				mg_printf(conn,
						"{ query: 'size', success: 1, message:[], size: {width: %d, height: %d} }",
						display.w, display.h);
	return ret;
}

static void send_ok(struct mg_connection *conn, const char* extra, int size) {
	if (size) {
		if (extra)
			mg_printf(conn,
					"HTTP/1.1 200 OK\r\nCache-Control: no-store, no-cache, must-revalidate\r\nCache-Control: post-check=0, pre-check=0\r\nPragma: no-cache\r\nConnection: close\r\nContent-Length: %d\r\n%s\r\n\r\n",
					size, extra);
		else
			mg_printf(conn,
					"HTTP/1.1 200 OK\r\nCache-Control: no-store, no-cache, must-revalidate\r\nCache-Control: post-check=0, pre-check=0\r\nPragma: no-cache\r\nConnection: close\r\nContent-Length: %d\r\n\r\n",
					size);
	} else {
		if (extra)
			mg_printf(conn,
					"HTTP/1.1 200 OK\r\nCache-Control: no-store, no-cache, must-revalidate\r\nCache-Control: post-check=0, pre-check=0\r\nPragma: no-cache\r\nConnection: close\r\n%s\r\n\r\n",
					extra);
		else
			mg_printf(conn,
					"HTTP/1.1 200 OK\r\nCache-Control: no-store, no-cache, must-revalidate\r\nCache-Control: post-check=0, pre-check=0\r\nPragma: no-cache\r\nConnection: close\r\n\r\n");
	}
}

static void send_jpg(struct mg_connection *conn) {
	if (server.hash != display.hash) {
		gray_to_jpeg_mem(display.w, display.h, display.buf, 50, &server.jpg_buf,
				&server.jpg_size);
		server.hash = display.hash;

		send_ok(conn,
				"Content-Type: image/jpeg; charset=UTF-8\r\nContent-Disposition: attachment;filename=gray.jpg",
				server.jpg_size);

		mg_write(conn, server.jpg_buf, server.jpg_size);
	} else {
		mg_printf(conn, "HTTP/1.1 304 Not Modified\r\n\r\n");
	}
}

static int do_command(struct mg_connection *conn, const char *cmd) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	int ret = 1;
	char param[32];
	if (urlcompare(cmd, "help")) {

		mg_printf(conn, "{query: 'help', success: 1, message:[%s]}", HELP);
	} else if (urlcompare(cmd, "aviablePlusDelay")) {

		mg_printf(conn,
				"{query: 'aviablePlusDelay', success: 1, message:[''], aviablePlusDelay: [%s]}",
				AVIABLEPLUSDELAY);

	} else if (urlcompare(cmd, "aviableFrequency")) {

		mg_printf(conn,
				"{query: 'aviableFrequency', success: 1, message:[''],  aviableFrequency: [%s]}",
				AVIABLEFREQUENCY);

	} else if (urlcompare(cmd, "aviableResolution")) {

		mg_printf(conn,
				"{query: 'aviableResolution', success: 1, message:[''],  aviableResolution: [%s]}",
				AVIABLERESOLUTION);

	} else if (urlcompare(cmd, "aviablePrecision")) {

		mg_printf(conn,
				"{query: 'aviablePrecision', success: 1, message:[''], aviablePrecision: \n [%s]}",
				AVIABLEPRECISION);
	} else if (urlcompare(cmd, "authorize")) {
		authorize(conn, request_info);
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
		send_size_info(conn);
	} else if (urlcompare(cmd, "angle")) {
		mg_printf(conn,
				"{ query: 'angle', success: 1, message:[''] angle: %8.4f}",
				server.ls300m->angle);
	} else if (urlcompare(cmd, "tilt")) {
		mg_printf(conn,
				"{ query: 'tilt', success: 1, message:[''] dx: %8.4f dy:%8.4f}",
				server.ls300m->tilt.dX, server.ls300m->tilt.dY);
	} else if (urlcompare(cmd, "temperature")) {
		mg_printf(conn,
				"{ query: 'temperature', success: 1, message:[''] temperature: %8.4f}",
				server.ls300m->temperature);
	} else if (urlcompare(cmd, "battery")) {
		mg_printf(conn,
				"{ query: 'battery', success: 1, message:[''] battery: %8.4f}",
				server.ls300m->battery);
	} else if (urlcompare(cmd, "config")) {
		int plusdelay, frequency;
		float resolution, start_angle_h, end_angle_h, start_angle_v,
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
			send_replay(conn, 0, cmd, "Laser scan config failed.device busy.");
		} else {
			send_replay(conn, 1, cmd, "Laser scan config successfull.");
		}
	} else if (urlcompare(cmd, "pointscan")) {
		ret = sj_scan_point(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, 0, cmd,
					"Laser scan start scan point cloud failed.device busy.");
		} else {
			send_replay(conn, 1, cmd,
					"Laser scan start scan point cloud successfull.");
		}
	} else if (urlcompare(cmd, "photoscan")) {
		ret = sj_scan_photo(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, 0, cmd,
					"Laser scan start scan photo failed.device busy.");
		} else {
			send_replay(conn, 1, cmd,
					"Laser scan start scan photo successfull.");
		}
	} else if (urlcompare(cmd, "cancel")) {
		ret = sj_cancel(server.ls300);
		if (e_failed(ret)) {
			send_replay(conn, 0, cmd, "Laser scan stop failed.");
		} else {
			send_replay(conn, 1, cmd, "Laser scan stopped.");
		}
	} else if (urlcompare(cmd, "led")) {
		get_qsvar(request_info, "color", param, sizeof(param));
		if (!strcmp(param, "red"))
			ret = lm_led_red();
		else if (!strcmp(param, "green"))
			ret = lm_led_green();
		else
			ret = lm_led_off();
		if (e_failed(ret)) {
			send_replay(conn, 0, cmd, "Operate led failed.");
		} else {
			send_replay(conn, 1, cmd, "Operate led successful.");
		}
	} else if (urlcompare(cmd, "turntable")) {
		float angle;
		get_request_param_float(request_info, param, angle, 0);
		if (angle < 1e-6)
			ret = lm_turntable_stop();
		else
			ret = lm_turntable_turn_async(angle);

		if (e_failed(ret)) {
			send_replay(conn, 0, cmd, "Operate turntable failed.");
		} else {
			send_replay(conn, 1, cmd, "Operate turntable successful.");
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

static int begin_request_handler(struct mg_connection *conn) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	char q[32] = { 0 };
	int is_jsonp;

	if (!is_authorized(conn, request_info)) {
		redirect_to_login(conn, request_info);
	} else if (strcmp(request_info->uri, authorize_url) == 0) {
		authorize(conn, request_info);
	}

	if (get_qsvar(request_info, "q", q, sizeof(q)) <= 0)
		return 0;

	if (!is_command_avaiable(q)) {
		send_replay(conn, 0, q, "unknown command.");
		goto done;
	}

	is_jsonp = handle_jsonp(conn, request_info);
	if (is_jsonp)
		mg_printf(conn, "%s", ajax_reply_start);

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
				"{ query: '%s', success: 0, message:['ls300 is not ready.'] }",
				q);
		goto done;
	}
	if (do_ls300_command(conn, q))
		goto done;

	done: if (is_jsonp) {
		mg_printf(conn, "%s", ")");
	}

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

	memset(&callbacks, 0, sizeof(callbacks));
	callbacks.begin_request = begin_request_handler;
	ctx = mg_start(&callbacks, NULL, options);
	DMSG((STDOUT,"web server running...\n"));
	return 1;
}

e_int32 webserver_stop(void) {
	if (!ctx)
		return 0;
	mg_stop(ctx);
	ctx = NULL;
	return 1;
}
