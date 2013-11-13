// Copyright (c) 2004-2012 Sergey Lyubka
// This file is a part of mongoose project, http://github.com/valenok/mongoose

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <stdarg.h>
#include <pthread.h>
#include <server/hd_websocket.h>
#include "mongoose.h"
#include "process.h"
#include <comm/hd_utils.h>
#include <jpg/hd_jpeg.h>

int urlcompare(const char* url, char* patt) {
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

static void websocket_ready_handler(struct mg_connection *conn) {
	int i;
	static const char *message = "server ready";
	mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, message, strlen(message));
}

static int json_printf(char *buf, int *ctx, int code, char *fmt, ...) {
	char msg[1024] = { 0 };
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	(*ctx) += sprintf(buf + (*ctx), "{\"code\":%d,\"msg\":\"%s\"},", code, msg);
	return 1;
}

static int msg_printf(char *buf, int *ctx, int code, char *fmt, ...) {
	char msg[1024] = { 0 };
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	(*ctx) += sprintf(buf + (*ctx), "code:%d,msg:%s", code, msg);
	return 0;
}

static int cmd_phrase(struct mg_connection *conn, char* data, size_t len) {
	char msg[1024] = { 0 };
	int ctx = 0;
	data[len] = 0;
	ctx = sprintf(msg, "{\"query\":\"%s\",\"ret\":[", data);
	on_command(data, msg, &ctx, json_printf);
	ctx += sprintf(msg + ctx, "{}]}");
	mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, msg, strlen(msg));
	return 1;
}

// Arguments:
//   flags: first byte of websocket frame, see websocket RFC,
//          http://tools.ietf.org/html/rfc6455, section 5.2
//   data, data_len: payload data. Mask, if any, is already applied.
static int websocket_data_handler(struct mg_connection *conn, int flags,
		char *data, size_t data_len) {
	(void) flags; // Unused
	int ctx = 0;
	char msg[1024] = { 0 };

	if (!memcmp(data, "data", 4)) {
#if BASE64
		mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, gray_string, sizeof(gray_string));
#else
		if (display.buf != NULL)
			mg_websocket_write(conn, WEBSOCKET_OPCODE_BINARY, display.buf,
					display.h * display.w);
#endif
	} else if (!memcmp(data, "size", 4)) {
		ctx = sprintf(msg, "{\"query\":\"size\",\"ret\":");
		if (display.buf == NULL)
			ctx += sprintf(msg + ctx,
					"[{\"code\":0,\"msg\":\"display buf is null.\"}]}");
		else
			ctx += sprintf(msg + ctx,
					"{\"code\":1,\"size\":{\"w\":\"%d\",\"h\":\"%d\"}}}",
					display.w, display.h);
		mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, msg, strlen(msg));
	} else if (!memcmp(data, "exit", 4)) {
		printf("websocket_data_handler exit.");
		return 0;
	} else {
		cmd_phrase(conn, data, data_len);
	}

	return 1;
}

void send_ok(struct mg_connection *conn, const char* extra, int size) {
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

struct jpg_ctx {
	int hash;
	char* jpg_buf;
	int jpg_size;
};

static struct jpg_ctx jctx = { -1, 0, 0 };

void send_jpg(struct mg_connection *conn) {

	if(jctx.hash != display.hash){
		gray_to_jpeg(display.w, display.h, display.buf, 50, &jctx.jpg_buf,
				&jctx.jpg_size);
		jctx.hash = display.hash;
	}

	send_ok(conn,
			"Content-Type: image/jpeg; charset=UTF-8\r\nContent-Disposition: attachment;filename=gray.jpg",
			jctx.jpg_size);

	mg_write(conn, jctx.jpg_buf, jctx.jpg_size);
}

static int begin_request_handler(struct mg_connection *conn) {
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	int processed = 1;

	if (strstr(request_info->uri, "gray.jpg")) {
		send_jpg(conn);
	} else {
		// No suitable handler found, mark as not processed. Mongoose will
		// try to serve the request.
		processed = 0;
	}
	return processed;
}

static struct mg_context *ctx = NULL;

int websocket_start(char *root_dir) {

	if (ctx)
		return 0;

	struct mg_callbacks callbacks;
	const char *options[] = { "listening_ports", "8080", "document_root",
			root_dir, NULL };

	memset(&callbacks, 0, sizeof(callbacks));
	callbacks.websocket_ready = websocket_ready_handler;
	callbacks.websocket_data = websocket_data_handler;
	callbacks.begin_request = begin_request_handler;
	ctx = mg_start(&callbacks, NULL, options);
	DMSG((STDOUT,"web server running...\n"));
	return 1;
}

e_int32 websocket_stop(void) {
	if (!ctx)
		return 0;
	mg_stop(ctx);
	ctx = NULL;
	return 1;
}
