#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include <ls300/hd_laser_scan.h>

scan_job_t* job;

//struct json_msg {
//	int ret_code;
//	char msg[];
//};

const char *get_sub_command(const char *cmd) {
	return cmd + 3;
}

static void sig_handler(int sig) {
	if (sig == SIGINT) {
		sj_cancel(job);
		sj_destroy(job);
		exit(0);
	}
}

int on_command(const char *cmd, char *ret_buf, int *ctx,
		int log_printf(char *buf, int *ctx, int code, char *fmt, ...)) {
	int ret = 1;
	//command select

	signal(SIGINT, sig_handler);

	if (!strncmp(cmd, "get", 3)) {
		const char *scmd = get_sub_command(cmd);
		if (!strcmp(scmd, "author")) {
			log_printf(ret_buf, ctx, ret, "uplusplus");
		} else if (!strcmp(scmd, "version")) {
			log_printf(ret_buf, ctx, ret, "1.0.0");
		} else {
			log_printf(ret_buf, ctx, 0, "unknown command: <b>%s</b>", cmd);
			ret = 0;
			goto end;
		}
	} else if (!strncmp(cmd, "connect", sizeof("connect"))) {
		if (job != NULL) {
			log_printf(ret_buf, ctx, 0, "Laser already connect..");
			return 0;
		}
		log_printf(ret_buf, ctx, ret, "Laser try create..");
		ret = sj_create(&job, NULL, NULL, NULL, NULL);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret, "Laser scan create failed.");
		} else {
			log_printf(ret_buf, ctx, ret, "Laser create successfull.");
		}
	} else if (!strncmp(cmd, "disconnect", sizeof("disconnect"))) {
		if (job == NULL) {
			log_printf(ret_buf, ctx, 0, "Laser already disconnect..");
			return 0;
		}
		log_printf(ret_buf, ctx, ret, "Laser try disconnect..");
		ret = sj_destroy(job);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret, "Laser scan disconnect failed.");
		} else {
			log_printf(ret_buf, ctx, ret, "Laser disconnect successfull.");
		}
		job = NULL;
	} else if (!strncmp(cmd, "config", sizeof("config"))) {
		log_printf(ret_buf, ctx, ret, "Laser scan try config..");
		sj_set_data_dir(job, "/sdcard/ls300/data/point_cloud",
				"/sdcard/ls300/data/image");
		ret = sj_config(job, 100, 0, 360, 5, 0.25, -45, 90);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret,
					"Laser scan config failed.device busy.");
		} else {
			log_printf(ret_buf, ctx, ret, "Laser scan config successfull.");
		}
	} else if (!strncmp(cmd, "pointscan", sizeof("pointscan"))) {
		log_printf(ret_buf, ctx, ret, "Laser scan try start..");
		ret = sj_scan_point(job);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret,
					"Laser scan start scan point cloud failed.device busy.");
		}
	} else if (!strncmp(cmd, "photoscan", sizeof("photoscan"))) {
		log_printf(ret_buf, ctx, ret, "Laser photo scan try start..");
		ret = sj_scan_photo(job);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret,
					"Laser scan start scan photo failed.device busy.");
		}
	} else if (!strncmp(cmd, "cancel", sizeof("cancel"))) {
		log_printf(ret_buf, ctx, ret, "Laser scan try stop..");
		ret = sj_cancel(job);
		if (e_failed(ret)) {
			log_printf(ret_buf, ctx, ret, "Laser scan stop failed.");
		}
	} else {
		log_printf(ret_buf, ctx, 0, "unknown command: <b>%s</b>", cmd);
		ret = 0;
		goto end;
	}

	end: return ret;
}
