/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AM_CAM_DEBUG_H_
#define AM_CAM_DEBUG_H_

#define DEBUG_SENTENCE_MAX		300

struct am_cam_debug_log {
	size_t			dsentence_pos;
	char			dsentence[DEBUG_SENTENCE_MAX];
};

void am_cam_dmsg_concate(struct am_cam_debug_log *log, const char *fmt, ...);
char * am_cam_dmsg_print(struct am_cam_debug_log *log);

/* #define USE_DLOG */

#ifdef USE_DLOG
#define DLOG_INIT()		struct am_cam_debug_log am_cam_debug_log = { .dsentence_pos = 0 }
#define DLOG(fmt, ...)		am_cam_dmsg_concate(&am_cam_debug_log, fmt, ##__VA_ARGS__)
#define DLOG_OUT()		am_cam_dmsg_print(&am_cam_debug_log)
#else
#define DLOG_INIT()
#define DLOG(fmt, ...)
#define DLOG_OUT()		""
#endif

#endif
