/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/delay.h>

#include "am-cam-config.h"
#include "am-cam-debug.h"

void am_cam_dmsg_concate(struct am_cam_debug_log *log, const char *fmt, ...)
{
	va_list ap;
	char term[50];
	u32 copy_len;

	va_start(ap, fmt);
	vsnprintf(term, sizeof(term), fmt, ap);
	va_end(ap);

	if (log->dsentence_pos >= DEBUG_SENTENCE_MAX) {
		am_cam_err("debug message(%zd) over max\n", log->dsentence_pos);
		return;
	}

	copy_len = min((DEBUG_SENTENCE_MAX - log->dsentence_pos - 1), strlen(term));
	strncpy(log->dsentence + log->dsentence_pos, term, copy_len);
	log->dsentence_pos += copy_len;
	log->dsentence[log->dsentence_pos] = 0;
}

char * am_cam_dmsg_print(struct am_cam_debug_log *log)
{
	log->dsentence_pos = 0;
	return log->dsentence;
}
