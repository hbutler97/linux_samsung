/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AM_CAM_CONFIG_H_
#define AM_CAM_CONFIG_H_

/* features */
#define USE_END_TASKLET
#define CHK_OPEN_CNT

/* debug */
#define MAP_KVADDR
/* #define DBG_STM_SPINLOCK */

/* log */
#define DEBUG_LOG_MEMORY

#define probe_info(fmt, ...)		pr_info("[@]" fmt, ##__VA_ARGS__)
#define probe_warn(fmt, args...)	pr_warning("[@][WRN]" fmt, ##args)
#define probe_err(fmt, args...) 	pr_err("[@][ERR]%s:%d:" fmt, __func__, __LINE__, ##args)

#ifdef DEBUG_LOG_MEMORY
#define am_cam_err_target(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define am_cam_warn_target(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define am_cam_info_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define am_cam_dbg_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
/* #define am_cam_devi_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__) */
#define am_cam_devi_target(fmt, ...)
#else
#define am_cam_err_target(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define am_cam_warn_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define am_cam_info_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define am_cam_dbg_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define am_cam_devi_target(fmt, ...)
#endif

#define am_cam_err(fmt, args...) \
	am_cam_err_target("[@][ERR]%s:%d:" fmt, __func__, __LINE__, ##args)

#define am_cam_ierr(fmt, vctx, args...) \
	am_cam_err_target("[@][I%d][ERR]%s:%d:" fmt, vctx->id, __func__, __LINE__, ##args)

#define am_cam_irerr(fmt, vctx, frame, args...) \
	am_cam_err_target("[@][I%d][F%d][ERR]%s:%d:" fmt, vctx->id, frame->id, __func__, __LINE__, ##args)

#define am_cam_warn(fmt, args...) \
	am_cam_warn_target("[@][WRN]%s:%d:" fmt, __func__, __LINE__, ##args)

#define am_cam_iwarn(fmt, vctx, args...) \
	am_cam_warn_target("[@][I%d][WRN]%s:%d:" fmt, vctx->id, __func__, __LINE__, ##args)

#define am_cam_irwarn(fmt, vctx, frame, args...) \
	am_cam_warn_target("[@][I%d][F%d][WRN]%s:%d:" fmt, vctx->id, frame->id, __func__, __LINE__, ##args)

#define am_cam_info(fmt, args...) \
	am_cam_info_target("[@]" fmt, ##args)

#define am_cam_iinfo(fmt, vctx, args...) \
	am_cam_info_target("[@][I%d]" fmt, vctx->id, ##args)

#define am_cam_irinfo(fmt, vctx, frame, args...) \
	am_cam_info_target("[@][I%d][F%d]" fmt, vctx->id, frame->id, ##args)

#define am_cam_dbg(fmt, args...) \
	am_cam_dbg_target("[@]" fmt, ##args)

#define am_cam_idbg(fmt, vctx, args...) \
	am_cam_dbg_target("[@][I%d]" fmt, vctx->id, ##args)

#define am_cam_irdbg(fmt, vctx, frame, args...) \
	am_cam_dbg_target("[@][I%d][F%d]" fmt, vctx->id, frame->id, ##args)

#define am_cam_devi(fmt, args...) \
	am_cam_devi_target("[@]" fmt, ##args)

#endif
