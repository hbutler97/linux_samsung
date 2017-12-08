/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AM_CAM_FRAMEMGR_H_
#define AM_CAM_FRAMEMGR_H_

#include <linux/kthread.h>

#define AM_CAM_MAX_FRAMES		12
#define AM_CAM_MAX_PLANES		4

#define framemgr_e_barrier_irqs(framemgr, index, flag) \
	framemgr->sindex |= index; spin_lock_irqsave(&framemgr->slock, flag)
#define framemgr_x_barrier_irqr(framemgr, index, flag) \
	spin_unlock_irqrestore(&framemgr->slock, flag); framemgr->sindex &= ~index
#define framemgr_e_barrier_irq(framemgr, index) \
	framemgr->sindex |= index; spin_lock_irq(&framemgr->slock)
#define framemgr_x_barrier_irq(framemgr, index) \
	spin_unlock_irq(&framemgr->slock); framemgr->sindex &= ~index
#define framemgr_e_barrier(framemgr, index) \
	framemgr->sindex |= index; spin_lock(&framemgr->slock)
#define framemgr_x_barrier(framemgr, index) \
	spin_unlock(&framemgr->slock); framemgr->sindex &= ~index

enum am_cam_frame_state {
	AM_CAM_FRAME_STATE_FREE = 1,
	AM_CAM_FRAME_STATE_REQUEST,
	AM_CAM_FRAME_STATE_PREPARE,
	AM_CAM_FRAME_STATE_PROCESS,
	AM_CAM_FRAME_STATE_COMPLETE,
	AM_CAM_FRAME_STATE_INVALID
};

struct am_cam_frame {
	struct list_head		list;
	struct kthread_work		work;
	u32				state;

	u32				dvaddr_buffer[AM_CAM_MAX_PLANES];
	ulong 				kvaddr_buffer[AM_CAM_MAX_PLANES];

	u32				id;
	u32				index;
	u32				findex;
};

struct am_cam_framemgr {
	u32				id;
	u32				sindex;
	spinlock_t			slock;
	struct am_cam_frame		frame[AM_CAM_MAX_FRAMES];

	struct list_head		fre_list;
	struct list_head		req_list;
	struct list_head		pre_list;
	struct list_head		pro_list;
	struct list_head		com_list;

	u32				tot_cnt;
	u32				fre_cnt;
	u32				req_cnt;
	u32				pre_cnt;
	u32				pro_cnt;
	u32				com_cnt;
};

void am_cam_frame_s_free(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_g_free(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_free_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_free_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_free_list(struct am_cam_framemgr *framemgr);
void am_cam_frame_search_g_free(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);

void am_cam_frame_s_request(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_g_request(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_request_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_request_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_request_list(struct am_cam_framemgr *framemgr);

void am_cam_frame_s_prepare(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_g_prepare(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_prepare_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_prepare_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_prepare_list(struct am_cam_framemgr *framemgr);

void am_cam_frame_s_process(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_g_process(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_process_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_process_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_process_list(struct am_cam_framemgr *framemgr);

void am_cam_frame_s_complete(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_g_complete(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_complete_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_complete_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_complete_list(struct am_cam_framemgr *framemgr);

void am_cam_frame_trans_fre_to_req(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_req_to_pre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_req_to_pro(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_req_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_req_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_pre_to_pro(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_pre_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_pre_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_pro_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_pro_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_com_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);
void am_cam_frame_trans_any_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame);

void am_cam_frame_pick_fre_to_req(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame);
void am_cam_frame_print_all(struct am_cam_framemgr *framemgr);

int am_cam_frame_init(struct am_cam_framemgr *framemgr);
int am_cam_frame_deinit(struct am_cam_framemgr *framemgr);
void am_cam_frame_flush(struct am_cam_framemgr *framemgr);

#endif
