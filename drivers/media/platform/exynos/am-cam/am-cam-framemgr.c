/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "am-cam-config.h"
#include "am-cam-framemgr.h"
#include "am-cam-debug.h"

void am_cam_frame_s_free(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	frame->state = AM_CAM_FRAME_STATE_FREE;

	list_add_tail(&frame->list, &framemgr->fre_list);
	framemgr->fre_cnt++;
}

void am_cam_frame_g_free(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->fre_cnt &&
		(*frame = container_of(framemgr->fre_list.next, struct am_cam_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->fre_cnt--;
		(*frame)->state = AM_CAM_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_search_g_free(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	struct am_cam_frame *temp_frame, *target_frame;

	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->fre_cnt) {
		list_for_each_entry_safe(target_frame, temp_frame, &framemgr->fre_list, list) {
			if (target_frame->index == (*frame)->index) {
				list_del(&(*frame)->list);
				framemgr->fre_cnt--;
			}
		}
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_free_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->fre_cnt) {
		*frame = container_of(framemgr->fre_list.next, struct am_cam_frame, list);
		/* printk("free(%d) (%p)\n", (*frame)->index, *frame); */
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_free_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->fre_cnt)
		*frame = container_of(framemgr->fre_list.prev, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_print_free_list(struct am_cam_framemgr *framemgr)
{
	struct am_cam_frame *frame, *temp;
	DLOG_INIT();

	DLOG("[FRM] fre(%d, %d) :", framemgr->id, framemgr->fre_cnt);
	list_for_each_entry_safe(frame, temp, &framemgr->fre_list, list) {
		DLOG("%d->", frame->index);
	}
	DLOG("X");

	am_cam_info("%s\n", DLOG_OUT());
}

void am_cam_frame_s_request(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	frame->state = AM_CAM_FRAME_STATE_REQUEST;

	list_add_tail(&frame->list, &framemgr->req_list);
	framemgr->req_cnt++;
}

void am_cam_frame_g_request(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->req_cnt &&
		(*frame = container_of(framemgr->req_list.next, struct am_cam_frame, list))) {
		/* list_del corruption */
		/* list_del(&(*frame)->list); */
		/* framemgr->req_cnt--; */
		(*frame)->state = AM_CAM_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_request_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->req_cnt)
		*frame = container_of(framemgr->req_list.next, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_request_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->req_cnt)
		*frame = container_of(framemgr->req_list.prev, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_print_request_list(struct am_cam_framemgr *framemgr)
{
	struct am_cam_frame *frame, *temp;
	int count = 0;
	DLOG_INIT();

	DLOG("[FRM] req(%d, %d) :", framemgr->id, framemgr->req_cnt);
	if (count == framemgr->req_cnt)
		return;

	list_for_each_entry_safe(frame, temp, &framemgr->req_list, list) {
		if (count == framemgr->req_cnt) {
			am_cam_err(" count(%d) req_cnt(%d)\n", count, framemgr->req_cnt);
			break;
		}

		DLOG("%d->", frame->index);
		count ++;

		if(count > 100)
			BUG();

	}
	DLOG("X");
	count = 0;

	am_cam_info("%s\n", DLOG_OUT());
}

void am_cam_frame_s_prepare(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	frame->state = AM_CAM_FRAME_STATE_PREPARE;

	list_add_tail(&frame->list, &framemgr->pre_list);
	framemgr->pre_cnt++;
}

void am_cam_frame_g_prepare(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->pre_cnt &&
		(*frame = container_of(framemgr->pre_list.next, struct am_cam_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->pre_cnt--;
		(*frame)->state = AM_CAM_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_prepare_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->pre_cnt)
		*frame = container_of(framemgr->pre_list.next, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_prepare_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->pre_cnt)
		*frame = container_of(framemgr->pre_list.prev, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_print_prepare_list(struct am_cam_framemgr *framemgr)
{
	struct am_cam_frame *frame, *temp;
	DLOG_INIT();

	DLOG("[FRM] pre(%d, %d) :", framemgr->id, framemgr->pre_cnt);
	list_for_each_entry_safe(frame, temp, &framemgr->pre_list, list) {
		DLOG("%d->", frame->index);
	}
	DLOG("X");

	am_cam_info("%s\n", DLOG_OUT());
}

void am_cam_frame_s_process(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	frame->state = AM_CAM_FRAME_STATE_PROCESS;

	list_add_tail(&frame->list, &framemgr->pro_list);
	framemgr->pro_cnt++;
}

void am_cam_frame_g_process(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->pro_cnt &&
		(*frame = container_of(framemgr->pro_list.next, struct am_cam_frame, list))) {
		/* list_del corruption */
		/* list_del(&(*frame)->list); */
		/* framemgr->pro_cnt--; */
		(*frame)->state = AM_CAM_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_process_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->pro_cnt)
		*frame = container_of(framemgr->pro_list.next, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_process_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->pro_cnt)
		*frame = container_of(framemgr->pro_list.prev, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_print_process_list(struct am_cam_framemgr *framemgr)
{
	struct am_cam_frame *frame, *temp;
	int count = 0;
	DLOG_INIT();

	DLOG("[FRM] pro(%d, %d) :", framemgr->id, framemgr->pro_cnt);
	am_cam_devi(" pro(%d, %d):", framemgr->id, framemgr->pro_cnt);
	if (count == framemgr->pro_cnt)
		return;

	list_for_each_entry_safe(frame, temp, &framemgr->pro_list, list) {
		if (count == framemgr->pro_cnt)
			break;

		DLOG("%d->", frame->index);
		am_cam_devi(" %d->", frame->index);
		count ++;

		if(count > 100)
			BUG();
	}
	DLOG("X");
	am_cam_devi("X\n");
	count = 0;

	am_cam_info("%s\n", DLOG_OUT());
}

void am_cam_frame_s_complete(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	frame->state = AM_CAM_FRAME_STATE_COMPLETE;

	list_add_tail(&frame->list, &framemgr->com_list);
	framemgr->com_cnt++;
}

void am_cam_frame_g_complete(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->com_cnt &&
		(*frame = container_of(framemgr->com_list.next, struct am_cam_frame, list))) {
		/* list_del corruption */
		/* list_del(&(*frame)->list); */
		/* framemgr->com_cnt--; */
		(*frame)->state = AM_CAM_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void am_cam_frame_complete_head(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->com_cnt)
		*frame = container_of(framemgr->com_list.next, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_complete_tail(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	if (framemgr->com_cnt)
		*frame = container_of(framemgr->com_list.prev, struct am_cam_frame, list);
	else
		*frame = NULL;
}

void am_cam_frame_print_complete_list(struct am_cam_framemgr *framemgr)
{
	struct am_cam_frame *frame, *temp;
	DLOG_INIT();

	DLOG("[FRM] com(%d, %d) :", framemgr->id, framemgr->com_cnt);
	list_for_each_entry_safe(frame, temp, &framemgr->com_list, list) {
		DLOG("%d->", frame->index);
	}
	DLOG("X");

	am_cam_info("%s\n", DLOG_OUT());
}

void am_cam_frame_trans_fre_to_req(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->fre_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_FREE);

	list_del(&frame->list);
	framemgr->fre_cnt--;
	am_cam_frame_s_request(framemgr, frame);
}

void am_cam_frame_trans_req_to_pre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->req_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->req_cnt--;
	am_cam_frame_s_prepare(framemgr, frame);
}

void am_cam_frame_trans_req_to_pro(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->req_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->req_cnt--;
	am_cam_frame_s_process(framemgr, frame);
}

void am_cam_frame_trans_req_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->req_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->req_cnt--;
	am_cam_frame_s_complete(framemgr, frame);
}

void am_cam_frame_trans_req_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->req_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->req_cnt--;
	am_cam_frame_s_free(framemgr, frame);
}

void am_cam_frame_trans_pre_to_pro(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->pre_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->pre_cnt--;
	am_cam_frame_s_process(framemgr, frame);
}

void am_cam_frame_trans_pre_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->pre_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->pre_cnt--;
	am_cam_frame_s_complete(framemgr, frame);
}

void am_cam_frame_trans_pre_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->pre_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->pre_cnt--;
	am_cam_frame_s_free(framemgr, frame);
}

void am_cam_frame_trans_pro_to_com(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->pro_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_PROCESS);

	list_del(&frame->list);
	framemgr->pro_cnt--;
	am_cam_frame_s_complete(framemgr, frame);
}

void am_cam_frame_trans_pro_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->pro_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_PROCESS);

	list_del(&frame->list);
	framemgr->pro_cnt--;
	am_cam_frame_s_free(framemgr, frame);
}

void am_cam_frame_trans_com_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->com_cnt);
	BUG_ON(frame->state != AM_CAM_FRAME_STATE_COMPLETE);

	list_del(&frame->list);
	framemgr->com_cnt--;
	am_cam_frame_s_free(framemgr, frame);
}

void am_cam_frame_trans_any_to_fre(struct am_cam_framemgr *framemgr, struct am_cam_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	list_del(&frame->list);
	switch (frame->state) {
	case AM_CAM_FRAME_STATE_REQUEST:
		framemgr->req_cnt--;
		break;
	case AM_CAM_FRAME_STATE_PREPARE:
		framemgr->pre_cnt--;
		break;
	case AM_CAM_FRAME_STATE_PROCESS:
		framemgr->pro_cnt--;
		break;
	case AM_CAM_FRAME_STATE_COMPLETE:
		framemgr->com_cnt--;
		break;
	default:
		BUG();
		break;
	}

	am_cam_frame_s_free(framemgr, frame);
}

void am_cam_frame_pick_fre_to_req(struct am_cam_framemgr *framemgr, struct am_cam_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	am_cam_frame_free_head(framemgr, frame);
	if (*frame)
		am_cam_frame_trans_fre_to_req(framemgr, *frame);
}

void am_cam_frame_print_all(struct am_cam_framemgr *framemgr)
{
	BUG_ON(!framemgr);

	am_cam_frame_print_request_list(framemgr);
	am_cam_frame_print_prepare_list(framemgr);
	am_cam_frame_print_process_list(framemgr);
	am_cam_frame_print_complete_list(framemgr);
	am_cam_frame_print_free_list(framemgr);
}

int am_cam_frame_init(struct am_cam_framemgr *framemgr)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&framemgr->slock, flags);

	INIT_LIST_HEAD(&framemgr->fre_list);
	INIT_LIST_HEAD(&framemgr->req_list);
	INIT_LIST_HEAD(&framemgr->pre_list);
	INIT_LIST_HEAD(&framemgr->pro_list);
	INIT_LIST_HEAD(&framemgr->com_list);

	framemgr->tot_cnt = AM_CAM_MAX_FRAMES;
	framemgr->fre_cnt = 0;
	framemgr->req_cnt = 0;
	framemgr->pre_cnt = 0;
	framemgr->pro_cnt = 0;
	framemgr->com_cnt = 0;

	for (index = 0; index < framemgr->tot_cnt; ++index) {
		framemgr->frame[index].index = index;
		framemgr->frame[index].findex = AM_CAM_MAX_FRAMES;
		am_cam_frame_s_free(framemgr, &framemgr->frame[index]);
	}

	spin_unlock_irqrestore(&framemgr->slock, flags);

	return ret;
}

int am_cam_frame_deinit(struct am_cam_framemgr *framemgr)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&framemgr->slock, flags);

	for (index = 0; index < framemgr->tot_cnt; ++index)
		am_cam_frame_s_free(framemgr, &framemgr->frame[index]);

	spin_unlock_irqrestore(&framemgr->slock, flags);

	return ret;
}

void am_cam_frame_flush(struct am_cam_framemgr *framemgr)
{
	unsigned long flag;
	struct am_cam_frame *frame, *temp;
	u32 index;

	BUG_ON(!framemgr);

	spin_lock_irqsave(&framemgr->slock, flag);

	am_cam_info("req(%d) pre(%d) pro(%d) com(%d) fre(%d) \n",
		framemgr->req_cnt,
		framemgr->pre_cnt,
		framemgr->pro_cnt,
		framemgr->com_cnt,
		framemgr->fre_cnt);

	list_for_each_entry_safe(frame, temp, &framemgr->req_list, list) {
		am_cam_info("req frame%d (count : %d)", frame->index, framemgr->req_cnt);
		list_del(&frame->list);
		am_cam_info("req frame%d is flushed(count : %d)\n", frame->index, framemgr->req_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->pre_list, list) {
		am_cam_info("pre frame%d (count : %d)", frame->index, framemgr->pre_cnt);
		list_del(&frame->list);
		am_cam_info("pre frame%d is flushed(count : %d)\n", frame->index, framemgr->pre_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->pro_list, list) {
		am_cam_info("pro frame%d (count : %d)", frame->index, framemgr->pro_cnt);
		list_del(&frame->list);
		am_cam_info("pro frame%d is flushed(count : %d)\n", frame->index, framemgr->pro_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->com_list, list) {
		am_cam_info("com frame%d (count : %d)", frame->index, framemgr->com_cnt);
		list_del(&frame->list);
		am_cam_info("com frame%d is flushed(count : %d)\n", frame->index, framemgr->com_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->fre_list, list) {
		am_cam_info("fre frame%d (count : %d)", frame->index, framemgr->fre_cnt);
		list_del(&frame->list);
		am_cam_info("fre frame%d is flushed(count : %d)\n", frame->index, framemgr->fre_cnt);
	}

	framemgr->req_cnt = 0;
	framemgr->pre_cnt = 0;
	framemgr->pro_cnt = 0;
	framemgr->com_cnt = 0;
	framemgr->fre_cnt = 0;

	for (index = 0; index < framemgr->tot_cnt; ++index) {
		framemgr->frame[index].index = index;
		framemgr->frame[index].findex = AM_CAM_MAX_FRAMES;
		am_cam_frame_s_free(framemgr, &framemgr->frame[index]);
	}
	am_cam_info("frame free_cnt(%d) total(%d)",
			frame->index, framemgr->com_cnt);

	spin_unlock_irqrestore(&framemgr->slock, flag);

	/* BUG_ON(framemgr->fre_cnt != (framemgr->tot_cnt - 1)); */
}
