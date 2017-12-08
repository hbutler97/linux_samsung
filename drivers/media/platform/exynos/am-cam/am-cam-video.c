/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <video/videonode.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/v4l2-mediabus.h>
#include <linux/bug.h>

#include "am-cam-config.h"
#include "am-cam-video.h"
#include "am-cam-device.h"
#include "sensor/sensor_module.h"
#include "am-cam-framemgr.h"
#include "hw/fimc-is-csi.h"

const struct v4l2_file_operations am_cam_video_fops;
const struct v4l2_ioctl_ops am_cam_video_ioctl_ops;
const struct vb2_ops am_cam_qops;

#ifdef DBG_FILL_COLOR
unsigned int debug_color;
#endif

static int __vref_open(struct am_cam_video *video)
{
	struct am_cam_device *device = container_of(video, struct am_cam_device, video);
	return am_cam_device_open(device);
}

static int __vref_close(struct am_cam_video *video)
{
	struct am_cam_device *device = container_of(video, struct am_cam_device, video);
	return am_cam_device_close(device);
}

static inline void __vref_init(struct am_cam_video_refcount *vref,
	struct am_cam_video *video, int (*first)(struct am_cam_video *video), int (*final)(struct am_cam_video *video))
{
	vref->video = video;
	vref->first = first;
	vref->final = final;
	atomic_set(&vref->refcount, 0);
}

static inline int __vref_get(struct am_cam_video_refcount *vref)
{
	return (atomic_inc_return(&vref->refcount) == 1) ? vref->first(vref->video) : 0;
}

static inline int __vref_put(struct am_cam_video_refcount *vref)
{
	return (atomic_dec_return(&vref->refcount) == 0) ? vref->final(vref->video) : 0;
}

int am_cam_video_probe(struct am_cam_video *video, struct v4l2_device *v4l2_dev)
{
	int ret = 0;
	char name[30];
	struct platform_device *pdev;
	struct am_cam_device *device;
	//device->pdev = pdev;

	BUG_ON(!video);

	snprintf(name, sizeof(name), "%s", "exynos-am-cam");

	mutex_init(&video->lock);
	__vref_init(&video->open_cnt, video, __vref_open, __vref_close);

	video->vb2_ops		= &am_cam_qops;
	video->vb2_mem_ops	= &vb2_ion_memops;

	//video->id		= video_number;
	//video->fimc_is_vb2_buf_ops = mem->fimc_is_vb2_buf_ops;
	//video->alloc_ctx	= mem->default_ctx;
	device = container_of(video, struct am_cam_device, video);
	pdev = device->pdev;
	video->alloc_ctx = vb2_ion_create_context(&pdev->dev,
			SZ_4K,
			VB2ION_CTX_IOMMU | VB2ION_CTX_VMCONTIG);

	snprintf(video->vd.name, sizeof(video->vd.name), "%s", "exynos-am-cam");
	video->vd.vfl_dir	= VFL_DIR_RX;
	video->vd.v4l2_dev	= v4l2_dev;
	video->vd.fops		= &am_cam_video_fops;
	video->vd.ioctl_ops	= &am_cam_video_ioctl_ops;
	video->vd.minor		= -1;
	video->vd.release	= video_device_release;
	video->vd.lock		= &video->lock;
	video_set_drvdata(&video->vd, video);

	ret = video_register_device(&video->vd, VFL_TYPE_GRABBER, EXYNOS_VIDEONODE_MODULE_AM_CAM);
	if (ret) {
		probe_err("Failed to register video device");
		goto p_err;
	}

#ifdef DBG_FILL_COLOR
	debug_color = 222;
#endif

#if defined(CONFIG_VIDEOBUF2_ION)
	vb2_ion_attach_iommu(video->alloc_ctx);
#endif

p_err:
	probe_info("%s():%d\n", __func__, ret);
	return ret;
}

/*
 * =============================================================================
 * Video File Opertation
 * =============================================================================
 */

static inline void vref_init(struct am_cam_video *video)
{
	atomic_set(&video->refcount, 0);
}

static inline int vref_get(struct am_cam_video *video)
{
	return atomic_inc_return(&video->refcount) - 1;
}

static inline int vref_put(struct am_cam_video *video,
	void (*release)(struct am_cam_video *video))
{
	int ret = 0;

	ret = atomic_sub_and_test(1, &video->refcount);
	if (ret)
		pr_debug("closed all instacne");

	return atomic_read(&video->refcount);
}

static int open_vctx(struct file *file,
	struct am_cam_video *video,
	struct am_cam_video_ctx **vctx)
{
	int ret = 0;

	BUG_ON(!file);
	BUG_ON(!video);

	if (atomic_read(&video->refcount) > AM_CAM_MAX_INSTANCES) {
		am_cam_err("can't open vctx, refcount is invalid");
		ret = -EINVAL;
		goto p_err;
	}

	*vctx = kzalloc(sizeof(struct am_cam_video_ctx), GFP_KERNEL);
	if (*vctx == NULL) {
		am_cam_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	(*vctx)->refcount = vref_get(video);
	(*vctx)->instance = atomic_read(&video->refcount);
	(*vctx)->queue.id = 0;
	(*vctx)->state = AM_CAM_VIDEO_CLOSE;

	file->private_data = *vctx;

p_err:
	return ret;
}

static int close_vctx(struct file *file,
	struct am_cam_video *video,
	struct am_cam_video_ctx *vctx)
{
	int ret = 0;

	kfree(vctx);
	file->private_data = NULL;
	ret = vref_put(video, NULL);

	return ret;
}

static int am_cam_video_open(struct file *file)
{
	int ret = 0;
	struct am_cam_video *video;
	struct am_cam_video_ctx *vctx;
	struct am_cam_queue *queue;
	struct vb2_queue *vbq;
#ifdef CHK_OPEN_CNT
	struct am_cam_video_refcount *vref;
#endif
	video = video_drvdata(file);

	ret = __vref_get(&video->open_cnt);
	if (ret) {
		am_cam_err("vref_get is fail(%d)", ret);
		return -1;
	}

#ifdef CHK_OPEN_CNT
	vref = &video->open_cnt;
	/* if (atomic_read(&vref->refcount) > 2) { */
	if (atomic_read(&vref->refcount) > 1) {
		am_cam_err("ref_count is not 1 (%d)", atomic_read(&vref->refcount));

		ret = __vref_put(&video->open_cnt);
		if (ret)
			am_cam_err("vref_put is fail(%d)", ret);

		return -1;
	}
#endif

	ret = open_vctx(file, video, &vctx);
	if (ret) {
		am_cam_err("open_vctx is fail(%d)", ret);
		goto p_err;
	}

	vctx->video		= video;
	//vctx->fimc_is_vb2_buf_ops = video->fimc_is_vb2_buf_ops;
	//vctx->vops.qbuf		= fimc_is_video_qbuf;
	//vctx->vops.dqbuf	= fimc_is_video_dqbuf;
	//vctx->vops.done 	= fimc_is_video_buffer_done;
	mutex_init(&vctx->lock);

	queue = &vctx->queue;
	queue->vbq = NULL;
	//queue->qops = qops;
	queue->buf_maxcount = 0;
	queue->buf_refcount = 0;
	queue->buf_rdycount = 0;
	queue->buf_req = 0;
	queue->buf_pre = 0;
	queue->buf_que = 0;
	queue->buf_com = 0;
	queue->buf_dqe = 0;
//	clear_bit(FIMC_IS_QUEUE_BUFFER_PREPARED, &queue->state);
//	clear_bit(FIMC_IS_QUEUE_BUFFER_READY, &queue->state);
//	clear_bit(FIMC_IS_QUEUE_STREAM_ON, &queue->state);
//	memset(&queue->framecfg, 0, sizeof(struct fimc_is_frame_cfg));
//	frame_manager_probe(&queue->framemgr, queue->id);

	vbq = kzalloc(sizeof(struct vb2_queue), GFP_KERNEL);
	if (!vbq) {
		am_cam_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	//vbq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	vbq->type       = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	vbq->io_modes		= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vbq->drv_priv		= vctx;
	//vbq->buf_struct_size	= sizeof(struct fimc_is_vb2_buf);
	vbq->ops		= video->vb2_ops;
	vbq->mem_ops		= video->vb2_mem_ops;
	vbq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(vbq);
	if (ret) {
		am_cam_err("vb2_queue_init fail(%d)", ret);
		goto p_err;
	}

	queue->vbq = vbq;

	vctx->state = BIT(AM_CAM_VIDEO_OPEN);

p_err:
	am_cam_info("%s():%d\n", __func__, ret);
	return ret;
}

static int am_cam_video_close(struct file *file)
{
	int ret = 0;
	int refcount;
	struct am_cam_video_ctx *vctx = file->private_data;
	struct am_cam_video *video;
	struct vb2_queue *vbq;

	BUG_ON(!vctx);

	video = vctx->video;
	vbq = vctx->queue.vbq;

	if (vctx->state < BIT(AM_CAM_VIDEO_OPEN)) {
		am_cam_err("already close(%lX)", vctx->state);
		return -ENOENT;
	}

#if 0
	ret = fimc_is_sensor_close(device);
	if (ret)
		merr("fimc_is_sensor_close is fail(%d)", device, ret);

	queue = &vctx->queue;
	queue->buf_maxcount = 0;
	queue->buf_refcount = 0;
	clear_bit(FIMC_IS_QUEUE_BUFFER_PREPARED, &queue->state);
	clear_bit(FIMC_IS_QUEUE_BUFFER_READY, &queue->state);
	clear_bit(FIMC_IS_QUEUE_STREAM_ON, &queue->state);
	frame_manager_close(&queue->framemgr);
#endif

	vb2_queue_release(vbq);
	kfree(vbq);

	refcount = close_vctx(file, video, vctx);
	if (refcount < 0)
		am_cam_err("close_vctx is fail(%d)", refcount);

	ret = __vref_put(&video->open_cnt);
	if (ret)
		am_cam_err("vref_put is fail(%d)", ret);

	am_cam_info("%s():%d\n", __func__, ret);
	return ret;
}

static int am_cam_video_mmap(struct file *file,
	struct vm_area_struct *vma)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;

	am_cam_info("%s():\n", __func__);

	ret = vb2_mmap(vctx->queue.vbq, vma);
	if (ret)
		am_cam_err("vb2_mmap is fail(%d)", ret);

	return ret;
}

const struct v4l2_file_operations am_cam_video_fops = {
	.owner		= THIS_MODULE,
	.open		= am_cam_video_open,
	.release	= am_cam_video_close,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= am_cam_video_mmap,
};

/*
 * =============================================================================
 * Video Ioctl Opertation
 * =============================================================================
 */

static int am_cam_video_querycap(struct file *file, void *fh,
	struct v4l2_capability *cap)
{
	/* Todo : add to query capability code */
	am_cam_info("%s():\n", __func__);

	strcpy(cap->driver, "exynos-am_cam");
	strcpy(cap->card, "exynos-am_cam");
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", "exynos-am_cam");

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
			V4L2_CAP_STREAMING |
			V4L2_CAP_READWRITE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			V4L2_CAP_STREAMING |
			V4L2_CAP_READWRITE |
			V4L2_CAP_DEVICE_CAPS;

	return 0;
}
static int am_cam_video_enum_fmt(struct file *file, void *priv,
	struct v4l2_fmtdesc *f)
{
	if (f->index >= 1)
		return -EINVAL;

	/* Todo : add to enumerate format code */
	am_cam_info("%s():\n", __func__);

	strlcpy(f->description, "BGR32", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_BGR32;

	return 0;
}

static int am_cam_video_enum_fmt_mplane(struct file *file, void *priv,
	struct v4l2_fmtdesc *f)
{
	/* Todo : add to enumerate format code */
	am_cam_info("%s():\n", __func__);

	f->pixelformat = V4L2_PIX_FMT_BGR32;

	return 0;
}

static int am_cam_video_enum_frameintervals(struct file *file, void *fh,
					 struct v4l2_frmivalenum *fival)
{
	int ret = 0;

	if (fival->index >= 1)
		return -EINVAL;

	am_cam_info("%s():\n", __func__);

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30;

	return ret;
}

static int am_cam_video_enum_framesizes(struct file *file, void *fh,
					 struct v4l2_frmsizeenum *fsize)
{

	if (fsize->index >= 1)
		return -EINVAL;

	am_cam_info("%s():\n", __func__);

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = 1920;
	fsize->discrete.height = 1080;

	return 0;
}

static int am_cam_video_try_fmt_vid_cap(struct file *file, void *fh,
    struct v4l2_format *f)
{

    am_cam_info("%s():\n", __func__);

    f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f->fmt.pix.width = 1920;
    f->fmt.pix.height = 1080;
    f->fmt.pix.pixelformat = V4L2_PIX_FMT_BGR32;
    f->fmt.pix.bytesperline = 1920 * 4;

    f->fmt.pix.sizeimage = (f->fmt.pix.bytesperline) * (f->fmt.pix.height);
    f->fmt.pix.field = V4L2_FIELD_NONE;

    return 0;
}

static int am_cam_video_get_format_mplane(struct file *file, void *fh,
	struct v4l2_format *format)
{
	/* Todo : add to get format code */

	return 0;
}
static int am_cam_video_set_format(struct file *file, void *fh,
	struct v4l2_format *format)
{

    am_cam_info("%s():\n", __func__);
    am_cam_info("width(%d)\n", format->fmt.pix.width);
    am_cam_info("height(%d)\n", format->fmt.pix.height);
    am_cam_info("sizeimage(%d)\n", format->fmt.pix.sizeimage);
    am_cam_info("bytesperline(%d)\n", format->fmt.pix.bytesperline);
    am_cam_info("pixelformat(%d)\n", format->fmt.pix.pixelformat);

    format->fmt.pix.sizeimage = 1920 * 1080 * 4;
    return 0;
}

static int am_cam_video_set_format_mplane(struct file *file, void *fh,
	struct v4l2_format *format)
{
#if 0
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;
	struct v4l2_pix_format_mplane *pix;

	pix = &format->fmt.pix_mp;
	fmt = fimc_is_find_format(pix->pixelformat, 0);
	if (!fmt) {
		err("pixel format is not found\n");
		ret = -EINVAL;
		goto p_err;
	}

	queue->framecfg.format			= fmt;
	queue->framecfg.colorspace		= pix->colorspace;
	queue->framecfg.width			= pix->width;
	queue->framecfg.height			= pix->height;

	/* for multi-buffer */
	if (pix->reserved[0] > 0 && pix->reserved[0] <= FIMC_IS_MAX_PLANES)
		queue->framecfg.num_buffers	= pix->reserved[0];
	else
		queue->framecfg.num_buffers	= 1;

	for (plane = 0; plane < fmt->hw_plane; ++plane) {
		if (pix->plane_fmt[plane].bytesperline) {
			queue->framecfg.bytesperline[plane] =
				pix->plane_fmt[plane].bytesperline;
		} else {
			queue->framecfg.bytesperline[plane] = 0;
		}
	}

	ret = CALL_QOPS(queue, s_format, device, queue);
	if (ret) {
		err("s_format is fail(%d)", ret);
		goto p_err;
	}

	info("[FID:%04X]pixelformat(%c%c%c%c), num_buffer(%d)\n", queue->id,
		(char)((fmt->pixelformat >> 0) & 0xFF),
		(char)((fmt->pixelformat >> 8) & 0xFF),
		(char)((fmt->pixelformat >> 16) & 0xFF),
		(char)((fmt->pixelformat >> 24) & 0xFF),
		queue->framecfg.num_buffers);

	vctx->state = BIT(FIMC_IS_VIDEO_S_FORMAT);

p_err:
	return ret;
#else
	return 0;
#endif
}

static int am_cam_video_cropcap(struct file *file, void *fh,
	struct v4l2_cropcap *cropcap)
{
	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Todo : add to crop capability code */
	am_cam_info("%s():\n", __func__);

	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	cropcap->bounds.left   = 0;
	cropcap->bounds.top    = 0;
	cropcap->bounds.width  = 1920;
	cropcap->bounds.height = 1080;
	cropcap->defrect.left     = 0;
	cropcap->defrect.top      = 0;
	cropcap->defrect.width    = 1920;
	cropcap->defrect.height   = 1080;

	return 0;
}

static int am_cam_video_get_crop(struct file *file, void *fh,
	struct v4l2_crop *crop)
{
	/* Todo : add to get crop control code */
	return 0;
}

static int am_cam_video_set_crop(struct file *file, void *fh,
	const struct v4l2_crop *crop)
{
	/* Todo : add to set crop control code */
	return 0;
}

static int am_cam_video_reqbufs(struct file *file, void *priv,
	struct v4l2_requestbuffers *buf)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;
	am_cam_info("%s():reqbufs count(%d)\n", __func__, buf->count);
	am_cam_info("%s():reqbufs type(%d)\n", __func__, buf->type);
	am_cam_info("%s():reqbufs memory(%d)\n", __func__, buf->memory);

	BUG_ON(!vctx);

	ret = vb2_reqbufs(vctx->queue.vbq, buf);
	if (ret) {
		am_cam_err("vb2_reqbufs is fail(%d)", ret);
		goto p_err;
	}

p_err:
    ret = 0;
	return ret;
}

static int am_cam_video_querybuf(struct file *file, void *priv,
	struct v4l2_buffer *buf)
{
	struct am_cam_video_ctx *vctx = file->private_data;
	am_cam_info("%s():\n", __func__);

	return vb2_querybuf(vctx->queue.vbq, buf);
}

static int am_cam_video_qbuf(struct file *file, void *priv,
	struct v4l2_buffer *buf)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;
	vctx = file->private_data;

	am_cam_devi("%s(): (%d)\n", __func__, buf->index);

	ret = vb2_qbuf(vctx->queue.vbq, buf);
	if (ret) {
		am_cam_err("vb2_qbuf is fail(index : %d, %d)", buf->index, ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int am_cam_video_dqbuf(struct file *file, void *priv,
	struct v4l2_buffer *buf)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;
	bool blocking = file->f_flags & O_NONBLOCK;

	ret = vb2_dqbuf(vctx->queue.vbq, buf, blocking);
	if (ret) {
		am_cam_err("vb2_dqbuf is fail(%d)", ret);
		goto p_err;
	}

	/* HACK */
	buf->bytesused = buf->length;
	buf->field =  V4L2_FIELD_NONE;
	am_cam_devi("%s(): (%d)\n", __func__, buf->index);
	am_cam_devi("%s(): timestamp(%ld us)\n", __func__,
			buf->timestamp.tv_sec * 1000000 + buf->timestamp.tv_usec);

#ifdef DBG_FILL_COLOR
{
	debug_color ++;
	if (debug_color > 255)
		debug_color = 0;

	memset(am_cam_vb[buf->index].kvaddr[0], debug_color, buf->length);

	am_cam_devi("%s(): debug_color(%d) \n", __func__, debug_color);
	am_cam_devi("%s(): kvaddr(%d)(%d)(0x%llx \n", __func__,
		buf->index,
		buf->length,
		am_cam_vb[buf->index].kvaddr[0]);
}
#endif

p_err:
	return ret;
}

static int am_cam_video_prepare(struct file *file, void *prev,
	struct v4l2_buffer *buf)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;

	am_cam_info("%s(): (%d)\n", __func__, buf->index);

	ret = vb2_prepare_buf(vctx->queue.vbq, buf);
	if (ret) {
		am_cam_err("vb2_prepare_buf is fail(index : %d, %d)", buf->index, ret);
		goto p_err;
	}

p_err:
	return ret;
}


static int am_cam_video_streamon(struct file *file, void *priv,
	enum v4l2_buf_type type)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;

	ret = vb2_streamon(vctx->queue.vbq, type);
	if (ret) {
		am_cam_err("vb2_streamon is fail(%d)", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int am_cam_video_streamoff(struct file *file, void *priv,
	enum v4l2_buf_type type)
{
	int ret = 0;
	struct am_cam_video_ctx *vctx = file->private_data;

	ret = vb2_streamoff(vctx->queue.vbq, type);
	if (ret) {
		am_cam_err("vb2_streamoff is fail(%d)", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int am_cam_video_enum_input(struct file *file, void *priv,
	struct v4l2_input *input)
{
	/* Todo: add to enumerate input code */
	if (input->index >= 1)
		return -EINVAL;

	am_cam_info("%s():input->index(%d)\n", __func__, input->index);

	input->type = V4L2_INPUT_TYPE_CAMERA;
	snprintf(input->name, sizeof(input->name), "exynos-am_cam");
	input->capabilities = 0;
	input->status = V4L2_IN_ST_VFLIP | V4L2_IN_ST_HFLIP;

	return 0;
}

static int am_cam_video_g_input(struct file *file, void *priv,
	unsigned int *input)
{
	/* Todo: add to get input control code */
//	struct am_cam_video_ctx *vctx = file->private_data;

	return 0;
}

static int am_cam_video_s_input(struct file *file, void *priv,
	unsigned int input)
{
	return 0;
}

static int am_cam_video_s_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{
	am_cam_info("%s(): ctrl->id(0x%x 0x%x) \n", __func__,
			ctrl->id, ctrl->value);

	/* Set the min buffer count */
	ctrl->value = 3;

	return 0;
}

static int am_cam_video_g_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{

	am_cam_info("%s(): ctrl->id(0x%x) \n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->value = 5; /* gstreamer + 3 = 8 */
		break;
	default:
		am_cam_err("unsupport ioctl (0x%x) \n", ctrl->id);
		break;
	}

	return 0;
}

static int am_cam_video_g_parm(struct file *file, void *priv,
	struct v4l2_streamparm *parm)
{
	am_cam_info("%s():\n", __func__);

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.readbuffers = 3;

	return 0;
}

static int am_cam_video_s_parm(struct file *file, void *priv,
	struct v4l2_streamparm *parm)
{
	return 0;
}

const struct v4l2_ioctl_ops am_cam_video_ioctl_ops = {
	.vidioc_querycap		= am_cam_video_querycap,
	.vidioc_enum_fmt_vid_out_mplane	= am_cam_video_enum_fmt_mplane,
	.vidioc_enum_fmt_vid_cap_mplane	= am_cam_video_enum_fmt_mplane,
	.vidioc_enum_fmt_vid_cap        = am_cam_video_enum_fmt, 
	.vidioc_enum_framesizes         = am_cam_video_enum_framesizes,
	.vidioc_enum_frameintervals     = am_cam_video_enum_frameintervals,
	.vidioc_try_fmt_vid_cap         = am_cam_video_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = am_cam_video_set_format,
	.vidioc_g_fmt_vid_out_mplane	= am_cam_video_get_format_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= am_cam_video_get_format_mplane,
	.vidioc_s_fmt_vid_out_mplane	= am_cam_video_set_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= am_cam_video_set_format_mplane,
	.vidioc_cropcap			= am_cam_video_cropcap,
	.vidioc_g_crop			= am_cam_video_get_crop,
	.vidioc_s_crop			= am_cam_video_set_crop,
	.vidioc_reqbufs			= am_cam_video_reqbufs,
	.vidioc_querybuf		= am_cam_video_querybuf,
	.vidioc_qbuf			= am_cam_video_qbuf,
	.vidioc_dqbuf			= am_cam_video_dqbuf,
	.vidioc_prepare_buf		= am_cam_video_prepare,
	.vidioc_streamon		= am_cam_video_streamon,
	.vidioc_streamoff		= am_cam_video_streamoff,
	.vidioc_enum_input		= am_cam_video_enum_input,
	.vidioc_g_input			= am_cam_video_g_input,
	.vidioc_s_input			= am_cam_video_s_input,
	.vidioc_s_ctrl			= am_cam_video_s_ctrl,
	.vidioc_g_ctrl			= am_cam_video_g_ctrl,
	.vidioc_g_parm			= am_cam_video_g_parm,
	.vidioc_s_parm			= am_cam_video_s_parm,
};

static int am_cam_queue_setup(struct vb2_queue *vbq,
	const struct v4l2_format *fmt,
	unsigned int *num_buffers,
	unsigned int *num_planes,
	unsigned int sizes[],
	void *allocators[])
{
	u32 ret = 0;
	u32 plane;

	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;

	BUG_ON(!vbq);
	BUG_ON(!vbq->drv_priv);

	vctx = vbq->drv_priv;
	video = vctx->video;
	/* queue = vctx->queue; */

/* HACK */
	*num_planes = 1;

	for (plane = 0; plane < *num_planes; plane++) {
		allocators[plane] = video->alloc_ctx;
/* HACK */
		sizes[plane] = 1920 * 1080 * 4;
	}

	am_cam_info("%s(): num_buffers(%d), num_planes(%d) \n",
			__func__,
			*num_buffers,
			*num_planes);

	return ret;
}

static int am_cam_buffer_init(struct vb2_buffer *vb)
{
	return 0;
}

static int am_cam_buffer_prepare(struct vb2_buffer *vb)
{
	am_cam_devi("%s(): \n", __func__);

	return 0;
}

static inline void am_cam_wait_prepare(struct vb2_queue *vbq)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;

	BUG_ON(!vbq);
	BUG_ON(!vbq->drv_priv);

	vctx = vbq->drv_priv;
	video = vctx->video;
	mutex_unlock(&video->lock);
}

static inline void am_cam_wait_finish(struct vb2_queue *vbq)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;

	BUG_ON(!vbq);
	BUG_ON(!vbq->drv_priv);

	vctx = vbq->drv_priv;
	video = vctx->video;
	mutex_lock(&video->lock);
}

static int am_cam_start_streaming(struct vb2_queue *vbq,
	unsigned int count)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;
	struct am_cam_device *device;

	BUG_ON(!vbq);
	BUG_ON(!vbq->drv_priv);

	am_cam_info("%s():\n", __func__);

	vctx = vbq->drv_priv;
	video = vctx->video;
	device = container_of(video, struct am_cam_device, video);

	/* csi_dma_set_buffer_start(&device->subdev_csi); */
	am_cam_info("11. csi stream on+\n");
	csi_s_stream(&device->subdev_csi, true);
	am_cam_info("11. csi stream on-\n");

	am_cam_info("12. sensor stream on+\n");
	sensor_module_stream_on();
	am_cam_info("12. sensor stream on-\n");

	return 0;
}

static void am_cam_stop_streaming(struct vb2_queue *vbq)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;
	struct am_cam_device *device;

	BUG_ON(!vbq);
	BUG_ON(!vbq->drv_priv);

	am_cam_info("%s():\n", __func__);

	vctx = vbq->drv_priv;
	video = vctx->video;
	device = container_of(video, struct am_cam_device, video);

	am_cam_info("1. sensor stream off+\n");
	sensor_module_stream_off();
	am_cam_info("1. sensor stream off-\n");

	am_cam_info("2. csi stream off+\n");
	csi_s_stream(&device->subdev_csi, false);
	am_cam_info("2. csi stream off-\n");

	return;
}

static dma_addr_t am_cam_vb2_ion_plane_dvaddr(
		struct vb2_buffer *vb2_buf, u32 plane)
{
	dma_addr_t dva = 0;
	WARN_ON(vb2_ion_dma_address(vb2_plane_cookie(vb2_buf, plane), &dva) != 0);

	return (ulong)dva;
}

#ifdef MAP_KVADDR
static void *am_cam_vb2_ion_plane_kvaddr(
		struct vb2_buffer *vb2_buf, u32 plane)
{
	return vb2_ion_private_vaddr(vb2_plane_cookie(vb2_buf, plane));
}
#endif

static void am_cam_buffer_queue(struct vb2_buffer *vb)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;
	struct am_cam_device *device;
	struct am_cam_buf am_cam_vb_data;

	vctx = vb->vb2_queue->drv_priv;
	video = vctx->video;
	device = container_of(video, struct am_cam_device, video);

	am_cam_devi("%s(): (%d) \n", __func__, vb->v4l2_buf.index);

	/* move to prepare */
	am_cam_vb_data.vb2_buf  = vb;
	am_cam_vb_data.dvaddr[0] = am_cam_vb2_ion_plane_dvaddr(vb, 0);

#ifdef MAP_KVADDR
	am_cam_vb_data.kvaddr[0] = am_cam_vb2_ion_plane_kvaddr(vb, 0);
#endif

#ifdef DBG_FILL_COLOR
{
	debug_color ++;
	if (debug_color > 200)
		debug_color = 0;

	memset(am_cam_vb[vb->v4l2_buf.index].kvaddr[0], debug_color, 1920 * 1080 * 2);
	am_cam_devi("%s(): debug_color(%d) \n", __func__, debug_color);

	am_cam_devi("%s(): kvaddr(%d) (0x%lx) (0x%llx) \n", __func__,
		vb->v4l2_buf.index,
		(long unsigned int)am_cam_vb_data.dvaddr[0],
		(long long unsigned int)am_cam_vb_data.kvaddr[0]);
}
#endif

	csi_s_buffer_addr(&device->subdev_csi,
			(void *)&am_cam_vb_data, NULL);

	return;
}

static void am_cam_buffer_finish(struct vb2_buffer *vb)
{
	struct am_cam_video_ctx *vctx;
	struct am_cam_video *video;
	struct am_cam_device *device;

	vctx = vb->vb2_queue->drv_priv;
	video = vctx->video;
	device = container_of(video, struct am_cam_device, video);

	am_cam_devi("%s(): (%d) \n", __func__, vb->v4l2_buf.index);

	csi_s_buffer_finish(&device->subdev_csi, NULL, NULL);

#ifdef DBG_FILL_COLOR
{
	debug_color ++;
	if (debug_color > 200)
		debug_color = 0;

	vb->v4l2_buf.length = 1920 * 1080 * 2;
	vb2_ion_sync_for_cpu(vb2_plane_cookie(vb, 0), 0, vb->v4l2_buf.length, DMA_BIDIRECTIONAL);

	memset(am_cam_vb[vb->v4l2_buf.index].kvaddr[0], debug_color, vb->v4l2_buf.length);

	vb2_ion_sync_for_device(vb2_plane_cookie(vb, 0), 0, vb->v4l2_buf.length, DMA_BIDIRECTIONAL);

	am_cam_devi("%s(): debug_color(%d) \n", __func__, debug_color);
}

	am_cam_devi("%s(): kvaddr(%d) length(%d) (0x%llx \n", __func__,
		vb->v4l2_buf.index,
		vb->v4l2_buf.length,
		am_cam_vb[vb->v4l2_buf.index].kvaddr[0]);
#endif

	return;
}

const struct vb2_ops am_cam_qops = {
	.queue_setup		= am_cam_queue_setup,
	.buf_init		= am_cam_buffer_init,
	.buf_prepare		= am_cam_buffer_prepare,
	.buf_queue		= am_cam_buffer_queue,
	.buf_finish		= am_cam_buffer_finish,
	.wait_prepare		= am_cam_wait_prepare,
	.wait_finish		= am_cam_wait_finish,
	.start_streaming	= am_cam_start_streaming,
	.stop_streaming		= am_cam_stop_streaming,
};
