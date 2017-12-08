/*
 * Samsung HDMI driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *
 * Ayoung Sim <a.sim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundiation. either version 2 of the License,
 * or (at your option) any later version
 */

#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#include "exynos_drm_hdmi.h"
#include "regs-hdmi.h"

const struct hdmi_config hdmi_config[] = {
	{V4L2_DV_BT_CEA_720X480P59_94,	720, 480,	0, 9,  6, 30,	0, 16, 62, 60,		0, 0,	2, 3},
	{V4L2_DV_BT_CEA_720X576P50,	720, 576,	0, 5, 5, 39,	0, 12, 64, 68,		0, 0,	17, 18},
	{V4L2_DV_BT_CEA_1280X720P60,	1280, 720,	1, 5, 5, 20,	1, 110, 40, 220,	0, 0,	4, 69},
	{V4L2_DV_BT_CEA_1280X720P50,	1280, 720,	1, 5, 5, 20,	1, 440, 40, 220,	0, 0,	19, 19},
	{V4L2_DV_BT_CEA_1920X1080P60,	1920, 1080,	1, 4, 5, 36,	1, 88, 44, 148,		0, 0,	16, 16},

	{V4L2_DV_BT_CEA_1920X1080P50,	1920, 1080,	1, 4, 5, 36,	1, 528, 44, 148,	0, 0,	31, 31},
	{V4L2_DV_BT_CEA_1920X1080P30,	1920, 1080,	0, 4, 5, 36,	0, 88, 44, 148,		0, 0,	34, 34},
	{V4L2_DV_BT_CEA_1920X1080P25,	1920, 1080,	0, 4, 5, 36,	0, 528, 44, 148,	0, 0,	33, 33},
	{V4L2_DV_BT_CEA_1920X1080P24,	1920, 1080,	1, 4, 5, 36,	1, 638, 44, 148,	0, 0,	32, 32},
	{V4L2_DV_BT_CEA_1920X1080I60,	1920, 540,	1, 2, 5, 15,	1, 88, 44, 148,		1, 550,	5, 5},
	{V4L2_DV_BT_CEA_1920X1080I50,	1920, 540,	1, 2, 5, 15,	1, 528, 44, 148,	1, 660,	20, 20},
	{V4L2_DV_BT_CEA_3840X2160P30,	3840, 2160,	1, 8, 10, 72,	1, 176, 88, 296,	0, 0,	95, 95},
	{V4L2_DV_BT_CEA_3840X2160P25,	3840, 2160,	1, 8, 10, 72,	1, 1056, 88, 296,	0, 0,	94, 94},
	{V4L2_DV_BT_CEA_3840X2160P24,	3840, 2160,	1, 8, 10, 72,	1, 1276, 88, 296,	0, 0,	93, 93},
};

const int hdmi_pre_cnt = ARRAY_SIZE(hdmi_config);

irqreturn_t hdmi_irq_handler(int irq, void *dev_data)
{
	struct hdmi_device *hdev = dev_data;
	u32 hpd_status, hpd_int;

	if (hdev->streaming == HDMI_STREAMING) {
		hpd_int = hdmi_read(hdev, HDMI20_CORE_INT);
		/* clearing flags for HPD plug/unplug */
		if (hpd_int) {
			hdmi_write_mask(hdev, HDMI20_CORE_INT, 0, 0);
			hpd_status = (hdmi_read(hdev, HDMI20_STATUS) & HDMI20_STATUS_HPD);
			if (hpd_status) {
				hdmi_info("PLUG interrupt\n");
			} else {
				hdmi_info("UNPLUG interrupt\n");
				if (hdev->hdcp_info.hdcp_start)
					hdcp_stop(hdev);
				hdmi_write_mask(hdev, HDMI20_CORE_INT, 0, 0);
				queue_work(hdev->hpd_wq, &hdev->hpd_work);
			}
		}
		hdcp_irq_handler(hdev);
	} else {
		pr_warn("HDMI interrupt shouldn't happen\n");
	}

	return IRQ_HANDLED;
}

static const u8 *hdmiphy_timing2conf(struct v4l2_dv_timings *timings)
{
	int i;

	for (i = 0; i < hdmiphy_conf_cnt; ++i)
		if (hdmi_match_timings(&hdmiphy_conf[i].dv_timings,
					timings, 0))
			break;

	if (i == hdmiphy_conf_cnt)
		return NULL;

	return hdmiphy_conf[i].data;
}

int hdmi_conf_apply(struct hdmi_device *hdmi_dev)
{
	struct v4l2_dv_timings timings;
	const u8 *data;
	u8 buffer[32];
	int tries;

	hdmi_dbg(4, "%s\n", __func__);

	/* configure presets */
	timings = hdmi_dev->cur_timings;

	data = hdmiphy_timing2conf(&timings);
	if (!data) {
		hdmi_err("format not supported\n");
		return -EINVAL;
	}

	memcpy(buffer, data, 32);
	hdmiphy_set_mode(hdmi_dev, 0);
	hdmiphy_conf_store(hdmi_dev, buffer);
	hdmiphy_set_mode(hdmi_dev, 1);

	/* waiting for HDMIPHY's PLL to get to steady state */
	for (tries = 100; tries; --tries) {
		if (is_hdmiphy_ready(hdmi_dev))
			break;

		msleep(1);
	}

	/* steady state not achieved */
	if (tries == 0) {
		hdmi_err("hdmiphy's pll could not reach steady state.\n");
		return -EIO;
	} else {
		hdmi_info("hdmiphy's pll is ready\n");
	}

	return 0;
}

void hdmi_timing_apply(struct hdmi_device *hdev, const struct hdmi_config *conf)
{
	u32 hpol, vpol;

	if (conf->vpol == 1)
		vpol = 0;
	else
		vpol = 1;

	if (conf->hpol == 1)
		hpol = 0;
	else
		hpol = 1;

	hdmi_write_mask(hdev, HDMI20_VIDEO_CTRL, hpol << 9, 0x1 << 9);
	hdmi_write_mask(hdev, HDMI20_VIDEO_CTRL, vpol << 8, 0x1 << 8);

	hdmi_dbg(3, "hdmi config: %dx%d, %d %d %d %d, %d %d %d %d, %d, %d\n",
			conf->h_line, conf->v_line,
			conf->vpol, conf->vfp, conf->vsp, conf->vbp,
			conf->hpol, conf->hfp, conf->hsp, conf->hbp,
			conf->interlace, conf->avi_vic);
}

void hdmi_video_pattern_enable(struct hdmi_device *hdev,
			const struct hdmi_config *conf)
{
	u32 hpol, vpol;

	hdmi_info("HDMI self pattern will be turned on\n");

	if (conf->vpol == 1)
		vpol = 0;
	else
		vpol = 1;

	if (conf->hpol == 1)
		hpol = 0;
	else
		hpol = 1;

	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_CONFIG, hpol << 9, 0x1 << 9);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_CONFIG, vpol << 8, 0x1 << 8);

	/* RGB ramping 24bit */
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_CONFIG, 0x2, 0xf);

	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_VFP, conf->vfp, 0xffff);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_VSYNC, conf->vsp, 0xffff);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_VBP, conf->vbp, 0xffff);

	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_VLINE, conf->v_line, 0xffff);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_DE, conf->h_line / 2, 0xffff);

	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_HFP, conf->hfp / 2, 0xffff);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_HSYNC, conf->hsp / 2, 0xffff);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_HBP, conf->hbp / 2, 0xffff);

	hdmi_write_mask(hdev, HDMI20_CTRL, 0x1 << 4, 0x1 << 4);
	hdmi_write_mask(hdev, HDMI20_PTTN_GEN_CTRL, 0x1, 0x1);

	hdmi_dbg(3, "end\n");
}

void hdmi_reg_init(struct hdmi_device *hdev)
{
	/* Deep color mode: 24bpp */
	hdmi_write_mask(hdev, HDMI20_VIDEO_CTRL, 0x0, 0x3);
	/* Normal video input */
	hdmi_write_mask(hdev, HDMI20_CTRL, 0x0 << 4, 0x1 << 4);
	/* HPD threshold */
	hdmi_write_mask(hdev, HDMI20_HPD_THRESHOLD_R, 0x7F, 0xFFFF);
	hdmi_write_mask(hdev, HDMI20_HPD_THRESHOLD_F, 0x7F, 0xFFFF);
}

void hdmi_set_dvi_mode(struct hdmi_device *hdev)
{
	hdmi_write_mask(hdev, HDMI20_VIDEO_CTRL, 0x1 << 4, 0x1 << 4);
	/* HDMI: EESS, DVI: OESS */
	hdmi_write_mask(hdev, HDMI20_VIDEO_CTRL, 0x1 << 7, 0x1 << 7);
}

void hdmi_avi_packet_config(struct hdmi_device *hdev)
{
	int i;
	u8 checksum;

	/* Refer CEA-861-F document */
	/* Inforframe version */
	hdmi_write_mask(hdev, HDMI20_AVI_CTRL, 0x2 << 8, 0xff << 8);
	/* Infoframe length */
	hdmi_write_mask(hdev, HDMI20_AVI_CTRL, 0xd << 16, 0xff << 16);

	for (i = 0; i < 3; i++)
		checksum += hdmi_read_bytes(hdev, HDMI20_AVI_CTRL, i);
	for (i = 1; i < 4; i++)
		checksum += hdmi_read_bytes(hdev, HDMI20_AVI_SUB_3_0, i);
	for (i = 0; i < 4; i++)
		checksum += hdmi_read_bytes(hdev, HDMI20_AVI_SUB_7_4, i);
	for (i = 0; i < 4; i++)
		checksum += hdmi_read_bytes(hdev, HDMI20_AVI_SUB_11_8, i);
	for (i = 0; i < 2; i++)
		checksum += hdmi_read_bytes(hdev, HDMI20_AVI_SUB_15_12, i);
	hdmi_dbg(4, "AVI checksum = 0x%x\n", (u8)(~checksum + 1));

	hdmi_write_mask(hdev, HDMI20_AVI_SUB_3_0, (u8)(~checksum + 1), 0xff);
	hdmi_write_mask(hdev, HDMI20_PKT_CTRL, 0x1 << 12, 0x1 << 12);
}

void hdmi_enable(struct hdmi_device *hdev, int on)
{
	if (on) {
		hdmi_write_mask(hdev, HDMI20_CTRL, 1, 1);
	} else {
		hdmi_write_mask(hdev, HDMI20_CTRL, 0, 1);
	}
}

int is_hdmiphy_ready(struct hdmi_device *hdev)
{
	u32 val = hdmi_read(hdev, HDMI20_STATUS);
	if (val & HDMI_PHY_STATUS_READY)
		return 1;

	return 0;
}

void hdmi_phy_sw_reset(struct hdmi_device *hdev)
{
	hdmi_write_mask(hdev, HDMI20_PHY_RESETN, 1, 0x1 << 0);
	mdelay(10);
	hdmi_write_mask(hdev, HDMI20_PHY_RESETN, 0, 0x1 << 0);
}

void hdmiphy_set_isolation(struct hdmi_device *hdev, int en)
{
	u32 val = 0;
	u32 old = readl(hdev->pmu_regs);

	val = (en & HDMI_ISOLATION_MASK) | (old & ~HDMI_ISOLATION_MASK);
	writel(val, hdev->pmu_regs);
}

void hdmiphy_set_mode(struct hdmi_device *hdev, int en)
{
	if (en)
		hdmiphy_write_mask(hdev, HDMI_PHY_REG84, 1 << 7, 1 << 7);
	else
		hdmiphy_write_mask(hdev, HDMI_PHY_REG84, 0 << 7, 1 << 7);
}

void hdmiphy_set_power(struct hdmi_device *hdev, int en)
{
	if (en)
		hdmi_write_mask(hdev, HDMI20_PHY_CON, 1, 1);
	else
		hdmi_write_mask(hdev, HDMI20_PHY_CON, 0, 1);
}

void hdmiphy_conf_store(struct hdmi_device *hdev, const char *buf)
{
	int i;

	hdmi_dbg(4, "hdmiphy configuration\n");
	for (i = 0; i < 32; i++) {
		writeb(buf[i], hdev->phy_regs + HDMI_PHY_REG00 + (i * 4));
		if (hdmi_dbg >= 3) {
			if (i % 8 == 0)
				printk("[");
			printk("%#x ", buf[i]);
			if (i % 8 == 7)
				printk("]\n");
		}
	}
}

void hdmi_hpd_clear_int(struct hdmi_device *hdev)
{
	hdmi_write_mask(hdev, HDMI20_CORE_INT, 0, 0);
}

void hdmi_reg_set_int_hpd(struct hdmi_device *hdev)
{
	hdmi_dbg(4, "start\n");

	s5p_v4l2_int_src_hdmi_hpd(hdev);
	hdmi_write_mask(hdev, HDMI20_CORE_INT_EN, 1, 0);
	hdmi_hpd_clear_int(hdev);

	hdmi_dbg(4, "end\n");
}

void hdmi_reg_set_ext_hpd(struct hdmi_device *hdev)
{
	hdmi_hpd_clear_int(hdev);
	hdmi_write_mask(hdev, HDMI20_CORE_INT_EN, 0, 0);
	s5p_v4l2_int_src_ext_hpd(hdev);
}

void hdmi_dumpregs(struct hdmi_device *hdev, char *prefix)
{
	pr_err("-----------%s) HDMI registers (SFR base = %p)\n",
				prefix, hdev->regs);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			hdev->regs + 0x0, 0xF10, false);
	pr_err("--------------------------------------------\n");
}
