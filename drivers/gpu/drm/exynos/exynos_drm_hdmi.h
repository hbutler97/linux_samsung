/*
 * Samsung HDMI interface driver
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *
 * Jiun Yu, <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundiation. either version 2 of the License,
 * or (at your option) any later version
 */

#ifndef SAMSUMG_HDMI_H
#define SAMSUNG_HDMI_H

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/switch.h>
#include <uapi/linux/v4l2-dv-timings.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

typedef unsigned char u8;

#define INFOFRAME_CNT          2

/* default preset configured on probe */
#define HDMI_DEFAULT_TIMINGS_IDX (4)

#define HDMI_VSI_VERSION	0x01
#define HDMI_AVI_VERSION	0x02
#define HDMI_AUI_VERSION	0x01
#define HDMI_VSI_LENGTH		0x05
#define HDMI_AVI_LENGTH		0x0d
#define HDMI_AUI_LENGTH		0x0a

#define AVI_UNDERSCAN			(2 << 0)
#define AVI_ACTIVE_FORMAT_VALID		(1 << 4)
#define AVI_PIC_ASPECT_RATIO_4_3	(1 << 4)
#define AVI_PIC_ASPECT_RATIO_16_9	(2 << 4)
#define AVI_SAME_AS_PIC_ASPECT_RATIO	8
#define AVI_LIMITED_RANGE		(1 << 2)
#define AVI_FULL_RANGE			(2 << 2)
#define AVI_ITU709			(2 << 6)

/* HDMI audio configuration value */
#define DEFAULT_SAMPLE_RATE		48000
#define DEFAULT_BITS_PER_SAMPLE		16
#define AUDIO_CHANNEL_MASK		(0xFF)
#define AUDIO_BIT_RATE_MASK		(0x7 << 16)
#define AUDIO_SAMPLE_RATE_MASK		(0x7F << 19)

/* HDMI pad definitions */
#define HDMI_PAD_SINK		0
#define HDMI_PADS_NUM		1

/* HPD state definitions */
#define HPD_LOW		0
#define HPD_HIGH	1

/*
 * HDMI timing distinguish definition
 * If timings has the same widh and height,
 * it is low definition as it is low pixelclock.
 */
#define HDMI_TIMINGS_FPS_24		24
#define HDMI_TIMINGS_FPS_25		25
#define HDMI_TIMINGS_FPS_30		30
#define HDMI_TIMINGS_FPS_50		50
#define HDMI_TIMINGS_FPS_59		59
#define HDMI_TIMINGS_FPS_60		60

#define HDMI_AUDIO_192KHZ	(1 << 6)
#define HDMI_AUDIO_176KHZ	(1 << 5)
#define HDMI_AUDIO_96KHZ	(1 << 4)
#define HDMI_AUDIO_88KHZ	(1 << 3)
#define HDMI_AUDIO_48KHZ	(1 << 2)
#define HDMI_AUDIO_44KHZ	(1 << 1)
#define HDMI_AUDIO_32KHZ	(1 << 0)

#define HDMI_AUDIO_24BIT	(1 << 2)
#define HDMI_AUDIO_20BIT	(1 << 1)
#define HDMI_AUDIO_16BIT	(1 << 0)

enum HDMI_DBG {
	HDMI_DEFAULT = 0,
	HDMI_HDCP_L1 = 1,
	HDMI_HDCP_L2 = 2,
};

extern int hdmi_dbg;

#define hdmi_dbg(level, fmt, args...)				\
	do {							\
		if (hdmi_dbg >= level)				\
			printk(KERN_DEBUG "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
	} while (0)

#define hdmi_err(fmt, args...)				\
	do {						\
		printk(KERN_ERR "%s:%d: " fmt,		\
		       __func__, __LINE__, ##args);	\
	} while (0)

#define hdmi_info(fmt, args...)				\
	do {							\
		printk(KERN_INFO "%s:%d: " fmt,	\
			__func__, __LINE__, ##args);		\
	} while (0)

enum HDMI_VIDEO_FORMAT {
	HDMI_VIDEO_FORMAT_2D = 0x0,
	/** refer to Table 8-12 HDMI_Video_Format in HDMI specification v1.4a */
	HDMI_VIDEO_FORMAT_UD = 0x1,
	HDMI_VIDEO_FORMAT_3D = 0x2
};

enum HDMI_3D_FORMAT {
	/** refer to Table 8-13 3D_Structure in HDMI specification v1.4a */

	/** Frame Packing */
	HDMI_3D_FORMAT_FP = 0x0,
	/** Top-and-Bottom */
	HDMI_3D_FORMAT_TB = 0x6,
	/** Side-by-Side Half */
	HDMI_3D_FORMAT_SB_HALF = 0x8
};

enum HDMI_3D_EXT_DATA {
	/* refer to Table H-3 3D_Ext_Data - Additional video format
	 * information for Side-by-side(half) 3D structure */

	/** Horizontal sub-sampleing */
	HDMI_H_SUB_SAMPLE = 0x1
};

enum HDMI_OUTPUT_FMT {
	HDMI_OUTPUT_RGB888 = 0x0,
	HDMI_OUTPUT_YUV444 = 0x2
};

enum HDMI_PACKET_TYPE {
	/** refer to Table 5-8 Packet Type in HDMI specification v1.4a */

	/** InfoFrame packet type */
	HDMI_PACKET_TYPE_INFOFRAME = 0X80,
	/** Vendor-Specific InfoFrame */
	HDMI_PACKET_TYPE_VSI = HDMI_PACKET_TYPE_INFOFRAME + 1,
	/** Auxiliary Video information InfoFrame */
	HDMI_PACKET_TYPE_AVI = HDMI_PACKET_TYPE_INFOFRAME + 2,
	/** Audio information InfoFrame */
	HDMI_PACKET_TYPE_AUI = HDMI_PACKET_TYPE_INFOFRAME + 4
};

enum HDMI_COLOR_RANGE {
	HDMI_RGB601_0_255,
	HDMI_RGB601_16_235,
	HDMI_RGB709_0_255,
	HDMI_RGB709_16_235
};

enum HDMI_PROBE_STATE {
	HDMI_PROBED,
	HDMI_PROBING
};

enum HDMI_AUDIO_CODEC {
	HDMI_AUDIO_PCM,
	HDMI_AUDIO_AC3,
	HDMI_AUDIO_MP3
};

enum HDMI_ASPECT_RATIO {
	HDMI_ASPECT_RATIO_16_9,
	HDMI_ASPECT_RATIO_4_3
};

enum HDMI_STREAMING_STATE {
	HDMI_STOP		= 0 << 1,
	HDMI_STREAMING		= 1 << 1
};

enum HDCP_EVENT {
	HDCP_EVENT_STOP			= 1 << 0,
	HDCP_EVENT_START		= 1 << 1,
	HDCP_EVENT_READ_BKSV_START	= 1 << 2,
	HDCP_EVENT_WRITE_AKSV_START	= 1 << 4,
	HDCP_EVENT_CHECK_RI_START	= 1 << 8,
	HDCP_EVENT_SECOND_AUTH_START	= 1 << 16
};

enum HDCP_STATE {
	NOT_AUTHENTICATED,
	RECEIVER_READ_READY,
	BCAPS_READ_DONE,
	BKSV_READ_DONE,
	AN_WRITE_DONE,
	AKSV_WRITE_DONE,
	FIRST_AUTHENTICATION_DONE,
	SECOND_AUTHENTICATION_RDY,
	SECOND_AUTHENTICATION_DONE,
};

enum HDCP_VERSION {
	HDCP_NONE,
	HDCP_VERSION_14,
	HDCP_VERSION_22,
};

#define DEFAULT_AUDIO_CODEC	HDMI_AUDIO_PCM

struct hdmi_resources {
	struct clk *hdmi;
	struct clk *pixel;
	struct clk *tmds;
	int gpio_hpd;
	int gpio_ls;
	int gpio_dcdc;
};


struct hdcp_info {
	u8 is_repeater;
	u32 hdcp_start;
	int hdcp_enable;

	enum HDCP_EVENT	event;
	enum HDCP_STATE	auth_status;
};

struct hdmi_device {
	struct drm_encoder		encoder;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct drm_connector		connector;
	struct device_node 		*panel_node;
	struct drm_panel 			*panel;
	bool				powered;
	struct drm_display_mode		current_mode;
	u8				cea_video_id;
	struct i2c_client			*ddc_port;

	/** base address of HDMI registers */
	void __iomem *regs;
	void __iomem *phy_regs;
	void __iomem *pmu_regs;
	void __iomem *otp_regs;

	/** HDMI interrupt */
	unsigned int int_irq;
	unsigned int ext_irq;

	/** configuration of current graphic mode */
	const struct hdmi_config *cur_conf;
	/** current preset */
	struct v4l2_dv_timings cur_timings;

	/** other resources */
	struct hdmi_resources res;
	/** supported HDMI InfoFrame */
	//struct hdmi_infoframe infoframe[INFOFRAME_CNT];
	/** audio on/off control flag */
	int audio_enable;
	/** number of audio channels */
	int audio_channel_count;
	/** audio sample rate */
	int sample_rate;
	/** audio bits per sample */
	int bits_per_sample;
	/** audio bit clock support or not */
	int audio_master_clk;
	/** RGB Quantization range and Colorimetry */
	enum HDMI_COLOR_RANGE color_range;
	/** HDMI is streaming or not */
	enum HDMI_STREAMING_STATE streaming;
	/** current probe state of hdmi driver */
	enum HDMI_PROBE_STATE probe_state;
	/** current audio codec type */
	enum HDMI_AUDIO_CODEC audio_codec;
	/** HDMI output format */
	enum HDMI_OUTPUT_FMT output_fmt;
	/** Aspect ratio information */
	enum HDMI_ASPECT_RATIO aspect;

	/* HPD releated */
	struct workqueue_struct *hpd_wq;
	struct workqueue_struct *hpd_wq_ext;
	struct work_struct hpd_work;
	struct delayed_work hpd_work_ext;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_int;
	struct pinctrl_state *pin_ext;

	/* choose DVI or HDMI mode */
	int dvi_mode;

	/** mutex for protection of fields below */
	struct mutex mutex;

	/** hdcp **/
	/** HDCP information */
	struct hdcp_info hdcp_info;
	struct work_struct work;
	struct workqueue_struct	*hdcp_wq;
};

struct hdmi_config {
	const struct v4l2_dv_timings dv_timings;

	u32		h_line;
	u32		v_line;

	u32		vpol;
	u32		vfp;
	u32		vsp;
	u32		vbp;

	u32		hpol;
	u32		hfp;
	u32		hsp;
	u32		hbp;

	u32		interlace;
	u32		vsync_offset;

	u8		avi_vic;	// CEA VIC
	u8		avi_vic_16_9;	// CEA VIC
};
extern const struct hdmi_config hdmi_config[];

struct hdmiphy_conf {
	const struct v4l2_dv_timings dv_timings;
	const u8 *data;
};
extern const struct hdmiphy_conf hdmiphy_conf[];
extern const int hdmi_pre_cnt;
extern const int hdmiphy_conf_cnt;

bool hdmi_match_timings(const struct v4l2_dv_timings *t1,
			  const struct v4l2_dv_timings *t2,
			  unsigned pclock_delta);

void s5p_v4l2_int_src_hdmi_hpd(struct hdmi_device *hdev);
void s5p_v4l2_int_src_ext_hpd(struct hdmi_device *hdev);

/** HDMI reg functions */
irqreturn_t hdmi_irq_handler(int irq, void *dev_data);
int hdmi_conf_apply(struct hdmi_device *hdmi_dev);
void hdmi_timing_apply(struct hdmi_device *hdev, const struct hdmi_config *conf);
void hdmi_video_pattern_enable(struct hdmi_device *hdev,
			const struct hdmi_config *conf);
void hdmi_reg_init(struct hdmi_device *hdev);
void hdmi_set_dvi_mode(struct hdmi_device *hdev);
void hdmi_avi_packet_config(struct hdmi_device *hdev);
void hdmi_enable(struct hdmi_device *hdev, int on);
int is_hdmiphy_ready(struct hdmi_device *hdev);
void hdmi_phy_sw_reset(struct hdmi_device *hdev);
void hdmiphy_set_isolation(struct hdmi_device *hdev, int en);
void hdmiphy_set_mode(struct hdmi_device *hdev, int en);
void hdmiphy_set_power(struct hdmi_device *hdev, int en);
void hdmiphy_conf_store(struct hdmi_device *hdev, const char *buf);
void hdmi_hpd_clear_int(struct hdmi_device *hdev);
void hdmi_reg_set_int_hpd(struct hdmi_device *hdev);
void hdmi_reg_set_ext_hpd(struct hdmi_device *hdev);
void hdmi_dumpregs(struct hdmi_device *hdev, char *prefix);

/** DDC functions**/
void hdmi_attach_ddc_client(struct i2c_client *ddc);
extern struct i2c_driver ddc_driver;

/** HDCP functions */
irqreturn_t hdcp_irq_handler(struct hdmi_device *hdev);
int hdcp_stop(struct hdmi_device *hdev);
int hdcp_start(struct hdmi_device *hdev);
int hdcp_prepare(struct hdmi_device *hdev);

static inline
void hdmi_write(struct hdmi_device *hdev, u32 reg_id, u32 value)
{
	writel(value, hdev->regs + reg_id);
}

static inline
void hdmi_write_mask(struct hdmi_device *hdev, u32 reg_id, u32 value, u32 mask)
{
	u32 old = readl(hdev->regs + reg_id);
	old &= ~mask;
	old |= value;
	writel(old, hdev->regs + reg_id);
}

static inline
void hdmiphy_write_mask(struct hdmi_device *hdev, u32 reg_id, u32 value, u32 mask)
{
	u32 old = readl(hdev->phy_regs + reg_id);
	old &= ~mask;
	old |= value;
	writel(old, hdev->phy_regs + reg_id);
}

static inline
void hdmi_writeb(struct hdmi_device *hdev, u32 reg_id, u8 value)
{
	writeb(value, hdev->regs + reg_id);
}

static inline void hdmi_write_bytes(struct hdmi_device *hdev, u32 reg_id,
		u8 *buf, int bytes)
{
	int i;

	for (i = 0; i < bytes; ++i)
		writeb(buf[i], hdev->regs + reg_id + i * 4);
}

static inline u32 hdmi_read(struct hdmi_device *hdev, u32 reg_id)
{
	return readl(hdev->regs + reg_id);
}

static inline u8 hdmi_readb(struct hdmi_device *hdev, u32 reg_id)
{
	return readb(hdev->regs + reg_id);
}

static inline u8 hdmi_read_bytes(struct hdmi_device *hdev, u32 reg_id, int bytes)
{
	u8 val;
	u32 old = readl(hdev->regs + reg_id);
	val = (old & (0xFF << (bytes * 8))) >> (bytes * 8);

	return val;
}

#endif /* SAMSUNG_HDMI_H */
