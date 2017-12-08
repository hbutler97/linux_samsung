/* linux/drivers/gpu/drm/exynos-hq/exynos_drm_dsi.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Samsung MIPI DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __EXYNOS_DRM_DSI_H__
#define __EXYNOS_DRM_DSI_H__

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic_helper.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/component.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"


/* Add header */
#include "regs-dsim.h"
#include <linux/types.h>

/* returns true iff both arguments logically differs */
#define NEQV(a, b) (!(a) ^ !(b))

#ifndef MHZ
#define MHZ (1000*1000)
#endif

/* This is for dual video mode that has 8 mipi data lanes */
#define DSI_8_DATA_LANES		8
#define DSI_MAX_BUS_WIDTH		4
#define DSI_NUM_VIRTUAL_CHANNELS	4
#define DSI_TX_FIFO_SIZE		2048
#define DSI_RX_FIFO_SIZE		256
#define DSI_XFER_TIMEOUT_MS		100
#define DSI_RX_FIFO_EMPTY		0x30800002

#define OLD_SCLK_MIPI_CLK_NAME "pll_clk"

#define REG_ADDR(dsi, reg_idx)		((dsi)->reg_base + \
					dsi->driver_data->reg_ofs[(reg_idx)])
#define DSI_WRITE(dsi, reg_idx, val)	writel((val), \
					REG_ADDR((dsi), (reg_idx)))
#define DSI_READ(dsi, reg_idx)		readl(REG_ADDR((dsi), (reg_idx)))

enum exynos_dsi_transfer_type {
	EXYNOS_DSI_TX,
	EXYNOS_DSI_RX,
};

/* operation state of dsim driver */
enum dsim_state {
	DSIM_STATE_HSCLKEN,	/* HS clock was enabled. */
	DSIM_STATE_ULPS,	/* DSIM was entered ULPS state */
	DSIM_STATE_SUSPEND	/* DSIM is suspend state */
};

struct exynos_dsi_transfer {
	struct list_head list;
	struct completion completed;
	int result;
	u8 data_id;
	u8 data[2];
	u16 flags;

	const u8 *tx_payload;
	u16 tx_len;
	u16 tx_done;

	u8 *rx_payload;
	u16 rx_len;
	u16 rx_done;
};

#define DSIM_STATE_ENABLED		BIT(0)
#define DSIM_STATE_INITIALIZED		BIT(1)
#define DSIM_STATE_CMD_LPM		BIT(2)
#define DSIM_STATE_VIDOUT_AVAILABLE	BIT(3)

struct exynos_dsi_driver_data {
	unsigned int *reg_ofs;
	unsigned int plltmr_reg;
	unsigned int has_freqband:1;
	unsigned int has_clklane_stop:1;
	unsigned int num_clks;
	unsigned int max_freq;
	unsigned int wait_for_reset;
	unsigned int num_bits_resol;
	unsigned int *reg_values;
};

/* decon_lcd.h + dsim_common.h */
enum decon_psr_mode {
	DECON_VIDEO_MODE = 0,
	DECON_DP_PSR_MODE = 1,
	DECON_MIPI_COMMAND_MODE = 2,
};

/* Mic ratio: 0: 1/2 ratio, 1: 1/3 ratio */
enum decon_mic_comp_ratio {
	MIC_COMP_RATIO_1_2 = 0,
	MIC_COMP_RATIO_1_3 = 1,
	MIC_COMP_BYPASS
};

enum mic_ver {
	MIC_VER_1_1,
	MIC_VER_1_2,
	MIC_VER_2_0,
};

enum type_of_ddi {
	TYPE_OF_SM_DDI = 0,
	TYPE_OF_MAGNA_DDI,
	TYPE_OF_NORMAL_DDI,
};

struct stdphy_pms {
	unsigned int p;
	unsigned int m;
	unsigned int s;
};

struct decon_lcd {
	enum decon_psr_mode mode;
	unsigned int vfp;
	unsigned int vbp;
	unsigned int hfp;
	unsigned int hbp;

	unsigned int vsa;
	unsigned int hsa;

	unsigned int xres;
	unsigned int yres;

	unsigned int width;
	unsigned int height;

	unsigned int hs_clk;
	struct stdphy_pms dphy_pms;
	unsigned int esc_clk;

	unsigned int fps;
	unsigned int mic_enabled;
	enum decon_mic_comp_ratio mic_ratio;
	unsigned int dsc_enabled;
	unsigned int dsc_cnt;
	unsigned int dsc_slice_num;
	enum mic_ver mic_ver;
	enum type_of_ddi ddi_type;
};

struct dsim_pll_param {
	u32 p;
	u32 m;
	u32 s;
	u32 pll_freq; /* in/out parameter: Mhz */
};

struct dsim_clks {
	u32 hs_clk;
	u32 esc_clk;
	u32 byte_clk;
};

struct dphy_timing_value {
	u32 bps;
	u32 clk_prepare;
	u32 clk_zero;
	u32 clk_post;
	u32 clk_trail;
	u32 hs_prepare;
	u32 hs_zero;
	u32 hs_trail;
	u32 lpx;
	u32 hs_exit;
	u32 b_dphyctl;
};

struct dsim_clks_param {
	struct dsim_clks clks;
	struct dsim_pll_param pll;
	struct dphy_timing_value t;

	u32 esc_div;
};

struct dsim_resources {
	struct clk *pclk;
	struct clk *dphy_esc;
	struct clk *dphy_byte;
	struct clk *rgb_vclk0;
	struct clk *pclk_disp;
	int lcd_power[2];
	int lcd_reset;
};

struct exynos_dsi {
	struct drm_encoder encoder;
	struct mipi_dsi_host dsi_host;
	struct drm_connector connector;
	struct device_node *panel_node;
	struct drm_panel *panel;
	struct device *dev;
	struct exynos_dsi_transfer xfer;

	void __iomem *reg_base;
	struct phy *phy;
	struct clk **clks;
	struct regulator_bulk_data supplies[2];
	int irq;
	int te_gpio;

	u32 pll_clk_rate;
	u32 burst_clk_rate;
	u32 esc_clk_rate;
	u32 lanes;
	u32 mode_flags;
	u32 format;
	struct videomode vm;

	//int state;
	struct drm_property *brightness;
	struct completion r_completed;

	spinlock_t transfer_lock; /* protects transfer_list */
	spinlock_t slock; /* protects transfer_list */
	struct list_head transfer_list;

	struct exynos_dsi_driver_data *driver_data;
	struct device_node *bridge_node;

	/* Add newly for 8890 */
	u32 id;
	u32 data_lane;
	u32 data_lane_cnt;
	struct decon_lcd lcd_info;
	struct dsim_clks_param clks_param;
	struct dsim_resources res;
	enum dsim_state state;
	bool force_disconnected;
};

#define host_to_dsi(host) container_of(host, struct exynos_dsi, dsi_host)
#define connector_to_dsi(c) container_of(c, struct exynos_dsi, connector)

static inline struct exynos_dsi *encoder_to_dsi(struct drm_encoder *e)
{
	return container_of(e, struct exynos_dsi, encoder);
}

enum reg_idx {
	DSIM_STATUS_REG,	/* Status register */
	DSIM_SWRST_REG,		/* Software reset register */
	DSIM_CLKCTRL_REG,	/* Clock control register */
	DSIM_TIMEOUT_REG,	/* Time out register */
	DSIM_CONFIG_REG,	/* Configuration register */
	DSIM_ESCMODE_REG,	/* Escape mode register */
	DSIM_MDRESOL_REG,
	DSIM_MVPORCH_REG,	/* Main display Vporch register */
	DSIM_MHPORCH_REG,	/* Main display Hporch register */
	DSIM_MSYNC_REG,		/* Main display sync area register */
	DSIM_INTSRC_REG,	/* Interrupt source register */
	DSIM_INTMSK_REG,	/* Interrupt mask register */
	DSIM_PKTHDR_REG,	/* Packet Header FIFO register */
	DSIM_PAYLOAD_REG,	/* Payload FIFO register */
	DSIM_RXFIFO_REG,	/* Read FIFO register */
	DSIM_FIFOCTRL_REG,	/* FIFO status and control register */
	DSIM_PLLCTRL_REG,	/* PLL control register */
	DSIM_PHYCTRL_REG,
	DSIM_PHYTIMING_REG,
	DSIM_PHYTIMING1_REG,
	DSIM_PHYTIMING2_REG,
	NUM_REGS
};

enum reg_value_idx {
	RESET_TYPE,
	PLL_TIMER,
	STOP_STATE_CNT,
	PHYCTRL_ULPS_EXIT,
	PHYCTRL_VREG_LP,
	PHYCTRL_SLEW_UP,
	PHYTIMING_LPX,
	PHYTIMING_HS_EXIT,
	PHYTIMING_CLK_PREPARE,
	PHYTIMING_CLK_ZERO,
	PHYTIMING_CLK_POST,
	PHYTIMING_CLK_TRAIL,
	PHYTIMING_HS_PREPARE,
	PHYTIMING_HS_ZERO,
	PHYTIMING_HS_TRAIL
};


/* For 8890 */
int dsim_write_data(struct exynos_dsi *dsi, unsigned int data_id,
		unsigned long data0, unsigned int data1);
int dsim_read_data(struct exynos_dsi *dsi, u32 data_id, u32 addr,
		u32 count, u8 *buf);
int dsim_wait_for_cmd_done(struct exynos_dsi *dsi);

#define MAX_DSI_CNT 3
extern struct exynos_dsi *dsi_drvdata[MAX_DSI_CNT];
static inline struct exynos_dsi *get_dsim_drvdata(u32 id)
{
	if (id < 0 || id > MAX_DSI_CNT)
		BUG();
	return dsi_drvdata[id];
}

static inline int dsim_rd_data(u32 id, u32 cmd_id,
	 u32 addr, u32 size, u8 *buf)
{
	int ret;
	struct exynos_dsi *dsim = get_dsim_drvdata(id);

	ret = dsim_read_data(dsim, cmd_id, addr, size, buf);
	if (ret)
		return ret;

	return 0;
}

static inline int dsim_wr_data(u32 id, u32 cmd_id, unsigned long d0, u32 d1)
{
	int ret;
	struct exynos_dsi *dsim = get_dsim_drvdata(id);

	ret = dsim_write_data(dsim, cmd_id, d0, d1);
	if (ret)
		return ret;

	return 0;
}

static inline int dsim_wait_for_cmd_completion(u32 id)
{
	int ret;
	struct exynos_dsi *dsim = get_dsim_drvdata(id);

	ret = dsim_wait_for_cmd_done(dsim);

	return ret;
}

/* register access subroutines */
static inline u32 dsim_read(u32 id, u32 reg_id)
{
	struct exynos_dsi *dsim = get_dsim_drvdata(id);
	return readl(dsim->reg_base + reg_id);
}

static inline u32 dsim_read_mask(u32 id, u32 reg_id, u32 mask)
{
	u32 val = dsim_read(id, reg_id);
	val &= (mask);
	return val;
}

static inline void dsim_write(u32 id, u32 reg_id, u32 val)
{
	struct exynos_dsi *dsim = get_dsim_drvdata(id);
	writel(val, dsim->reg_base + reg_id);
}

static inline void dsim_write_mask(u32 id, u32 reg_id, u32 val, u32 mask)
{
	struct exynos_dsi *dsim = get_dsim_drvdata(id);
	u32 old = dsim_read(id, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, dsim->reg_base + reg_id);
}

/* dsim_common.h */
#define DSIM_PIXEL_FORMAT_RGB24		0x7
#define DSIM_PIXEL_FORMAT_RGB18		0x6
#define DSIM_PIXEL_FORMAT_RGB18_PACKED	0x5
#define DSIM_RX_FIFO_MAX_DEPTH		64

/* define DSI lane types. */
enum {
	DSIM_LANE_CLOCK	= (1 << 0),
	DSIM_LANE_DATA0	= (1 << 1),
	DSIM_LANE_DATA1	= (1 << 2),
	DSIM_LANE_DATA2	= (1 << 3),
	DSIM_LANE_DATA3	= (1 << 4),
};

/* DSI Error report bit definitions */
enum {
	MIPI_DSI_ERR_SOT			= (1 << 0),
	MIPI_DSI_ERR_SOT_SYNC			= (1 << 1),
	MIPI_DSI_ERR_EOT_SYNC 			= (1 << 2),
	MIPI_DSI_ERR_ESCAPE_MODE_ENTRY_CMD	= (1 << 3),
	MIPI_DSI_ERR_LOW_POWER_TRANSMIT_SYNC	= (1 << 4),
	MIPI_DSI_ERR_HS_RECEIVE_TIMEOUT		= (1 << 5),
	MIPI_DSI_ERR_FALSE_CONTROL		= (1 << 6),
	/* Bit 7 is reserved */
	MIPI_DSI_ERR_ECC_SINGLE_BIT		= (1 << 8),
	MIPI_DSI_ERR_ECC_MULTI_BIT		= (1 << 9),
	MIPI_DSI_ERR_CHECKSUM			= (1 << 10),
	MIPI_DSI_ERR_DATA_TYPE_NOT_RECOGNIZED	= (1 << 11),
	MIPI_DSI_ERR_VCHANNEL_ID_INVALID	= (1 << 12),
	MIPI_DSI_ERR_INVALID_TRANSMIT_LENGTH	= (1 << 13),
	/* Bit 14 is reserved */
	MIPI_DSI_ERR_PROTOCAL_VIOLATION		= (1 << 15),
	/* DSI_PROTOCAL_VIOLATION[15] is for protocol violation that is caused EoTp
	 * missing So this bit is egnored because of not supportung @S.LSI AP */
	/* FALSE_ERROR_CONTROL[6] is for detect invalid escape or turnaround sequence.
	 * This bit is not supporting @S.LSI AP because of non standard
	 * ULPS enter/exit sequence during power-gating */
	/* Bit [14],[7] is reserved */
	MIPI_DSI_ERR_BIT_MASK			= (0x3f3f), /* Error_Range[13:0] */
};


/* CAL APIs list */
int dsim_reg_init(u32 id, struct decon_lcd *lcd_info,
			u32 data_lane_cnt, struct dsim_clks *clks);
void dsim_reg_init_probe(u32 id, struct decon_lcd *lcd_info,
			u32 data_lane_cnt, struct dsim_clks *clks);
int dsim_reg_set_clocks(u32 id, struct dsim_clks *clks, struct stdphy_pms *dphy_pms, u32 en);
int dsim_reg_set_lanes(u32 id, u32 lanes, u32 en);
int dsim_reg_set_hs_clock(u32 id, u32 en);
void dsim_reg_set_int(u32 id, u32 en);
int dsim_reg_set_ulps(u32 id, u32 en, u32 lanes);
int dsim_reg_set_smddi_ulps(u32 id, u32 en, u32 lanes);
/* RX related APIs list */
u32 dsim_reg_rx_fifo_is_empty(u32 id);
u32 dsim_reg_get_rx_fifo(u32 id);
int dsim_reg_rx_err_handler(u32 id, u32 rx_fifo);

/* CAL raw functions list */
void dsim_reg_sw_reset(u32 id);
void dsim_reg_dphy_reset(u32 id);
void dsim_reg_funtion_reset(u32 id);
void dsim_reg_dp_dn_swap(u32 id, u32 en);
void dsim_reg_set_num_of_lane(u32 id, u32 lane);
void dsim_reg_enable_lane(u32 id, u32 lane, u32 en);
void dsim_reg_set_pll_freq(u32 id, u32 p, u32 m, u32 s);
void dsim_reg_pll_stable_time(u32 id);
void dsim_reg_set_dphy_timing_values(u32 id, struct dphy_timing_value *t);
void dsim_reg_clear_int(u32 id, u32 int_src);
void dsim_reg_clear_int_all(u32 id);
void dsim_reg_set_pll(u32 id, u32 en);
u32 dsim_reg_is_pll_stable(u32 id);
int dsim_reg_enable_pll(u32 id, u32 en);
void dsim_reg_set_byte_clock(u32 id, u32 en);
void dsim_reg_set_esc_clk_prescaler(u32 id, u32 en, u32 p);
void dsim_reg_set_esc_clk_on_lane(u32 id, u32 en, u32 lane);
u32 dsim_reg_wait_lane_stop_state(u32 id);
void dsim_reg_set_stop_state_cnt(u32 id);
void dsim_reg_set_bta_timeout(u32 id);
void dsim_reg_set_lpdr_timeout(u32 id);
void dsim_reg_set_porch(u32 id, struct decon_lcd *lcd);
void dsim_reg_set_pixel_format(u32 id, u32 pixformat);
void dsim_reg_set_config(u32 id, struct decon_lcd *lcd_info, u32 data_lane_cnt);
void dsim_reg_set_cmd_transfer_mode(u32 id, u32 lp);
void dsim_reg_set_multipix(u32 id, u32 multipix);
void dsim_reg_set_vc_id(u32 id, u32 vcid);
void dsim_reg_set_video_mode(u32 id, u32 mode);
void dsim_reg_enable_dsc(u32 id, u32 en);
void dsim_reg_disable_hsa(u32 id, u32 en);
void dsim_reg_disable_hbp(u32 id, u32 en);
void dsim_reg_disable_hfp(u32 id, u32 en);
void dsim_reg_disable_hse(u32 id, u32 en);
void dsim_reg_set_hsync_preserve(u32 id, u32 en);
void dsim_reg_set_burst_mode(u32 id, u32 burst);
void dsim_reg_set_sync_inform(u32 id, u32 inform);
void dsim_reg_set_cmdallow(u32 id, u32 cmdallow);
void dsim_reg_set_stable_vfp(u32 id, u32 stablevfp);
void dsim_reg_set_vbp(u32 id, u32 vbp);
void dsim_reg_set_hfp(u32 id, u32 hfp);
void dsim_reg_set_hbp(u32 id, u32 hbp);
void dsim_reg_set_vsa(u32 id, u32 vsa);
void dsim_reg_set_hsa(u32 id, u32 hsa);
void dsim_reg_set_vresol(u32 id, u32 vresol);
void dsim_reg_set_hresol(u32 id, u32 hresol, struct decon_lcd *lcd);
void dsim_reg_set_multi_packet_count(u32 id, u32 multipacketcnt);
void dsim_reg_set_command_control(u32 id, u32 cmdcontrol);
void dsim_reg_set_time_stable_vfp(u32 id, u32 stablevfp);
void dsim_reg_set_time_vsync_timeout(u32 id, u32 vsynctout);
void dsim_reg_set_time_te_protect_on(u32 id, u32 teprotecton);
void dsim_reg_set_time_te_timeout(u32 id, u32 tetout);
void dsim_reg_set_hsync_timeout(u32 id, u32 hsynctout);
void dsim_reg_enable_mflush(u32 id, u32 en);
void dsim_reg_enable_noncontinuous_clock(u32 id, u32 en);
void dsim_reg_enable_clocklane_stop_start(u32 id, u32 en);
void dsim_reg_enable_packetgo(u32 id, u32 en);
void dsim_reg_set_packetgo_ready(u32 id);
void dsim_reg_enable_multi_command_packet(u32 id, u32 en);
void dsim_reg_enable_shadow(u32 id, u32 en);
void dsim_reg_enable_hs_clock(u32 id, u32 en);
void dsim_reg_enable_byte_clock(u32 id, u32 en);
u32 dsim_reg_is_hs_clk_ready(u32 id);
void dsim_reg_enable_per_frame_read(u32 id, u32 en);
void dsim_reg_enable_qchannel(u32 id, u32 en);
int dsim_reg_wait_hs_clk_ready(u32 id);
u32 dsim_reg_is_writable_fifo_state(u32 id);
u32 dsim_reg_header_fifo_is_empty(u32 id);
int dsim_reg_wait_for_hdr_fifo_empty(u32 id);
void dsim_reg_set_fifo_ctrl(u32 id, u32 cfg);
void dsim_reg_force_dphy_stop_state(u32 id, u32 en);
void dsim_reg_wr_tx_header(u32 id, u32 data_id, unsigned long data0, u32 data1);
void dsim_reg_wr_tx_payload(u32 id, u32 payload);
void dsim_reg_enter_ulps(u32 id, u32 enter);
void dsim_reg_exit_ulps(u32 id, u32 exit);
int dsim_reg_set_ulps_by_ddi(u32 id, u32 ddi_type, u32 lanes, u32 en);
int dsim_reg_wait_enter_ulps_state(u32 id, u32 lanes);
int dsim_reg_wait_exit_ulps_state(u32 id);
void dsim_reg_set_standby(u32 id, u32 en);
void dsim_reg_set_bist(u32 id, u32 en, u32 vfp, u32 format, u32 type);
void dsim_reg_set_packet_ctrl(u32 id);
void dsim_reg_enable_loopback(u32 id, u32 en);
void dsim_reg_set_loopback_id(u32 id, u32 en);
void dsim_reg_set_pkt_go_enable(u32 id, bool en);
void dsim_reg_set_pkt_go_ready(u32 id);
void dsim_reg_set_pkt_go_cnt(u32 id, unsigned int count);
void dsim_reg_set_shadow(u32 id, u32 en);
void dsim_reg_shadow_update(u32 id);
int dsim_reg_exit_ulps_and_start(u32 id, u32 ddi_type, u32 lanes);
int dsim_reg_stop_and_enter_ulps(u32 id, u32 ddi_type, u32 lanes);
void dsim_reg_start(u32 id, struct dsim_clks *clks, u32 lanes);
void dsim_reg_stop(u32 id, u32 lanes);
void dsim_reg_set_partial_update(u32 id, struct decon_lcd *lcd_info);

#endif /* __EYXNOS_DRM_DSI_H__ */
