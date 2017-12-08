/* linux/drivers/gpu/drm/exynos-hq/exynos_drm_dsi.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung MIPI DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "exynos_drm_dsi.h"
#include <linux/completion.h>

#ifndef MHZ
#define MHZ	(1000*1000)
#endif

struct exynos_dsi *dsi_drvdata[MAX_DSI_CNT];

char *clk_names[3] = {
	"dsim_pclk",
	"dphy_escclk",
	"dphy_byteclk" };

static struct exynos_dsi_driver_data exynos8890_dsi_driver_data = {
	.num_clks = 3,
};

static struct of_device_id exynos_dsi_of_match[] = {
	{ .compatible = "samsung,exynos-mipi-dsi",
	  .data = &exynos8890_dsi_driver_data },
	{ }
};

static int __maybe_unused exynos_dsi_suspend(struct device *dev);
static int __maybe_unused exynos_dsi_resume(struct device *dev);

static inline struct exynos_dsi_driver_data *exynos_dsi_get_driver_data(
						struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(exynos_dsi_of_match, &pdev->dev);

	return (struct exynos_dsi_driver_data *)of_id->data;
}

static void exynos_dsi_disable_clock(struct exynos_dsi *dsi)
{
	/* It is fixed that dsim id is zero and dsim used 4 data lanes */
	u32 lanes = DSIM_LANE_CLOCK | dsi->data_lane;

	DRM_DEBUG_KMS("%s +\n", __func__);

	/* unset standby and disable HS clock */
	dsim_reg_set_standby(dsi->id, 0);
	dsim_reg_set_hs_clock(dsi->id, 0);
	dsim_reg_set_lanes(dsi->id, lanes, 0);
	dsim_reg_set_esc_clk_on_lane(dsi->id, 0, lanes);
	dsim_reg_enable_byte_clock(dsi->id, 0);
	dsim_reg_set_clocks(dsi->id, NULL, NULL, 0);
	dsim_reg_sw_reset(dsi->id);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

void exynos_dsi_enable_lane(struct exynos_dsi *dsi, u32 lane)
{
	/* It is fixed that dsim id is zero and dsim used 4 data lanes */
	u32 lanes = DSIM_LANE_CLOCK | dsi->data_lane;

	DRM_DEBUG_KMS("%s +\n", __func__);

	dsim_reg_set_lanes(dsi->id, lanes, 0);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void exynos_dsi_set_display_mode(struct exynos_dsi *dsi)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	dsim_reg_init(dsi->id, &dsi->lcd_info, dsi->data_lane_cnt,
		&dsi->clks_param.clks);
	if (dsi->lanes == DSI_8_DATA_LANES) {
		u32 id = !dsi->id;
		dsim_reg_init(id, &dsi->lcd_info, dsi->data_lane_cnt,
				&dsi->clks_param.clks);
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void exynos_dsi_set_display_enable(struct exynos_dsi *dsi, bool enable)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	if (enable) {
		dsim_reg_start(dsi->id, &dsi->clks_param.clks,
			DSIM_LANE_CLOCK | dsi->data_lane);
		if (dsi->lanes == DSI_8_DATA_LANES) {
			u32 id = !dsi->id;
			dsim_reg_start(id, &dsi->clks_param.clks,
					DSIM_LANE_CLOCK | dsi->data_lane);
		}
	} else {
		dsim_reg_stop(dsi->id, DSIM_LANE_CLOCK | dsi->data_lane);
		if (dsi->lanes == DSI_8_DATA_LANES) {
			u32 id = !dsi->id;
			dsim_reg_stop(id, DSIM_LANE_CLOCK | dsi->data_lane);
		}
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void exynos_dsi_transfer_start(struct exynos_dsi *dsi)
{
	struct exynos_dsi_transfer *xfer = &dsi->xfer;
	const u8 *payload = xfer->tx_payload;
	u16 length = xfer->tx_len;
	u32 reg;

	DRM_DEBUG_KMS("%s +\n", __func__);

	DRM_DEBUG_KMS("< xfer %p: tx len %u, done %u, rx len %u, done %u\n",
		xfer, xfer->tx_len, xfer->tx_done, xfer->rx_len, xfer->rx_done);

	dsim_reg_clear_int(dsi->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Send payload */
	while (length >= 4) {
		reg = (payload[3] << 24) | (payload[2] << 16)
					| (payload[1] << 8) | payload[0];
		dsim_reg_wr_tx_payload(dsi->id, reg);
		payload += 4;
		length -= 4;
	}

	reg = 0;
	switch (length) {
	case 3:
		reg |= payload[2] << 16;
		/* Fall through */
	case 2:
		reg |= payload[1] << 8;
		/* Fall through */
	case 1:
		reg |= payload[0];
		dsim_reg_wr_tx_payload(dsi->id, reg);
		break;
	case 0:
		/* Do nothing */
		break;
	}

	dsim_reg_wr_tx_header(dsi->id, xfer->data_id, xfer->data[0], xfer->data[1]);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void exynos_dsi_dump(struct exynos_dsi *dsi)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			dsi->reg_base, 0xC0, false);
}


static int exynos_dsi_transfer(struct exynos_dsi *dsi)
{
	struct exynos_dsi_transfer *xfer = &dsi->xfer;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	reinit_completion(&xfer->completed);

	exynos_dsi_transfer_start(dsi);

	ret = wait_for_completion_timeout(&xfer->completed,
				    msecs_to_jiffies(DSI_XFER_TIMEOUT_MS));
	if (!ret) {
		if (dsim_reg_header_fifo_is_empty(dsi->id)) {
			reinit_completion(&xfer->completed);
			dsim_reg_clear_int(dsi->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		DRM_ERROR("xfer timed out: %*ph %*ph\n", 2, xfer->data,
			xfer->tx_len, xfer->tx_payload);
		exynos_dsi_dump(dsi);
		return -ETIMEDOUT;
	}

	/* Also covers hardware timeout condition */
	DRM_DEBUG_KMS("%s -\n", __func__);

	return 0;
}

static int exynos_dsi_get_rx_payload(struct exynos_dsi *dsi,
		struct exynos_dsi_transfer *xfer)
{
	u32 rx_fifo = 0, rx_size = 0;
	int i, j, ret = 0;
	int count = xfer->rx_len;
	u8 *buf = xfer->rx_payload;
	u32 rx_fifo_depth = DSIM_RX_FIFO_MAX_DEPTH;

	do {
		rx_fifo = dsim_reg_get_rx_fifo(dsi->id);

		/* Parse the RX packet data types */
		switch (rx_fifo & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_reg_rx_err_handler(dsi->id, rx_fifo);
			if (ret < 0) {
				exynos_dsi_dump(dsi);
				return ret;
			}
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			DRM_DEBUG_KMS("EoTp was received from LCD module.\n");
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			DRM_INFO("short pkt was received\n");
			for (i = 0; i <= count; i++)
				buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			DRM_INFO("long pkt was received\n");
			rx_size = (rx_fifo & 0x00ffff00) >> 8;
			DRM_INFO("rx fifo : %8x, rsp : %x, rx_size : %d\n",
					rx_fifo, rx_fifo & 0xff, rx_size);
			/* Read data from RX packet payload */
			for (i = 0; i < rx_size >> 2; i++) {
				rx_fifo = dsim_reg_get_rx_fifo(dsi->id);
				for (j = 0; j < 4; j++)
					buf[(i*4)+j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			if (rx_size % 4) {
				rx_fifo = dsim_reg_get_rx_fifo(dsi->id);
				for (j = 0; j < rx_size % 4; j++)
					buf[4 * i + j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			break;
		default:
			DRM_ERROR("Packet format is invaild.\n");
			ret = -EBUSY;
			break;
		}
	} while (!dsim_reg_rx_fifo_is_empty(dsi->id) && --rx_fifo_depth);

	ret = rx_size;
	if (!rx_fifo_depth) {
		DRM_ERROR("Check DPHY values about HS clk.\n");
		exynos_dsi_dump(dsi);
		ret = -EBUSY;
	}

	return ret;
}

static irqreturn_t exynos_dsi_irq(int irq, void *dev_id)
{
	struct exynos_dsi *dsi = dev_id;
	struct exynos_dsi_transfer *xfer = &dsi->xfer;
	u32 status;

	spin_lock(&dsi->slock);

#if 0
	if (dsi->state == DSIM_STATE_SUSPEND) {
		dev_warn(dsi->dev, "dsim power is off, state(0x%x)\n", dsi->state);
		spin_unlock(&dsi->slock);
		return IRQ_HANDLED;
	}
#endif

	status = readl(dsi->reg_base + DSIM_INTSRC);
	DRM_DEBUG("%s: interrupt status(0x%x)\n", __func__, status);
	if (!status) {
		static unsigned long int j;
		if (printk_timed_ratelimit(&j, 500))
			dev_warn(dsi->dev, "spurious interrupt\n");
		spin_unlock(&dsi->slock);
		return IRQ_HANDLED;
	}

	if (status & DSIM_INTSRC_SFR_PH_FIFO_EMPTY)
		complete(&xfer->completed);

	if (status & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsi->r_completed);

	dsim_reg_clear_int(0, status);

	spin_unlock(&dsi->slock);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_dsi_te_irq_handler(int irq, void *dev_id)
{
	struct exynos_dsi *dsi = (struct exynos_dsi *)dev_id;
	struct drm_encoder *encoder = &dsi->encoder;

	if (dsi->state == DSIM_STATE_HSCLKEN)
		exynos_drm_crtc_te_handler(encoder->crtc);

	return IRQ_HANDLED;
}

void exynos_dsi_enable_irq(struct exynos_dsi *dsi)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	enable_irq(dsi->irq);

	if (gpio_is_valid(dsi->te_gpio))
		enable_irq(gpio_to_irq(dsi->te_gpio));

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void exynos_dsi_disable_irq(struct exynos_dsi *dsi)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	if (gpio_is_valid(dsi->te_gpio))
		disable_irq(gpio_to_irq(dsi->te_gpio));

	disable_irq(dsi->irq);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static int exynos_dsi_register_te_irq(struct exynos_dsi *dsi)
{
	int ret;
	int te_gpio_irq;

	DRM_DEBUG_KMS("%s +\n", __func__);

	dsi->te_gpio = of_get_named_gpio(dsi->panel_node, "te-gpios", 0);
	if (!gpio_is_valid(dsi->te_gpio)) {
		DRM_ERROR("no te-gpios specified\n");
		ret = dsi->te_gpio;
		goto out;
	}

	ret = gpio_request(dsi->te_gpio, "te_gpio");
	if (ret) {
		DRM_ERROR("gpio request failed with %d\n", ret);
		goto out;
	}

	te_gpio_irq = gpio_to_irq(dsi->te_gpio);
	//irq_set_status_flags(te_gpio_irq, IRQ_NOAUTOEN);

	ret = request_threaded_irq(te_gpio_irq, exynos_dsi_te_irq_handler, NULL,
					IRQF_TRIGGER_RISING, "TE", dsi);
	if (ret) {
		DRM_ERROR("request interrupt failed with %d\n", ret);
		gpio_free(dsi->te_gpio);
		goto out;
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
out:
	return ret;
}

static void exynos_dsi_unregister_te_irq(struct exynos_dsi *dsi)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	if (gpio_is_valid(dsi->te_gpio)) {
		free_irq(gpio_to_irq(dsi->te_gpio), dsi);
		gpio_free(dsi->te_gpio);
		dsi->te_gpio = -ENOENT;
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static int exynos_dsi_host_attach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct exynos_dsi *dsi = host_to_dsi(host);
	int ret = 0;
	DRM_DEBUG_KMS("%s +\n", __func__);

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;
	dsi->panel_node = device->dev.of_node;

	/*
	 * This is a temporary solution and should be made by more generic way.
	 *
	 * If attached panel device is for command mode one, dsi should register
	 * TE interrupt handler.
	 */
	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		ret = exynos_dsi_register_te_irq(dsi);

		if (ret)
			return ret;
	}

	if (dsi->connector.dev) {
		drm_helper_hpd_irq_event(dsi->connector.dev);
	}

	DRM_DEBUG_KMS("%s -\n", __func__);

	return 0;
}

static int exynos_dsi_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct exynos_dsi *dsi = host_to_dsi(host);

	DRM_DEBUG_KMS("%s +\n", __func__);

	exynos_dsi_unregister_te_irq(dsi);

	dsi->panel_node = NULL;

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

/* distinguish between short and long DSI packet types */
static bool exynos_dsi_is_short_dsi_type(u8 type)
{
	DRM_DEBUG_KMS("%s \n", __func__);

	return (type & 0x0f) <= 8;
}

static void exynos_dsi_read_transfer_pre(struct exynos_dsi *dsi)
{
	dsim_reg_set_fifo_ctrl(dsi->id, DSIM_FIFOCTRL_INIT_RX);
	dsim_reg_clear_int(dsi->id, DSIM_INTSRC_RX_DATA_DONE);
	reinit_completion(&dsi->r_completed);
	DRM_DEBUG_KMS("%s xfer %p: cmd %d, txlen %u, rxlen %u\n",
		__func__, &dsi->xfer,
		dsi->xfer.data_id, dsi->xfer.tx_len, dsi->xfer.rx_len);
}

static int exynos_dsi_read_transfer_post(struct exynos_dsi *dsi)
{
	int ret;

	ret = wait_for_completion_timeout(&dsi->r_completed,
			msecs_to_jiffies(DSI_XFER_TIMEOUT_MS));
	if (!ret) {
		DRM_ERROR("xfer %p read timed out, cmd %d, txlen %u, rxlen %u\n",
				&dsi->xfer, dsi->xfer.data_id,
				dsi->xfer.tx_len, dsi->xfer.rx_len);
		return -ETIMEDOUT;
	}

	ret = exynos_dsi_get_rx_payload(dsi, &dsi->xfer);
	return ret;
}


static ssize_t exynos_dsi_host_transfer(struct mipi_dsi_host *host,
				        const struct mipi_dsi_msg *msg)
{
	struct exynos_dsi *dsi = host_to_dsi(host);
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (dsi->state == DSIM_STATE_SUSPEND) {
		DRM_INFO("dsim is disabled.. cannot transfer dsi command\n");
		return -EINVAL;
	}

	if (msg->tx_len == 0)
		return -EINVAL;

	dsi->xfer.data_id = msg->type;

	if (exynos_dsi_is_short_dsi_type(msg->type)) {
		const char *tx_buf = msg->tx_buf;

		if (msg->tx_len > 2)
			return -EINVAL;
		dsi->xfer.tx_len = 0;
		dsi->xfer.data[0] = tx_buf[0];
		dsi->xfer.data[1] = (msg->tx_len == 2) ? tx_buf[1] : 0;
	} else {
		dsi->xfer.tx_len = msg->tx_len;
		dsi->xfer.data[0] = msg->tx_len & 0xff;
		dsi->xfer.data[1] = msg->tx_len >> 8;
		dsi->xfer.tx_payload = msg->tx_buf;
	}

	dsi->xfer.rx_len = msg->rx_len;
	dsi->xfer.rx_payload = msg->rx_buf;
	dsi->xfer.flags = msg->flags;

	if (msg->type == MIPI_DSI_DCS_READ)
		exynos_dsi_read_transfer_pre(dsi);

	ret = exynos_dsi_transfer(dsi);

	if (msg->type == MIPI_DSI_DCS_READ)
		ret = exynos_dsi_read_transfer_post(dsi);

	DRM_DEBUG_KMS("%s +\n", __func__);
	if (ret < 0)
		return ret;
	else
		return msg->type == MIPI_DSI_DCS_READ ?
			dsi->xfer.rx_len : dsi->xfer.tx_len;
}

static const struct mipi_dsi_host_ops exynos_dsi_ops = {
	.attach = exynos_dsi_host_attach,
	.detach = exynos_dsi_host_detach,
	.transfer = exynos_dsi_host_transfer,
};

static void exynos_dsi_enable(struct drm_encoder *encoder)
{
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	int ret;

	DRM_INFO("%s +\n", __func__);

	if (dsi->state == DSIM_STATE_HSCLKEN) {
		dev_info(dsi->dev, "dsi is already enabled\n");
		return;
	}

	enable_irq(dsi->irq);

	ret = drm_panel_prepare(dsi->panel);
	if (ret < 0) {
		pm_runtime_put_sync(dsi->dev);
		return;
	}

	phy_power_on(dsi->phy);
	dsim_reg_set_clocks(dsi->id, &dsi->clks_param.clks, &dsi->lcd_info.dphy_pms, 1);
	dsim_reg_set_lanes(dsi->id, DSIM_LANE_CLOCK | dsi->data_lane, 1);

	if (dsi->lanes == DSI_8_DATA_LANES) {
		u32 id = !dsi->id;
		phy_power_on(dsi_drvdata[id]->phy);
		dsim_reg_set_clocks(id, &dsi->clks_param.clks,
				&dsi->lcd_info.dphy_pms, 1);
		dsim_reg_set_lanes(id, DSIM_LANE_CLOCK | dsi->data_lane, 1);
	}

	drm_panel_ready(dsi->panel);

	exynos_dsi_set_display_mode(dsi);
	exynos_dsi_set_display_enable(dsi, true);

	dsi->state = DSIM_STATE_HSCLKEN;
	ret = drm_panel_enable(dsi->panel);
	if (ret < 0) {
		exynos_dsi_set_display_enable(dsi, false);
		drm_panel_unprepare(dsi->panel);
		pm_runtime_put_sync(dsi->dev);
		return;
	}
	//exynos_dsi_enable_irq(dsi);

	DRM_INFO("%s -\n", __func__);
}

static void exynos_dsi_disable(struct drm_encoder *encoder)
{
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (dsi->state == DSIM_STATE_SUSPEND) {
		dev_info(dsi->dev, "dsi is already disabled\n");
		return;
	}

	drm_panel_disable(dsi->panel);
	exynos_dsi_set_display_enable(dsi, false);
	drm_panel_unprepare(dsi->panel);

	phy_power_off(dsi->phy);
	if (dsi->lanes == DSI_8_DATA_LANES) {
		u32 id = !dsi->id;
		phy_power_off(dsi_drvdata[id]->phy);
	}

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_put_sync(dsi->dev);
#else
	exynos_dsi_suspend(dsi->dev);
#endif
	dsi->state = DSIM_STATE_SUSPEND;

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static enum drm_connector_status
exynos_dsi_detect(struct drm_connector *connector, bool force)
{
	struct exynos_dsi *dsi = connector_to_dsi(connector);

	DRM_INFO("%s +\n", __func__);

	if (!dsi->panel) {
		dsi->panel = of_drm_find_panel(dsi->panel_node);
		if (dsi->panel)
			drm_panel_attach(dsi->panel, &dsi->connector);
	} else if (!dsi->panel_node) {
		struct drm_encoder *encoder;

		encoder = platform_get_drvdata(to_platform_device(dsi->dev));
		exynos_dsi_disable(encoder);
		drm_panel_detach(dsi->panel);
		dsi->panel = NULL;
	}

	if (dsi->panel && !dsi->force_disconnected)
		return connector_status_connected;

	DRM_INFO("%s -\n", __func__);

	return connector_status_disconnected;
}

static void exynos_dsi_connector_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	connector->dev = NULL;

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static const struct drm_connector_funcs exynos_dsi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = exynos_dsi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = exynos_dsi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int exynos_dsi_get_modes(struct drm_connector *connector)
{
	struct exynos_dsi *dsi = connector_to_dsi(connector);

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (dsi->panel)
		return dsi->panel->funcs->get_modes(dsi->panel);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static struct drm_encoder *
exynos_dsi_best_encoder(struct drm_connector *connector)
{
	struct exynos_dsi *dsi = connector_to_dsi(connector);

	DRM_DEBUG_KMS("%s \n", __func__);

	return &dsi->encoder;
}

static const struct drm_connector_helper_funcs exynos_dsi_connector_helper_funcs = {
	.get_modes = exynos_dsi_get_modes,
	.best_encoder = exynos_dsi_best_encoder,
};

static int exynos_dsi_create_connector(struct drm_encoder *encoder)
{
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_connector *connector = &dsi->connector;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector,
				 &exynos_dsi_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_dsi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static bool exynos_dsi_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("%s \n", __func__);
	return true;
}

static void exynos_dsi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	struct videomode *vm = &dsi->vm;
	struct drm_display_mode *m = adjusted_mode;

	DRM_DEBUG_KMS("%s +\n", __func__);

	vm->hactive = m->hdisplay;
	vm->vactive = m->vdisplay;
	vm->vfront_porch = m->vsync_start - m->vdisplay;
	vm->vback_porch = m->vtotal - m->vsync_end;
	vm->vsync_len = m->vsync_end - m->vsync_start;
	vm->hfront_porch = m->hsync_start - m->hdisplay;
	vm->hback_porch = m->htotal - m->hsync_end;
	vm->hsync_len = m->hsync_end - m->hsync_start;

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static const struct drm_encoder_helper_funcs exynos_dsi_encoder_helper_funcs = {
	.mode_fixup = exynos_dsi_mode_fixup,
	.mode_set = exynos_dsi_mode_set,
	.enable = exynos_dsi_enable,
	.disable = exynos_dsi_disable,
};

static const struct drm_encoder_funcs exynos_dsi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};
MODULE_DEVICE_TABLE(of, exynos_dsi_of_match);

/* of_* functions will be removed after merge of of_graph patches */
static struct device_node *
of_get_child_by_name_reg(struct device_node *parent, const char *name, u32 reg)
{
	struct device_node *np;

	DRM_DEBUG_KMS("%s +\n", __func__);

	for_each_child_of_node(parent, np) {
		u32 r;

		if (!np->name || of_node_cmp(np->name, name))
			continue;

		if (of_property_read_u32(np, "reg", &r) < 0)
			r = 0;

		if (reg == r)
			break;
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
	return np;
}

static struct device_node *of_graph_get_port_by_reg(struct device_node *parent,
						    u32 reg)
{
	struct device_node *ports, *port;

	DRM_DEBUG_KMS("%s +\n", __func__);

	ports = of_get_child_by_name(parent, "ports");
	if (ports)
		parent = ports;

	port = of_get_child_by_name_reg(parent, "port", reg);

	of_node_put(ports);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return port;
}

static struct device_node *
of_graph_get_endpoint_by_reg(struct device_node *port, u32 reg)
{
	DRM_DEBUG_KMS("%s \n", __func__);
	return of_get_child_by_name_reg(port, "endpoint", reg);
}

static int exynos_dsi_of_read_u32(const struct device_node *np,
				  const char *propname, u32 *out_value)
{
	int ret = of_property_read_u32(np, propname, out_value);

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (ret < 0)
		pr_err("%s: failed to get '%s' property\n", np->full_name,
		       propname);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return ret;
}

enum {
	DSI_PORT_IN,
	DSI_PORT_OUT
};

static void dsi_parse_lcd_info(struct exynos_dsi *dsi, struct device *dev)
{
	u32 res[3];
	struct device_node *node;

	DRM_DEBUG_KMS("%s +\n", __func__);

	node = of_parse_phandle(dsi->dev->of_node, "lcd_info", 0);

	if (!node)
		return;

	of_property_read_u32(node, "mode", &dsi->lcd_info.mode);
	dev_dbg(dev, "%s mode\n", dsi->lcd_info.mode ? "command" : "video");

	of_property_read_u32_array(node, "resolution", res, 2);
	dsi->lcd_info.xres = res[0];
	dsi->lcd_info.yres = res[1];
	dev_info(dev, "LCD(%s) resolution: xres(%d), yres(%d)\n",
			of_node_full_name(node), res[0], res[1]);

	of_property_read_u32_array(node, "size", res, 2);
	dsi->lcd_info.width = res[0];
	dsi->lcd_info.height = res[1];
	dev_dbg(dev, "LCD size: width(%d), height(%d)\n", res[0], res[1]);

	of_property_read_u32(node, "timing,refresh", &dsi->lcd_info.fps);
	dev_dbg(dev, "LCD refresh rate(%d)\n", dsi->lcd_info.fps);

	of_property_read_u32_array(node, "timing,h-porch", res, 3);
	dsi->lcd_info.hbp = res[0];
	dsi->lcd_info.hfp = res[1];
	dsi->lcd_info.hsa = res[2];
	dev_dbg(dev, "hbp(%d), hfp(%d), hsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32_array(node, "timing,v-porch", res, 3);
	dsi->lcd_info.vbp = res[0];
	dsi->lcd_info.vfp = res[1];
	dsi->lcd_info.vsa = res[2];
	dev_dbg(dev, "vbp(%d), vfp(%d), vsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-hs-clk", &dsi->lcd_info.hs_clk);
	dsi->clks_param.clks.hs_clk = dsi->lcd_info.hs_clk;
	dev_dbg(dev, "requested hs clock(%d)\n", dsi->lcd_info.hs_clk);

	of_property_read_u32_array(node, "timing,pms", res, 3);
	dsi->lcd_info.dphy_pms.p = res[0];
	dsi->lcd_info.dphy_pms.m = res[1];
	dsi->lcd_info.dphy_pms.s = res[2];
	dev_dbg(dev, "p(%d), m(%d), s(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-escape-clk",
			&dsi->lcd_info.esc_clk);
	dsi->clks_param.clks.esc_clk = dsi->lcd_info.esc_clk;
	dev_dbg(dev, "requested escape clock(%d)\n", dsi->lcd_info.esc_clk);

	of_property_read_u32(node, "mic_en", &dsi->lcd_info.mic_enabled);
	dev_dbg(dev, "mic enabled (%d)\n", dsi->lcd_info.mic_enabled);

	of_property_read_u32(node, "mic_ratio", &dsi->lcd_info.mic_ratio);
	switch (dsi->lcd_info.mic_ratio) {
		case MIC_COMP_RATIO_1_2:
			dev_dbg(dev, "mic ratio (%s)\n", "MIC_COMP_RATIO_1_2");
			break;
		case MIC_COMP_RATIO_1_3:
			dev_dbg(dev, "mic ratio (%s)\n", "MIC_COMP_RATIO_1_3");
			break;
		default: /*MIC_COMP_BYPASS*/
			dev_dbg(dev, "mic ratio (%s)\n", "MIC_COMP_BYPASS");
			break;
	}

	of_property_read_u32(node, "mic_ver", &dsi->lcd_info.mic_ver);
	dev_dbg(dev, "mic version(%d)\n", dsi->lcd_info.mic_ver);

	of_property_read_u32(node, "type_of_ddi", &dsi->lcd_info.ddi_type);
	dev_dbg(dev, "ddi type(%d)\n", dsi->lcd_info.ddi_type);

	of_property_read_u32(node, "dsc_en", &dsi->lcd_info.dsc_enabled);
	dev_dbg(dev, "dsc is %s\n", dsi->lcd_info.dsc_enabled ? "enabled" : "disabled");

	if (dsi->lcd_info.dsc_enabled) {
		of_property_read_u32(node, "dsc_cnt", &dsi->lcd_info.dsc_cnt);
		dev_dbg(dev, "dsc count(%d)\n", dsi->lcd_info.dsc_cnt);
		of_property_read_u32(node, "dsc_slice_num",
				&dsi->lcd_info.dsc_slice_num);
		dev_dbg(dev, "dsc slice count(%d)\n", dsi->lcd_info.dsc_slice_num);
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static int exynos_dsi_parse_dt(struct exynos_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *node = dev->of_node;
	struct device_node *port, *ep;
	int ret = 0;

	DRM_DEBUG_KMS("%s +\n", __func__);

	dsi->force_disconnected = IS_ENABLED(CONFIG_DRM_EXYNOS_MAIN_LCD_BYPASS);

	port = of_graph_get_port_by_reg(node, DSI_PORT_OUT);
	if (!port) {
		dev_info(dev, "no output port specified\n");
		ret = 0;
		goto panel_done;
	}

	ret = exynos_dsi_of_read_u32(node, "samsung,pll-clock-frequency",
				     &dsi->pll_clk_rate);
	if (ret < 0)
		return ret;

	ep = of_graph_get_endpoint_by_reg(port, 0);
	of_node_put(port);
	if (!ep) {
		dev_err(dev, "no endpoint specified in output port\n");
		return -EINVAL;
	}

	ret = exynos_dsi_of_read_u32(ep, "samsung,burst-clock-frequency",
				     &dsi->burst_clk_rate);
	if (ret < 0)
		goto end;

	ret = exynos_dsi_of_read_u32(ep, "samsung,esc-clock-frequency",
				     &dsi->esc_clk_rate);
	if (ret < 0)
		goto end;

	ep = of_graph_get_next_endpoint(node, NULL);
	if (!ep) {
		ret = -EINVAL;
		goto end;
	}
	of_node_put(ep);

	dsi->bridge_node = of_graph_get_remote_port_parent(ep);
	if (!dsi->bridge_node) {
		ret = -EINVAL;
		goto end;
	}

	/* 8890 */
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dev_err(dev, "no device tree information\n");
		return -EINVAL;
	}

panel_done:
	dsi->dev = dev;

	of_property_read_u32(dev->of_node, "data_lane_cnt", &dsi->data_lane_cnt);
	dev_info(dev, "using data lane count(%d)\n", dsi->data_lane_cnt);

	dsi_parse_lcd_info(dsi, dev);

	DRM_DEBUG_KMS("%s -\n", __func__);
end:
	of_node_put(ep);

	return ret;
}

static int exynos_dsi_bind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	struct device_node *node = dev->of_node;
	struct drm_device *drm_dev = data;
	struct drm_bridge *bridge;
	int ret;

	DRM_INFO("%s +\n", __func__);

	if (!of_graph_get_port_by_reg(node, DSI_PORT_OUT)) {
		DRM_INFO("%s there is no port info.\n", dev_name(dev));
		return 0;
	}

	ret = exynos_drm_crtc_get_pipe_from_type(drm_dev, DECON_OUT_DSI);
	if (ret < 0)
		return ret;

	encoder->possible_crtcs = 1 << ret;

	DRM_INFO("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	drm_encoder_init(drm_dev, encoder, &exynos_dsi_encoder_funcs,
			 DRM_MODE_ENCODER_DSI, NULL);

	drm_encoder_helper_add(encoder, &exynos_dsi_encoder_helper_funcs);

	ret = exynos_dsi_create_connector(encoder);
	if (ret) {
		DRM_ERROR("failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	bridge = of_drm_find_bridge(dsi->bridge_node);
	if (bridge) {
		encoder->bridge = bridge;
		drm_bridge_attach(drm_dev, bridge);
	}

	DRM_INFO("%s -\n", __func__);
	return mipi_dsi_host_register(&dsi->dsi_host);
}

static void exynos_dsi_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);

	DRM_DEBUG_KMS("%s +\n", __func__);

	exynos_dsi_disable(encoder);

	mipi_dsi_host_unregister(&dsi->dsi_host);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static const struct component_ops exynos_dsi_component_ops = {
	.bind	= exynos_dsi_bind,
	.unbind	= exynos_dsi_unbind,
};

static int __maybe_unused exynos_dsi_suspend(struct device *dev)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	struct exynos_dsi_driver_data *driver_data = dsi->driver_data;
	int i;

	DRM_DEBUG_KMS("%s +\n", __func__);

	usleep_range(10000, 20000);

	if (dsi->state & DSIM_STATE_INITIALIZED) {
		dsi->state &= ~DSIM_STATE_INITIALIZED;

		exynos_dsi_disable_clock(dsi);

		exynos_dsi_disable_irq(dsi);
	}

	dsi->state &= ~DSIM_STATE_CMD_LPM;

	phy_power_off(dsi->phy);

	for (i = driver_data->num_clks - 1; i > -1; i--)
		clk_disable_unprepare(dsi->clks[i]);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int __maybe_unused exynos_dsi_resume(struct device *dev)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct exynos_dsi *dsi = encoder_to_dsi(encoder);
	struct exynos_dsi_driver_data *driver_data = dsi->driver_data;
	int ret, i;

	DRM_DEBUG_KMS("%s +\n", __func__);

	for (i = 0; i < driver_data->num_clks; i++) {
		ret = clk_prepare_enable(dsi->clks[i]);
		if (ret < 0) {
			DRM_ERROR("cannot prepare_enable clk %d\n", ret);
			goto err_clk;
		}
	}

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		DRM_ERROR("cannot enable phy %d\n", ret);
		goto err_clk;
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;

err_clk:
	while (--i > -1)
		clk_disable_unprepare(dsi->clks[i]);

	return ret;
}

static const struct dev_pm_ops exynos_dsi_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_dsi_suspend, exynos_dsi_resume, NULL)
};

static int exynos_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct exynos_dsi *dsi;
	int ret, i;

	DRM_INFO("%s +\n", __func__);

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	/* To be checked as invalid one */
	dsi->te_gpio = -ENOENT;

	init_completion(&dsi->r_completed);
	init_completion(&dsi->xfer.completed);
	spin_lock_init(&dsi->transfer_lock);
	spin_lock_init(&dsi->slock);
	INIT_LIST_HEAD(&dsi->transfer_list);

	dsi->dsi_host.ops = &exynos_dsi_ops;
	dsi->dsi_host.dev = dev;

	dsi->dev = dev;
	dsi->driver_data = exynos_dsi_get_driver_data(pdev);

	dsi->id = of_alias_get_id(dev->of_node, "dsi");
	DRM_INFO("dsi(%d) probe start..\n", dsi->id);

	ret = exynos_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	dsi->clks = devm_kzalloc(dev,
			sizeof(*dsi->clks) * dsi->driver_data->num_clks,
			GFP_KERNEL);
	if (!dsi->clks)
		return -ENOMEM;

	for (i = 0; i < dsi->driver_data->num_clks; i++) {
		dsi->clks[i] = devm_clk_get(dev, clk_names[i]);
		if (IS_ERR(dsi->clks[i])) {
			if (strcmp(clk_names[i], "sclk_mipi") == 0) {
				strcpy(clk_names[i], OLD_SCLK_MIPI_CLK_NAME);
				i--;
				continue;
			}

			dev_info(dev, "failed to get the clock: %s\n",
					clk_names[i]);
			return PTR_ERR(dsi->clks[i]);
		}
	}

	dsi_drvdata[dsi->id] = dsi;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->reg_base)) {
		dev_err(dev, "failed to remap io region\n");
		return PTR_ERR(dsi->reg_base);
	}

	dsi->phy = devm_phy_get(dev, "dsim_dphy");
	if (IS_ERR(dsi->phy)) {
		dev_info(dev, "failed to get dsim phy\n");
		return PTR_ERR(dsi->phy);
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		dev_err(dev, "failed to request dsi irq resource\n");
		return dsi->irq;
	}

	irq_set_status_flags(dsi->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, dsi->irq, NULL,
					exynos_dsi_irq, IRQF_ONESHOT,
					dev_name(dev), dsi);
	if (ret) {
		dev_err(dev, "failed to request dsi irq\n");
		return ret;
	}

	switch (dsi->data_lane_cnt) {
		case 1:
			dsi->data_lane = DSIM_LANE_DATA0;
			break;
		case 2:
			dsi->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1;
			break;
		case 3:
			dsi->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1 |
				DSIM_LANE_DATA2;
			break;
		case 4:
			dsi->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1 |
				DSIM_LANE_DATA2 | DSIM_LANE_DATA3;
			break;
		default:
			dev_info(&pdev->dev, "data lane is invalid.\n");
			return -EINVAL;
	};

	platform_set_drvdata(pdev, &dsi->encoder);

	phy_init(dsi->phy);

	pm_runtime_enable(dev);

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_get_sync(dsi->dev);
#else
	exynos_dsi_resume(dsi->dev);
#endif

	dsi->state = DSIM_STATE_SUSPEND;

#if 1
	{
		DRM_INFO("exynos_dsi dt parse data dump\n");
		DRM_INFO("id : %d \n", dsi->id);
		DRM_INFO("%s: %ld Mhz\n", __clk_get_name(dsi->res.pclk),
				clk_get_rate(dsi->res.pclk) / MHZ);
		DRM_INFO("%s: %ld Mhz\n", __clk_get_name(dsi->res.dphy_esc),
				clk_get_rate(dsi->res.dphy_esc) / MHZ);
		DRM_INFO("%s: %ld Mhz\n", __clk_get_name(dsi->res.dphy_byte),
				clk_get_rate(dsi->res.dphy_byte) / MHZ);
		DRM_INFO("data lane count(%d)\n", dsi->data_lane_cnt);
		DRM_INFO("%s mode\n", dsi->lcd_info.mode ? "command" : "video");
		DRM_INFO("resolution: xres(%d), yres(%d)\n", dsi->lcd_info.xres, dsi->lcd_info.yres);
		DRM_INFO("LCD size: width(%d), height(%d)\n", dsi->lcd_info.width, dsi->lcd_info.height);
		DRM_INFO("LCD refresh rate(%d)\n", dsi->lcd_info.fps);
		DRM_INFO("hbp(%d), hfp(%d), hsa(%d)\n", dsi->lcd_info.hbp, dsi->lcd_info.hfp, dsi->lcd_info.hsa);
		DRM_INFO("vbp(%d), vfp(%d), vsa(%d)\n", dsi->lcd_info.vbp, dsi->lcd_info.vfp, dsi->lcd_info.vsa);
		DRM_INFO("requested hs clock(%d)\n", dsi->lcd_info.hs_clk);
		DRM_INFO("p(%d), m(%d), s(%d)\n", dsi->lcd_info.dphy_pms.p, dsi->lcd_info.dphy_pms.m, dsi->lcd_info.dphy_pms.s);
		DRM_INFO("requested escape clock(%d)\n", dsi->lcd_info.esc_clk);
		DRM_INFO("mic enabled (%d)\n", dsi->lcd_info.mic_enabled);

		switch (dsi->lcd_info.mic_ratio) {
			case MIC_COMP_RATIO_1_2:
				DRM_INFO("mic ratio (%s)\n", "MIC_COMP_RATIO_1_2");
				break;
			case MIC_COMP_RATIO_1_3:
				DRM_INFO("mic ratio (%s)\n", "MIC_COMP_RATIO_1_3");
				break;
			default: /*MIC_COMP_BYPASS*/
				DRM_INFO("mic ratio (%s)\n", "MIC_COMP_BYPASS");
				break;
		}

		DRM_INFO("mic version(%d)\n", dsi->lcd_info.mic_ver);
		DRM_INFO("ddi type(%d)\n", dsi->lcd_info.ddi_type);
		DRM_INFO("dsc is %s\n", dsi->lcd_info.dsc_enabled ? "enabled" : "disabled");

		if (dsi->lcd_info.dsc_enabled) {
			DRM_INFO("dsc count(%d)\n", dsi->lcd_info.dsc_cnt);
			DRM_INFO("dsc slice count(%d)\n", dsi->lcd_info.dsc_slice_num);
		}

		DRM_INFO("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);
		DRM_INFO("DRM dt parse end \n");

	}
#endif

	DRM_INFO("%s -\n", __func__);
	return component_add(dev, &exynos_dsi_component_ops);
}

static int exynos_dsi_remove(struct platform_device *pdev)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &exynos_dsi_component_ops);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}


struct platform_driver dsi_driver = {
	.probe = exynos_dsi_probe,
	.remove = exynos_dsi_remove,
	.driver = {
		   .name = "exynos-dsi",
		   .owner = THIS_MODULE,
		   .pm = &exynos_dsi_pm_ops,
		   .of_match_table = exynos_dsi_of_match,
	},
};

MODULE_AUTHOR("Tomasz Figa <t.figa@samsung.com>");
MODULE_AUTHOR("Andrzej Hajda <a.hajda@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC MIPI DSI Master");
MODULE_LICENSE("GPL v2");
