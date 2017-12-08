/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "panel-drv.h"

struct i2c_client *ptr_i2c;
static struct of_device_id i2c_panel_of_match[] = {
	{ .compatible = "samsung,i2c-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, i2c_panel_of_match);

static struct i2c_device_id panel_idtable[] = {
	{"i2c-panel", 0},
};
MODULE_DEVICE_TABLE(i2c, panel_idtable);

static int i2c_panel_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	ptr_i2c = client;
	dev_info(&client->adapter->dev,
		"attached %s into i2c adapter successfully\n", client->name);

	return 0;
}

static int i2c_panel_remove(struct i2c_client *client)
{
	ptr_i2c = NULL;
	dev_info(&client->adapter->dev,
		"attached %s into i2c adapter removed\n", client->name);

	return 0;
}

static struct i2c_driver i2c_panel_driver = {
	.id_table = panel_idtable,
	.probe = i2c_panel_probe,
	.remove = i2c_panel_remove,
	.driver = {
		.name = "i2c-panel",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(i2c_panel_of_match),
	},
};

static void __amsa20jw01_i2c_write(struct exynos_panel *ctx, const void *buf, size_t bytes)
{
	struct device *dev = ctx->dev;
	struct i2c_client *i2c = ptr_i2c;
	u8 msg[bytes];
	int ret, i = 0;

	memcpy(&msg[0], buf, bytes);

	if (!i2c) {
		dev_err(dev, "%s: i2c_client is NULL\n", __func__);
		return;
	}

	ret = i2c_master_send(i2c, msg, bytes);
	if (ret != bytes) {
		dev_err(dev, "%s: can't write data\n", __func__);
		goto write_err;
	}

	dev_dbg(dev, "##### write addr 0x%x%x\n ", msg[0], msg[1]);
	i = 2;
	do {
		dev_dbg(dev, "write data: idx(%d), data(0x%x)\n", i - 2, msg[i]);
	} while(++i < bytes);
	return;

write_err:
	dev_err(dev, "%s: can't write data, timeout\n", __func__);
}

int amsa20jw01_i2c_read(struct exynos_panel *ctx, u8 offset1, u8 offset2, int bytes, u8 *buf)
{
	struct device *dev = ctx->dev;
	struct i2c_client *i2c = ptr_i2c;
	int ret, i = 0;
	u8 addr[2] = {offset1, offset2};

	struct i2c_msg msg[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 2,
			.buf = addr
		},
		[1] = {
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = bytes,
			.buf = buf
		}
	};

	ret = i2c_transfer(i2c->adapter, msg, 2);

	if (ret != 2) {
		dev_dbg(dev, "%s: can't read data, error %d\n",
				__func__, ret);
		goto read_err;
	}

	dev_info(dev, "##### read addr 0x%x%x\n ", addr[0], addr[1]);
	do {
		dev_info(dev, "read data: idx(%d), data(0x%x)\n", i, buf[i]);
	} while(++i < bytes);
	dev_info(dev, "%s: read data ok\n", __func__);

	return 0;

read_err:
	dev_err(dev, "%s: can't read data, timeout\n", __func__);
	return -ETIME;
}


#define amsa20jw01_i2c_write(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	__amsa20jw01_i2c_write(ctx, d, ARRAY_SIZE(d));\
})

void amsa20jw01_info_dump(struct exynos_panel *ctx)
{
	u8 rd[10];
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x00, 0x53, 1, rd);
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x07, 0x7B, 4, rd);
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x07, 0x7F, 2, rd);
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x01, 0xA2, 1, rd);
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x01, 0xA3, 1, rd);
	memset(rd, 0, sizeof(rd));
	amsa20jw01_i2c_read(ctx, 0x01, 0xA4, 1, rd);
}

static void amsa20jw01_gamma_condition_set(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x08, 0xDF,
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x00,
	0x80, 0x80,
	0x00, /* RED */
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x00,
	0x80, 0x80,
	0x00, /* GREEN */
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x80,
	0x80, 0x00,
	0x80, 0x80,
	0x00); /* BLUE */
}

static void amsa20jw01_width_set(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x00, 0x53,
	0x38);
}

static void amsa20jw01_aid_set(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0xD9,
	0x00, 0x00, 0x0E);
}

static void amsa20jw01_elvss_set(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x67,
	0x0F);
}

static void amsa20jw01_gamma_update(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x35,
	0x01);
}

static void amsa20jw01_acl_on(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x36,
	0x12);
}

static void amsa20jw01_acl_off(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x36,
	0x10);
}

static void amsa20jw01_opr_avr(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x4D,
	0x04);
}

static void amsa20jw01_caps_on(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x6D,
	0x85);
}

int amsa20jw01_update_gamma(struct exynos_panel *ctx,
					unsigned int brightness)
{
	DRM_DEBUG_KMS("%s +\n", __func__);
	amsa20jw01_i2c_write(ctx, 0x08, 0xDF,
	0x00, 0x00, 0x0E);
	amsa20jw01_i2c_write(ctx, 0x01, 0xD9,
	0x00, 0x00, 0x0E);
	/* TODO: ELVSS_DIM_OFFSET */
	amsa20jw01_i2c_write(ctx, 0x01, 0x4D,
	0x05);
	amsa20jw01_acl_on(ctx);
	amsa20jw01_caps_on(ctx);
	amsa20jw01_gamma_update(ctx);
	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int amsa20jw01_init(struct exynos_panel *ctx)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	/* 2176(default width) -> 2160 */
	amsa20jw01_width_set(ctx);

	/* 4.2.1 Max 360nit */
	amsa20jw01_gamma_condition_set(ctx);
	amsa20jw01_aid_set(ctx);
	amsa20jw01_elvss_set(ctx);
	amsa20jw01_opr_avr(ctx);
	amsa20jw01_acl_off(ctx);
	amsa20jw01_caps_on(ctx);
	amsa20jw01_gamma_update(ctx);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int amsa20jw01_disp_on(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x98,
	0x04);
	return 0;
}

static int amsa20jw01_disp_off(struct exynos_panel *ctx)
{
	amsa20jw01_i2c_write(ctx, 0x01, 0x98,
	0x00);
	return 0;
}

static const struct exynos_panel_funcs amsa20jw01_panel_funcs = {
	.init = amsa20jw01_init,
	.disp_on = amsa20jw01_disp_on,
	.disp_off = amsa20jw01_disp_off,
};

static int amsa20jw01_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct exynos_panel *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->funcs = &amsa20jw01_panel_funcs;

	if (i2c_add_driver(&i2c_panel_driver)) {
		DRM_ERROR("failed to register i2c driver\n");
		ret = -ENOENT;
		goto error;
	}

	dsi->lanes = 8;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	ret = panel_helper_probe(ctx, dsi);
	if (ret < 0) {
		i2c_del_driver(&i2c_panel_driver);
		goto error;
	}

	return ret;
error:
	kfree(ctx);
	return ret;
}

static int amsa20jw01_remove(struct mipi_dsi_device *dsi)
{
	i2c_del_driver(&i2c_panel_driver);
	panel_helper_remove(dsi);

	return 0;
}

static struct of_device_id amsa20jw01_of_match[] = {
	{ .compatible = "samsung,panel-amsa20jw01" },
	{ }
};
MODULE_DEVICE_TABLE(of, amsa20jw01_of_match);

static struct mipi_dsi_driver amsa20jw01_driver = {
	.probe = amsa20jw01_probe,
	.remove = amsa20jw01_remove,
	.driver = {
		.name = "panel_amsa20jw01",
		.owner = THIS_MODULE,
		.of_match_table = amsa20jw01_of_match,
	},
};
module_mipi_dsi_driver(amsa20jw01_driver);

MODULE_DESCRIPTION("MIPI-DSI based AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
