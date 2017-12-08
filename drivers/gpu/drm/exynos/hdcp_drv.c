/* linux/drivers/gpu/drm/exynos-hp/hdmi/hdcp_drv.c
 *
 * Copyright (c) 2016 Samsung Electronics
 *		http://www.samsung.com/
 *
 * HDCP function for Samsung TV driver
 * v1.4 Supported
 *
 * This program is free software. you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/smc.h>

#include "exynos_drm_hdmi.h"
#include "regs-hdmi.h"
#include "regs-hdcp.h"

#define AN_SIZE			8
#define AKSV_SIZE		5
#define BKSV_SIZE		5
#define BCAPS_SIZE		1
#define MAX_KEY_SIZE		16

#define RETRY_CNT		100
#define DDC_DELAY		25

#define KEY_LOAD_RETRY_CNT	1000
#define ENCRYPT_CHECK_CNT	10

/* offset of HDCP port */
#define HDCP_BKSV		0x00
#define HDCP_RI			0x08
#define HDCP_AKSV		0x10
#define HDCP_AN			0x18
#define HDCP_SHA1		0x20
#define HDCP_BCAPS		0x40

#define REPEATER_DONE			(0)
#define REPEATER_ILLEGAL_DEVICE_ERROR	(-4)
#define REPEATER_TIMEOUT_ERROR		(-5)

#ifdef CONFIG_OF
static const struct of_device_id hdcp_device_table[] = {
	        { .compatible = "samsung,exynos8-hdcp" },
		{},
};
MODULE_DEVICE_TABLE(of, hdcp_device_table);
#endif

struct i2c_client *g_ptr_i2c;

int hdcp_i2c_read(struct hdmi_device *hdev, u8 offset, int bytes, u8 *buf)
{
	struct device *dev = hdev->dev;
	struct i2c_client *i2c = g_ptr_i2c;
	int ret, cnt = 0, num;

	struct i2c_msg msg[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &offset
		},
		[1] = {
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = bytes,
			.buf = buf
		}
	};

	num = sizeof(msg) / sizeof(struct i2c_msg);
	do {
		ret = i2c_transfer(i2c->adapter, msg, num);

		if (ret != num)
			dev_dbg(dev, "%s: can't read data, retry %d\n",
					__func__, cnt);
		else
			break;

		if (hdev->hdcp_info.auth_status == FIRST_AUTHENTICATION_DONE
				|| hdev->hdcp_info.auth_status
					== SECOND_AUTHENTICATION_DONE)
			goto read_err;

		msleep(DDC_DELAY);
	} while (++cnt < RETRY_CNT);

	if (cnt == RETRY_CNT)
		goto read_err;

	dev_dbg(dev, "%s: read data ok\n", __func__);
	return 0;

read_err:
	dev_err(dev, "%s: can't read data(0x%x),timeout\n", __func__, offset);
	return -ETIME;
}

int hdcp_i2c_write(struct hdmi_device *hdev, u8 offset, int bytes, u8 *buf)
{
	struct device *dev = hdev->dev;
	struct i2c_client *i2c = g_ptr_i2c;
	u8 msg[bytes + 1]; /* offset + buf */
	int ret, cnt = 0;

	msg[0] = offset;
	memcpy(&msg[1], buf, bytes);

	do {
		ret = i2c_master_send(i2c, msg, bytes + 1);

		if (ret != bytes + 1)
			dev_dbg(dev, "%s: can't write data, retry %d\n",
					__func__, cnt);
		else
			break;

		msleep(DDC_DELAY);
	} while (++cnt < RETRY_CNT);

	if (cnt == RETRY_CNT)
		goto write_err;

	dev_dbg(dev, "%s: write data ok\n", __func__);
	return 0;

write_err:
	dev_err(dev, "%s: can't write data(0x%x),timeout\n", __func__, offset);
	return -ETIME;
}

static int hdcp_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct device *dev = &client->adapter->dev;

	g_ptr_i2c = client;
	dev_info(dev, "attached %s into i2c adapter\n", client->name);
	return ret;
}

static int hdcp_remove(struct i2c_client *client)
{
	struct device *dev = &client->adapter->dev;

	g_ptr_i2c = NULL;
	dev_info(dev, "detached %s from i2c adapter\n", client->name);
	return 0;
}

static struct i2c_device_id hdcp_idtable[] = {
	{"exynos-hdcp", 0},
};
MODULE_DEVICE_TABLE(i2c, hdcp_idtable);

static struct i2c_driver hdcp_driver = {
	.driver = {
		.name = "exynos-hdcp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hdcp_device_table),
	},
	.id_table	= hdcp_idtable,
	.probe		= hdcp_probe,
	.remove         = hdcp_remove,
};

static int __init hdcp_init(void)
{
	return i2c_add_driver(&hdcp_driver);
}

static void __exit hdcp_exit(void)
{
	i2c_del_driver(&hdcp_driver);
}

module_init(hdcp_init);
module_exit(hdcp_exit);

/* internal functions of HDCP */
static void hdcp_reg_encryption(struct hdmi_device *hdev, bool on)
{
	if (on)
		hdmi_write_mask(hdev, HDCP14_ENC_EN, ~0, HDMI_HDCP_ENC_ENABLE);
	else
		hdmi_write_mask(hdev, HDCP14_ENC_EN, 0, HDMI_HDCP_ENC_ENABLE);
}

static void hdcp_reg_swreset(struct hdmi_device *hdev)
{
	u32 cnt = 1000;

	hdmi_write(hdev, HDCP14_SYS_EN, 0);
	/* wait until this value automatically becomes 1 */
	do {
		if (hdmi_readb(hdev, HDCP14_SYS_EN))
			break;
		msleep(10);
	} while (cnt--);

	if (cnt == 0)
		hdmi_err("HDCP could not reach sw reset.\n");
}

static void hdmi_reg_set_bcaps(struct hdmi_device *hdev, u8 buf)
{
	hdmi_writeb(hdev, HDCP14_BCAPS, buf);
}

static void hdmi_reg_set_bksv(struct hdmi_device *hdev, u8 *buf)
{
	hdmi_write_bytes(hdev, HDCP14_BKSV_0, buf, BKSV_SIZE);
}

static int hdcp_write_key(struct hdmi_device *hdev, int size,
		int reg, int offset)
{
	struct device *dev = hdev->dev;
	u8 buf[MAX_KEY_SIZE];
	int i;

	memset(buf, 0, sizeof(buf));
	for (i = 0; i < size; i++)
		buf[i] = hdmi_readb(hdev, reg + i * 4);

	if (!buf[0]) {
		dev_err(dev, "%s: %s is null\n", __func__,
				offset == HDCP_AN ? "An" : "Aksv");
		goto write_key_err;
	}

	if (hdcp_i2c_write(hdev, offset, size, buf) < 0)
		goto write_key_err;

	for (i = 0; i < size + 1; i++)
		hdmi_dbg(5, "[i2c] %s[%d] : 0x%02x\n",
				offset == HDCP_AN ? "An" : "Aksv", i, buf[i]);

	return 0;

write_key_err:
	dev_err(dev, "%s: write %s is failed\n", __func__,
			offset == HDCP_AN ? "An" : "Aksv");
	return -1;
}

static int hdcp_read_bcaps(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	u8 bcaps = 0;

	if (hdcp_i2c_read(hdev, HDCP_BCAPS, BCAPS_SIZE, &bcaps) < 0)
		goto bcaps_read_err;

	hdmi_reg_set_bcaps(hdev, bcaps);
	hdmi_dbg(5, "[i2c] bcaps : 0x%02x\n", bcaps);

	if (bcaps & HDMI_HDCP_BCAPS_REPEATER)
		hdev->hdcp_info.is_repeater = 1;
	else
		hdev->hdcp_info.is_repeater = 0;

	hdmi_dbg(4, " is completed, dev(%s)\n",
			hdev->hdcp_info.is_repeater ? "REPEAT" : "SINK");
	return 0;

bcaps_read_err:
	dev_err(dev, "can't read bcaps : timeout\n");
	return -ETIME;
}

static int hdcp_read_bksv(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	u8 bksv[BKSV_SIZE];
	int i;

	memset(bksv, 0, sizeof(bksv));

	if (hdcp_i2c_read(hdev, HDCP_BKSV, BKSV_SIZE, bksv) < 0)
		goto bksv_read_err;

	for (i = 0; i < BKSV_SIZE; i++)
		hdmi_dbg(5, "[i2c] bksv[%d]: 0x%x\n", i, bksv[i]);

	hdmi_reg_set_bksv(hdev, bksv);

	hdmi_dbg(4, " is completed\n");
	return 0;

bksv_read_err:
	dev_err(dev, "%s: can't read bksv \n", __func__);
	return -EBUSY;
}

static int hdcp_read_ri(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	u8 ri[2] = {0, 0};
	u8 rj[2] = {0, 0};

	ri[0] = hdmi_readb(hdev, HDCP14_Ri_0);
	ri[1] = hdmi_readb(hdev, HDCP14_Ri_1);

	if (hdcp_i2c_read(hdev, HDCP_RI, 2, rj) < 0)
		goto compare_err;

	if ((ri[0] == rj[0]) && (ri[1] == rj[1]) && (ri[0] | ri[1])) {
		hdmi_writeb(hdev, HDCP14_CHECK_RESULT, HDCP14_RI_MATCH);
		hdmi_writeb(hdev, HDCP14_CHECK_RESULT, 0x0);
	} else {
		hdmi_writeb(hdev, HDCP14_CHECK_RESULT, HDCP14_RI_NOT_MATCH);
		goto compare_err;
	}

	hdmi_dbg(4, "ri and ri' are matched\n");
	return 0;

compare_err:
	hdev->hdcp_info.event = HDCP_EVENT_STOP;
	hdev->hdcp_info.auth_status = NOT_AUTHENTICATED;
	dev_err(dev, "%s: ri and ri' are mismatched\n", __func__);
	hdmi_info("Tx -> ri[0]: 0x%02x, ri[1]: 0x%02x\n", ri[0], ri[1]);
	hdmi_info("Rx -> rj[0]: 0x%02x, rj[1]: 0x%02x\n", rj[0], rj[1]);
	return -EBUSY;
}

static int hdcp_reset_auth(struct hdmi_device *hdev)
{
	mutex_lock(&hdev->mutex);

	hdev->hdcp_info.event		= HDCP_EVENT_STOP;
	hdev->hdcp_info.auth_status	= NOT_AUTHENTICATED;

	hdmi_write(hdev, HDCP14_CTRL1, 0x0);
	hdmi_write(hdev, HDCP14_CTRL2, 0x0);
	hdcp_reg_encryption(hdev, 0);
	hdmi_write(hdev, HDCP14_STATUS_EN, (~HDMI_INT_EN_ALL));
	hdmi_writeb(hdev, HDCP14_CHECK_RESULT, HDMI_HDCP_CLR_ALL_RESULTS);

	/* need some delay (at least 1 frame) */
	msleep(16);

	hdcp_reg_swreset(hdev);

	hdmi_dbg(4, "reset authentication\n");
	mutex_unlock(&hdev->mutex);

	return 0;
}

static int hdcp_start_encryption(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	u8 val;
	u32 cnt = 0;

	do {
		val = hdmi_readb(hdev, HDCP14_STATUS);

		if (val & HDMI_AUTHEN_ACK_AUTH) {
			hdcp_reg_encryption(hdev, 1);
			break;
		}

		mdelay(1);
	} while (++cnt < ENCRYPT_CHECK_CNT);

	if (cnt >= ENCRYPT_CHECK_CNT)
		goto encrypt_err;

	hdmi_dbg(4, "encryption is started\n");
	return 0;

encrypt_err:
	hdcp_reg_encryption(hdev, 0);
	dev_err(dev, "%s: encryption is failed\n", __func__);
	return -EBUSY;
}

static int hdmi_check_repeater(struct hdmi_device *hdev)
{
	/* TODO */
	return REPEATER_DONE;
}

static int hdcp_bksv(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;

	hdev->hdcp_info.auth_status = RECEIVER_READ_READY;

	if (hdcp_read_bcaps(hdev) < 0)
		goto bksv_start_err;

	hdev->hdcp_info.auth_status = BCAPS_READ_DONE;

	if (hdcp_read_bksv(hdev) < 0)
		goto bksv_start_err;

	hdev->hdcp_info.auth_status = BKSV_READ_DONE;

	return 0;

bksv_start_err:
	dev_err(dev, "%s: failed to start bksv\n", __func__);
	msleep(100);
	return -EBUSY;
}

static int hdcp_second_auth(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	int ret = 0;

	if (!hdev->hdcp_info.hdcp_start)
		goto second_auth_err;

	ret = hdmi_check_repeater(hdev);

	if (!ret) {
		hdev->hdcp_info.auth_status = SECOND_AUTHENTICATION_DONE;
		hdcp_start_encryption(hdev);
	} else {
		switch (ret) {

		case REPEATER_ILLEGAL_DEVICE_ERROR:
			hdmi_writeb(hdev, HDCP14_CTRL2, 0x1);
			hdmi_writeb(hdev, HDCP14_CTRL2, 0x0);

			hdmi_dbg(4, "repeater : illegal device\n");
			break;
		case REPEATER_TIMEOUT_ERROR:
			hdmi_write_mask(hdev, HDCP14_CTRL1, ~0,
					HDMI_HDCP_SET_REPEATER_TIMEOUT);
			hdmi_write_mask(hdev, HDCP14_CTRL1, 0,
					HDMI_HDCP_SET_REPEATER_TIMEOUT);

			hdmi_dbg(4, "repeater : timeout\n");
			break;
		default:
			break;
		}

		hdev->hdcp_info.auth_status = NOT_AUTHENTICATED;
		goto second_auth_err;
	}

	hdmi_dbg(4, "second authentication is done\n");
	return 0;

second_auth_err:
	dev_dbg(dev, "%s: second authentication is failed\n", __func__);
	return -1;
}

static int hdcp_write_an_aksv(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	dev_dbg(dev, "%s\n", __func__);

	if (hdev->hdcp_info.auth_status != BKSV_READ_DONE) {
		dev_err(dev, "%s: bksv is not ready\n", __func__);
		goto aksv_write_err;
	}
	if (hdcp_write_key(hdev, AN_SIZE, HDCP14_An_0, HDCP_AN) < 0)
		goto aksv_write_err;

	hdev->hdcp_info.auth_status = AN_WRITE_DONE;
	hdmi_dbg(4, "write An is done\n");

	if (hdcp_write_key(hdev, AKSV_SIZE, HDCP14_AKSV_0, HDCP_AKSV) < 0)
		goto aksv_write_err;

	msleep(100);
	hdev->hdcp_info.auth_status = AKSV_WRITE_DONE;
	hdmi_dbg(4, " is completed\n");
	return 0;

aksv_write_err:
	dev_err(dev, "%s: an/aksv start is failed\n", __func__);
	return -1;
}

static int hdcp_check_ri(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	if (hdev->hdcp_info.auth_status < AKSV_WRITE_DONE) {
		dev_dbg(dev, "%s: ri check is not ready\n", __func__);
		goto check_ri_err;
	}

	if (hdcp_read_ri(hdev) < 0)
		goto check_ri_err;

	if (hdev->hdcp_info.is_repeater)
		hdev->hdcp_info.auth_status = SECOND_AUTHENTICATION_RDY;
	else {
		hdev->hdcp_info.auth_status = FIRST_AUTHENTICATION_DONE;
		hdcp_start_encryption(hdev);
	}

	hdmi_dbg(4, " is completed\n");
	return 0;

check_ri_err:
	dev_err(dev, "%s: ri check is failed\n", __func__);
	return -1;
}

static void hdcp_work(struct work_struct *work)
{
	struct hdmi_device *hdev = container_of(work, struct hdmi_device, work);
	struct device *dev = hdev->dev;

	if (!hdev->hdcp_info.hdcp_start) {
		dev_dbg(dev, "%s: hdcp is not started\n", __func__);
		return;
	}

	if (hdev->hdcp_info.event & HDCP_EVENT_READ_BKSV_START) {
		if (hdcp_bksv(hdev) < 0)
			goto work_err;
		else
			hdev->hdcp_info.event &= ~HDCP_EVENT_READ_BKSV_START;
	}

	if (hdev->hdcp_info.event & HDCP_EVENT_SECOND_AUTH_START) {
		if (hdcp_second_auth(hdev) < 0)
			goto work_err;
		else
			hdev->hdcp_info.event &= ~HDCP_EVENT_SECOND_AUTH_START;
	}

	if (hdev->hdcp_info.event & HDCP_EVENT_WRITE_AKSV_START) {
		if (hdcp_write_an_aksv(hdev) < 0)
			goto work_err;
		else
			hdev->hdcp_info.event  &= ~HDCP_EVENT_WRITE_AKSV_START;
	}

	if (hdev->hdcp_info.event & HDCP_EVENT_CHECK_RI_START) {
		if (hdcp_check_ri(hdev) < 0)
			goto work_err;
		else
			hdev->hdcp_info.event &= ~HDCP_EVENT_CHECK_RI_START;
	}
	return;
work_err:
	if (!hdev->hdcp_info.hdcp_start)
		return;
	hdcp_reset_auth(hdev);
}

/* HDCP APIs for hdmi driver */
irqreturn_t hdcp_irq_handler(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	u32 event = 0;
	u8 flag;
	event = 0;

	flag = hdmi_readb(hdev, HDCP14_STATUS);
	if (!flag)
		return IRQ_HANDLED;

	hdmi_dbg(4, "HDCP interrupt flag = 0x%x\n", flag);

	if (flag & HDMI_I2C_INIT_INT_OCC) {
		event |= HDCP_EVENT_READ_BKSV_START;
	}

	if (flag & HDMI_WRITE_INT_OCC) {
		event |= HDCP_EVENT_WRITE_AKSV_START;
	}

	if (flag & HDMI_UPDATE_RI_INT_OCC) {
		event |= HDCP_EVENT_CHECK_RI_START;
	}

	if (flag & HDMI_WATCHDOG_INT_OCC) {
		event |= HDCP_EVENT_SECOND_AUTH_START;
	}

	/* Clear All interrupts */
	hdmi_write_mask(hdev, HDCP14_STATUS, ~0, flag);

	if (!event)
		dev_dbg(dev, "%s: unknown irq\n", __func__);

	if (hdev->streaming == HDMI_STREAMING) {
		hdev->hdcp_info.event |= event;
		queue_work(hdev->hdcp_wq, &hdev->work);
	}

	return IRQ_HANDLED;
}

int hdcp_prepare(struct hdmi_device *hdev)
{
	hdev->hdcp_wq = create_singlethread_workqueue("khdcpd");
	if (hdev->hdcp_wq == NULL)
		return -ENOMEM;

	INIT_WORK(&hdev->work, hdcp_work);

	if (hdev->dev->of_node)
		of_property_read_u32(hdev->dev->of_node, "hdcp-en",
				&hdev->hdcp_info.hdcp_enable);
	return 0;
}

int hdcp_loadkey(struct hdmi_device *hdev)
{
	struct device *dev = hdev->dev;
	int cnt = 0;
	u32 val;

	do {
		val = hdmi_read(hdev, HDCP_OTP_STATUS);
		if (val & HDCP14_ECC_DONE)
			break;
		mdelay(1);
	} while (++cnt < KEY_LOAD_RETRY_CNT);

	if (cnt >=  KEY_LOAD_RETRY_CNT) {
		dev_err(dev, "%s: load key is failed. OTP_STATUS=0x%x\n",
			__func__, hdmi_read(hdev, HDCP_OTP_STATUS));
		return -EBUSY;
	}

	hdmi_dbg(4, " is completed\n");
	return 0;
}

/* #define OTP_SMC_NOT_SUPPORT */
int hdcp_otp_control(struct hdmi_device *hdev, u32 en)
{
	int ret = 0;

	hdmi_dbg(5, "+\n");
	if (en) {
		hdmi_write(hdev, OTP_SETUP_WIDTH, 0x014B);
		hdmi_write(hdev, OTP_RSTB_ASSERT, 0x00D8);
		hdmi_write(hdev, OTP_RSTB_DEASSERT, 0x0148);
		hdmi_write(hdev, OTP_CEB_ASSERT, 0x0002);
		hdmi_write(hdev, OTP_CEB_DEASSERT, 0x014A);
		hdmi_write(hdev, OTP_READ_WIDTH, 0x001E);
		hdmi_write(hdev, OTP_DOUT_OFFSET, 0x0019);
		hdmi_write(hdev, OTP_READ_OFFSET, 0x001A);
		hdmi_write(hdev, OTP_BASE_ADDRESS_H, 0x14);
		hdmi_write(hdev, OTP_BASE_ADDRESS_L, 0x00);
	}

#ifdef OTP_SMC_NOT_SUPPORT
	if (hdev->hdcp_info.hdcp_enable == HDCP_VERSION_22) {
		val = readl(hdev->otp_regs + OTP_CON_CONTROL);
		if (en) {
			hdmi_write(hdev, HDCP_OTP_CTRL, 0x1);
			val |= 0x00000020; /* OTP enable[5] */
			writel(val, hdev->otp_regs + OTP_CON_CONTROL);
			if (hdcp_loadkey(hdev) < 0)
				return -1;
		} else {
			val &= ~(0x00000020);
			writel(val, hdev->otp_regs + OTP_CON_CONTROL);
		}
		writel(val, hdev->otp_regs + OTP_CON_CONTROL);
	}
#else
	/* ret == 0 is success */
	ret = exynos_smc(0x82001014, 0, 0x2004, en);
	if (ret)
		hdmi_err("failed to OTP exynos_smc, en(%d) ret(%d)\n", en, ret);
#endif
	hdmi_dbg(5, "-\n");
	return ret;
}

int hdcp_start(struct hdmi_device *hdev)
{
	int ret;
	hdmi_dbg(5, "+\n");

	hdev->hdcp_info.event = HDCP_EVENT_START;
	hdev->hdcp_info.auth_status = NOT_AUTHENTICATED;

	/* Select HDCP14 */
	hdmi_write(hdev, HDMI20_HDCP_SEL, HDCP14_SEL_EN);
	hdcp_reg_encryption(hdev, 0);
	hdcp_reg_swreset(hdev);

	if (hdev->otp_regs) {
		ret = hdcp_otp_control(hdev, 1);
		if (ret)
			return ret;
	}

	hdev->hdcp_info.hdcp_start = 1;
	/* Enable interrupt and HDCP */
	hdmi_write(hdev, HDCP14_STATUS_EN, HDMI_INT_EN_ALL);
	hdmi_write_mask(hdev, HDCP14_CTRL1, HDCP_DESIRED, HDCP_DESIRED);

	dev_info(hdev->dev, "%s is ready\n", __func__);
	hdmi_dbg(5, "-\n");
	return 0;
}

int hdcp_stop(struct hdmi_device *hdev)
{
	int ret = 0;
	hdmi_dbg(5, "+\n");

	if (!hdev->hdcp_info.hdcp_start)
		return 0;

	hdev->hdcp_info.event		= HDCP_EVENT_STOP;
	hdev->hdcp_info.auth_status	= NOT_AUTHENTICATED;
	hdev->hdcp_info.hdcp_start	= false;

	if (hdev->otp_regs)
		ret = hdcp_otp_control(hdev, 0);

	hdmi_write(hdev, HDCP14_CTRL1, 0x0);
	hdmi_write(hdev, HDCP14_CTRL2, 0x0);
	hdcp_reg_encryption(hdev, 0);
	hdmi_write(hdev, HDCP14_STATUS_EN, (~HDMI_INT_EN_ALL));
	hdmi_writeb(hdev, HDCP14_CHECK_RESULT, HDMI_HDCP_CLR_ALL_RESULTS);

	hdcp_reg_swreset(hdev);

	dev_info(hdev->dev, "%s is stoped\n", __func__);
	hdmi_dbg(5, "-\n");
	return ret;
}
