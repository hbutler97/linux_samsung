/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <video/videonode.h>
#include <media/exynos_mc.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/pm_qos.h>
#include <linux/bug.h>
#include <linux/v4l2-mediabus.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "i2c_api.h"

int bI2c_not_work = 0;
void i2c_udelay(unsigned usec)
{
	udelay(usec);
}

struct fimc_is_i2c_dev i2c_dev;

unsigned long mcubase;

#if 0
static void dump_reg(void)
{
	volatile uint32_t value = 0;
	u32 offset = 0;

//while(1) {
	offset = 0;
	while(offset <= 0x40) {
		value  = readl((volatile void *)i2c_dev.regbase + offset);
		pr_info("[%X] : %X\n", offset, value);
		offset += 4;
	}

	printk(KERN_INFO "INT : %X %X\n", readl((volatile void *)mcubase + 0x38), readl((volatile void *)mcubase + 0x3C));
	msleep(1000);
//}
}
#endif

static irqreturn_t i2c_handle_irq(int irq, void *data)
{
	struct fimc_is_i2c_dev *dev = data;
	int eStatus; 
	int bReadAck;
	PI2C_PacketStr	cur_packet = dev->cur_packet;

	{
		volatile uint32_t src = 0;
		src = ISP_I2C_GetIntStatus(i2c_dev.channel);
		dev->intr_status |= src;
		ISP_I2C_ClrIntStatus(i2c_dev.channel, src);
		eStatus = dev->intr_status;
	}

	//printk(KERN_INFO "I2C interrupt %X\n", eStatus);

	if(cur_packet->eDirection == I2C_TRANSACTION_WRITE)
	{
		// Send data
		// Check sending is complete.
		if(dev->cur_buf_idx != cur_packet->uiDataLen)
		{
			if(ISP_I2C_IsXferAck(i2c_dev.channel))
			{
				// check start condition, and send slave addr...
				I2CIntrSending(i2c_dev.channel, cur_packet->pucData[dev->cur_buf_idx++]);
			}
			else
			{
				i2c_loge("Sensor I2C doesn't work. I2C ch(%d),Addr:0x%02x - Interrupt Status Register: 0x%X \n", i2c_dev.channel, cur_packet->ucI2cAddr, eStatus);
                bI2c_not_work = 1;
				dev->irq_ack = 1;
				wake_up(&dev->waitq);
			}
		}
		else
		{
			I2CIntrEnd(i2c_dev.channel);
			dev->irq_ack = 1;
			wake_up(&dev->waitq);
		}
	}
	else if(cur_packet->eDirection == I2C_TRANSACTION_READ)
	{
		// Receive data
		if(dev->is_first_read_intr)
		{
			bReadAck = (dev->cur_buf_idx == (cur_packet->uiDataLen-1)) ? 0 : 1;
			I2CIntrRecvClear(i2c_dev.channel, bReadAck);
			dev->is_first_read_intr = 0;
		}
		else
		{
			cur_packet->pucData[dev->cur_buf_idx++] = ISP_I2C_ReadData(i2c_dev.channel);

			// Check recving is complete.
			if(dev->cur_buf_idx != cur_packet->uiDataLen)
			{
				if(ISP_I2C_IsXferAck(i2c_dev.channel))
				{
					bReadAck = (dev->cur_buf_idx == (cur_packet->uiDataLen-1)) ? 0 : 1;
					I2CIntrRecvClear(i2c_dev.channel, bReadAck);
				}
				else
				{
					i2c_loge("Sensor I2C doesn't work.\n");

					dev->irq_ack = 1;
					wake_up(&dev->waitq);
					dev->is_first_read_intr = 1;
				}
			}
			else
			{
				I2CIntrEnd(i2c_dev.channel);

				dev->irq_ack = 1;
				wake_up(&dev->waitq);
				dev->is_first_read_intr = 1;
			}
		}
	}

	//atomic_clr(&dev->intr_status, eStatus);
	dev->intr_status &= ~eStatus;

	return IRQ_HANDLED;
}

void i2c_init(struct device *dev)
{
	int ret = 0;

	mcubase = (unsigned long) ioremap_nocache(0x14370000, 0x100);

	writel(0, (volatile void *)(mcubase + 0x34));
	printk(KERN_INFO "MASK2 : %X\n", readl((volatile void *)(mcubase + 0x34)));

	i2c_dev.intr = 32 + 141;
	i2c_dev.iid = -1;
	i2c_dev.physbase = 0x14310000;
	i2c_dev.regbase = (unsigned long) ioremap_nocache(i2c_dev.physbase, 0x100);
	i2c_dev.slave_addr = 0x5A;
	i2c_dev.channel = ISP_I2C_Ch0;
	i2c_dev.sclk = 400000;
	i2c_dev.intr_status = 0;

	i2c_dev.cur_buf_idx = 0;
	i2c_dev.is_first_read_intr = 1;
    bI2c_not_work = 0;

	mutex_init(&i2c_dev.i2c_mutex);
	init_waitqueue_head(&i2c_dev.waitq);

	i2c_driver_init(i2c_dev.channel);
	ISP_I2C_SoftwareReset(i2c_dev.channel);
	i2c_clock_init();

	ret = request_irq(i2c_dev.intr,
			i2c_handle_irq,
			IRQF_SHARED,
			"isp-i2c0",
			&i2c_dev);
	if (ret)
		printk(KERN_INFO "request_irq(i2c 141) is fail(%d)", ret);

/* HACK */
//	iounmap((void *)mcubase);
}

void i2c_deinit(struct device *dev)
{
	free_irq(i2c_dev.intr, &i2c_dev);
	iounmap((void *)i2c_dev.regbase);
}

void i2c_SetInit(uint8_t eCh)
{
    ISP_I2C_CONTEXT *sContext = ISP_I2C_GetContext((ISP_I2C_CHANNEL)eCh);

    if(sContext)
    {
        ISP_I2C_SetFilter((ISP_I2C_CHANNEL)eCh, 1);
        ISP_I2C_SetOutputDelay((ISP_I2C_CHANNEL)eCh, sContext->eOutputDelay);
        ISP_I2C_SetClock((ISP_I2C_CHANNEL)eCh, sContext->ePrescaler, sContext->nPrescaler);
    } else {
	    i2c_loge("[%s][Ch%d] sContext(I2C) is NULL.\n", __func__, eCh);
    }
}

void i2c_CalcInit(uint8_t eCh, uint32_t uiI2CSpeed, int bEnDiffSpeed)
{
    ISP_I2C_PRESCALER ePrescaler = ISP_I2C_PRESCALER_UNKNOWN;
    ISP_I2C_OUTPUT_DELAY eOutputDelay = ISP_I2C_OD_0CLK;
    uint32_t nPclk = 0, nPrescaler = 0;
    uint32_t uOpI2CSpeed = 0;
	ISP_I2C_CONTEXT *sContext = ISP_I2C_GetContext((ISP_I2C_CHANNEL)eCh);
	i2c_assert(sContext, "[%s][Ch%d] sContext(I2C) is NULL.\n", __func__, eCh);

    i2c_assert(i2c_dev.sclk != 0 , "[I2CHAL_CalcInit] i2c_dev.sclk is NULL.\n");

    nPclk = 26000000;
    
    

    if (!ISP_I2C_CalcPrescaler(nPclk, uiI2CSpeed, &ePrescaler, &nPrescaler, &uOpI2CSpeed))
    {
        i2c_assert(uiI2CSpeed != 0 , "[I2CHAL_CalcInit] I2CSpeed is NULL\n");
        i2c_assert(uOpI2CSpeed != 0, "[I2CHAL_CalcInit] OpI2CSpeed is NULL\n");
        i2c_assert(nPclk != 0 , "[I2CHAL_CalcInit] i2c_dev.sclk is NULL.\n");

        i2c_loge("[%s](Ch:%d, DevNum:%d)Invalid TX Clock!!! Check I2C SPPED(%dKhz).. (SCLK: %dKhz, I2C Input Clk: %dMhz)\n",
            __func__, eCh, sContext->nCurDevice, uiI2CSpeed/K_UNIT, uOpI2CSpeed/K_UNIT, nPclk/M_UNIT);
        return;
    }

    i2c_assert(uiI2CSpeed != 0 , "[I2CHAL_CalcInit] I2CSpeed is NULL\n");
    i2c_assert(uOpI2CSpeed != 0 , "[I2CHAL_CalcInit] OpI2CSpeed is NULL\n");
    i2c_assert(nPclk != 0 , "[I2CHAL_CalcInit] i2c_dev.sclk is NULL.\n");

    ISP_I2C_CalculateOutputDelay(nPclk, uiI2CSpeed, &eOutputDelay);

    if(sContext)
    {
        if (bEnDiffSpeed == 0)
        {
            sContext->nOpClock = uOpI2CSpeed;   //same value : nPclk/ePrescaler/(nPrescaler+1);
            sContext->ePrescaler = ePrescaler;
            sContext->nPrescaler = nPrescaler;
            sContext->nTimeOut = ISP_I2C_TIMEOUT_INFINITY;
            sContext->eError = ISP_I2C_SUCCESS;
            sContext->eOutputDelay = eOutputDelay;
        }
    }
}

int i2c_init_clk(uint8_t eCh, uint32_t uiI2CSpeed)
{
    i2c_CalcInit(eCh, uiI2CSpeed, 0);
    i2c_SetInit(eCh);
    return 1;
}

void i2c_clock_init(void)
{
	i2c_init_clk(i2c_dev.channel, i2c_dev.sclk);
	ISP_I2C_ClrIntStatus(i2c_dev.channel, 0xFFFFFFFF); // Clear all of interrupt
	printk(KERN_INFO "[I2C MODE] %s (Ch:%d)\n", "NON FIFO", i2c_dev.channel);
};

int PushDataPrompt(PI2C_PacketStr pWPacket)
{
	int bRet = 0;

	//mutex_lock(&i2c_dev.i2c_mutex);

	i2c_dev.cur_packet = pWPacket;
	i2c_dev.cur_buf_idx = 0;

	if(!I2CIntrSendStart(i2c_dev.channel, i2c_dev.cur_packet->ucI2cAddr)){
		i2c_loge("[I2C ][%s] I2CIntrSendStart fail\n", __func__);

	    if (bI2c_not_work) {
		    i2c_loge("[I2C ][%s] I2C module does not exist\n", __func__);
		    return 1;
	    }
	}
#if 0
	{
		volatile uint32_t value = 0;
		u32 offset = 0;

		while(1) {
			offset = 0;
			while(offset <= 0x40) {
				value  = readl((volatile void *)i2c_dev.regbase + offset);
				pr_info("[%X] : %X\n", offset, value);
				offset += 4;
			}
			printk(KERN_INFO "INT : %X %X\n", readl((volatile void *)mcubase + 0x38), readl((volatile void *)mcubase + 0x3C));
			msleep(1000);
		}
	}
#endif

	i2c_dev.irq_ack = 0;
	wait_event(i2c_dev.waitq, (i2c_dev.irq_ack != 0));
	bRet = 0;

	//mutex_unlock(&i2c_dev.i2c_mutex);

	return bRet;
}

int PopData(PI2C_PacketStr pRPacket)
{
	int bRet = 0;

	mutex_lock(&i2c_dev.i2c_mutex);

	i2c_dev.cur_packet = pRPacket;
	i2c_dev.cur_buf_idx = 0;

	if(!I2CIntrRecvStart(i2c_dev.channel, i2c_dev.cur_packet->ucI2cAddr))
	{
		i2c_loge("[I2C ][%s] I2CIntrRecvStart fail\n", __func__);
	    mutex_unlock(&i2c_dev.i2c_mutex);

		if (bI2c_not_work) {
		    i2c_loge("[I2C ][%s] I2C module does not exist\n", __func__);
			return 1;
		}
	}

	i2c_dev.irq_ack = 0;
	wait_event(i2c_dev.waitq, (i2c_dev.irq_ack != 0));

	mutex_unlock(&i2c_dev.i2c_mutex);

	return bRet;
}

int i2c_read_addr16data8(int slaveAddr, uint16_t addr, uint8_t *val)
{
	uint8_t data[2] = {addr >> 8, addr & 0xFF};

	// write addr
	I2C_PacketStr wPacket, RdPacket;
	wPacket.ucI2cAddr = slaveAddr;
	wPacket.eDirection = I2C_TRANSACTION_WRITE;	// write
	wPacket.uiDataLen = 2;
	wPacket.pucData = data;

	PushDataPrompt(&wPacket);

	// read val
	RdPacket.ucI2cAddr = slaveAddr;
	RdPacket.eDirection = I2C_TRANSACTION_READ;	// read
	RdPacket.uiDataLen = 1;
	RdPacket.pucData = val;

	PopData(&RdPacket);

	return 0;
}

int i2c_read_addr16data16(int slaveAddr, uint16_t addr, uint16_t *val)
{
    int bRet = 0;
	uint8_t wdata[2] = {addr >> 8, addr & 0xFF};
	uint8_t rdata[2] = {0};

	// write addr
	I2C_PacketStr wPacket, RdPacket;
	wPacket.ucI2cAddr = slaveAddr;
	wPacket.eDirection = I2C_TRANSACTION_WRITE;	// write
	wPacket.uiDataLen = 2;
	wPacket.pucData = wdata;

	bRet = PushDataPrompt(&wPacket);

	RdPacket.ucI2cAddr = slaveAddr;
	RdPacket.eDirection = I2C_TRANSACTION_READ;	// read
	RdPacket.uiDataLen = 2;
	RdPacket.pucData = rdata;

	bRet = PopData(&RdPacket);

	*val = ((rdata[0] << 8) | rdata[1]);

	return bRet;
}

int i2c_write_addr16data8(int slaveAddr, uint16_t addr, uint8_t val)
{
	int bRet = 0;
	uint8_t data[3] = {addr >> 8, addr & 0xFF, val};

	I2C_PacketStr wPacket;
	wPacket.ucI2cAddr = slaveAddr;
	wPacket.eDirection = I2C_TRANSACTION_WRITE;	// write
	wPacket.uiDataLen = 3;
	wPacket.pucData = data;

	bRet = PushDataPrompt(&wPacket);

	return bRet;
}

int i2c_write_addr16data16(int slaveAddr, uint16_t addr, uint16_t val)
{
	int bRet =0;
	uint8_t wdata[4] = {addr >> 8, addr & 0xFF, val >> 8, val & 0xFF};

	I2C_PacketStr wPacket;
	wPacket.ucI2cAddr = slaveAddr;
	wPacket.eDirection = I2C_TRANSACTION_WRITE;	// write
	wPacket.uiDataLen = 4;
	wPacket.pucData = wdata;

	bRet = PushDataPrompt(&wPacket);

	return bRet;
}


int i2c_write_burst(int slaveAddr, uint8_t *wdata, uint32_t size)
{
	I2C_PacketStr wPacket;
	wPacket.ucI2cAddr = slaveAddr;
	wPacket.eDirection = I2C_TRANSACTION_WRITE;	// write
	wPacket.uiDataLen = size;
	wPacket.pucData = wdata;

	PushDataPrompt(&wPacket);

	return 0;
}
