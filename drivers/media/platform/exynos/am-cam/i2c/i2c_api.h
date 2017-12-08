/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __ISP_I2C_API_H__
#define __ISP_I2C_API_H__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/wait.h>

#include "ISP_I2C_Driver.h"

typedef enum
{
	I2C_TRANSACTION_WRITE = 0,
	I2C_TRANSACTION_READ = 1,
	I2C_TRANSACTION_WAIT = 2,
	I2C_TRANSACTION_BOTH = 3,
	I2C_TRANSACTION_UNKNOWN,
	I2C_TRANSACTION_TOTALCOUNT
} eI2C_Transcaction;

typedef struct
{
	uint8_t ucI2cAddr;
	eI2C_Transcaction eDirection; // 0 : write, 1 : read, 2: waiting, 3: both
	uint32_t uiDataLen;
	uint8_t *pucData;
	uint8_t uWaitTime; // [hc0105.kim, 2014/02/17] If eDirection set to 2, then I2C sleeps about uWaitTime
}I2C_PacketStr, *PI2C_PacketStr;

struct fimc_is_i2c_dev {
	int				intr;
	int				iid;
	unsigned            		physbase;
	uintptr_t           		regbase;
	unsigned            		slave_addr;
	ISP_I2C_CHANNEL			channel;
	uint32_t			sclk;


	volatile int			irq_ack;
	struct mutex			i2c_mutex;
	wait_queue_head_t		waitq;
	unsigned int			intr_id;
	unsigned int			intr_status;

	PI2C_PacketStr			cur_packet;
	int				cur_buf_idx;
	int				is_first_read_intr;

#ifdef POLLING_MODE
	volatile int			i2c_thread_running;
#endif
};

void i2c_thread_start(void);
void i2c_thread_stop(void);

void i2c_init(struct device *dev);
void i2c_deinit(struct device *dev);
void i2c_request_irq(struct fimc_is_i2c_dev *dev);
void i2c_clock_init(void);

int i2c_read_addr16data8(int slaveAddr, uint16_t addr, uint8_t *val);
int i2c_read_addr16data16(int slaveAddr, uint16_t addr, uint16_t *val);
int i2c_write_addr16data8(int slaveAddr, uint16_t addr, uint8_t val);
int i2c_write_addr16data16(int slaveAddr, uint16_t addr, uint16_t val);

int i2c_write_burst(int slaveAddr, uint8_t *wdata, uint32_t size);
#endif
