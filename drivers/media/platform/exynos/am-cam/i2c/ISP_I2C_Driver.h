/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __ISP_I2C_DRIVER_H__
#define __ISP_I2C_DRIVER_H__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>

#define I2C_MAX_CHANNEL			2

#define FIMC_ISPBLK_BASE        (0x14000000)
#define I2C0_BASE                   (FIMC_ISPBLK_BASE + 0x00310000)
#define I2C1_BASE                   (FIMC_ISPBLK_BASE + 0x00320000)
#define I2C2_BASE                   (FIMC_ISPBLK_BASE + 0x00330000)

#define ENABLE_FIXED_I2C_CLK 1

#define NUM_I2C0 (32 + 141)
#define NUM_I2C1 (32 + 142)
#define NUM_I2C2 (32 + 143)
#define NUM_I2C3 (32 + 144)

#if 0
#define i2c_logd printf
#else
#define i2c_logd(fmt, args...)
#endif

#define i2c_loge pr_info
#define i2c_logw pr_info
#define i2c_logi pr_info
#define i2c_assert(a, fmt, args...) do { if (!a) printk(fmt, ##args); } while(0)

#define ISP_I2C_DEVICE_NUM  4
#define ISP_I2C_SPFIFO_START	0x200
#define ISP_I2C_SPFIFO_STOP		0x100
#define ISP_I2C_SPIFO_TX_SIZE   64

/*
	Register Map
*/
#define ISP_I2C_CON					0x00
#define ISP_I2C_STAT				0x04
#define ISP_I2C_ADD					0x08
#define ISP_I2C_DS					0x0C
#define ISP_I2C_LCON				0x10

#define ISP_I2C_CLK_BYPASS			0x14
#define ISP_I2C_FIFO_CON			0x18
#define ISP_I2C_FIFO_STAT			0x1c
#define ISP_I2C_INT_STAT			0x20
#define ISP_I2C_VER					0x24
#define ISP_I2C_NCLK_DIV20			0x28
#define ISP_I2C_SW_RESET			0x2c
#define ISP_I2C_BUS_CON				0x30
#define ISP_I2C_TIMEOUT_CON			0x34
#define ISP_I2C_SP_DELAY			0x38

///////////////////////
// [2014/11/26, ys_sim]
#define K_UNIT  1000
#define M_UNIT  1000000
#define G_UNIT  1000000000

#define MAX_I2C_SPEED_400K      400000    // 400Khz
#define MAX_I2C_SPEED_800K      800000    // 800Khz

#define MAX_I2C_SPEED           MAX_I2C_SPEED_800K

#define SDA_OUTPUT_DELAY_SETP   4
#define SDA_OUTPUT_DELAY_UNIT   16  // 16clk (clk is PLCK or NCLK accroding to INPUT Clock)
#define BASE_INTERVAL           250 // base interval between SCL and SDA (measured value, unit : ns)

/* FIFO control */
#define I2C_ACK_CHECK		(1 << 8)
#define I2C_TX_TRIGGER_0	(0 << 6)
#define I2C_TX_TRIGGER_4	(1 << 6)
#define I2C_TX_TRIGGER_8	(2 << 6)
#define I2C_RX_TRIGGER_4	(0 << 4)
#define I2C_RX_TRIGGER_8	(1 << 4)

/* Value */
#define SP_FIFO_INT_EN		(1 << 17)
#define NACK_INT_EN			(1 << 14)
#define TX_INT_EN			(1 << 11)
#define RX_INT_EN			(1 << 10)
#define BUS_HOLD_INT_EN	(1 << 8)

#define ACK_GEN_EN			(1 << 7)

#define RELEASE_BUS_HOLD	(1 << 4)

#define ACK_CHECK			(1 << 8)
#define I2C_TX_FIFO_RESET   (1 << 3)
#define I2C_RX_FIFO_RESET	(1 << 2)
#define SP_FIFO_ENABLE		(1 << 1)
#define FIFO_ENABLE			(1 << 0)

#define MASTER_RX			(2 << 6)
#define MASTER_TX			(3 << 6)
#define START_GENERATION	(1 << 5)
#define ENABLE_TRANSFER	(1 << 4)

#define SP_FIFO_DONE_INT	(1 << 15)
#define NACK_INT			(1 << 14)
#define STOP_INT			(1 << 13)
#define START_INT			(1 << 12)
#define I2C_TX_INT          (1 << 11)
#define I2C_RX_INT			(1 << 10)
#define BUS_HOLD_INT		(1 << 8)
///////////////////////

/*
	Parameter
*/
#define MASTER_RX			(2 << 6)
#define MASTER_TX			(3 << 6)
#define START_GENERATION	(1 << 5)
#define ENABLE_TRANSFER	(1 << 4)

#define ISP_I2C_TX_CLOCK_125KHZ					125000
#define ISP_I2C_TX_CLOCK_200KHZ					200000
#define ISP_I2C_TX_CLOCK_300KHZ					300000
#define ISP_I2C_TX_CLOCK_400KHZ					400000
#define ISP_I2C_TX_CLOCK_DEFAULT				ISP_I2C_TX_CLOCK_300KHZ

#define ISP_I2C_TIMEOUT_INFINITY				0xFFFFFFFF
#define ISP_I2C_TIMEOUT_DEFAULT					0xFFFF
//#include "osbase.h"

#define ISP_I2C_BUS_READY_TIMEOUT_LOOP_CNT          10000

#define ISP_I2C_BUS_READY_TIMEOUT_CNT          5
#define ISP_I2C_BUS_READY_TIMEOUT                   1//2/MS_PER_TICK // 1ms

//#define I2C_CLOCK_FOR_FPGA
#if defined(I2C_CLOCK_FOR_FPGA)
#define I2C_nCLOCK_FREQ					12000000 // 12Mhz for FPGA
#else
//#define ISP_I2C_nCLOCK_FREQ					100000000 // 12Mhz for FPGA
#define I2C_nCLOCK_FREQ					8000000 // ISP I2C clock source comes from ACLK_DIV1 of ACLK200
#endif


/*
	Enumultion & Structure
*/
typedef enum
{
	ISP_I2C_SUCCESS,
    ISP_I2C_FAIL_BUS_ARBITRATION,
	ISP_I2C_INVALID_TX_CLOCK,
	ISP_I2C_INVALID_ADDRESS,
	ISP_I2C_TIMEOUT_BUS_READY_START,
	ISP_I2C_TIMEOUT_SLAVE_ADDRESS,
	ISP_I2C_TIMEOUT_WRITE_ADDRESS,
	ISP_I2C_TIMEOUT_WRITE_DATA,
	ISP_I2C_TIMEOUT_READ_DATA,
	ISP_I2C_TIMEOUT_BUS_READY_STOP,
} ISP_I2C_ERROR;

typedef enum
{
	ISP_I2C_DISABLE,
	ISP_I2C_MASTER_TX_MODE,
	ISP_I2C_MASTER_RX_MODE,
	ISP_I2C_SLAVE_TX_MODE,
	ISP_I2C_SLAVE_RX_MODE,
} ISP_I2C_MODE;

typedef enum
{
	ISP_I2C_START_CONDITION,
	ISP_I2C_STOP_CONDITION,
} ISP_I2C_CONDITION;

typedef enum
{
    // KJ_120720 : 32 and 256 is added to prescaler at gaia evt1.
	ISP_I2C_PRESCALER_16 = 16,
	ISP_I2C_PRESCALER_32 = 32,
	ISP_I2C_PRESCALER_256 = 256,
	ISP_I2C_PRESCALER_512 = 512,
	ISP_I2C_PRESCALER_UNKNOWN = 0
} ISP_I2C_PRESCALER;

typedef enum
{
	ISP_I2C_OD_0CLK,
	ISP_I2C_OD_16CLK,
	ISP_I2C_OD_32CLK,
	ISP_I2C_OD_48CLK,
} ISP_I2C_OUTPUT_DELAY;

typedef struct
{
    uint32_t uI2cAddr;
    uint32_t uI2cSpeed;
    uint32_t nOpClock;
    ISP_I2C_PRESCALER ePrescaler;
    uint32_t nPrescaler;
    ISP_I2C_OUTPUT_DELAY nDelay;
} ISP_I2C_SpeedInfo;

typedef struct
{
    unsigned long uBase;
    uint32_t nOpClock;
    ISP_I2C_PRESCALER ePrescaler;
    uint32_t nPrescaler;

    uint32_t nTimeOut;
    ISP_I2C_ERROR eError;

    ISP_I2C_OUTPUT_DELAY eOutputDelay;                  // [2013/12/16, ys_sim] add
    uint32_t nCurDevice;
    ISP_I2C_SpeedInfo sI2CSpeedInfo[ISP_I2C_DEVICE_NUM];   // [2014/11/20, ys_sim] add
} ISP_I2C_CONTEXT;

typedef enum
{
	ISP_I2C_Ch0,
	ISP_I2C_Ch1,
#if (I2C_MAX_CHANNEL>=3)
	ISP_I2C_Ch2,
#endif
#if (I2C_MAX_CHANNEL>=4)
	ISP_I2C_Ch3,
#endif
	ISP_I2C_CHANNEL_MAX,
} ISP_I2C_CHANNEL;

typedef enum
{
	ISP_I2C_INT_ACK			= (1 << 7),
	ISP_I2C_INT_BUS_HOLD		= (1 << 8),
	ISP_I2C_INT_TIMEOUT		= (1 << 9),
	ISP_I2C_INT_RX				= (1 << 10),
	ISP_I2C_INT_TX				= (1 << 11),
	ISP_I2C_INT_START			= (1 << 12),
	ISP_I2C_INT_STOP			= (1 << 13),
	ISP_I2C_INT_NACK			= (1 << 14),
	ISP_I2C_INT_SP_FIFO		= (1 << 15),
	ISP_I2C_INT_ALL			= (0xFF << 8),
} ISP_I2C_INTERRUPT;

typedef enum
{
	TL_1 = 0, 				// Trigger Level 1 - (TX:0B,RX:4B),
	TL_2 ,	 				// Trigger Level 2 - (TX:4B,RX:8B),
	TL_3 ,	 				// Trigger Level 3 - (TX:8B,RX:Reserved),
}ISP_I2C_TRIGGER_LEVEL;

void i2c_driver_init(int ch);
ISP_I2C_CONTEXT *ISP_I2C_GetContext(ISP_I2C_CHANNEL eCh);
int ISP_I2C_CalcPrescaler(uint32_t uPClk, uint32_t uiI2CSpeed, ISP_I2C_PRESCALER *pePrescaler, uint32_t *pnPrescaler, uint32_t *uOpI2CSpeed);
int ISP_I2C_WaitForBusReady(ISP_I2C_CHANNEL eCh, uint32_t nTimeOut);
void ISP_I2C_SetClock(ISP_I2C_CHANNEL eCh, ISP_I2C_PRESCALER eSource, uint32_t nPrescaler);
void ISP_I2C_SetAckGen(ISP_I2C_CHANNEL eCh, int bEnable);
void ISP_I2C_SetFilter(ISP_I2C_CHANNEL eCh, int bEnable);
void ISP_I2C_SetOutputDelay(ISP_I2C_CHANNEL eCh, ISP_I2C_OUTPUT_DELAY eOutputDelay);
void ISP_I2C_CalculateOutputDelay(uint32_t uPClk, uint32_t uiI2CSpeed, ISP_I2C_OUTPUT_DELAY *peOutputDelay);
void ISP_I2C_SetMode(ISP_I2C_CHANNEL eCh, ISP_I2C_MODE eMode);
void ISP_I2C_GenerateSignal(ISP_I2C_CHANNEL eCh, ISP_I2C_CONDITION eCondition);
int ISP_I2C_IsBusReady(ISP_I2C_CHANNEL eCh);
int ISP_I2C_IsXferAck(ISP_I2C_CHANNEL eCh);
void ISP_I2C_WriteAddress(ISP_I2C_CHANNEL eCh, uint8_t nAddress);
void ISP_I2C_WriteData(ISP_I2C_CHANNEL eCh, uint8_t nData);
uint8_t ISP_I2C_ReadData(ISP_I2C_CHANNEL eCh);

void ISP_I2C_EnableInt(ISP_I2C_CHANNEL eCh, ISP_I2C_INTERRUPT eInt);
void ISP_I2C_DisableInt(ISP_I2C_CHANNEL eCh, ISP_I2C_INTERRUPT eInt);
uint32_t ISP_I2C_GetIntStatus(ISP_I2C_CHANNEL eCh);
void ISP_I2C_ClrBusHoldStatus(ISP_I2C_CHANNEL eCh);
void ISP_I2C_SoftwareReset(ISP_I2C_CHANNEL eCh);
void ISP_I2C_ClrIntStatus(ISP_I2C_CHANNEL eCh, uint32_t uInt);
int I2CIntrSendStart(ISP_I2C_CHANNEL eCh, uint8_t ucSlvAddr);
void I2CIntrSending(ISP_I2C_CHANNEL eCh, uint8_t ucData);
int I2CIntrEnd(ISP_I2C_CHANNEL eCh);
int I2CIntrRecvStart(ISP_I2C_CHANNEL eCh, uint8_t ucSlvAddr);
void I2CIntrRecvClear(ISP_I2C_CHANNEL eCh, int bAck);

#endif /* __ISP_I2C_DRIVER_H__ */

