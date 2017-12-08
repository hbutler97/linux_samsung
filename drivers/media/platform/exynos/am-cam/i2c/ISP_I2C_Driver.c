/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/platform_device.h>
#include <linux/io.h>

#include "ISP_I2C_Driver.h"

#define MAX_I2C_ERROR_CNT           5

#define DEBUGMSG(cond,printf_exp)   ((void)((cond)?(i2c_logd printf_exp),1:0))
#define HW_REG32(base, offset)		(*(volatile uint32_t *)(base + (offset)))
#define HW_REG16(base, offset)		(*(volatile u16 *)(base + (offset)))
#define HW_REG8(base, offset)		(*(volatile uint8_t *)(base + (offset)))

#define I2C_SPEED_HW_MARGIN     30000   // 30Khz

static ISP_I2C_CONTEXT g_aI2c[] =
{
	{ 0, 0, ISP_I2C_PRESCALER_UNKNOWN, 0, 0, ISP_I2C_SUCCESS, ISP_I2C_OD_0CLK },
	{ 0, 0, ISP_I2C_PRESCALER_UNKNOWN, 0, 0, ISP_I2C_SUCCESS, ISP_I2C_OD_0CLK },
#if (I2C_MAX_CHANNEL>=3)
	{ 0, 0, ISP_I2C_PRESCALER_UNKNOWN, 0, 0, ISP_I2C_SUCCESS, ISP_I2C_OD_0CLK },
#endif
#if (I2C_MAX_CHANNEL>=4)
	{ 0, 0, ISP_I2C_PRESCALER_UNKNOWN, 0, 0, ISP_I2C_SUCCESS, ISP_I2C_OD_0CLK },
#endif
};

uint32_t uI2CErrCount[3] = {0, 0, 0};

void i2c_driver_init(int ch)
{
	if (ch == ISP_I2C_Ch0)
		g_aI2c[ch].uBase = (unsigned long) ioremap_nocache(I2C0_BASE, 0x100);
	else if (ch == ISP_I2C_Ch1)
		g_aI2c[ch].uBase = (unsigned long) ioremap_nocache(I2C1_BASE, 0x100);
#if (I2C_MAX_CHANNEL>=3)
	else if (ch == ISP_I2C_Ch2)
		g_aI2c[ch].uBase = (unsigned long) ioremap_nocache(I2C2_BASE, 0x100);
#endif
}

void ISP_I2C_CalculateOutputDelay(uint32_t uPClk, uint32_t uiI2CSpeed, ISP_I2C_OUTPUT_DELAY *peOutputDelay)
{
    uint32_t Interval = 0;       // interval (ns) btw SCL and SDA when SCL : 400Khz
    uint32_t uCalcValue, uStep;
    uint32_t uPeriod =0;
    uint32_t uTotaldelay =0;

    uPeriod = G_UNIT/uPClk;  // ns

    Interval = (G_UNIT/uiI2CSpeed)/2 - BASE_INTERVAL;   // (ns)

    for(uStep = 0; uStep < SDA_OUTPUT_DELAY_SETP; uStep++)
    {
        uCalcValue = uPeriod * (SDA_OUTPUT_DELAY_UNIT * uStep);
        if(uCalcValue > Interval)
            break;
    }

    switch(uStep)
    {
        case 1:
            *peOutputDelay = ISP_I2C_OD_0CLK;
            break;
        case 2:
            *peOutputDelay = ISP_I2C_OD_16CLK;
            break;
        case 3:
            *peOutputDelay = ISP_I2C_OD_32CLK;
            break;
        case 4:
            *peOutputDelay = ISP_I2C_OD_48CLK;
            break;
        default:
            *peOutputDelay = ISP_I2C_OD_0CLK;
            break;
    }

    uTotaldelay = Interval +  (*peOutputDelay)*SDA_OUTPUT_DELAY_UNIT;

    if (uTotaldelay < Interval)
        i2c_loge("[%s] Wrong SDA delay!! : uTotaldelay(%d) <= Interval(%d)\n", __func__, uTotaldelay, Interval);
}

int ISP_I2C_CalcPrescaler(uint32_t uPClk, uint32_t uiI2CSpeed, ISP_I2C_PRESCALER *pePrescaler, uint32_t *pnPrescaler, uint32_t *uOpI2CSpeed)
{
	uint32_t aTable[64];
	uint32_t uCalcValue;
	uint32_t i;

	if ( (uiI2CSpeed == 0) || (uiI2CSpeed > MAX_I2C_SPEED)) {
		i2c_loge("[%s] I2C SPEED is Wrong!!(%dhz)\n", __func__, uiI2CSpeed);
		return 0;
	}

	for (i = 0; i < 32; i++) {
		if (i < 16)
			aTable[i] = uPClk / ISP_I2C_PRESCALER_32 / (i + 1);
		else if (i < 32)
			aTable[i] = uPClk / ISP_I2C_PRESCALER_512 / (i - 16 + 1);
	}

	for (i = 0; i < 32; i++) {
		uCalcValue = aTable[i];

		if (uCalcValue > MAX_I2C_SPEED)
			continue;

		if (uCalcValue <= uiI2CSpeed + I2C_SPEED_HW_MARGIN) {

			if (i < 16) {
				*pePrescaler = ISP_I2C_PRESCALER_32;
				*pnPrescaler = i;
			} else if (i < 32) {
				*pePrescaler = ISP_I2C_PRESCALER_512;
				*pnPrescaler = i - 16;
			}
			*uOpI2CSpeed = uCalcValue;

			return 1;
		}
	}

	*pePrescaler = ISP_I2C_PRESCALER_UNKNOWN;
	*pnPrescaler = 0;
	*uOpI2CSpeed = uCalcValue;

	return 0;
}

int ISP_I2C_WaitForBusReady(ISP_I2C_CHANNEL eCh, uint32_t nTimeOut)
{
	nTimeOut = ISP_I2C_BUS_READY_TIMEOUT_LOOP_CNT;
	while (--nTimeOut) {
		if (nTimeOut <= ISP_I2C_BUS_READY_TIMEOUT_CNT) {
			mdelay(ISP_I2C_BUS_READY_TIMEOUT);
		}
		if (ISP_I2C_IsBusReady(eCh)) {
			uI2CErrCount[eCh] = 0;
			return 1;
		}
	}

	if (uI2CErrCount[eCh] < MAX_I2C_ERROR_CNT)
		i2c_loge("[ISP_I2C_WaitForBusReady][ch %d] timeout (ErrCnt:%d)\n", eCh, uI2CErrCount[eCh]+1);

	uI2CErrCount[eCh]++;

	return 0;
}

void ISP_I2C_SetClock(ISP_I2C_CHANNEL eCh, ISP_I2C_PRESCALER eSource, uint32_t nPrescaler)
{
	if (ENABLE_FIXED_I2C_CLK == 1) {
		switch(eSource) {
		case ISP_I2C_PRESCALER_32:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CLK_BYPASS) = 0x1;
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) &= ~(0x1 << 6);
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_NCLK_DIV20) = 0x0;
			break;
		case ISP_I2C_PRESCALER_512:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CLK_BYPASS) = 0x1;
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (0x1 << 6);
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_NCLK_DIV20) = 0x0;
			break;
		default:
			i2c_assert(0, "[I2C] I2C : wrong prescaler value\n");
			break;
		}
	} else {
		if (eSource == ISP_I2C_PRESCALER_32) {
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) &= ~(0x1 << 6);
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_NCLK_DIV20) = 0x0;
		} else if (eSource == ISP_I2C_PRESCALER_512) {
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (0x1 << 6);
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_NCLK_DIV20) = 0x0;
		}
	}

	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) &= ~(0xF);
	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (nPrescaler & 0xF);
}

void ISP_I2C_SetAckGen(ISP_I2C_CHANNEL eCh, int bEnable)
{
	if (bEnable)
		HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (0x1 << 7);
	else
		HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) &= ~(0x1 << 7);
}

void ISP_I2C_SetFilter(ISP_I2C_CHANNEL eCh, int bEnable)
{
	if (bEnable) {
	 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) &= ~(0xf<<12);
	 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) &= ~(0xf<<16);
	 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) |= ((0x0<<12) | (0x0<<16));
	} else {
	 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) &= ~(0xf<<12);
	 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) &= ~(0xf<<16);
	}
}

void ISP_I2C_SetOutputDelay(ISP_I2C_CHANNEL eCh, ISP_I2C_OUTPUT_DELAY eOutputDelay)
{
	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) &= ~(0x3);
	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_BUS_CON) |= (eOutputDelay & 0x3);
}

void ISP_I2C_SetMode(ISP_I2C_CHANNEL eCh, ISP_I2C_MODE eMode)
{
	uint32_t temp;

	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) &= ~(0x1 << 4);		// Disable I2C

	temp = HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) & ~(0x3 << 6);		// Clear Mode

	switch (eMode)
	{
		case ISP_I2C_MASTER_TX_MODE:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) = (temp | (0x3 << 6));	// Set Mode
			break;

		case ISP_I2C_MASTER_RX_MODE:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) = (temp | (0x2 << 6));	// Set Mode
			break;

		case ISP_I2C_SLAVE_TX_MODE:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) = (temp | (0x1 << 6));	// Set Mode
			break;

		case ISP_I2C_SLAVE_RX_MODE:
			HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) = (temp | (0x0 << 6));	// Set Mode
			break;

		default:
			return;
	}

	ISP_I2C_ClrBusHoldStatus(eCh);
	ISP_I2C_DisableInt(eCh, ISP_I2C_INT_START);			// Disable Interrupt
	ISP_I2C_EnableInt(eCh, ISP_I2C_INT_BUS_HOLD);
#if 0
	ISP_I2C_EnableInt(eCh, ISP_I2C_INT_ACK);
	ISP_I2C_EnableInt(eCh, ISP_I2C_INT_NACK);
	ISP_I2C_EnableInt(eCh, ISP_I2C_INT_START);
	ISP_I2C_EnableInt(eCh, ISP_I2C_INT_TIMEOUT);
#endif
	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) |= (0x1 << 4);	// Enable I2C
}

void ISP_I2C_EnableInt(ISP_I2C_CHANNEL eCh, ISP_I2C_INTERRUPT eInt)
{
 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (eInt);
}

void ISP_I2C_DisableInt(ISP_I2C_CHANNEL eCh, ISP_I2C_INTERRUPT eInt)
{
 	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) &= ~(eInt);
}

uint32_t ISP_I2C_GetIntStatus(ISP_I2C_CHANNEL eCh)
{
	return HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_INT_STAT) & 0xFFFF;
}

void ISP_I2C_ClrBusHoldStatus(ISP_I2C_CHANNEL eCh)
{
    HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_INT_STAT) |= (0x1 << 8);
    HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON) |= (0x1 << 4);
}

void ISP_I2C_GenerateSignal(ISP_I2C_CHANNEL eCh, ISP_I2C_CONDITION eCondition)
{
	if (eCondition == ISP_I2C_START_CONDITION)
		HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) |= (0x1 << 5);
	else
		HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) &= ~(0x1 << 5);
}

int ISP_I2C_IsBusReady(ISP_I2C_CHANNEL eCh)
{
	if (HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) & (0x1 << 5))
		return 0;
	else
		return 1;
}

int ISP_I2C_IsXferAck(ISP_I2C_CHANNEL eCh)
{
	if (HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_STAT) & (0x1 << 0))
		return 0;
	else
		return 1;
}

void ISP_I2C_WriteAddress(ISP_I2C_CHANNEL eCh, uint8_t nAddress)
{
	HW_REG8(g_aI2c[eCh].uBase, ISP_I2C_DS) = nAddress & 0xFE;
}

void ISP_I2C_WriteData(ISP_I2C_CHANNEL eCh, uint8_t nData)
{
	HW_REG8(g_aI2c[eCh].uBase, ISP_I2C_DS) = nData;
}

uint8_t ISP_I2C_ReadData(ISP_I2C_CHANNEL eCh)
{
	return HW_REG8(g_aI2c[eCh].uBase, ISP_I2C_DS);
}

void ISP_I2C_SoftwareReset(ISP_I2C_CHANNEL eCh)
{
	printk(KERN_INFO "IKY before %lX\n", g_aI2c[eCh].uBase);
	
	HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_SW_RESET) |= (1 << 0);
	msleep(10);
	printk(KERN_INFO "IKY after %lX\n", g_aI2c[eCh].uBase);
}

void ISP_I2C_ClrIntStatus(ISP_I2C_CHANNEL eCh, uint32_t uInt)
{

    HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_INT_STAT) &= uInt ;
}

int I2CIntrSendStart(ISP_I2C_CHANNEL eCh, uint8_t ucSlvAddr)
{
	if (g_aI2c[eCh].eError == ISP_I2C_INVALID_TX_CLOCK)
	{
		return 0;
	}

	// I2C Phase - Check BUS Status
	if (ISP_I2C_WaitForBusReady(eCh, g_aI2c[eCh].nTimeOut) == 0) {
		g_aI2c[eCh].eError = ISP_I2C_TIMEOUT_BUS_READY_START;
		if (uI2CErrCount[eCh] < MAX_I2C_ERROR_CNT) {
			i2c_loge("[%s][ch %d]Not Bus(SCL) Ready to Start(CON:0x%X, INI:0x%X)\n", __func__,
					eCh, HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON),
					HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_INT_STAT));
		}
		return 0;
	}

	// I2C Phase - START
	ISP_I2C_SetMode(eCh, ISP_I2C_MASTER_TX_MODE);		// Set Mode
	ISP_I2C_WriteAddress(eCh, ucSlvAddr);			// Write Slave Address
	ISP_I2C_GenerateSignal(eCh, ISP_I2C_START_CONDITION);	// Send START Signal

	return 1;
}

void I2CIntrSending(ISP_I2C_CHANNEL eCh, uint8_t ucData)
{
    ISP_I2C_ClrBusHoldStatus(eCh);
    ISP_I2C_WriteData(eCh, ucData);
}

int I2CIntrEnd(ISP_I2C_CHANNEL eCh)
{
	// Send STOP Signal
	ISP_I2C_GenerateSignal(eCh, ISP_I2C_STOP_CONDITION);
	ISP_I2C_ClrBusHoldStatus(eCh);

	// I2C Phase - Check BUS Status
	if (ISP_I2C_WaitForBusReady(eCh, g_aI2c[eCh].nTimeOut) == 0) {
		if (uI2CErrCount[eCh] < MAX_I2C_ERROR_CNT) {
			i2c_loge("[%s][ch %d]Not Bus(SCL) Ready to Stop(CON:0x%X, INI:0x%X)\n", __func__,
					eCh, HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_CON),
					HW_REG32(g_aI2c[eCh].uBase, ISP_I2C_INT_STAT));
		}
		return 0;
	}

	return 1;
}

int I2CIntrRecvStart(ISP_I2C_CHANNEL eCh, uint8_t ucSlvAddr)
{
	if (g_aI2c[eCh].eError == ISP_I2C_INVALID_TX_CLOCK)
		return 0;

	if (ISP_I2C_WaitForBusReady(eCh, g_aI2c[eCh].nTimeOut) == 0) {
		g_aI2c[eCh].eError = ISP_I2C_TIMEOUT_BUS_READY_START;
		return 0;
	}

	// I2C Phase - START
	ISP_I2C_SetMode(eCh, ISP_I2C_MASTER_RX_MODE);		// Set Mode
	ISP_I2C_WriteAddress(eCh, ucSlvAddr);			// Write Slave Address
	ISP_I2C_GenerateSignal(eCh, ISP_I2C_START_CONDITION);	// Send START Signal

	return 1;
}

void I2CIntrRecvClear(ISP_I2C_CHANNEL eCh, int bAck)
{
	ISP_I2C_SetAckGen(eCh, bAck);
	ISP_I2C_ClrBusHoldStatus(eCh);
}

ISP_I2C_CONTEXT *ISP_I2C_GetContext(ISP_I2C_CHANNEL eCh)
{
	if (eCh >= ISP_I2C_CHANNEL_MAX) {
		i2c_loge("[%s] Wrong Channel!(%d)", __func__, eCh);
		return NULL;
	} else {
		return &(g_aI2c[eCh]);
	}

	return NULL;
}
