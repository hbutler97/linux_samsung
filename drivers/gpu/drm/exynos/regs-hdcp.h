/*
 * Copyright (c) 2016 Samsung Electronics
 *		http://www.samsung.com
 *
 * HDCP register header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __REGS_HDCP_H__
#define __REGS_HDCP_H__

/** HDCP1.4 */
#define HDCP14_SEL_EN				(1 << 0)
#define HDCP22_SEL_EN				(0 << 0)
#define HDCP14_SHA1_0				(0x4000)		// total 160bit, 8bit x 20
#define HDCP14_KSV_LIST_0			(0x4050)		// total 40bit, 8bit x 5
#define HDCP14_KSV_LIST_CON			(0x4064)
#define HDCP14_SHA_RESULT			(0x4070)
#define HDCP14_CTRL1				(0x4080)
#define HDCP_DESIRED				(1 << 1)
#define HDCP14_CTRL2				(0x4084)		// bit[4] is hiddened
#define HDCP14_STATUS				(0x4088)
#define HDCP14_STATUS_EN			(0x408C)
#define HDCP14_CHECK_RESULT			(0x4090)
#define HDCP14_RI_NOT_MATCH			(2)
#define HDCP14_RI_MATCH				(3)

#define HDCP14_BKSV_0				(0x40A0)		// total 40bit, 8bit x 5
#define HDCP14_AKSV_0				(0x40C0)		// total 40bit, 8bit x 5
#define HDCP14_An_0				(0x40E0)		// total 64bit, 8bit x 8
#define HDCP14_BCAPS				(0x4100)
#define HDCP14_BSTATUS_0			(0x4110)
#define HDCP14_BSTATUS_1			(0x4114)
#define HDCP14_Ri_0				(0x4140)
#define HDCP14_Ri_1				(0x4144)

#define HDCP14_I2C_INT				(0x4180)
#define HDCP14_AN_INT				(0x4190)
#define HDCP14_WATCHDOG_INT			(0x41A0)
#define HDCP14_RI_INT				(0x41B0)

#define HDCP14_Ri_COMPARE_0			(0x41D0)
#define HDCP14_Ri_COMPARE_1			(0x41D4)

#define HDCP14_FRAME_COUNT			(0x41E0)
#define HDCP14_SYS_EN				(0x41E4)
#define HDCP14_ENC_EN				(0x41E8)
#define HDCP14_AN_SEL				(0x41EC)
#define HDCP14_ENC_ENABLE			(1 << 0)

#define HDCP14_AN_SEED_0			(0x41F0)
#define HDCP14_AN_SEED_1			(0x41F4)
#define HDCP14_AN_SEED_2			(0x41F8)
#define HDCP14_SHA1_CLKGATE_CTRL		(0x41FC)		// Hidden

// ** HDCP2.2 ** //
#define HDCP22_SYS_EN				(0x5000)
#define HDCP22_STATUS				(0x5004)
#define HDCP22_INT_FLAG				(0x5008)
#define HDCP22_INT_CLEAR			(0x500C)
#define HDCP22_INT_EN				(0x5010)
#define HDCP22_INPUT_CTR_RESET			(0x501C)
#define HDCP22_AES_ADDR				(0x5020)
#define HDCP22_LC128_ADDR			(0x5024)
#define HDCP22_AES_CLK_GATING			(0x5028)

#define HDCP22_ENC_EN				(0x5064)

#define HDCP22_RIV_0				(0x5104)
#define HDCP22_RIV_1				(0x5108)

#define HDCP22_AES_KEY_0			(0x5200)		// Secure
#define HDCP22_AES_KEY_1			(0x5204)		// Secure
#define HDCP22_AES_KEY_2			(0x5208)		// Secure
#define HDCP22_AES_KEY_3			(0x520C)		// Secure
#define HDCP22_KS_KEY_0				(0x5210)		// Secure
#define HDCP22_KS_KEY_1				(0x5214)		// Secure
#define HDCP22_KS_KEY_2				(0x5218)		// Secure
#define HDCP22_KS_KEY_3				(0x521C)		// Secure

// ** OTP ** //
#define HDCP_OTP_CTRL				(0x7000)
#define HDCP_OTP_STATUS				(0x7004)
#define HDCP14_ECC_DONE				(1 << 0)
#define HDCP14_ECC_BUSY				(1 << 1)
#define HDCP14_ECC_FAIL				(1 << 2)

#define OTP_SETUP_WIDTH				(0x780C)
#define OTP_RSTB_ASSERT				(0x7814)
#define OTP_RSTB_DEASSERT			(0x781C)
#define OTP_CEB_ASSERT				(0x7824)
#define OTP_CEB_DEASSERT			(0x782C)
#define OTP_READ_WIDTH				(0x7834)
#define OTP_READEN_ASSERT			(0x783C)
#define OTP_READEN_DEASSERT			(0x7844)
#define OTP_DOUT_OFFSET				(0x784C)
#define OTP_READ_OFFSET				(0x7854)
#define OTP_BASE_ADDRESS_H			(0x7858)
#define OTP_BASE_ADDRESS_L			(0x785C)
#define OTP_READ_DATA_ORDER			(0x7880)
//** OTP_CON_TOP, base address = 0x101E0000 **//
#define OTP_CON_CONTROL				(0x0010)

#endif  // __REGS_HDCP_H__
