/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/phy/phy.h>
#include <media/videobuf2-core.h>


#include "am-cam-config.h"
#include "am-cam-framemgr.h"
#include "am-cam-device.h"
#include "am-cam-video.h"
#include "fimc-is-csi.h"
#include "fimc-is-hw-csi-v4_0.h"

struct am_cam_buf am_cam_buf_list[AM_CAM_MAX_FRAMES];

void csi_get_timestamp(struct timeval *timestamp)
{
	struct timespec cur_time;

	ktime_get_ts(&cur_time);

	timestamp->tv_sec = cur_time.tv_sec;
	timestamp->tv_usec = cur_time.tv_nsec/1000;
}

void csi_hw_s_cfg(struct fimc_is_device_csi *csi)
{
	volatile unsigned long addr_csi = (unsigned long)csi->regs;
	uint32_t val;
	uint32_t channel = 0;
	uint32_t parallel = 0;
	uint32_t width = csi->cfg.width;
	uint32_t height = csi->cfg.height;

	val = *(unsigned int *)(addr_csi + csi_regs[CSIS_R_ISP_CONFIG_CH0 + (channel * 3)].sfr_offset);

	val = update_field(val, &csi_fields[CSIS_F_VIRTUAL_CHANNEL], 0);
	val = update_field(val, &csi_fields[CSIS_F_DATAFORMAT], HW_FORMAT_RGB888);
	/* val = update_field(val, &csi_fields[CSIS_F_DATAFORMAT], HW_FORMAT_YUV422_8BIT); */
	val = update_field(val, &csi_fields[CSIS_F_PARALLEL], parallel);
	val = update_field(val, &csi_fields[CSIS_F_DOUBLE_CMPNT], CSIS_REG_DOUBLE_CLK_EN);

	*(unsigned int *)(addr_csi + csi_regs[CSIS_R_ISP_CONFIG_CH0 + (channel * 3)].sfr_offset) = val;

	val = *(unsigned int *)(addr_csi + csi_regs[CSIS_R_ISP_RESOL_CH0 + (channel * 3)].sfr_offset);

	val = update_field(val, &csi_fields[CSIS_F_VRESOL], height);
	val = update_field(val, &csi_fields[CSIS_F_HRESOL], width);            

	*(unsigned int *)(addr_csi + csi_regs[CSIS_R_ISP_RESOL_CH0 + (channel * 3)].sfr_offset) = val;
}

void csi_s_dma_buffer(struct fimc_is_device_csi *csi)
{
	int vc;
	unsigned long addr;
	volatile unsigned long addr_csi = (unsigned long)csi->regs;

	/* update all VC */
	for (vc = 0; vc < CSI_VIRTUAL_CH_MAX; vc++) {
		unsigned int val = 0;

		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL + (vc * 15)].sfr_offset;
		val = *(unsigned int*)(addr);

		val = update_field(val, &csi_fields[CSIS_F_DMA_N_DISABLE], 1);
		val = update_field(val, &csi_fields[CSIS_F_DMA_N_UPDT_PTR_EN], 0);
		*(unsigned int*)(addr_csi + csi_regs[CSIS_R_DMA0_CTRL + (vc * 15)].sfr_offset) = val;
	}

	return;
}

static inline void csi_frame_start_inline(struct fimc_is_device_csi *csi)
{
	/* frame start interrupt */
	/* dev->sw_checker = EXPECT_FRAME_END; */
	/* atomic_add(&dev->fcount, 1); */

	/* csi_s_dma_buffer(csi); */
	/* am_cam_info("start interrupt\n"); */
}

static inline void csi_frame_end_inline(struct fimc_is_device_csi *csi)
{
	/* am_cam_info("end interrupt\n"); */
}

void csi_dma_set_buffer_start(struct v4l2_subdev *subdev)
{
	unsigned long flags;
	struct fimc_is_device_csi *csi;
	struct am_cam_framemgr *framemgr;
	struct am_cam_frame *frame_qbuf;

	BUG_ON(!subdev);

	am_cam_devi("%s(): \n", __func__);

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL \n");
		BUG();
	}

	framemgr = csi->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_e(%d)\n", __LINE__);
	am_cam_frame_print_request_list(framemgr);
#endif

	/* buffer set */
	am_cam_frame_g_request(framemgr, &frame_qbuf);
	if (frame_qbuf) {
		int i = 0;
		unsigned int vc = 0;
		unsigned long addr;
		unsigned long buf_addr;
		volatile unsigned long addr_csi;
		volatile uint32_t src = 0;
		addr_csi = (unsigned long)csi->regs;

		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src &= ~(0x7 << 2);
		src |= (0x1 << 1);
		writel(src, (void *)addr);

		buf_addr = (unsigned long)frame_qbuf->dvaddr_buffer[0];
		frame_qbuf->state = AM_CAM_FRAME_STATE_REQUEST;
		am_cam_frame_trans_req_to_pro(framemgr, frame_qbuf);

		am_cam_devi("q(%d)(0x%lx)(0x%x) (%p)\n",
			frame_qbuf->index,
			buf_addr,
			frame_qbuf->dvaddr_buffer[0],
			frame_qbuf);

		for (i = 0; i < 8; i++) {
			addr = addr_csi + csi_regs[CSIS_R_DMA0_ADDR1 + (vc * 15) + i].sfr_offset;
			buf_addr = (unsigned long)frame_qbuf->dvaddr_buffer[0];
			writel(buf_addr, (void *)addr);
		}

		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src &= ~(0x1 << 0);
		writel(src, (void *)addr);
	} else {
		unsigned long addr;
		volatile unsigned long addr_csi;
		volatile uint32_t src = 0;
		addr_csi = (unsigned long)csi->regs;

		/* Disable Next Frame's DMA out */
		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src |= (0x1 << 0);
		writel(src, (void *)addr);
		/* am_cam_err("f N2 \n"); */
		/* BUG(); */
	}

#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_x(%d)\n", __LINE__);
#endif
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}

static void tasklet_csis_str(unsigned long data)
{
	unsigned long flags;
	struct fimc_is_device_csi *csi;
	struct am_cam_framemgr *framemgr;
	struct am_cam_frame *frame_qbuf;
	struct v4l2_subdev *subdev;

	am_cam_devi("%s(): \n", __func__);

	subdev = (struct v4l2_subdev *)data;
	csi = v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL \n");
		BUG();
	}

	framemgr = csi->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_e(%d)\n", __LINE__);
	am_cam_frame_print_request_list(framemgr);
#endif

	if (framemgr->req_cnt < 2) {
		unsigned long addr;
		volatile unsigned long addr_csi;
		volatile uint32_t src = 0;
		addr_csi = (unsigned long)csi->regs;

		/* Disable Next Frame's DMA out */
		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src |= (0x1 << 0);
		writel(src, (void *)addr);

		am_cam_devi("req%d\n", framemgr->req_cnt);
		/* BUG(); */
		framemgr_x_barrier_irqr(framemgr, 0, flags);

		return;
	}

	/* buffer set */
	am_cam_frame_g_request(framemgr, &frame_qbuf);
	if (frame_qbuf) {
		int i = 0;
		unsigned int vc = 0;
		unsigned long addr;
		unsigned long buf_addr;
		volatile unsigned long addr_csi;
		volatile uint32_t src = 0;
		addr_csi = (unsigned long)csi->regs;

		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src &= ~(0x7 << 2);
		src |= (0x1 << 1);
		writel(src, (void *)addr);

		buf_addr = (unsigned long)frame_qbuf->dvaddr_buffer[0];
		frame_qbuf->state = AM_CAM_FRAME_STATE_REQUEST;
		am_cam_frame_trans_req_to_pro(framemgr, frame_qbuf);

		am_cam_devi("q(%d)(0x%lx)(0x%x) (%p)\n",
			frame_qbuf->index,
			buf_addr,
			frame_qbuf->dvaddr_buffer[0],
			frame_qbuf);

		for (i = 0; i < 8; i++) {
			addr = addr_csi + csi_regs[CSIS_R_DMA0_ADDR1 + (vc * 15) + i].sfr_offset;
			buf_addr = (unsigned long)frame_qbuf->dvaddr_buffer[0];
			writel(buf_addr, (void *)addr);
		}

		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src &= ~(0x1 << 0);
		writel(src, (void *)addr);
	} else {
		unsigned long addr;
		volatile unsigned long addr_csi;
		volatile uint32_t src = 0;
		addr_csi = (unsigned long)csi->regs;

		/* Disable Next Frame's DMA out */
		addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
		src = readl((void *)addr);
		src |= (0x1 << 0);
		writel(src, (void *)addr);
		am_cam_devi("f N2 \n");
		/* BUG(); */
	}

#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_x(%d)\n", __LINE__);
#endif
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}

#ifdef USE_END_TASKLET
static void tasklet_csis_end(unsigned long data)
{
	unsigned long flags;
	struct fimc_is_device_csi *csi;
	struct am_cam_framemgr *framemgr;
	struct am_cam_frame *frame_dqbuf;
	struct am_cam_frame *frame_done;
	struct v4l2_subdev *subdev;

	am_cam_devi("%s(): \n", __func__);

	subdev = (struct v4l2_subdev *)data;
	csi = v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL \n");
		BUG();
	}

	framemgr = csi->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_e(%d)\n", __LINE__);
	am_cam_frame_print_process_list(framemgr);
#endif

	/* buffer done */
	am_cam_frame_g_process(framemgr, &frame_dqbuf);

	if (frame_dqbuf) {
		struct am_cam_buf *am_cam_vb;
		int vb_idx = 0;
		am_cam_vb = &am_cam_buf_list[frame_dqbuf->index];
		frame_done = frame_dqbuf;

		frame_dqbuf->state = AM_CAM_FRAME_STATE_PROCESS;
		am_cam_frame_trans_pro_to_com(framemgr, frame_dqbuf);
		vb_idx = am_cam_vb->vb2_buf->v4l2_buf.index;

		am_cam_devi("dq(%d) (%p)\n",
			frame_dqbuf->index,
			frame_dqbuf);

		csi_get_timestamp(&am_cam_vb->vb2_buf->v4l2_buf.timestamp);

		vb2_buffer_done(am_cam_buf_list[vb_idx].vb2_buf,
				VB2_BUF_STATE_DONE);

#ifdef MAP_KVADDR
		am_cam_devi("%s(): kvaddr(%llx) \n", __func__,
				(long long unsigned int)am_cam_vb->kvaddr[0]);
#endif
	} else {
		/* am_cam_err("f N1 \n"); */
		/* BUG(); */
	}

#ifdef DBG_STM_SPINLOCK
	am_cam_devi("barr x(%d)\n", __LINE__);
#endif
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}
#endif

static irqreturn_t csi_isr(int irq, void *data)
{
	struct fimc_is_device_csi *csi = data;
	volatile unsigned long addr = ((ulong)csi->regs + csi_regs[CSIS_R_CSIS_INT_SRC0].sfr_offset);
	volatile unsigned long addr2 = ((ulong)csi->regs + csi_regs[CSIS_R_CSIS_INT_SRC1].sfr_offset);
	unsigned int status;
	unsigned int dma_status;
	volatile uint32_t src = 0;
	uint32_t frame_start;
	uint32_t frame_end;
	uint32_t dma_start;
	uint32_t dma_end;
	uint32_t err;

	src = readl((void *)addr);
	status = src;
	writel(src, (void *)addr);

	src = readl((void *)addr2);
	dma_status = src;
	writel(src, (void *)addr2);

	/* TODO: Currently only consider VC CH0 */
	frame_start = status & (1 << 24);
	frame_end = status & (1 << 20);
	err = status & (0xffff);

	if (err) {
		am_cam_devi("csi intr status %x\n", err);
		return IRQ_HANDLED;
	}

	dma_start = dma_status & (1 << 4);
	dma_end = dma_status & (1 << 8);
	/* am_cam_devi("f(0x%x) d(0x%x)\n", status, dma_status); */

	/* err = dma_status & (0xffff); */

	if (frame_start)
		tasklet_schedule(&csi->tasklet_csis_str);
		/* start or end ?? */
#ifdef USE_END_TASKLET
	if (dma_end)
		tasklet_schedule(&csi->tasklet_csis_end);
#endif

	if (frame_start) {
		csi_frame_start_inline(csi);
	} 

	if (frame_end) {
		csi_frame_end_inline(csi);
	} 

	return IRQ_HANDLED;
}

/* value : module enum */
static int csi_init(struct v4l2_subdev *subdev, u32 value)
{
	return 0;
}

int csi_s_power(struct v4l2_subdev *subdev, int on)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;
	volatile unsigned long addr = 0;
	int offset = 0;
	int val = 0;

	BUG_ON(!subdev);

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	/* HACK : */
	/* PHY pwr on */
	addr = (unsigned long) ioremap_nocache(EXYNOS_REG_PMU_BASE + 0x700, 0x2000);
	// MIPI_PHY_M4S4_CONTROL 0x105C070C
	offset = 0x0C;
	val = *(unsigned int *)(addr + offset);
	val |= 0x1;
	*(unsigned int *)(addr + offset) = val;

	// MIPI_PHY_M4S0_CONTROL 0x105C0710
	// MIPI_PHY_M0S4_CONTROL 0x105C0714
	// MIPI_PHY_M0S1_CONTROL 0x105C0718

	iounmap((void*)addr);

	/* Reset */
	addr = (unsigned long) ioremap_nocache(EXYNOS_REG_SYSREG_CAM0 + 0x1000, 0x2000);
	// CAM0_MIPI_PHY_CON 0x144F1040 [0]-M0S2, [1]-M4S4
	offset = 0x40;
	val = *(unsigned int *)(addr + offset);
	val |= 0x1 << 1;
	*(unsigned int *)(addr + offset) = val;

	iounmap((void*)addr);

	// CAM1_MIPI_PHY_CON 0x145F1050 [0]-M0S1, [1]-M0S4

//p_err:
	am_cam_devi("%s(%d):%d\n", __func__, on, ret);
	return ret;
}

static const struct v4l2_subdev_core_ops core_ops = {
	.init = csi_init,
	.s_power = csi_s_power
};

static int csi_stream_on(struct v4l2_subdev *subdev,
	struct fimc_is_device_csi *csi)
{
	int ret = 0;
	unsigned long addr;
	uint32_t settle = csi->cfg.settle;
	volatile unsigned long addr_csi = (unsigned long)csi->regs;

	tasklet_init(&csi->tasklet_csis_str, tasklet_csis_str, (unsigned long)subdev);
#ifdef USE_END_TASKLET
	tasklet_init(&csi->tasklet_csis_end, tasklet_csis_end, (unsigned long)subdev);
#endif

	ret = request_irq(csi->irq, csi_isr, IRQF_GIC_MULTI_TARGET, "mipi-csi0", csi);
	if (ret) {
		am_cam_err("request_irq(IRQ_MIPICSI %d) is fail(%d)" , csi->irq, ret);
		goto p_err;
	}

	/* reset */
	addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_SW_RESET], 1);

	/* DMA buffer setting: enable or disable for all VC */
	csi_s_dma_buffer(csi);

	/* settle */
	addr = addr_csi + csi_regs[CSIS_R_DPHY_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_HSSETTLE], settle);

	/* lane */
	{
		int lane = 0;
		int deskew = 0;
		u32 val = 0;
		addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
		val = *(unsigned int *)(addr);

		val = update_field(val, &csi_fields[CSIS_F_LANE_NUMBER], csi->cfg.lanes);
		val = update_field(val, &csi_fields[CSIS_F_DESKEW_ENABLE], deskew);

		*(unsigned int *)(addr) = val;

		switch (csi->cfg.lanes) {                                               
		case CSI_DATA_LANES_1:                                         
			/* lane 0 */                                           
			lane = (0x1);                                          
			break;                                                 
		case CSI_DATA_LANES_2:                                         
			/* lane 0 + lane 1 */                                  
			lane = (0x3);                                          
			break;                                                 
		case CSI_DATA_LANES_3:                                         
			/* lane 0 + lane 1 + lane 2 */                         
			lane = (0x7);                                          
			break;                                                 
		case CSI_DATA_LANES_4:                                         
			/* lane 0 + lane 1 + lane 2 + lane 3 */                
			lane = (0xF);                                          
			break;                                                 
		default:                                                       
			am_cam_err("lanes is invalid(%d)", csi->cfg.lanes);                    
			lane = (0xF);                                          
			break;                                                 
		}                                                              

		addr = addr_csi + csi_regs[CSIS_R_DPHY_CMN_CTRL].sfr_offset;
		set_field(addr, &csi_fields[CSIS_F_ENABLE_DAT], lane);
	}

	/* CSIS_CTRL_INTERLEAVE_MODE */
	addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_INTERLEAVE_MODE], CSI_MODE_VC_ONLY);

	csi_hw_s_cfg(csi);

	// TODO: interrupt unmask
	{
		u32 otf_msk, dma_msk;

		/* base interrupt setting */
		otf_msk = CSIS_IRQ_MASK0;
		dma_msk = CSIS_IRQ_MASK1;

//		dma_msk = update_field(dma_msk, &csi_fields[CSIS_F_MSK_LINE_END], 0xF);
//		otf_msk = update_field(otf_msk, &csi_fields[CSIS_F_FRAMEEND], 0x0);

		*(unsigned int *)(addr_csi + csi_regs[CSIS_R_CSIS_INT_MSK0].sfr_offset) = otf_msk;
		*(unsigned int *)(addr_csi + csi_regs[CSIS_R_CSIS_INT_MSK1].sfr_offset) = dma_msk;
	}

	/* int csi_hw_enable(u32 __iomem *base_reg) */
	/* update shadow */
	addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_UPDATE_SHADOW], 0xF);

	/* DPHY on */
	addr = addr_csi + csi_regs[CSIS_R_DPHY_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_ENABLE_CLK], 1);

	/* csi enable */
	addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_CSI_EN], 1);

#if 0
	{
		
		volatile uint32_t value = 0;
		u32 offset = 0;

		while(offset <= 0xB8) {
			value  = readl((void *)(addr_csi + offset));
			pr_info("CSI[%X] : %X\n", offset, value);
			offset += 4;
		}
	}
#endif

p_err:
	am_cam_devi("%s(%d):%d\n", __func__, csi->irq, ret);
	return ret;
}

static int csi_stream_off(struct v4l2_subdev *subdev,
	struct fimc_is_device_csi *csi)
{
	int ret = 0;
	unsigned long flags;
	struct am_cam_frame *frame, *temp;
	/* u32 __iomem *base_reg; */

	unsigned long addr;
	volatile unsigned long addr_csi = (unsigned long)csi->regs;

	BUG_ON(!csi);

//	if (!test_bit(CSIS_START_STREAM, &csi->state)) {
//		am_cam_err("[CSI] already stop", csi);
//		ret = -EINVAL;
//		goto perr;
//	}

//	base_reg = csi->regs;

	/* csi_hw_s_irq_msk(base_reg, false); */
	// TODO: interrupt unmask
	{
		u32 otf_msk, dma_msk;

		/* base interrupt setting */
		otf_msk = 0;
		dma_msk = 0;

		*(unsigned int *)(addr_csi + csi_regs[CSIS_R_CSIS_INT_MSK0].sfr_offset) = otf_msk;
		*(unsigned int *)(addr_csi + csi_regs[CSIS_R_CSIS_INT_MSK1].sfr_offset) = dma_msk;
	}

	/* csi_hw_disable(base_reg); */
{
	/* DPHY off */
	addr = addr_csi + csi_regs[CSIS_R_DPHY_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_ENABLE_CLK], 0);
	addr = addr_csi + csi_regs[CSIS_R_DPHY_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_ENABLE_DAT], 0);

	/* csi enable */
	addr = addr_csi + csi_regs[CSIS_R_CSIS_CMN_CTRL].sfr_offset;
	set_field(addr, &csi_fields[CSIS_F_CSI_EN], 0);
}

//	if (!test_bit(CSIS_DMA_ENABLE, &csi->state))
//		goto p_dma_skip;

	tasklet_kill(&csi->tasklet_csis_str);
#ifdef USE_END_TASKLET
	tasklet_kill(&csi->tasklet_csis_end);
#endif

{
	struct am_cam_framemgr *framemgr = csi->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	list_for_each_entry_safe(frame, temp, &framemgr->req_list, list) {
		struct am_cam_buf *am_cam_vb;
		int vb_idx = 0;
		am_cam_vb = &am_cam_buf_list[frame->index];

		am_cam_devi("req frame%d (count : %d)\n", frame->index, framemgr->req_cnt);

//		frame_dqbuf->state = AM_CAM_FRAME_STATE_PROCESS;
//		am_cam_frame_trans_pro_to_com(framemgr, frame_dqbuf);
		
		vb_idx = am_cam_vb->vb2_buf->v4l2_buf.index;
		vb2_buffer_done(am_cam_buf_list[vb_idx].vb2_buf,
				VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->pro_list, list) {
		struct am_cam_buf *am_cam_vb;
		int vb_idx = 0;
		am_cam_vb = &am_cam_buf_list[frame->index];

		am_cam_devi("pro frame%d (count : %d)\n", frame->index, framemgr->pro_cnt);

		vb_idx = am_cam_vb->vb2_buf->v4l2_buf.index;
		vb2_buffer_done(am_cam_buf_list[vb_idx].vb2_buf,
				VB2_BUF_STATE_ERROR);
	}

	framemgr_x_barrier_irqr(framemgr, 0, flags);

	am_cam_frame_flush(framemgr);

}

	free_irq(csi->irq, csi);

	am_cam_devi("%s():%d\n", __func__, ret);

//p_dma_skip:
//	clear_bit(CSIS_START_STREAM, &csi->state);
//p_err:
	return ret;
}

int csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;

	BUG_ON(!subdev);

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	if (enable) {
		ret = csi_stream_on(subdev, csi);
		if (ret) {
			am_cam_err("[CSI] csi_stream_on is fail(%d)", ret);
			goto p_err;
		}
	} else {
		ret = csi_stream_off(subdev, csi);
		if (ret) {
			am_cam_err("[CSI] csi_stream_off is fail(%d)", ret);
			goto p_err;
		}
	}

p_err:
	return 0;
}

static int csi_s_param(struct v4l2_subdev *subdev, struct v4l2_streamparm *param)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;
	struct v4l2_captureparm *cp;
	struct v4l2_fract *tpf;

	BUG_ON(!subdev);
	BUG_ON(!param);

	cp = &param->parm.capture;
	tpf = &cp->timeperframe;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	csi->image.framerate = tpf->denominator / tpf->numerator;

	return ret;
}

static int csi_s_format(struct v4l2_subdev *subdev,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;

	BUG_ON(!subdev);
	BUG_ON(!fmt);

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	csi->image.window.offs_h = 0;
	csi->image.window.offs_v = 0;
	csi->image.window.width = fmt->format.width;
	csi->image.window.height = fmt->format.height;
	csi->image.window.o_width = fmt->format.width;
	csi->image.window.o_height = fmt->format.height;
	csi->image.format.pixelformat = fmt->format.code;
	csi->image.format.field = fmt->format.field;

	am_cam_err("%s(%dx%d, %X)\n", __func__, fmt->format.width, fmt->format.height, fmt->format.code);
	return ret;
}

int csi_s_buffer(struct v4l2_subdev *subdev, void *buf, unsigned int *size)
{
	int ret = 0;
	unsigned long addr;
	struct fimc_is_device_csi *csi;
	volatile unsigned long addr_csi;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	addr_csi = (unsigned long)csi->regs;

	addr = addr_csi + csi_regs[CSIS_R_DMA0_CTRL].sfr_offset;
	writel(0x0, (void *)addr);

	addr = addr_csi + csi_regs[CSIS_R_DMA1_CTRL].sfr_offset;
	writel(0x0, (void *)addr);

	addr = addr_csi + csi_regs[CSIS_R_DMA2_CTRL].sfr_offset;
	writel(0x0, (void *)addr);

	addr = addr_csi + csi_regs[CSIS_R_DMA3_CTRL].sfr_offset;
	writel(0x0, (void *)addr);

	am_cam_devi("%s():%d\n", __func__, ret);
	return ret;
}

int csi_s_buffer_addr(struct v4l2_subdev *subdev, void *buf, unsigned int *size)
{
	struct fimc_is_device_csi *csi;
	struct am_cam_buf *am_cam_vb;

	unsigned long flags;
	struct am_cam_frame *frame;
	struct am_cam_framemgr *framemgr;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	am_cam_vb = (struct am_cam_buf *)buf;
	framemgr = csi->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_e(%d)\n", __LINE__);
#endif

	frame = &framemgr->frame[am_cam_vb->vb2_buf->v4l2_buf.index];
	am_cam_devi("f_s(%d %d %d)\n",
			am_cam_vb->vb2_buf->v4l2_buf.index,
			frame->index, frame->state);

	frame->dvaddr_buffer[0] = am_cam_vb->dvaddr[0];
	/* to debug #define MAP_KVADDR */
	frame->kvaddr_buffer[0] = (ulong)am_cam_vb->kvaddr[0];

	am_cam_buf_list[am_cam_vb->vb2_buf->v4l2_buf.index].dvaddr[0] =
		am_cam_vb->dvaddr[0];
	am_cam_buf_list[am_cam_vb->vb2_buf->v4l2_buf.index].kvaddr[0] =
		am_cam_vb->kvaddr[0];
	am_cam_buf_list[am_cam_vb->vb2_buf->v4l2_buf.index].vb2_buf =
		am_cam_vb->vb2_buf;

	am_cam_devi("sb(%d)(%p)\n", am_cam_vb->vb2_buf->v4l2_buf.index, frame);

	am_cam_frame_search_g_free(framemgr, &frame);

	am_cam_frame_s_request(framemgr, frame);

#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_x(%d)\n", __LINE__);
#endif
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return 0;
}

int csi_s_buffer_finish(struct v4l2_subdev *subdev,
		void *buf,
		unsigned int *size)
{
	struct fimc_is_device_csi *csi;

	unsigned long flags;
	struct am_cam_frame *frame;
	struct am_cam_framemgr *framemgr;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}
	framemgr = csi->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_e(%d)\n", __LINE__);
#endif

	am_cam_frame_g_complete(framemgr, &frame);
	if (frame) {
		frame->state = AM_CAM_FRAME_STATE_COMPLETE;
		am_cam_frame_trans_com_to_fre(framemgr, frame);
	} else {
		am_cam_devi("frame is NULL, stream off is already called.\n");
	}

#ifdef DBG_STM_SPINLOCK
	am_cam_devi("b_x(%d)\n", __LINE__);
#endif
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return 0;
}
static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = csi_s_stream,
	.s_parm = csi_s_param,
	.s_rx_buffer = csi_s_buffer,
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt = csi_s_format
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops
};

int fimc_is_csi_probe(void *parent)
{
	int ret = 0;
	struct v4l2_subdev *subdev;
	struct fimc_is_device_csi *csi;
	struct am_cam_device *device;
	struct platform_device *pdev;
	struct device *dev;
	struct resource *res;
	void __iomem *iomem;
	int irq;

	BUG_ON(!parent);

	device = parent;
	subdev = &device->subdev_csi;
	pdev = device->pdev;
	dev = &pdev->dev;

	csi = kzalloc(sizeof(struct fimc_is_device_csi), GFP_KERNEL);
	if (!csi) {
		probe_err("csi is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	csi->cfg.width = 1920;
	csi->cfg.height = 1080;
	csi->cfg.lanes = CSI_DATA_LANES_2;
	csi->cfg.settle = 14;
	csi->cfg.framerate = 30;
	csi->cfg.mode = 0;
	csi->cfg.mipi_speed = 0;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		probe_err("platform_get_irq(0) is fail(%d)", irq);
		ret = -EINVAL;
		goto p_err;
	}

	csi->irq = irq;
	am_cam_info("[CSI] irq : %d\n", irq);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		probe_err("platform_get_resource(0) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(0) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	csi->regs = iomem;
	csi->regs_size = resource_size(res);
	am_cam_info("[CSI] reg : %p\n", iomem);

/* HACK */
/*
	csi->phy = devm_phy_get(&pdev->dev, "csis_dphy");
	if (IS_ERR(csi->phy))
		return PTR_ERR(csi->phy);
*/
	/* pointer to me from device sensor */
	//csi->subdev = &device->subdev;

	v4l2_subdev_init(subdev, &subdev_ops);
	v4l2_set_subdevdata(subdev, csi);
	v4l2_set_subdev_hostdata(subdev, device);
	snprintf(subdev->name, V4L2_SUBDEV_NAME_SIZE, "csi-subdev.0");
	ret = v4l2_device_register_subdev(&device->v4l2_dev, subdev);
	if (ret) {
		am_cam_err("v4l2_device_register_subdev is fail(%d)", ret);
		goto p_err;
	}

p_err:
	am_cam_info("%s(%d)\n", __func__, ret);
	return ret;
}

int fimc_is_csi_open(struct v4l2_subdev *subdev,
	struct am_cam_framemgr *framemgr)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;

	BUG_ON(!subdev);

	csi = v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	memset(&csi->image, 0, sizeof(struct fimc_is_image));

	csi->framemgr = framemgr;

p_err:
	return ret;
}

int fimc_is_csi_close(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_device_csi *csi;

	BUG_ON(!subdev);

	csi = v4l2_get_subdevdata(subdev);
	if (!csi) {
		am_cam_err("csi is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	if (csi->framemgr != NULL) {
		am_cam_frame_deinit(csi->framemgr);
		kfree(csi->framemgr);
	}
p_err:
	return ret;
}
