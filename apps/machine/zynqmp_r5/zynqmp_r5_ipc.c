/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2021 Xilinx, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       zynqmp_r5_a53_irq.c
 *
 * DESCRIPTION
 *
 *       This file defines Xilinx ZynqMP R5 to A53 platform specific 
 *       interprocessor communication  implementation.
 *
 **************************************************************************/

#include <metal/atomic.h>
#include <metal/assert.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include "platform_info.h"
#ifndef RPMSG_NO_IPI
/* IPI REGs OFFSET */
#define IPI_TRIG_OFFSET          0x00000000    /* IPI trigger register offset */
#define IPI_OBS_OFFSET           0x00000004    /* IPI observation register offset */
#define IPI_ISR_OFFSET           0x00000010    /* IPI interrupt status register offset */
#define IPI_IMR_OFFSET           0x00000014    /* IPI interrupt mask register offset */
#define IPI_IER_OFFSET           0x00000018    /* IPI interrupt enable register offset */
#define IPI_IDR_OFFSET           0x0000001C    /* IPI interrupt disable register offset */

atomic_int kicked;

static metal_phys_addr_t kick_phys_addr = POLL_BASE_ADDR;
static struct metal_device kick_dev =
{
	.name = "kick",
	.bus = NULL,
	.num_regions = 1,
	.regions = {
			{
			.virt = (void *)POLL_BASE_ADDR,
			.physmap = &kick_phys_addr,
                        .size = 0x1000,
                        .page_shift = -1UL,
                        .page_mask = -1UL,
                        .mem_flags = DEVICE_NONSHARED | PRIV_RW_USER_RW,
                        .ops = {NULL},
			},
		   },
	.node = {NULL},
	.irq_num = 1,
	.irq_info = (void *)IPI_IRQ_VECT_ID,
};

static struct metal_io_region *kick_io;

static int zynqmp_r5_a53_proc_irq_handler(int vect_id, void *data)
{
	unsigned int ipi_intr_status;

	(void)vect_id;
	(void)data;
	ipi_intr_status = (unsigned int)metal_io_read32(kick_io,
							IPI_ISR_OFFSET);
	if (ipi_intr_status & IPI_CHN_BITMASK) {
		atomic_flag_clear(&kicked);
		metal_io_write32(kick_io, IPI_ISR_OFFSET,
				 IPI_CHN_BITMASK);
		return METAL_IRQ_HANDLED;
	}
	return METAL_IRQ_NOT_HANDLED;
}
#endif /* !RPMSG_NO_IPI */

int
metal_ipc_init(void)
{
	unsigned int irq_vect;
	int status;
	struct metal_device *dev = &kick_dev;

	status = metal_register_generic_device(&kick_dev);
        if (status != 0) {
                return status;
        }
        status = metal_device_open("generic", "kick", &dev);
        if (status != 0) {
                return status;
        }
	kick_io = metal_device_io_region(dev, 0);

#ifndef RPMSG_NO_IPI
	atomic_store(&kicked, 1);
	/* Register interrupt handler and enable interrupt */
	irq_vect = IPI_IRQ_VECT_ID;
	status = metal_irq_register(irq_vect, zynqmp_r5_a53_proc_irq_handler, 0);
        if (status != 0) {
                return status;
        }
	kick_io = metal_device_io_region(dev, 0);
xil_printf("kick_io = 0x%p\r\n", kick_io);


	metal_irq_enable(irq_vect);

	metal_io_write32(kick_io, IPI_IER_OFFSET, IPI_CHN_BITMASK);

#else
	(void)irq_vect;
	metal_io_write32(kick_io, 0, !POLL_STOP);
#endif /* !RPMSG_NO_IPI */

	return 0;
}

int metal_ipc_notify(void *rproc, uint32_t id)
{

	(void)id;
	(void)rproc;

#ifdef RPMSG_NO_IPI
	metal_io_write32(kick_io, 0, POLL_STOP);
#else
	metal_io_write32(kick_io, IPI_TRIG_OFFSET, IPI_CHN_BITMASK);
#endif /* RPMSG_NO_IPI */
	return 0;
}

