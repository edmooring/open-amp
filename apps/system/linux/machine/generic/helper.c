
/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 *
 * Copyright (c) 2015 Xilinx, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>
#include <stdio.h>
#include <metal/sys.h>
#include <metal/shmem.h>
#include <metal/device.h>
#include <metal/io.h>
#include <openamp/openamp.h>

#define SHM_FILE "openamp.shm"
#define SHM_SIZE 0x800000
#define SHARED_BUF_PA   0x10000UL
#define SHARED_BUF_SIZE 0x40000

static struct metal_io_region *io;

static struct OPENAMP_config_data oa_data = {
	.shm_size = SHM_SIZE,
	.vring_buff_address = SHARED_BUF_PA,
	.vring_size = SHARED_BUF_SIZE,
};

int init_system()
{
	struct metal_init_params metal_param = METAL_INIT_DEFAULTS;
	int ret;
	metal_param.log_level = LOG_DEBUG;
	metal_init(&metal_param);

	ret = metal_shmem_open(SHM_FILE, SHM_SIZE, &io);
	if (ret)
		return ret;
	oa_data.shm_start_address = io->virt;

	return 0;
}

void cleanup_system()
{
	metal_finish();
}

struct OPENAMP_config_data *OPENAMP_get_config(void)
{
	return &oa_data;
}
