/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2016 Xilinx, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       platform_info.c
 *
 * DESCRIPTION
 *
 *       This file implements APIs to get platform specific
 *       information for OpenAMP.
 *
 **************************************************************************/

#include <metal/alloc.h>
#include <metal/atomic.h>
#include <metal/io.h>
#include <metal/irq.h>
#include <metal/shmem.h>
#include <metal/utilities.h>
#include <openamp/openamp.h>
#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include "rsc_table.h"

#define IPI_CHAN_NUMS 2
#define IPI_CHAN_SEND 0
#define IPI_CHAN_RECV 1
#define UNIX_PREFIX "unix:"
#define UNIXS_PREFIX "unixs:"

#define RSC_MEM_PA  0x0UL
#define SHARED_BUF_PA   0x10000UL
#define SHARED_BUF_SIZE 0x40000UL

#define _rproc_wait() metal_cpu_yield()

struct vring_ipi_info {
	/* Socket file path */
	const char *path;
	int fd;
	atomic_int sync;
};

struct remoteproc_priv {
	const char *shm_file;
	int shm_size;
	struct metal_io_region *shm_old_io;
	struct metal_io_region shm_new_io;
	struct remoteproc_mem shm;
	struct vring_ipi_info ipi;
};

static struct remoteproc_priv rproc_priv_table [] = {
	{
		.shm_file = "openamp.shm",
		.shm_size = 0x80000,
		.ipi = {
			.path = "unixs:/tmp/openamp.event.0",
		},
	},
	{
		.shm_file = "openamp.shm",
		.shm_size = 0x80000,
		.ipi = {
			.path = "unix:/tmp/openamp.event.0",
		},
	},
};

atomic_int *kicked;

static int myrole;

/* External functions */
extern int init_system(void);
extern void cleanup_system(void);


static int sk_unix_client(const char *descr)
{
	struct sockaddr_un addr;
	int fd;

	fd = socket(AF_UNIX, SOCK_STREAM, 0);

	memset(&addr, 0, sizeof addr);
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, descr + strlen(UNIX_PREFIX),
		sizeof addr.sun_path);
	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) >= 0) {
		printf("connected to %s\r\n", descr + strlen(UNIX_PREFIX));
		return fd;
	}

	close(fd);
	return -1;
}

static int sk_unix_server(const char *descr)
{
	struct sockaddr_un addr;
	int fd, nfd;

	fd = socket(AF_UNIX, SOCK_STREAM, 0);

	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, descr + strlen(UNIXS_PREFIX),
		sizeof addr.sun_path);
	unlink(addr.sun_path);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		goto fail;
	}

	listen(fd, 5);
	printf("Waiting for connection on %s\r\n", addr.sun_path);
	nfd = accept(fd, NULL, NULL);
	close(fd);
	return nfd;
fail:
	close(fd);
	return -1;
}

static inline int is_sk_unix_server(const char *descr)
{
	if (memcmp(UNIXS_PREFIX, descr, strlen(UNIXS_PREFIX)))
		return 0;
	else
		return 1;
}

static int event_open(const char *descr)
{
	int fd = -1;
	int i;

	if (descr == NULL) {
		return fd;
	}

	if (!is_sk_unix_server(descr)) {
		/* UNIX client.  Retry to connect a few times to give the peer
		 *  a chance to setup.  */
		for (i = 0; i < 100 && fd == -1; i++) {
			fd = sk_unix_client(descr);
			if (fd == -1)
				usleep(i * 10 * 1000);
		}
	} else {
		/* UNIX server. */
		fd = sk_unix_server(descr);
	}
	printf("Open IPI: %s\r\n", descr);
	return fd;
}

static int linux_proc_irq_handler(int vect_id, void *data)
{
	char dummy_buf[32];
	struct vring_ipi_info *ipi = data;

	read(vect_id, dummy_buf, sizeof(dummy_buf));
	atomic_flag_clear(&ipi->sync);
	return 0;
}

int metal_ipc_init(int role)
{
	struct remoteproc_priv *prproc;
	struct metal_io_region *io;
	struct remoteproc_mem *shm;
	struct vring_ipi_info *ipi;
	int ret;

	if (role > 2)
		return -EINVAL;
	prproc = &rproc_priv_table[role];
	myrole = role;
#if 0
	/* Create shared memory io */
	ret = metal_shmem_open(prproc->shm_file, prproc->shm_size, &io);
	if (ret) {
		printf("Failed to init ipc, failed to open shm %s ret = %d.\r\n",
		       prproc->shm_file, ret);
		return ret;
	}
	prproc->shm_old_io = io;
	shm = &prproc->shm;
	shm->pa = 0;
	shm->da = 0;
	shm->size = prproc->shm_size;
	metal_io_init(&prproc->shm_new_io, io->virt, &shm->pa,
		      shm->size, -1, 0, NULL);
	shm->io = &prproc->shm_new_io;
#endif

	/* Open IPI */
	ipi = &prproc->ipi;
	if (!ipi->path) {
		fprintf(stderr,
			"ERROR: No IPI sock path specified.\r\n");
		ret = -EINVAL;
		goto err;
	}
	ipi->fd = event_open(ipi->path);
	if (ipi->fd < 0) {
		fprintf(stderr,
			"ERROR: Failed to open sock %s for IPI.\r\n",
			ipi->path);
		ret = errno;
		goto err;
	}
	metal_irq_register(ipi->fd, linux_proc_irq_handler, ipi);
	metal_irq_enable(ipi->fd);
	kicked = &ipi->sync;
	return 0;

err:
	return ret;
}


static void linux_proc_remove(struct remoteproc *rproc)
{
	struct remoteproc_priv *prproc;
	struct vring_ipi_info *ipi;
	struct metal_io_region *io;

	if (!rproc)
		return;
	prproc = rproc->priv;

	/* Close IPI */
	ipi = &prproc->ipi;
	if (ipi->fd >= 0) {
		metal_irq_disable(ipi->fd);
		metal_irq_unregister(ipi->fd);
		close(ipi->fd);
	}

	/* Close shared memory */
	io = prproc->shm_old_io;
	if (io && io->ops.close) {
		io->ops.close(io);
		prproc->shm_old_io = NULL;
	}
}

#if 0
static void *
linux_proc_mmap(struct remoteproc *rproc, metal_phys_addr_t *pa,
		metal_phys_addr_t *da, size_t size,
		unsigned int attribute, struct metal_io_region **io)
{
	struct remoteproc_mem *mem;
	struct remoteproc_priv *prproc;
	metal_phys_addr_t lpa, lda;
	void *va;

	(void)attribute;
	(void)size;
	lpa = *pa;
	lda = *da;

	if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
		return NULL;
	if (lpa == METAL_BAD_PHYS)
		lpa = lda;
	if (lda == METAL_BAD_PHYS)
		lda = lpa;

	if (!rproc)
		return NULL;
	prproc = rproc->priv;
	mem = &prproc->shm;
	va = metal_io_phys_to_virt(mem->io, lpa);
	if (va) {
		if (io)
			*io = mem->io;
		metal_list_add_tail(&rproc->mems, &mem->node);
	}
	return va;
}
#endif

int metal_ipc_notify(struct remoteproc *rproc, uint32_t id)
{
	struct remoteproc_priv *prproc;
	struct vring_ipi_info *ipi;
	char dummy = 1;

	(void)id;
	(void)rproc;
	prproc = &rproc_priv_table[myrole];
	ipi = &prproc->ipi;
	send(ipi->fd, &dummy, 1, MSG_NOSIGNAL);
	return 0;
}

static int platform_slave_setup_resource_table(const char *shm_file,
					       int shm_size,
					       void *rsc_table, int rsc_size,
					       metal_phys_addr_t rsc_pa)
{
	struct metal_io_region *io;
	void *rsc_shm;
	int ret;

	ret = metal_shmem_open(shm_file, shm_size, &io);
	if (ret) {
		printf("Failed to init rproc, failed to open shm %s.\r\n",
		       shm_file);
		return -1;
	}
	rsc_shm = metal_io_virt(io, rsc_pa);
	memcpy(rsc_shm, rsc_table, rsc_size);
	io->ops.close(io);
	free(io);
	return 0;
}



int metal_linux_poll(void *priv)
{
	struct remoteproc *rproc = priv;
	struct remoteproc_priv *prproc;
	struct vring_ipi_info *ipi;
	unsigned int flags;

	prproc = rproc->priv;
	ipi = &prproc->ipi;
	while(1) {
		flags = metal_irq_save_disable();
		if (!(atomic_flag_test_and_set(&ipi->sync))) {
			metal_irq_restore_enable(flags);
			remoteproc_get_notification(rproc, RSC_NOTIFY_ID_ANY);
			break;
		}
		_rproc_wait();
		metal_irq_restore_enable(flags);
	}
	return 0;
}

