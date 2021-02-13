/**
  ******************************************************************************
  * @file   openamp.c
  * @author  MCD Application Team
  * @brief  Code for openamp applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#define METAL_MAX_DEVICE_REGIONS 2
#include "openamp.h"
#include "rsc_table.h"
#include "metal/sys.h"
#include "metal/device.h"

// Get these from System Device Tree later
#define SHM_START_ADDRESS 0x3ED00000
#define SHM_SIZE 0X100000

#define SHM_DEVICE_NAME "openamp-shm"

static struct metal_io_region *shm_io;
static struct metal_io_region *rsc_io;
static struct remote_resource_table *rsc_table;
static struct rpmsg_virtio_shm_pool shpool;
static struct rpmsg_virtio_device rvdev;

extern int zynqmp_r5_a53_proc_notify(void *priv, uint32_t id);
extern int zynqmp_r5_ipc_init(void);

static metal_phys_addr_t shm_physmap;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		    {.virt = NULL},	/* shared memory */
		    {.virt = NULL},	/* rsc_table memory */
		    },
	.node = {NULL},
	.irq_num = 0,
	.irq_info = NULL
};

static int OPENAMP_shmem_init(int RPMsgRole)
{
	int status = 0;
	struct metal_device *device = NULL;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	void *rsc_tab_addr = NULL;
	int rsc_size = 4096;

	metal_init(&metal_params);

	status = metal_register_generic_device(&shm_device);
xil_printf("%s:%d status = %d\r\n", __FUNCTION__, __LINE__, status);
	if (status != 0) {
		return status;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
xil_printf("%s:%d status = %d\r\n", __FUNCTION__, __LINE__, status);
	if (status != 0) {
		return status;
	}

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);

	shm_physmap = SHM_START_ADDRESS;
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDRESS,
		      &shm_physmap, SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (shm_io == NULL) {
		return -1;
	}

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	/* Initialize resources table variables */
	rsc_table = get_resource_table(0, 0);
	if (!rsc_table) {
		return -1;
	}

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *) rsc_table, rsc_size, -1U, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (rsc_io == NULL) {
		return -1;
	}

	return 0;
}

int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc = NULL;
	struct virtio_device *vdev = NULL;
	int status = 0;

	//MAILBOX_Init();

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	/* Libmetal Initialization */
	status = OPENAMP_shmem_init(RPMsgRole);
	if (status) {
		return status;
	}
	/* initialize IPC */
	// Need to move this to generic libmetal code --TODO
	zynqmp_r5_a53_ipc_init();

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	vdev =
	    rproc_virtio_create_vdev(RPMsgRole, VDEV_ID, &rsc_table->rpmsg_vdev,
				     rsc_io, NULL, zynqmp_r5_a53_proc_notify,
				     NULL);
	if (vdev == NULL) {
		return -1;
	}

	rproc_virtio_wait_remote_ready(vdev);
xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);

	vring_rsc = &rsc_table->rpmsg_vring0;
xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	status = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
					 (void *)vring_rsc->da, shm_io,
					 vring_rsc->num, vring_rsc->align);
	if (status != 0) {
		return status;
	}

	vring_rsc = &rsc_table->rpmsg_vring1;
xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	status = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
					 (void *)vring_rsc->da, shm_io,
					 vring_rsc->num, vring_rsc->align);
	if (status != 0) {
		return status;
	}
xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);

	rpmsg_virtio_init_shm_pool(&shpool, (void *)VRING_BUFF_ADDRESS,
				   (size_t) SHM_SIZE);
	rpmsg_init_vdev(&rvdev, vdev, ns_bind_cb, shm_io, &shpool);

xil_printf("%s:%d\r\n", __FUNCTION__, __LINE__);
	return 0;
}

void OPENAMP_DeInit()
{
	rpmsg_deinit_vdev(&rvdev);

	metal_finish();
}

void OPENAMP_init_ept(struct rpmsg_endpoint *ept)
{
	rpmsg_init_ept(ept, "", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
}

int OPENAMP_create_endpoint(struct rpmsg_endpoint *ept, const char *name,
			    uint32_t dest, rpmsg_ept_cb cb,
			    rpmsg_ns_unbind_cb unbind_cb)
{
	int ret = 0;
	ret = rpmsg_create_ept(ept, &rvdev.rdev, name, RPMSG_ADDR_ANY, dest, cb,
			       unbind_cb);
xil_printf("%s() returns %d\r\n", __FUNCTION__, ret);
	return ret;
}
extern atomic_int kicked;

int OPENAMP_poll(void)
{
        unsigned int flags;
        int ret;

        while(1) {
#ifdef RPMSG_NO_IPI
                if (metal_io_read32(prproc->kick_io, 0)) {
                        ret = remoteproc_get_notification(rproc,
                                                          RSC_NOTIFY_ID_ANY);
                        if (ret)
                                return ret;
                        break;
                }
                (void)flags;
#else /* !RPMSG_NO_IPI */
                flags = metal_irq_save_disable();
                if (!(atomic_flag_test_and_set(&kicked))) {
                        metal_irq_restore_enable(flags);
                        ret = rproc_virtio_notified(rvdev.vdev, RSC_NOTIFY_ID_ANY);
                        if (ret)
                                return ret;
                        break;
                }
                metal_irq_restore_enable(flags);
// This should be a libmetal API.
                asm("wfi");
#endif /* RPMSG_NO_IPI */
        }
        return 0;
}


void OPENAMP_check_for_message(void)
{
	//MAILBOX_Poll(rvdev.vdev);
}

void OPENAMP_Wait_EndPointready(struct rpmsg_endpoint *rp_ept)
{
	while (!is_rpmsg_ept_ready(rp_ept)) {
		//MAILBOX_Poll(rvdev.vdev);
	}
}
