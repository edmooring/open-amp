/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This is a sample demonstration application that showcases usage of rpmsg
This application is meant to run on the remote CPU running baremetal code.
This application echoes back data that was sent to it by the master core. */

#include <stdio.h>
#include <metal/alloc.h>
#include <openamp/openamp.h>
#include "rpmsg-echo.h"

extern int init_system(void);
extern void cleanup_system(void);

#define SHUTDOWN_MSG	0xEF56A55A

#define LPRINTF(format, ...) printf(format, ##__VA_ARGS__)
//#define LPRINTF(format, ...)
#define LPERROR(format, ...) LPRINTF("ERROR: " format, ##__VA_ARGS__)

static struct rpmsg_endpoint lept;
static int shutdown_req = 0;

/*-----------------------------------------------------------------------------*
 *  RPMSG endpoint callbacks
 *-----------------------------------------------------------------------------*/
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
			     uint32_t src, void *priv)
{
	(void)priv;
	(void)src;

	/* On reception of a shutdown we signal the application to terminate */
	if ((*(unsigned int *)data) == SHUTDOWN_MSG) {
		LPRINTF("shutdown message is received.\r\n");
		shutdown_req = 1;
		return RPMSG_SUCCESS;
	}

	/* Send data back to master */
	if (rpmsg_send(ept, data, len) < 0) {
		LPERROR("rpmsg_send failed\r\n");
	}
	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
	(void)ept;
	LPRINTF("unexpected Remote endpoint destroy\r\n");
	shutdown_req = 1;
}

/*-----------------------------------------------------------------------------*
 *  Application entry point
 *-----------------------------------------------------------------------------*/
int main(void)
{
	int ret;

	LPRINTF("Starting application...\r\n");

	/* Initialize platform */
	ret = init_system();
	if (ret) {
		LPERROR("Failed to initialize platform.\r\n");
		ret = -1;
	} else {
		ret = MX_OPENAMP_Init(RPMSG_REMOTE, 0);
		if (ret) {
			LPERROR("Failed to initialize OpenAMAP (%d).\r\n", ret);
			ret = -1;
		} else {
			ret = OPENAMP_create_endpoint(&lept, RPMSG_SERVICE_NAME,
						      RPMSG_ADDR_ANY,
						      rpmsg_endpoint_cb,
						      rpmsg_service_unbind);
			if (ret) {
				LPERROR("Failed to create endpoint.\r\n");
				return -1;
			}
			while(1) {
				OPENAMP_poll();
				/* we got a shutdown request, exit */
				if (shutdown_req) {
					break;
				}
			}
			rpmsg_destroy_ept(&lept);
		}
	}

	LPRINTF("Stopping application...\r\n");
	cleanup_system();

	return ret;
}
