/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Contains top-level SMC handler for Power Management
*********************************************************************/

#include <debug.h>
#include <gic_v2.h>
#include <runtime_svc.h>
#include "pm_api_sys.h"
#include "pm_client.h"
#include "pm_svc_main.h"

/* Global PM context structure. Contains local data (at ATF) for PM. */
struct pm_context pm_ctx;

/* PM service setup, called from sip_svc_setup */
uint32_t pm_setup(void)
{
	/* Place here initialisation functions for PM service */
	pm_ipi_init();

	INFO("BL3-1: PM Service Init Complete\n");
	return 0;
}

/*
 * Top-level PM Service SMC handler.
 */
uint64_t pm_smc_handler(uint32_t smc_fid,
					uint64_t x1,
					uint64_t x2,
					uint64_t x3,
					uint64_t x4,
					void *cookie,
					void *handle,
					uint64_t flags)
{
	switch (smc_fid & FUNCID_NUM_MASK) {
	case PM_SMC_INIT:
		VERBOSE("PM_SMC_INIT: Initialize pm_notify handler,\
			IRQ: %d\n", x1);

		pm_ctx.pm_notify_irq = x1;
		gicd_set_isenabler(RDO_GICD_BASE, x1);

		SMC_RET1(handle, 0);

	case PM_SMC_NOTIFY:
		VERBOSE("PM_SMC_NOTIFY\n");

		/* Set additional return argument in x4 reg */
		/* By SMC CC only x0-3 is used for return value */
		SMC_SET_GP(handle, CTX_GPREG_X4, pm_ctx.pld->api_id);
		SMC_RET4(handle,
			pm_ctx.pld->arg[0], pm_ctx.pld->arg[1],
			pm_ctx.pld->arg[2], pm_ctx.pld->arg[3]);

	/* "Forward" interrupt (IRQ) from linux to ATF, in lack of FIQ! */
	case PM_SMC_IRQ:
		VERBOSE("PM_SMC_IRQ\n");
		ipi_fiq_handler();
		SMC_RET1(handle, 0);

	case PM_REQ_SUSPEND:
		pm_req_suspend(x1, x2, x3, x4);
		SMC_RET1(handle, 0);

	case PM_SELF_SUSPEND:
		pm_self_suspend(x1, x2, x3, x4);
		SMC_RET1(handle, 0);

	case PM_FORCE_POWERDOWN:
		pm_force_powerdown(x1, x2);
		SMC_RET1(handle, 0);

	case PM_ABORT_SUSPEND:
		pm_abort_suspend(x1);
		SMC_RET1(handle, 0);

	case PM_REQ_WAKEUP:
		pm_req_wakeup(x1, x2);
		SMC_RET1(handle, 0);

	case PM_SET_WAKEUP_SOURCE:
		pm_set_wakeup_source(x1, x2, x3);
		SMC_RET1(handle, 0);

	case PM_SYSTEM_SHUTDOWN:
		pm_system_shutdown(x1);
		SMC_RET1(handle, 0);

	case PM_REQ_NODE:
		pm_req_node(x1, x2, x3, x4);
		SMC_RET1(handle, 0);

	case PM_RELEASE_NODE:
		pm_release_node(x1, x2);
		SMC_RET1(handle, 0);

	case PM_SET_REQUIREMENT:
		pm_set_requirement(x1, x2, x3, x4);
		SMC_RET1(handle, 0);

	case PM_SET_MAX_LATENCY:
		pm_set_max_latency(x1, x2);
		SMC_RET1(handle, 0);

	case PM_GET_API_VERSION:
		pm_get_api_version();
		SMC_RET1(handle, 0);

	case PM_SET_CONFIGURATION:
	case PM_GET_NODE_STATUS:
		pm_get_node_status(x1);
		SMC_RET1(handle, 0);

	case PM_GET_OP_CHARACTERISTIC:
	case PM_REGISTER_NOTIFIER:
		pm_register_notifier(x1, x2, x3);
		SMC_RET1(handle, 0);

	default:
		WARN("Unimplemented PM Service Call: 0x%x \n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
