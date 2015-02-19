/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Contains Top SMC handler for SiP calls. Dispatch PM calls to
* PM SMC handler.
*********************************************************************/

#include <debug.h>
#include <runtime_svc.h>
#include <uuid.h>
#include "pm_svc_main.h"
#include "sip_svc.h"

/* SIP Service UUID */
DEFINE_SVC_UUID(zynqmp_sip_uid,
		0x0, 0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0, 0x0, 0x0);

/* Setup SIP Services */
static int32_t sip_svc_setup(void)
{
	/* PM implementation as SIP Service*/
	return pm_setup();
}

/*
 * Top-level SIP Service SMC handler.
 */
uint64_t sip_svc_smc_handler(uint32_t smc_fid,
				uint64_t x1,
				uint64_t x2,
				uint64_t x3,
				uint64_t x4,
				void *cookie,
				void *handle,
				uint64_t flags)
{
	/*
	 * Dispatch PM calls to PM SMC handler and return its return
	 * value
	 */
	if (is_pm_fid(smc_fid)) {
		return pm_smc_handler(smc_fid, x1, x2, x3, x4, cookie,
				handle, flags);
	}

	switch (smc_fid) {
#if 0
	case ZYNQMP_SIP_SVC_CALL_COUNT:
		/*
		 * Return the number of SIP Service Calls. PM is the only SIP service implemented
		 */
		SMC_RET1(handle, PM_API_MAX);
#endif
	case ZYNQMP_SIP_SVC_UID:
		/* Return UID to the caller */
		SMC_UUID_RET(handle, zynqmp_sip_uid);

	case ZYNQMP_SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET2(handle, SIP_SVC_VERSION_MAJOR, SIP_SVC_VERSION_MINOR);

	default:
		WARN("Unimplemented Sip Service Call: 0x%x \n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}

/* Register PM Service Calls as runtime service */
DECLARE_RT_SVC(
		sip_svc,
		OEN_SIP_START,
		OEN_SIP_END,
		SMC_TYPE_FAST,
		sip_svc_setup,
		sip_svc_smc_handler);
