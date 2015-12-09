/*
 * Copyright (c) 2013-2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Top level SMC handler for SiP calls. Dispatch PM calls to PM SMC handler. */

#include <debug.h>
#include <runtime_svc.h>
#include <uuid.h>
#include "pm_svc_main.h"
#include "sip_svc.h"

/* SiP Service UUID */
DEFINE_SVC_UUID(zynqmp_sip_uid,
		0x0, 0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0, 0x0, 0x0);

/**
 * sip_svc_setup() - Setup SiP Service
 *
 * Invokes PM setup
 */
static int32_t sip_svc_setup(void)
{
	/* PM implementation as SiP Service */
	return pm_setup();
}

/**
 * sip_svc_smc_handler() - Top-level SiP Service SMC handler
 *
 * Handler for all SiP SMC calls. Handles standard SIP requests
 * and calls PM SMC handler if the call is for a PM-API function.
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
	/* Let PM SMC handler deal with PM-related requests */
	if (is_pm_fid(smc_fid)) {
		return pm_smc_handler(smc_fid, x1, x2, x3, x4, cookie, handle,
				      flags);
	}

	switch (smc_fid) {
	case ZYNQMP_SIP_SVC_CALL_COUNT:
		/*
		 * Return the number of SiP Service Calls.
		 * For now PM is the only SiP service implemented.
		 */
		SMC_RET1(handle, PM_API_MAX);

	case ZYNQMP_SIP_SVC_UID:
		/* Return UID to the caller */
		SMC_UUID_RET(handle, zynqmp_sip_uid);

	case ZYNQMP_SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET2(handle, SIP_SVC_VERSION_MAJOR, SIP_SVC_VERSION_MINOR);

	default:
		WARN("Unimplemented SiP Service Call: 0x%x\n", smc_fid);
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
