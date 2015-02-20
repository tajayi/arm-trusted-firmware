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

#ifndef _PM_DEFS_H_
#define _PM_DEFS_H_

/*********************************************************************
 * Macro definitions
 ********************************************************************/

/* Capabilities for RAM */
#define CAPABILITY_ACCESSIBLE		0x00000001U
#define CAPABILITY_PRESERVE_CONTEXT	0x00000002U

#define MAX_LATENCY	(~0U)
#define MAX_QOS		100U

/*********************************************************************
 * Enum definitions
 ********************************************************************/

enum pm_api_id {
	/* API for suspending of PUs: */
	PM_REQ_SUSPEND = 1,
	PM_SELF_SUSPEND,
	PM_FORCE_POWERDOWN,
	PM_POWERDOWN,
	PM_ABORT_SUSPEND,
	PM_REQ_WAKEUP,
	PM_SET_WAKEUP_SOURCE,
	PM_SYSTEM_SHUTDOWN,
	/* API for managing PM slaves: */
	PM_REQ_NODE,
	PM_RELEASE_NODE,
	PM_SET_REQUIREMENT,
	PM_SET_MAX_LATENCY,
	/* Miscellaneous API functions: */
	PM_GET_API_VERSION,
	PM_SET_CONFIGURATION,
	PM_GET_NODE_STATUS,
	PM_GET_OP_CHARACTERISTIC,
	PM_REGISTER_NOTIFIER,
	/* Direct control API functions: */
	PM_CLOCK_REQUEST,
	PM_CLOCK_RELEASE,
	PM_CLOCK_SET_RATE,
	PM_CLOCK_GET_RATE,
	PM_CLOCK_GET_RATE_INFO,
	PM_RESET_ASSERT,
	PM_RESET_GET_STATUS,
	PM_MMIO_WRITE,
	PM_MMIO_READ,
};

enum pm_api_cb_id {
	PM_INIT_SUSPEND_CB = 30,
	PM_ACKNOWLEDGE_CB,
	PM_NOTIFY_CB,
};

enum pm_node_id {
	NODE_UNKNOWN = 0,
	NODE_APU,
	NODE_APU_0,
	NODE_APU_1,
	NODE_APU_2,
	NODE_APU_3,
	NODE_RPU,
	NODE_RPU_0,
	NODE_RPU_1,
	NODE_PL,
	NODE_FPD,
	NODE_OCM_BANK_0,
	NODE_OCM_BANK_1,
	NODE_OCM_BANK_2,
	NODE_OCM_BANK_3,
	NODE_TCM_0_A,
	NODE_TCM_0_B,
	NODE_TCM_1_A,
	NODE_TCM_1_B,
	NODE_L2,
	NODE_GPU_PP_0,
	NODE_GPU_PP_1,
	NODE_USB_0,
	NODE_USB_1,
};

enum pm_request_ack {
	REQ_ACK_NO = 1,
	REQ_ACK_STANDARD,
	REQ_ACK_ERROR,
	REQ_ACK_DETAILED,
};

enum pm_abort_reason {
	ABORT_REASON_WKUP_EVENT = 100,
	ABORT_REASON_PU_BUSY,
	ABORT_REASON_NO_PWRDN,
	ABORT_REASON_UNKNOWN,
};

enum pm_suspend_reason {
	SUSPEND_REASON_PU_REQ = 201,
	SUSPEND_REASON_ALERT,
	SUSPEND_REASON_SYS_SHUTDOWN,
};

enum pm_ram_state {
	PM_RAM_STATE_OFF = 1,
	PM_RAM_STATE_RETENTION,
	PM_RAM_STATE_ON,
};

#endif /* _PM_DEFS_H_ */
