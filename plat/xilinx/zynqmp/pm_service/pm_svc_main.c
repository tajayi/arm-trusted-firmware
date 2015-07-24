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

/*
 * Top-level SMC handler for ZynqMP power management calls and
 * IPI setup functions for communication with PMU.
 */

#include <debug.h>
#include <gic_v2.h>
#include <runtime_svc.h>
#include "ipi.h"
#include "pm_api_sys.h"
#include "pm_common.h"
#include "pm_client.h"
#include "pm_svc_main.h"
#include "../zynqmp_def.h"
#include "../zynqmp_private.h"

/* Global PM context structure. Contains data for power management */
struct pm_context pm_ctx;

/**
 * read_ipi_buffer() - Read from IPI buffer registers
 * @proc - Pointer to the processor who expected data
 * @pld - array of 5 elements
 *
 * Read value from ipi buffer registers (from PMU) and store
 * in PM context payload structure
 */
void read_ipi_buffer(const struct pm_proc *const proc, uint32_t *pld)
{
	uint32_t i;
	uint32_t offset = 0;
	uint32_t buffer_base = proc->ipi->buffer_base +
		IPI_BUFFER_TARGET_PMU_OFFSET +
		IPI_BUFFER_RESP_OFFSET;

	/* Read from IPI buffer and store into payload array */
	for (i = 0; i < PAYLOAD_ARG_CNT; i++) {
		pld[i] = pm_read(buffer_base + offset);
		offset += PAYLOAD_ARG_SIZE;
	}
}

/**
 * trigger_callback_irq() - Set interrupt for non-secure EL1/EL2
 * @irq_num - entrance in GIC
 *
 * Inform non-secure software layer (EL1/2) that PMU responsed on acknowledge
 * or demands suspend action.
 */
static void trigger_callback_irq(uint32_t irq_num)
{
	/* Set interrupt for non-secure EL1/EL2 */
	gicd_set_ispendr(BASE_GICD_BASE, irq_num);
	gicd_set_isactiver(BASE_GICD_BASE, irq_num);
}

/**
 * ipi_fiq_handler() - IPI Handler for PM-API callbacks
 * @id - 	number of the highest priority pending interrupt of the type
 *		that this handler was registered for
 * @flags - 	security state, bit[0]
 * @handler - 	pointer to 'cpu_context' structure of the current CPU for the
 * 	      	security state specified in the 'flags' parameter
 * @cookie  - 	unused
 *
 * Function registered as INTR_TYPE_EL3 interrupt handler
 *
 * PMU sends IPI interrupts for PM-API callbacks.
 * This handler reads data from payload buffers and
 * based on read data decodes type of callback and call proper function.
 *
 * In presence of non-secure software layers (EL1/2) sets the interrupt
 * at registered entrance in GIC and informs that PMU responsed or demands
 * action
 */
static uint64_t ipi_fiq_handler(uint32_t id,
				uint32_t flags,
				void *handle,
				void *cookie)
{
	const struct pm_proc *proc = pm_get_proc(pm_this_cpuid());
	uint32_t ipi_apu_isr_reg = pm_read(IPI_APU_ISR);

	/* Read PM-API Arguments */
	read_ipi_buffer(proc, pm_ctx.payload);

	/*
	 * Inform non-secure software layer (EL1/2) by setting the interrupt
	 * at registered entrance in GIC, that PMU responsed or demands action
	 */
	trigger_callback_irq(pm_ctx.callback_irq);

	/* Clear IPI_APU_ISR bit */
	pm_write(IPI_APU_ISR, ipi_apu_isr_reg & IPI_PMU_PM_INT_MASK);

	return 0;
}

/**
 * pm_ipi_init() - Initialize IPI peripheral for communication with PMU
 *
 * @return - 	On success, the initialization function must return 0.
 *		Any other return value will cause the framework to ignore
 *		the service
 *
 * Enable interrupts at registered entrance in IPI peripheral
 * Called from pm_setup initialization function
 */
int32_t pm_ipi_init(void)
{
	uint32_t ipi_apu_ier_reg;

	bakery_lock_init(&pm_secure_lock);

	/* IPI Interrupts Enable */
	ipi_apu_ier_reg = pm_read(IPI_APU_IER);
	pm_write(IPI_APU_IER,
		 ipi_apu_ier_reg | IPI_APU_IER_PMU_0_MASK);

	/* Register IPI interrupt as INTR_TYPE_EL3 */
	return request_intr_type_el3(IRQ_SEC_IPI_APU, ipi_fiq_handler);
}

/**
 * pm_setup() - PM service setup
 *
 * @return - 	On success, the initialization function must return 0.
 *		Any other return value will cause the framework to ignore
 *		the service
 *
 * Initialization functions for ZynqMP power management for
 * communicaton with PMU.
 *
 * Called from sip_svc_setup initialization function with the
 * rt_svc_init signature.
 *
 */
int32_t pm_setup(void)
{
	int32_t status;

	/* initialize IPI interrupts */
	status = pm_ipi_init();

	if (status == 0)
		INFO("BL3-1: PM Service Init Complete: API v%d.%d\n",
		     PM_VERSION_MAJOR, PM_VERSION_MINOR);
	else
		INFO("BL3-1: PM Service Init Failed, Error Code %d!\n", status);

	return status;
}

/**
 * pm_smc_handler() - SMC handler for PM-API calls coming from EL1/EL2.
 * @smc_fid - Function Identifier
 * @x1 - x4 - Arguments
 * @cookie  - Unused
 * @handler - Pointer to caller's context structure
 *
 * @return  - Unused
 *
 * Determines that smc_fid is valid and supported PM SMC Function ID from the
 * list of pm_api_ids, otherwise completes the request with
 * the Unknow SMC Function ID
 *
 * The SMC calls for PM service are forwarded from SIP Service SMC handler
 * function with rt_svc_handle signature
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
	enum pm_ret_status ret;

	switch (smc_fid & FUNCID_NUM_MASK) {
	case PM_F_INIT:
		VERBOSE("Initialize pm callback, irq: %d\n", x1);

		/* Save pm callback irq number */
		pm_ctx.callback_irq = x1;
		gicd_set_isenabler(BASE_GICD_BASE, x1);
		SMC_RET1(handle, (uint64_t)PM_RET_SUCCESS);

	case PM_F_GETARGS:
	{
		uint64_t callback_arg;

		/*
		* According to SMC calling convention the return values are
		* stored in registers x0-x3
		* x0 = pm_api_id
		* x1 = arg0
		* x2 = arg1
		* x3 lower 32bit = arg2
		* x3 higher 32bit = arg3
		*/
		callback_arg = pm_ctx.payload[4];
		callback_arg = (callback_arg << 32) + pm_ctx.payload[3];
		SMC_RET4(handle,
			 pm_ctx.payload[0], pm_ctx.payload[1],
			 pm_ctx.payload[2], callback_arg);
	}

	/* PM API Functions */
	case PM_SELF_SUSPEND:
		ret = pm_self_suspend(x1, x2, x3);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_REQ_SUSPEND:
		ret = pm_req_suspend(x1, x2, x3, x4);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_REQ_WAKEUP:
		ret = pm_req_wakeup(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_FORCE_POWERDOWN:
		ret = pm_force_powerdown(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_ABORT_SUSPEND:
		ret = pm_abort_suspend(x1);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_SET_WAKEUP_SOURCE:
		ret = pm_set_wakeup_source(x1, x2, x3);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_SYSTEM_SHUTDOWN:
		ret = pm_system_shutdown(x1);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_REQ_NODE:
		ret = pm_req_node(x1, x2, x3, x4);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_RELEASE_NODE:
		ret = pm_release_node(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_SET_REQUIREMENT:
		ret = pm_set_requirement(x1, x2, x3, x4);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_SET_MAX_LATENCY:
		ret = pm_set_max_latency(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_GET_API_VERSION:
		/* Check is PM API version already verified */
		if (pm_ctx.api_version == PM_VERSION)
			SMC_RET2(handle, (uint64_t)PM_RET_SUCCESS,
					  PM_VERSION);

		ret = pm_get_api_version(&pm_ctx.api_version);
		SMC_RET2(handle, (uint64_t)ret,
				 (uint64_t)pm_ctx.api_version);

	case PM_SET_CONFIGURATION:
		ret = pm_set_configuration(x1);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_GET_NODE_STATUS:
		ret = pm_get_node_status(x1);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_GET_OP_CHARACTERISTIC:
		ret = pm_get_op_characteristic(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_REGISTER_NOTIFIER:
		ret = pm_register_notifier(x1, x2, x3, x4);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_RESET_ASSERT:
		ret = pm_reset_assert(x1, x2);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_RESET_GET_STATUS:
	{
		uint32_t reset_status;

		ret = pm_reset_get_status(x1, &reset_status);
		SMC_RET2(handle, (uint64_t)ret, (uint64_t)reset_status);
	}

	/* PM memory access functions */
	case PM_MMIO_WRITE:
		ret = pm_mmio_write(x1, x2, x3);
		SMC_RET1(handle, (uint64_t)ret);

	case PM_MMIO_READ:
	{
		uint32_t value;

		ret = pm_mmio_read(x1, x2, &value);
		SMC_RET1(handle, (uint64_t)ret);
	}
	default:
		WARN("Unimplemented PM Service Call: 0x%x\n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
