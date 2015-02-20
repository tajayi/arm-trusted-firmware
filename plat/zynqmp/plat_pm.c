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

#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <psci.h>
#include <errno.h>
#include "zynqmp_private.h"
#include "pm_api_sys.h"

/*******************************************************************************
 * Private ZynqMP function to program the mailbox for a cpu before it is released
 * from reset.
 ******************************************************************************/
static void zynqmp_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	mailbox_t *zynqmp_mboxes;

	linear_id = platform_get_core_pos(mpidr);
	zynqmp_mboxes = (mailbox_t *)MBOX_BASE;
	zynqmp_mboxes[linear_id].value = address;
	flush_dcache_range((unsigned long) &zynqmp_mboxes[linear_id],
			   sizeof(unsigned long));
}

/*******************************************************************************
 * Private ZynqMP function which is used to determine if any platform actions
 * should be performed for the specified affinity instance given its
 * state. Nothing needs to be done if the 'state' is not off or if this is not
 * the highest affinity level which will enter the 'state'.
 ******************************************************************************/
static int32_t zynqmp_do_plat_actions(uint32_t afflvl, uint32_t state)
{
	uint32_t max_phys_off_afflvl;

	assert(afflvl <= MPIDR_AFFLVL1);

	if (state != PSCI_STATE_OFF)
		return -EAGAIN;

	/*
	 * Find the highest affinity level which will be suspended and postpone
	 * all the platform specific actions until that level is hit.
	 */
	max_phys_off_afflvl = psci_get_max_phys_off_afflvl();
	assert(max_phys_off_afflvl != PSCI_INVALID_DATA);
	assert(psci_get_suspend_afflvl() >= max_phys_off_afflvl);
	if (afflvl != max_phys_off_afflvl)
		return -EAGAIN;

	return 0;
}

/*******************************************************************************
 * ZynqMP handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
static int32_t zynqmp_affinst_on(uint64_t mpidr,
				  uint64_t sec_entrypoint,
				  uint32_t afflvl,
				  uint32_t state)
{
	uint32_t r;
	uint32_t node_id = platform_get_core_pos(mpidr) + 2;

	/*
	 * PMU takes care of powering up higher affinity levels so we
	 * only need to care about level 0
	 */
	if (afflvl != MPIDR_AFFLVL0)
		return PSCI_E_SUCCESS;

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up
	 */
	zynqmp_program_mailbox(mpidr, sec_entrypoint);

	mmio_write_32(R_RVBAR_L_0 + mpidr * 8, sec_entrypoint);
	mmio_write_32(R_RVBAR_H_0 + mpidr * 8, sec_entrypoint >> 32);
	dsb();

	if (!zynqmp_is_pmu_up()) {
		r = mmio_read_32(CRF_APB_RST_FPD_APU);
		r &= ~(0x401 << mpidr);
		mmio_write_32(CRF_APB_RST_FPD_APU, r);
	} else {
		/* Sent request to PMU to wake up selected APU CPU core */
		pm_req_wakeup((enum pm_node_id)node_id, REQ_ACK_NO/*REQ_ACK_STANDARD*/);
	}

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * ZynqMP handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take
 * appropriate actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across calls
 * to this function as each affinity level is dealt with. So do not write & read
 * global variables across calls. It will be wise to do flush a write to the
 * global to prevent unpredictable results.
 ******************************************************************************/
static void zynqmp_affinst_off(uint32_t afflvl, uint32_t state)
{
	uint32_t r;
	uint64_t mpidr = read_mpidr_el1();
	uint32_t node_id = platform_get_core_pos(mpidr) + 2;

	/* Determine if any platform actions need to be executed */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	if (!zynqmp_is_pmu_up()) {
		/* Program the power controller to power off this cpu. */
		r = mmio_read_32(CRF_APB_RST_FPD_APU);
		r |= 1 << (mpidr & 0xf);
		mmio_write_32(CRF_APB_RST_FPD_APU, r);
	} else {
		if (afflvl > MPIDR_AFFLVL0) {
			/* Send request to PMU to suspend APU subsystem */
			NOTICE("call pm_self_suspend(NODE_APU)\n");
			pm_self_suspend(NODE_APU, REQ_ACK_NO, MAX_LATENCY, 0);
		}

		/* Send request to PMU to power down the appropriate APU CPU core */
		NOTICE("call pm_self_suspend(node_id)\n");
		pm_self_suspend((enum pm_node_id)node_id, REQ_ACK_NO, MAX_LATENCY, 0);
	}
}

/*******************************************************************************
 * ZynqMP handler called when an affinity instance is about to be suspended. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions. The 'sec_entrypoint' determines the address in BL3-1 from where
 * execution should resume.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across calls
 * to this function as each affinity level is dealt with. So do not write & read
 * global variables across calls. It will be wise to do flush a write to the
 * global to prevent unpredictable results.
 ******************************************************************************/
static void zynqmp_affinst_suspend(uint64_t sec_entrypoint,
				       uint32_t afflvl,
				       uint32_t state)
{
	uint32_t r;
	uint64_t mpidr = read_mpidr_el1();
	uint32_t node_id = platform_get_core_pos(mpidr) + 2;

	/* Determine if any platform actions need to be executed. */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up.
	 */
	zynqmp_program_mailbox(mpidr, sec_entrypoint);

	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	if (!zynqmp_is_pmu_up()) {
		/* Program the power controller to power off this cpu. */
		r = mmio_read_32(CRF_APB_RST_FPD_APU);
		r |= 1 << (mpidr & 0xf);
		mmio_write_32(CRF_APB_RST_FPD_APU, r);
	} else {
		/* APU is to be turned off */
		if (afflvl > MPIDR_AFFLVL0) {
			/* Send request to PMU to suspend the APU CPU */
			pm_self_suspend(NODE_APU, REQ_ACK_NO, MAX_LATENCY, 0);
		}
		/* Send request to PMU to suspend the appropriate APU CPU core */
		pm_self_suspend((enum pm_node_id)node_id, REQ_ACK_NO, MAX_LATENCY, 0);
	}
}

/*******************************************************************************
 * ZynqMP handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
static void zynqmp_affinst_on_finish(uint32_t afflvl, uint32_t state)
{
	uint64_t mpidr = read_mpidr_el1();

	/* Determine if any platform actions need to be executed. */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: Is this setup only needed after a cold boot? */
	arm_gic_pcpu_distif_setup();

	/* Clear the mailbox for this cpu. */
	zynqmp_program_mailbox(mpidr, 0);
}

/*******************************************************************************
 * ZynqMP handler called when an affinity instance has just been powered on after
 * having been suspended earlier. The level and mpidr determine the affinity
 * instance.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
static void zynqmp_affinst_suspend_finish(uint32_t afflvl, uint32_t state)
{
	zynqmp_affinst_on_finish(afflvl, state);
}

/*******************************************************************************
 * ZynqMP handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 zynqmp_system_off(void)
{
#if 0
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_SHUTDOWN));
	wfi();
#endif

	if (zynqmp_is_pmu_up()) {
		/* Send the power down request to the PMU */
		pm_system_shutdown(0);

		wfi();
	}

	ERROR("ZynqMP System Off: operation not handled.\n");
	panic();
}

static void __dead2 zynqmp_system_reset(void)
{
#if 0
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_REBOOT));
	wfi();
#endif

	if (zynqmp_is_pmu_up()) {
		/* Send the system reset request to the PMU */
		pm_system_shutdown(1);

		wfi();
	}

	ERROR("ZynqMP System Reset: operation not handled.\n");
	panic();
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t zynqmp_ops = {
	.affinst_on		= zynqmp_affinst_on,
	.affinst_off		= zynqmp_affinst_off,
	.affinst_suspend	= zynqmp_affinst_suspend,
	.affinst_on_finish	= zynqmp_affinst_on_finish,
	.affinst_suspend_finish	= zynqmp_affinst_suspend_finish,
	.system_off		= zynqmp_system_off,
	.system_reset		= zynqmp_system_reset
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &zynqmp_ops;
	return 0;
}
