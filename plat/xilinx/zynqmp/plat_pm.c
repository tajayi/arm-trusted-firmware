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
#include <plat_arm.h>
#include <platform.h>
#include <psci.h>
#include <errno.h>
#include "zynqmp_private.h"
#include "pm_client.h"
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
 * ZynqMP handler called when an affinity instance is about to enter standby.
 * Mandatory
 ******************************************************************************/
void zynqmp_affinst_standby(unsigned int power_state)
{
	unsigned int target_afflvl;

	/* Sanity check the requested state */
	target_afflvl = psci_get_pstate_afflvl(power_state);

	/*
	 * It's possible to enter standby only on affinity level 0 i.e. a cpu
	 * on the ZynqMP. Ignore any other affinity level.
	 */
	if (target_afflvl != MPIDR_AFFLVL0)
		return;

	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();
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
	uint32_t linear_id = platform_get_core_pos(mpidr);

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

	dsb();

	if (!zynqmp_is_pmu_up()) {
		uint32_t r;

		/* program RVBAR */
		mmio_write_32(APU_RVBAR_L_0 + (linear_id << 3), sec_entrypoint);
		mmio_write_32(APU_RVBAR_H_0 + (linear_id << 3),
			      sec_entrypoint >> 32);

		/* clear VINITHI */
		r = mmio_read_32(APU_CONFIG_0);
		r &= ~(1 << APU_CONFIG_0_VINITHI_SHIFT << linear_id);
		mmio_write_32(APU_CONFIG_0, r);

		/* clear power down request */
		r = mmio_read_32(APU_PWRCTL);
		r &= ~(1 << linear_id);
		mmio_write_32(APU_PWRCTL, r);

		/* power up island */
		mmio_write_32(PMU_GLOBAL_REQ_PWRUP_EN, 1 << linear_id);
		mmio_write_32(PMU_GLOBAL_REQ_PWRUP_TRIG, 1 << linear_id);
		/* FIXME: we should have a way to break out */
		while (mmio_read_32(PMU_GLOBAL_REQ_PWRUP_STATUS) & (1 << linear_id))
			;

		/* release core reset */
		r = mmio_read_32(CRF_APB_RST_FPD_APU);
		r &= ~((CRF_APB_RST_FPD_APU_ACPU_PWRON_RESET |
				CRF_APB_RST_FPD_APU_ACPU_RESET) << linear_id);
		mmio_write_32(CRF_APB_RST_FPD_APU, r);
	} else {
		const struct pm_proc *proc = pm_get_proc(linear_id);

		/* Send request to PMU to wake up selected APU CPU core */
		pm_req_wakeup(proc->node_id, 1, sec_entrypoint, REQ_ACK_NO);
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
	uint64_t mpidr = read_mpidr_el1();
	uint32_t linear_id = platform_get_core_pos(mpidr);

	/* Determine if any platform actions need to be executed */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	if (!zynqmp_is_pmu_up()) {
		uint32_t r;

		/* set power down request */
		r = mmio_read_32(APU_PWRCTL);
		r |= (1 << linear_id);
		mmio_write_32(APU_PWRCTL, r);
	} else {
		const struct pm_proc *proc = pm_get_proc(linear_id);

		/*
		 * Send request to PMU to power down the appropriate APU CPU
		 * core.
		 * According to PSCI specification, CPU_off function does not
		 * have resume address and CPU core can only be woken up
		 * invoking CPU_on function, during which resume address will
		 * be set.
		 */
		pm_self_suspend(proc->node_id, MAX_LATENCY, 0, 0);
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
	uint64_t mpidr = read_mpidr_el1();
	uint32_t linear_id = platform_get_core_pos(mpidr);
	const struct pm_proc* proc = pm_get_proc(linear_id);

	/* Determine if any platform actions need to be executed. */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up.
	 */
	zynqmp_program_mailbox(mpidr, sec_entrypoint);

	if (!zynqmp_is_pmu_up()) {
		/* Program the power controller to power off this cpu. */
		uint32_t r = mmio_read_32(CRF_APB_RST_FPD_APU);
		r |= 1 << (linear_id);
		mmio_write_32(CRF_APB_RST_FPD_APU, r);
	} else {
		/* Send request to PMU to suspend the appropriate APU CPU core */
		pm_self_suspend(proc->node_id, MAX_LATENCY, 0,
				(uint64_t)bl31_entrypoint);

		/* APU is to be turned off */
		if (afflvl > MPIDR_AFFLVL0) {
			/* Power down L2 cache */
			pm_set_requirement(NODE_L2, 0, 0, REQ_ACK_NO);
			/* Send request for OCM retention state */
			set_ocm_retention();
		}
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
 ******************************************************************************/
static void zynqmp_affinst_suspend_finish(uint32_t afflvl, uint32_t state)
{
	uint64_t mpidr = read_mpidr_el1();
	uint32_t linear_id = platform_get_core_pos(mpidr);
	const struct pm_proc *proc = pm_get_proc(linear_id);

	/* Determine if any platform actions need to be executed. */
	if (zynqmp_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	if (afflvl > MPIDR_AFFLVL0) {
		/*
		 * FPD domain was turned off, reinitialize GIC settings
		 * Initialize the gic cpu and distributor interfaces
		 */
		plat_arm_gic_init();
		arm_gic_setup();
	}

	/* Clear the APU power control register for this cpu */
	pm_client_wakeup(proc);

	/* Clear the mailbox for this cpu. */
	zynqmp_program_mailbox(mpidr, 0);
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
	.affinst_standby	= zynqmp_affinst_standby,
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
