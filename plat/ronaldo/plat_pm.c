/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
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
#include "drivers/pwrc/fvp_pwrc.h"
#include "fvp_private.h"

/*******************************************************************************
 * Private Ronaldo function to program the mailbox for a cpu before it is released
 * from reset.
 ******************************************************************************/
static void ronaldo_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	mailbox_t *fvp_mboxes;

	linear_id = platform_get_core_pos(mpidr);
	fvp_mboxes = (mailbox_t *)MBOX_BASE;
	fvp_mboxes[linear_id].value = address;
	flush_dcache_range((unsigned long) &fvp_mboxes[linear_id],
			   sizeof(unsigned long));
}

/*******************************************************************************
 * Function which implements the common Ronaldo specific operations to power down a
 * cpu in response to a CPU_OFF or CPU_SUSPEND request.
 ******************************************************************************/
static void ronaldo_cpu_pwrdwn_common(void)
{
	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	/* Program the power controller to power off this cpu. */
	fvp_pwrc_write_ppoffr(read_mpidr_el1());
}

/*******************************************************************************
 * Private Ronaldo function which is used to determine if any platform actions
 * should be performed for the specified affinity instance given its
 * state. Nothing needs to be done if the 'state' is not off or if this is not
 * the highest affinity level which will enter the 'state'.
 ******************************************************************************/
static int32_t ronaldo_do_plat_actions(uint32_t afflvl, uint32_t state)
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
 * Ronaldo handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
static int32_t ronaldo_affinst_on(uint64_t mpidr,
				  uint64_t sec_entrypoint,
				  uint64_t ns_entrypoint,
				  uint32_t afflvl,
				  uint32_t state)
{
	uint32_t psysr;

	/*
	 * It's possible to turn on only affinity level 0 i.e. a cpu
	 * on the Ronaldo. Ignore any other affinity level.
	 */
	if (afflvl != MPIDR_AFFLVL0)
		return PSCI_E_SUCCESS;

	/*
	 * Ensure that we do not cancel an inflight power off request
	 * for the target cpu. That would leave it in a zombie wfi.
	 * Wait for it to power off, program the jump address for the
	 * target cpu and then program the power controller to turn
	 * that cpu on
	 */
	do {
		psysr = fvp_pwrc_read_psysr(mpidr);
	} while (psysr & PSYSR_AFF_L0);

	ronaldo_program_mailbox(mpidr, sec_entrypoint);

	mmio_write_32(R_RVBAR_L_0 + mpidr * 8, sec_entrypoint);
	mmio_write_32(R_RVBAR_H_0 + mpidr * 8, sec_entrypoint >> 32);
	dsb();

	fvp_pwrc_write_pponr(mpidr);

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Ronaldo handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take
 * appropriate actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across calls
 * to this function as each affinity level is dealt with. So do not write & read
 * global variables across calls. It will be wise to do flush a write to the
 * global to prevent unpredictable results.
 ******************************************************************************/
static int32_t ronaldo_affinst_off(uint64_t mpidr,
				   uint32_t afflvl,
				   uint32_t state)
{
	/* Determine if any platform actions need to be executed */
	if (ronaldo_do_plat_actions(afflvl, state) == -EAGAIN)
		return PSCI_E_SUCCESS;

	/*
	 * If execution reaches this stage then this affinity level will be
	 * suspended. Perform at least the cpu specific actions followed the
	 * cluster specific operations if applicable.
	 */
	ronaldo_cpu_pwrdwn_common();

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Ronaldo handler called when an affinity instance is about to be suspended. The
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
static int32_t ronaldo_affinst_suspend(uint64_t mpidr,
				       uint64_t sec_entrypoint,
				       uint64_t ns_entrypoint,
				       uint32_t afflvl,
				       uint32_t state)
{
	/* Determine if any platform actions need to be executed. */
	if (ronaldo_do_plat_actions(afflvl, state) == -EAGAIN)
		return PSCI_E_SUCCESS;

	/* Program the jump address for the target cpu */
	ronaldo_program_mailbox(read_mpidr_el1(), sec_entrypoint);

	/* Program the power controller to enable wakeup interrupts. */
	fvp_pwrc_set_wen(mpidr);

	/* Perform the common cpu specific operations */
	ronaldo_cpu_pwrdwn_common();

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Ronaldo handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
static int32_t ronaldo_affinst_on_finish(uint64_t mpidr,
					 uint32_t afflvl,
					 uint32_t state)
{
	/* Determine if any platform actions need to be executed. */
	if (ronaldo_do_plat_actions(afflvl, state) == -EAGAIN)
		return PSCI_E_SUCCESS;

	/*
	 * Clear PWKUPR.WEN bit to ensure interrupts do not interfere
	 * with a cpu power down unless the bit is set again
	 */
	fvp_pwrc_clr_wen(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	ronaldo_program_mailbox(read_mpidr_el1(), 0);

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: This setup is needed only after a cold boot */
	arm_gic_pcpu_distif_setup();

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Ronaldo handler called when an affinity instance has just been powered on after
 * having been suspended earlier. The level and mpidr determine the affinity
 * instance.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
static int32_t ronaldo_affinst_suspend_finish(uint64_t mpidr,
					      uint32_t afflvl,
					      uint32_t state)
{
	return ronaldo_affinst_on_finish(mpidr, afflvl, state);
}


/*******************************************************************************
 * Ronaldo handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 ronaldo_system_off(void)
{
#if 0
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_SHUTDOWN));
	wfi();
#endif
	ERROR("Ronaldo System Off: operation not handled.\n");
	panic();
}

static void __dead2 ronaldo_system_reset(void)
{
#if 0
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_REBOOT));
	wfi();
#endif
	ERROR("Ronaldo System Reset: operation not handled.\n");
	panic();
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t ronaldo_ops = {
	.affinst_on		= ronaldo_affinst_on,
	.affinst_off		= ronaldo_affinst_off,
	.affinst_suspend	= ronaldo_affinst_suspend,
	.affinst_on_finish	= ronaldo_affinst_on_finish,
	.affinst_suspend_finish	= ronaldo_affinst_suspend_finish,
	.system_off		= ronaldo_system_off,
	.system_reset		= ronaldo_system_reset
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &ronaldo_ops;
	return 0;
}
