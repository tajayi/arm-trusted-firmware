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

#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <bl_common.h>
#include <cci400.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <plat_config.h>
#include <xlat_tables.h>
#include "../zynqmp_def.h"

/*******************************************************************************
 * plat_config holds the characteristics of the differences between the three
 * ZYNQMP platforms (Base, A53_A57 & Foundation). It will be populated during cold
 * boot at each boot stage by the primary before enabling the MMU (to allow cci
 * configuration) & used thereafter. Each BL will have its own copy to allow
 * independent operation.
 ******************************************************************************/
plat_config_t plat_config;

/*
 * Table of regions to map using the MMU.
 * This doesn't include TZRAM as the 'mem_layout' argument passed to
 * configure_mmu_elx() will give the available subset of that,
 */
const mmap_region_t zynqmp_mmap[] = {
	{ ZYNQMP_SHARED_RAM_BASE,	ZYNQMP_SHARED_RAM_BASE,	ZYNQMP_SHARED_RAM_SIZE,
						MT_MEMORY | MT_RW | MT_SECURE },
	{ ZYNQMP_TRUSTED_DRAM_BASE, ZYNQMP_TRUSTED_DRAM_BASE,	ZYNQMP_TRUSTED_DRAM_SIZE,
						MT_MEMORY | MT_RW | MT_SECURE },
	{ DEVICE0_BASE,	DEVICE0_BASE,	DEVICE0_SIZE,
						MT_DEVICE | MT_RW | MT_SECURE },
	{ DEVICE1_BASE,	DEVICE1_BASE,	DEVICE1_SIZE,
						MT_DEVICE | MT_RW | MT_SECURE },
	{ APB_BASE,	APB_BASE,	APB_SIZE,
						MT_DEVICE | MT_RW | MT_SECURE },
	{ DRAM1_BASE,	DRAM1_BASE,	DRAM1_SIZE,
						MT_MEMORY | MT_RW | MT_NS },
	{0}
};

/* Array of secure interrupts to be configured by the gic driver */
const unsigned int irq_sec_array[] = {
	IRQ_TZ_WDOG,
	IRQ_SEC_PHY_TIMER,
	IRQ_SEC_SGI_0,
	IRQ_SEC_SGI_1,
	IRQ_SEC_SGI_2,
	IRQ_SEC_SGI_3,
	IRQ_SEC_SGI_4,
	IRQ_SEC_SGI_5,
	IRQ_SEC_SGI_6,
	IRQ_SEC_SGI_7
};

const unsigned int num_sec_irqs = sizeof(irq_sec_array) /
	sizeof(irq_sec_array[0]);

/*******************************************************************************
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 ******************************************************************************/
#define DEFINE_CONFIGURE_MMU_EL(_el)					\
	void zynqmp_configure_mmu_el##_el(unsigned long total_base,	\
				   unsigned long total_size,		\
				   unsigned long ro_start,		\
				   unsigned long ro_limit,		\
				   unsigned long coh_start,		\
				   unsigned long coh_limit)		\
	{								\
		mmap_add_region(total_base, total_base,			\
				total_size,				\
				MT_MEMORY | MT_RW | MT_SECURE);		\
		mmap_add_region(ro_start, ro_start,			\
				ro_limit - ro_start,			\
				MT_MEMORY | MT_RO | MT_SECURE);		\
		mmap_add_region(coh_start, coh_start,			\
				coh_limit - coh_start,			\
				MT_DEVICE | MT_RW | MT_SECURE);		\
		mmap_add(zynqmp_mmap);					\
		init_xlat_tables();					\
									\
		enable_mmu_el##_el(0);					\
	}

/* Define EL1 and EL3 variants of the function initialising the MMU */
DEFINE_CONFIGURE_MMU_EL(1)
DEFINE_CONFIGURE_MMU_EL(3)

#define ZYNQMP_SILICON_VER_MASK   0xF000
#define ZYNQMP_SILICON_VER_SHIFT  12
#define ZYNQMP_CSU_VERSION_SILICON      0x0
#define ZYNQMP_CSU_VERSION_EP108        0x1
#define ZYNQMP_CSU_VERSION_VELOCE       0x2
#define ZYNQMP_CSU_VERSION_QEMU         0x3

#define ZYNQMP_RTL_VER_MASK   0xFF0
#define ZYNQMP_RTL_VER_SHIFT  4

#define ZYNQMP_CSU_BASEADDR		0xFFCA0000
#define ZYNQMP_CSU_VERSION_OFFSET	0x44

static unsigned int zynqmp_get_silicon_ver(void)
{
	uint32_t ver;

	ver = mmio_read_32(ZYNQMP_CSU_BASEADDR + ZYNQMP_CSU_VERSION_OFFSET);
	ver &= ZYNQMP_SILICON_VER_MASK;
	ver >>= ZYNQMP_SILICON_VER_SHIFT;

	return ver;
}

static uint32_t zynqmp_get_rtl_ver(void)
{
	uint32_t ver;

	ver = mmio_read_32(ZYNQMP_CSU_BASEADDR + ZYNQMP_CSU_VERSION_OFFSET);
	ver &= ZYNQMP_RTL_VER_MASK;
	ver >>= ZYNQMP_RTL_VER_SHIFT;

	return ver;
}

uint32_t zynqmp_get_uart_clk(void)
{
	uint32_t ver = zynqmp_get_silicon_ver();

	switch (ver) {
	case ZYNQMP_CSU_VERSION_VELOCE:
		return  96000;
	case ZYNQMP_CSU_VERSION_EP108:
		return 25000000;
	}

	return 133000000;
}

static unsigned int zynqmp_get_silicon_freq(void)
{
	uint32_t ver = zynqmp_get_silicon_ver();

	switch (ver) {
	case ZYNQMP_CSU_VERSION_VELOCE:
		return 20000;
	case ZYNQMP_CSU_VERSION_EP108:
		return 4000000;
	}

	/* FIXME Qemu is exporting incorrect bit in this reg. 0 is allocated to real silicon */
	return 50000000;
}

static void zynqmp_print_platform_name(void)
{
	uint32_t ver = zynqmp_get_silicon_ver();
	uint32_t rtl = zynqmp_get_rtl_ver();

	switch (ver) {
	case ZYNQMP_CSU_VERSION_VELOCE:
		NOTICE("ATF running on VELOCE/RTL%d.%d\n", (rtl & 0xf0) >> 4, rtl & 0xf);
		return;
	case ZYNQMP_CSU_VERSION_EP108:
		NOTICE("ATF running on EP108/RTL%d.%d\n", (rtl & 0xf0) >> 4, rtl & 0xf);
		return;
	}

	NOTICE("ATF running on QEMU/RTL%d.%d\n", (rtl & 0xf0) >> 4, rtl & 0xf);
}

#define PMU_GLOBAL_CNTRL	0xFFD80000
#define FW_IS_PRESENT		(1 << 4)

/*
 * zynqmp_is_pmu_up - Find if PMU firmware is up and running
 *
 * Return 0 if firmware is not available, non 0 otherwise
 */
uint32_t zynqmp_is_pmu_up(void)
{
	uint32_t ver;

	ver = mmio_read_32(PMU_GLOBAL_CNTRL);
	ver &= FW_IS_PRESENT;

	return ver;
}

#define ZYNQMP_CRL_APB_BASEADDR				0xFF5E0000
#define ZYNQMP_CRL_APB_TIMESTAMP_REF_CTRL_OFFSET	0x128
#define ZYNQMP_CRL_APB_TIMESTAMP_REF_CTRL_CLKACT_BIT	(1 << 24)

#define ZYNQMP_IOU_SCNTRS_BASEADDR		0xFF260000
#define ZYNQMP_IOU_SCNTRS_CONTROL_OFFSET	0x0
#define ZYNQMP_IOU_SCNTRS_CONTROL_EN		(1 << 0)
#define ZYNQMP_IOU_SCNTRS_BASEFREQ_OFFSET	0x20
/*******************************************************************************
 * A single boot loader stack is expected to work on both the Foundation ZYNQMP
 * models and the two flavours of the Base ZYNQMP models (AEMv8 & Cortex). The
 * SYS_ID register provides a mechanism for detecting the differences between
 * these platforms. This information is stored in a per-BL array to allow the
 * code to take the correct path.Per BL platform configuration.
 ******************************************************************************/
int zynqmp_config_setup(void)
{
	uint32_t val;

	plat_config.gicd_base = RDO_GICD_BASE;
	plat_config.gicc_base = RDO_GICC_BASE;
	plat_config.gich_base = RDO_GICH_BASE;
	plat_config.gicv_base = RDO_GICV_BASE;

	zynqmp_print_platform_name();

	/* Global timer init - Program time stamp reference clk */
	val = mmio_read_32(ZYNQMP_CRL_APB_BASEADDR +
			   ZYNQMP_CRL_APB_TIMESTAMP_REF_CTRL_OFFSET);
	val |= ZYNQMP_CRL_APB_TIMESTAMP_REF_CTRL_CLKACT_BIT;
	mmio_write_32(ZYNQMP_CRL_APB_BASEADDR +
		      ZYNQMP_CRL_APB_TIMESTAMP_REF_CTRL_OFFSET, val);

	/* Program freq register in System counter  and enable system counter. */
	mmio_write_32(ZYNQMP_IOU_SCNTRS_BASEADDR +
		      ZYNQMP_IOU_SCNTRS_BASEFREQ_OFFSET,
		      zynqmp_get_silicon_freq());
	mmio_write_32(ZYNQMP_IOU_SCNTRS_BASEADDR +
	              ZYNQMP_IOU_SCNTRS_CONTROL_OFFSET,
		      ZYNQMP_IOU_SCNTRS_CONTROL_EN);

	plat_config.flags |= CONFIG_HAS_CCI;

	return 0;
}

unsigned long plat_get_ns_image_entrypoint(void)
{
	return NS_IMAGE_OFFSET;
}

uint64_t plat_get_syscnt_freq(void)
{
	uint64_t counter_base_frequency;

	/* FIXME: Read the frequency from Frequency modes table */
	counter_base_frequency = zynqmp_get_silicon_freq();

	return counter_base_frequency;
}

void zynqmp_cci_init(void)
{
	/*
	 * Initialize CCI-400 driver
	 */
	if (plat_config.flags & CONFIG_HAS_CCI)
		cci_init(CCI400_BASE,
			CCI400_SL_IFACE3_CLUSTER_IX,
			CCI400_SL_IFACE4_CLUSTER_IX);
}

void zynqmp_cci_enable(void)
{
	/*
	 * Enable CCI-400 coherency for this cluster. No need
	 * for locks as no other cpu is active at the
	 * moment
	 */
	if (plat_config.flags & CONFIG_HAS_CCI)
		cci_enable_cluster_coherency(read_mpidr());
}

void zynqmp_gic_init(void)
{
	arm_gic_init(plat_config.gicc_base,
		plat_config.gicd_base,
		1,
		irq_sec_array,
		num_sec_irqs);
}


/*******************************************************************************
 * Gets SPSR for BL32 entry
 ******************************************************************************/
uint32_t zynqmp_get_spsr_for_bl32_entry(void)
{
	/*
	 * The Secure Payload Dispatcher service is responsible for
	 * setting the SPSR prior to entry into the BL32 image.
	 */
	return 0;
}

/*******************************************************************************
 * Gets SPSR for BL33 entry
 ******************************************************************************/
uint32_t zynqmp_get_spsr_for_bl33_entry(void)
{
	unsigned long el_status;
	unsigned int mode;
	uint32_t spsr;

	/* Figure out what mode we enter the non-secure world in */
	el_status = read_id_aa64pfr0_el1() >> ID_AA64PFR0_EL2_SHIFT;
	el_status &= ID_AA64PFR0_ELX_MASK;

	if (el_status)
		mode = MODE_EL2;
	else
		mode = MODE_EL1;

	/*
	 * TODO: Consider the possibility of specifying the SPSR in
	 * the FIP ToC and allowing the platform to have a say as
	 * well.
	 */
	spsr = SPSR_64(mode, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
	return spsr;
}
