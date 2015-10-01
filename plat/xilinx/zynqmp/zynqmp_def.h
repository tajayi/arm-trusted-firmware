/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
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

#ifndef __ZYNQMP_DEF_H__
#define __ZYNQMP_DEF_H__

/* Firmware Image Package */
#define FIP_IMAGE_NAME			"fip.bin"
#define ZYNQMP_PRIMARY_CPU			0x0

/* Memory location options for Shared data and TSP in ZYNQMP */
#define ZYNQMP_IN_TRUSTED_SRAM		0
#define ZYNQMP_IN_TRUSTED_DRAM		1

/*******************************************************************************
 * ZYNQMP memory map related constants
 ******************************************************************************/

#define ZYNQMP_TRUSTED_SRAM_BASE	0xFFFC0000
#define ZYNQMP_TRUSTED_SRAM_SIZE	0x00040000

/* Location of trusted dram on the base zynqmp */
#define ZYNQMP_TRUSTED_DRAM_BASE	0x30000000 /* Can't overlap TZROM area */
#define ZYNQMP_TRUSTED_DRAM_SIZE	0x10000000

#define FLASH0_BASE		0x08000000
#define FLASH0_SIZE		TZROM_SIZE

#define FLASH1_BASE		0x0c000000
#define FLASH1_SIZE		0x04000000

#define PSRAM_BASE		0x14000000
#define PSRAM_SIZE		0x04000000

#define VRAM_BASE		0x18000000
#define VRAM_SIZE		0x02000000

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE		0xFF000000
#define DEVICE0_SIZE		0x00E00000
#define DEVICE1_BASE		0xF9000000
#define DEVICE1_SIZE		0x01000000

/* For cpu reset APU space here too 0xFE5F1000 CRF_APB*/
#define APB_BASE		0xFD1A0000
#define APB_SIZE		0x00600000

#define APU_BASE		(0xFD5C0000) /* APU */
#define R_RVBAR_L_0		(APU_BASE + 0x40)
#define R_RVBAR_H_0		(APU_BASE + 0x44)
#define CRF_APB_RST_FPD_APU	(APB_BASE + 0X00000104)

#define NSRAM_BASE		0x2e000000
#define NSRAM_SIZE		0x10000

/* 4KB shared memory */
#define ZYNQMP_SHARED_RAM_SIZE	0x1000

/* Location of shared memory */
#if (ZYNQMP_SHARED_DATA_LOCATION_ID == ZYNQMP_IN_TRUSTED_DRAM)
/* Shared memory at the base of Trusted DRAM */
# define ZYNQMP_SHARED_RAM_BASE		ZYNQMP_TRUSTED_DRAM_BASE
# define ZYNQMP_TRUSTED_SRAM_LIMIT		(ZYNQMP_TRUSTED_SRAM_BASE \
					+ ZYNQMP_TRUSTED_SRAM_SIZE)
#elif (ZYNQMP_SHARED_DATA_LOCATION_ID == ZYNQMP_IN_TRUSTED_SRAM)
# if (ZYNQMP_TSP_RAM_LOCATION_ID == ZYNQMP_IN_TRUSTED_DRAM)
#  error "Shared data in Trusted SRAM and TSP in Trusted DRAM is not supported"
# endif
/* Shared memory at the top of the Trusted SRAM */
# define ZYNQMP_SHARED_RAM_BASE		(ZYNQMP_TRUSTED_SRAM_BASE \
					+ ZYNQMP_TRUSTED_SRAM_SIZE \
					- ZYNQMP_SHARED_RAM_SIZE)
# define ZYNQMP_TRUSTED_SRAM_LIMIT		ZYNQMP_SHARED_RAM_BASE
#else
# error "Unsupported ZYNQMP_SHARED_DATA_LOCATION_ID value"
#endif

#define DRAM1_BASE		0x00000000ull
#define DRAM1_SIZE		0x10000000ull
#define DRAM1_END		(DRAM1_BASE + DRAM1_SIZE - 1)
#define DRAM1_SEC_SIZE		0x01000000ull

#define DRAM_BASE		DRAM1_BASE
#define DRAM_SIZE		DRAM1_SIZE

#define DRAM2_BASE		0x880000000ull
#define DRAM2_SIZE		0x780000000ull
#define DRAM2_END		(DRAM2_BASE + DRAM2_SIZE - 1)

#define PCIE_EXP_BASE		0x40000000
#define TZRNG_BASE		0x7fe60000
#define TZNVCTR_BASE		0x7fe70000
#define TZROOTKEY_BASE		0x7fe80000

/* Load address of BL33 in the ZYNQMP port */
#define PLAT_ARM_NS_IMAGE_OFFSET	(DRAM1_BASE + 0x8000000) /* DRAM + 128MB */

/* Special value used to verify platform parameters from BL2 to BL3-1 */
#define ZYNQMP_BL31_PLAT_PARAM_VAL	0x0f1e2d3c4b5a6978ULL

/*
 * V2M sysled bit definitions. The values written to this
 * register are defined in arch.h & runtime_svc.h. Only
 * used by the primary cpu to diagnose any cold boot issues.
 *
 * SYS_LED[0]   - Security state (S=0/NS=1)
 * SYS_LED[2:1] - Exception Level (EL3-EL0)
 * SYS_LED[7:3] - Exception Class (Sync/Async & origin)
 *
 */
#define SYS_LED_SS_SHIFT		0x0
#define SYS_LED_EL_SHIFT		0x1
#define SYS_LED_EC_SHIFT		0x3

#define SYS_LED_SS_MASK		0x1
#define SYS_LED_EL_MASK		0x3
#define SYS_LED_EC_MASK		0x1f

/* V2M sysid register bits */
#define SYS_ID_REV_SHIFT	28
#define SYS_ID_HBI_SHIFT	16
#define SYS_ID_BLD_SHIFT	12
#define SYS_ID_ARCH_SHIFT	8
#define SYS_ID_FPGA_SHIFT	0

#define SYS_ID_REV_MASK	0xf
#define SYS_ID_HBI_MASK	0xfff
#define SYS_ID_BLD_MASK	0xf
#define SYS_ID_ARCH_MASK	0xf
#define SYS_ID_FPGA_MASK	0xff

#define SYS_ID_BLD_LENGTH	4

#define HBI_ZYNQMP_BASE		0x020
#define REV_ZYNQMP_BASE_V0		0x0

#define HBI_FOUNDATION		0x010
#define REV_FOUNDATION_V2_0	0x0
#define REV_FOUNDATION_V2_1	0x1

#define BLD_GIC_VE_MMAP	0x0
#define BLD_GIC_A53A57_MMAP	0x1

/* ZYNQMP Power controller base address*/
#define PWRC_BASE		0x1c100000


/*******************************************************************************
 * CCI-400 related constants
 ******************************************************************************/
#define PLAT_ARM_CCI_BASE		0xFD6E0000
#define PLAT_ARM_CCI_CLUSTER0_SL_IFACE_IX	3
#define PLAT_ARM_CCI_CLUSTER1_SL_IFACE_IX	4

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
#define BASE_GICD_BASE		0xF9010000
#define BASE_GICC_BASE		0xF9020000
#define BASE_GICH_BASE		0xF9040000
#define BASE_GICV_BASE		0xF9060000

#define IRQ_SEC_IPI_APU			67
#define ARM_IRQ_SEC_PHY_TIMER		29

#define ARM_IRQ_SEC_SGI_0		8
#define ARM_IRQ_SEC_SGI_1		9
#define ARM_IRQ_SEC_SGI_2		10
#define ARM_IRQ_SEC_SGI_3		11
#define ARM_IRQ_SEC_SGI_4		12
#define ARM_IRQ_SEC_SGI_5		13
#define ARM_IRQ_SEC_SGI_6		14
#define ARM_IRQ_SEC_SGI_7		15

#define MAX_INTR_EL3			128

/*******************************************************************************
 * UART related constants
 ******************************************************************************/
#define RDO_UART0_BASE         0xFF000000
#define RDO_UART1_BASE         0xFF001000

#define PLAT_ARM_CRASH_UART_BASE	RDO_UART0_BASE
/* impossible to call C routine how it is done now - hardcode any value */
#define	PLAT_ARM_CRASH_UART_CLK_IN_HZ	25000000 /* FIXME */

/* Must be non zero */
#define CADENCE_UART_BAUDRATE (115200)

#define ARM_CONSOLE_BAUDRATE	CADENCE_UART_BAUDRATE
/*******************************************************************************
 *  Shared Data
 ******************************************************************************/

/* Entrypoint mailboxes */
#define MBOX_BASE		ZYNQMP_SHARED_RAM_BASE
#define MBOX_SIZE		0x200

/* Base address where parameters to BL31 are stored */
#define PARAMS_BASE		(MBOX_BASE + MBOX_SIZE)

#define APU_PWRCTL		(APU_BASE + 0x00000090)

#define APU_0_PWRCTL_CPUPWRDWNREQ_MASK		1
#define APU_1_PWRCTL_CPUPWRDWNREQ_MASK		2
#define APU_2_PWRCTL_CPUPWRDWNREQ_MASK		4
#define APU_3_PWRCTL_CPUPWRDWNREQ_MASK		8

/* IPI Base Address */
#define IPI_BASEADDR		0XFF300000

/* APU's IPI registers */
#define IPI_APU_ISR		(IPI_BASEADDR + 0X00000010)
#define IPI_APU_IER		(IPI_BASEADDR + 0X00000018)
#define IPI_APU_IDR		(IPI_BASEADDR + 0X0000001C)
#define IPI_APU_ISR_PMU_0_MASK		0X00010000
#define IPI_APU_IER_PMU_0_MASK		0X00010000

#endif /* __ZYNQMP_DEF_H__ */
