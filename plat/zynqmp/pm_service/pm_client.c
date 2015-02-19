/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Client-Side Power Management Library - IPI Management Functions
**********************************************************************/

#include <bakery_lock.h>
#include <debug.h>
#include <gic_v2.h>
#include <mmio.h>
#include "pm_client.h"
#include "pm_svc_main.h"
#include "reg-ipi.h"
#include "reg-ipibuf.h"
#include "zynqmp-map.dtsh"

static bakery_lock_t pm_secure_lock __attribute__ ((section("tzfw_coherent_mem")));

int pm_ipi_init(void)
{
	bakery_lock_init(&pm_secure_lock);

	/*
	 * Ipi Interrupts Enable
	 */
	mmio_write_32(IPI + IPI_PMU_3_IER, ~0);
	mmio_write_32(IPI + IPI_APU_IER, ~0);

	return 0;
}

/*
 * Write to ipi buffer registers
 * @args
 */
static void write_ipi_buffer(const uint32_t base, const uint32_t api_id,
				const uint32_t arg0, const uint32_t arg1,
				const uint32_t arg2, const uint32_t arg3)
{
	mmio_write_32(base + IPIBUF_API_ID_OFFSET, api_id);
	mmio_write_32(base + IPIBUF_ARG1_OFFSET, arg0);
	mmio_write_32(base + IPIBUF_ARG2_OFFSET, arg1);
	mmio_write_32(base + IPIBUF_ARG3_OFFSET, arg2);
	mmio_write_32(base + IPIBUF_ARG4_OFFSET, arg3);
}

/* Read from ipi buffer register and populate global payload structure
 * @base - base address of ipi buffer registers
 *
 * Read value from ipi buffer registers and store
 * in PM context payload structure
 */
static void read_ipi_buffer(const uint32_t base)
{
	pm_ctx.pld->api_id = mmio_read_32(base + IPIBUF_API_ID_OFFSET);
	pm_ctx.pld->arg[0] = mmio_read_32(base + IPIBUF_ARG1_OFFSET);
	pm_ctx.pld->arg[1] = mmio_read_32(base + IPIBUF_ARG2_OFFSET);
	pm_ctx.pld->arg[2] = mmio_read_32(base + IPIBUF_ARG3_OFFSET);
	pm_ctx.pld->arg[3] = mmio_read_32(base + IPIBUF_ARG4_OFFSET);
}

void pm_ipi_msg_start()
{
	uint32_t ipi_base = IPI;
	bakery_lock_get(&pm_secure_lock);

	/* Wait until previous interrupt is handled by PMU */
	while (mmio_read_32(ipi_base + IPI_PU_BASE + IPI_OBS_OFFSET)
		& IPI_PMU_PM_INT_MASK)
		;
}

void pm_ipi_msg_end()
{
	bakery_lock_release(&pm_secure_lock);
}

void pm_ipi_send(const struct payload *pload)
{
	uint32_t ipi_base = IPI;
	uint32_t ipibuf_base = IPIBUF_BASEADDR;

	/* Write payload into memory buffer */
	write_ipi_buffer(ipibuf_base + IPIBUF_PU_BASE + IPIBUF_PMU_REQ_OFFSET,
			pload->api_id,
			pload->arg[0], pload->arg[1],
			pload->arg[2], pload->arg[3]);
	/* Generate IPI to PMU */
	mmio_write_32(ipi_base + IPI_PU_BASE + IPI_TRIG_OFFSET,
			IPI_PMU_PM_INT_MASK);
}

void pm_ipi_handler(const uint32_t api_id)
{
	/* Read data from IPIBUF reg and populate payload structure */
	read_ipi_buffer(IPIBUF_BASEADDR + IPIBUF_PMU_OFFSET + IPIBUF_APU_REQ_OFFSET);

	/*Set interrupt for Non secure EL1, apu_pm_notify handler in Linux*/
	gicd_set_ispendr(RDO_GICD_BASE, pm_ctx.pm_notify_irq);
	gicd_set_isactiver(RDO_GICD_BASE, pm_ctx.pm_notify_irq);

	VERBOSE("%s\n payload: %d, %d, %d, %d, %d\n", __func__,
			pm_ctx.pld->api_id,
			pm_ctx.pld->arg[0], pm_ctx.pld->arg[1],
			pm_ctx.pld->arg[2], pm_ctx.pld->arg[3]);
}

/* *************************************************
 * Top level IPI FIQ interrupt handler
 * Forwarded from Linux
 * Temporary here in lack of FIQ
 ***************************************************/
enum irqreturn_t ipi_fiq_handler(void)
{
	uint32_t ipi_apu_isr_reg;

	ipi_apu_isr_reg = mmio_read_32(IPI + IPI_APU_ISR);

	switch (ipi_apu_isr_reg) {
	case IPI_APU_ISR_PL_0_MASK:
		break;
	case IPI_APU_ISR_PL_1_MASK:
		break;
	case IPI_APU_ISR_PL_2_MASK:
		break;
	case IPI_APU_ISR_PL_3_MASK:
		break;
	case IPI_APU_ISR_PMU_0_MASK:
		break;
	case IPI_APU_ISR_PMU_1_MASK:
		break;
	case IPI_APU_ISR_PMU_2_MASK:
		break;
	case IPI_APU_ISR_PMU_3_MASK:
		pm_ipi_handler(0);
		break;
	case IPI_APU_ISR_RPU_0_MASK:
		break;
	case IPI_APU_ISR_RPU_1_MASK:
		break;
	case IPI_APU_ISR_APU_MASK:
		break;

	default:
		INFO("Unknowk Interrupt, APU_ISR 0x%x\n", ipi_apu_isr_reg);
		return IRQ_FAIL;
	}

	/* clear  IPI_APU_ISR */
	mmio_write_32((IPI + IPI_APU_ISR), ipi_apu_isr_reg);

	return IRQ_OK;
}
