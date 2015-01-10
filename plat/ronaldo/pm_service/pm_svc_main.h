/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Contains PM's flags, macros and data for communicaton
* with non-secure part
*********************************************************************/

#ifndef _PM_PRIVATE_H_
#define _PM_PRIVATE_H_

/* PM Function identifiers  */
#define PM_SMC_INIT			0xa01
#define PM_SMC_NOTIFY			0xa02
/* Temporary, in lack of FIQs */
#define PM_SMC_IRQ			0xa03

/*******************************************************************************
 * Structure which contains data for power management
 * 'pm_notify_irq'	-entrance in Gic used for pm_notify action
 * '*pld'		-pointer to payload structure which contains received
 * 			data from ipi buffer registers
 ******************************************************************************/
struct pm_context {
	uint32_t pm_notify_irq;
	struct payload *pld;
};

extern struct pm_context pm_ctx;

uint32_t pm_setup(void);
uint64_t pm_smc_handler(uint32_t smc_fid,
			uint64_t x1,
			uint64_t x2,
			uint64_t x3,
			uint64_t x4,
			void *cookie,
			void *handle,
			uint64_t flags);

#endif /*  _PM_PRIVATE_H_ */
