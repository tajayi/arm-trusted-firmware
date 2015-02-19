/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Client-Side Power Management Library - IPI Management Functions
*********************************************************************/

#ifndef _PM_CLIENT_H_
#define _PM_CLIENT_H_

#define IPI_APU_MASK		0x00000001
#define IPI_RPU_0_MASK		0x00000100
#define IPI_RPU_1_MASK		0x00000200

#define IPI_TRIG_OFFSET		0x00000000
#define IPI_OBS_OFFSET		0x00000004
#define IPI_ISR_OFFSET		0x00000010

/* Power Management IPI interrupt number, to configure here */
#define PM_INT_NUM		0
#define IPI_PMU_PM_INT_BASE	(IPI_PMU_0_TRIG + (PM_INT_NUM * 0x1000))
#define IPI_PMU_PM_INT_MASK	(IPI_APU_ISR_PMU_0_MASK << PM_INT_NUM)
#if (PM_INT_NUM < 0 || PM_INT_NUM > 3)
	#error PM_INT_NUM value out of range
#endif

#define IPI_PU_BASE	0x00000000
#define IPIBUF_PU_BASE	IPIBUF_APU_OFFSET

#define IPIBUF_API_ID_OFFSET	0x0
#define IPIBUF_ARG1_OFFSET	(IPIBUF_API_ID_OFFSET + 0x4)
#define IPIBUF_ARG2_OFFSET	(IPIBUF_API_ID_OFFSET + 0x8)
#define IPIBUF_ARG3_OFFSET	(IPIBUF_API_ID_OFFSET + 0xC)
#define IPIBUF_ARG4_OFFSET	(IPIBUF_API_ID_OFFSET + 0x10)

#define PM_ARRAY_SIZE(x)	(sizeof(x) / sizeof(x[0]))

enum irqreturn_t {
	IRQ_OK = 1,
	IRQ_FAIL,
};

struct payload {
	uint32_t api_id;
	uint32_t arg[4];
};

/*******************************************
 *Function Prototypes
 *******************************************/
extern int pm_ipi_init(void);
extern void pm_ipi_msg_start(void);
extern void pm_ipi_msg_end(void);
extern void pm_ipi_send(const struct payload *pload);
/* Temporary here in lack of FIQs */
enum irqreturn_t ipi_fiq_handler(void);

#endif /* _PM_CLIENT_H_ */
