/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* System Level PM API function implementations
*********************************************************************/

#include <arch_helpers.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include "pm_api_sys.h"
#include "pm_client.h"
#include "reg-apu.h"
#include "zynqmp-map.dtsh"

#define DISABLE_PWRDN_REQ	0U
#define ENABLE_PWRDN_REQ	1U

#define NODE_ID_ERROR	0
#define NODE_ID_OK	1

#define CPU_PWRDN_REQ_ERROR	0U
#define CPU_PWRDN_REQ_OK	1U

#define UNDEFINED_CPUID	(~0)

static const enum pm_node_id cpu_node_ids[] = { NODE_APU_0, NODE_APU_1, NODE_APU_2, NODE_APU_3 };

/* Send API request via IPI layer */
static void send_ipi_request(uint32_t _api_id, uint32_t _arg0,
				uint32_t _arg1, uint32_t _arg2,
				uint32_t  _arg3)
{
	struct payload pload;

	pm_ipi_msg_start();
	pload.api_id = _api_id;
	pload.arg[0] = _arg0;
	pload.arg[1] = _arg1;
	pload.arg[2] = _arg2;
	pload.arg[3] = _arg3;
	pm_ipi_send(&pload);
	pm_ipi_msg_end();
}

/* Determine PM Node ID from CPU ID (cpu0..3) */
enum pm_node_id pm_get_node_id(const uint32_t cpuid)
{
	if (cpuid >= 0 && cpuid < PM_ARRAY_SIZE(cpu_node_ids)) {
		return cpu_node_ids[cpuid];
	}
	return NODE_UNKNOWN;
}

/* Determine CPU ID (0..3) from PM Node ID */
static uint32_t pm_get_cpuid(const enum pm_node_id node)
{
	uint32_t i;
	for (i = 0; i < PM_ARRAY_SIZE(cpu_node_ids); i++) {
		if (cpu_node_ids[i] == node) {
			return i;
		}
	}
	return UNDEFINED_CPUID;
}

/* Power-down request for an individual cpu */
static void pm_cpu_pwrdn_req(const uint32_t cpuid, const uint32_t eflag)
{
	uint32_t val = mmio_read_32(APU + APU_PWRCTL);
	if (eflag == ENABLE_PWRDN_REQ) {
		val |= /*APU_PWRCTL_CPUPWRDWNREQ_MASK &*/ (1 << cpuid);
	} else {
		val &= ~(/*APU_PWRCTL_CPUPWRDWNREQ_MASK &*/ (1 << cpuid));
	}
	mmio_write_32(APU + APU_PWRCTL, val);
	tf_printf("id = %d, 0x%x\n", cpuid, val);
}

/****************************************************************
 * PM-API functions:
 ****************************************************************/

/* APIs for suspending of PUs: */
void pm_req_suspend(const enum pm_node_id node, const enum pm_request_ack ack,
			const uint32_t latency, const uint8_t state)
{
	send_ipi_request(PM_REQ_SUSPEND, node, ack, latency, state);
	VERBOSE("ATF:%s(%d, %d, %d, %d)\n", __func__,
				node, ack, latency, state);
}

void pm_self_suspend(const enum pm_node_id node, const enum pm_request_ack ack,
			const uint32_t latency, const uint8_t state)
{
	uint32_t cpuid = platform_get_core_pos(read_mpidr_el1());

	tf_printf("CPUID = %d\n", cpuid);
	if ((0 == cpuid && NODE_APU == node) || (pm_get_node_id(cpuid) == node)) {
		pm_cpu_pwrdn_req(cpuid, ENABLE_PWRDN_REQ);
		send_ipi_request(PM_SELF_SUSPEND, node, ack, latency, state);
		VERBOSE("ATF:%s(%d, %d, %d, %d)\n", __func__,
					node, ack, latency, state);
	} else {
		VERBOSE("ATF:ERROR in %s(%d, %d, %d, %d)\n", __func__,
					node, ack, latency, state);
	}
}

void pm_force_powerdown(const enum pm_node_id node, const enum pm_request_ack ack)
{
	send_ipi_request(PM_FORCE_POWERDOWN, node, ack, 0, 0);
	VERBOSE("ATF: %s(%d, %d)\n", __func__, node, ack);
}

void pm_abort_suspend(const enum pm_abort_reason reason)
{
	send_ipi_request(PM_ABORT_SUSPEND, reason, 0, 0, 0);
	VERBOSE("ATF: %s(%d)\n", __func__, reason);
}

void pm_req_wakeup(const enum pm_node_id node, const enum pm_request_ack ack)
{
	uint32_t cpuid = pm_get_cpuid(node);
	if (UNDEFINED_CPUID != cpuid) {
		pm_cpu_pwrdn_req(cpuid, DISABLE_PWRDN_REQ);
	}
	send_ipi_request(PM_REQ_WAKEUP, node, ack, 0, 0);
	VERBOSE("ATF: %s(%d, %d)\n", __func__, node, ack);
}

void pm_set_wakeup_source(const enum pm_node_id target,
			const enum pm_node_id wkup_node,
			const uint8_t enable)
{
	send_ipi_request(PM_SET_WAKEUP_SOURCE, target, wkup_node, enable, 0);
	VERBOSE("ATF: %s(%d, %d, %d)\n", __func__, target, wkup_node, enable);
}

void pm_system_shutdown(const uint8_t restart)
{
	send_ipi_request(PM_SYSTEM_SHUTDOWN, restart, 0, 0, 0);
	VERBOSE("ATF: %s(%d)\n", __func__, restart);
}

/* APIs for managing PM slaves: */
void pm_req_node(const enum pm_node_id node, const uint32_t capabilities,
			const uint32_t qos, const enum pm_request_ack ack)
{
	send_ipi_request(PM_REQ_NODE, node, capabilities, qos, ack);
	VERBOSE("ATF: %s(%d, %d, %d, %d)\n", __func__,
				node, capabilities, qos, ack);
}
void pm_release_node(const enum pm_node_id node, const uint32_t latency)
{
	send_ipi_request(PM_RELEASE_NODE, node, latency, 0, 0);
	VERBOSE("ATF: %s(%d, %d)\n", __func__, node, latency);
}

void pm_set_requirement(const enum pm_node_id node,
			const uint32_t capabilities, const uint32_t qos,
			const enum pm_request_ack ack)
{
	send_ipi_request(PM_SET_REQUIREMENT, node, capabilities, qos, ack);
	VERBOSE("ATF: %s(%d, %d, %d, %d)\n",
				__func__, node, capabilities, qos, ack);
}

void pm_set_max_latency(const enum pm_node_id node,
			const uint32_t latency)
{
	send_ipi_request(PM_SET_MAX_LATENCY, node, latency, 0, 0);
	VERBOSE("ATF: %s(%d, %d, %d, %d)\n", __func__, node, latency, 0, 0);
}

/* Miscellaneous API functions */
void pm_get_api_version(void)
{
	send_ipi_request(PM_GET_API_VERSION, 0, 0, 0, 0);
	VERBOSE("ATF :%s\n", __func__);
}

void pm_get_node_status(const enum pm_node_id node)
{
	send_ipi_request(PM_GET_NODE_STATUS, node, 0, 0, 0);
	VERBOSE("ATF: %s\n(%d)", __func__, node);
}

void pm_register_notifier(const enum pm_node_id node,
			const uint32_t event,
			const uint8_t enable)
{
	send_ipi_request(PM_GET_API_VERSION, node, event, enable, 0);
	VERBOSE("ATF: %s\n(%d, %d, %d)", __func__, node, event, enable);
}
