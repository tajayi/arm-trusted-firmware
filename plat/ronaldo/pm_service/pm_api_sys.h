/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* System Level PM API functions:
*
* PM-related calls to be invoked from the lowest software
* layer executing on a PU, i.e. firmware.
* API implementation might cause IPI based communication
* between a PU and PMU.
*********************************************************************/

#ifndef _PM_API_SYS_H_
#define _PM_API_SYS_H_

#include "pm_defs.h"

enum pm_node_id pm_get_node_id(const uint32_t cpuid);

/**********************************************************
 * System-level API function declarations
 **********************************************************/
void pm_req_suspend(const enum pm_node_id node,
			const enum pm_request_ack ack,
			const uint32_t latency,
			const uint8_t state);

void pm_self_suspend(const enum pm_node_id node,
			const enum pm_request_ack ack,
			const uint32_t latency,
			const uint8_t state);

void pm_force_powerdown(const enum pm_node_id node,
			const enum pm_request_ack ack);

void pm_abort_suspend(const enum pm_abort_reason reason);

void pm_req_wakeup(const enum pm_node_id node,
			const enum pm_request_ack ack);

void pm_set_wakeup_source(const enum pm_node_id target,
			const enum pm_node_id wkup_node,
			const uint8_t enable);

void pm_system_shutdown(const uint8_t restart);

void pm_init_suspend_cb(const enum pm_suspend_reason reason,
			const uint8_t save_context,
			const uint32_t timeout);

/* API functions for managing PM Slaves */
void pm_req_node(const enum pm_node_id node, const uint32_t capabilities,
			const uint32_t qos, const enum pm_request_ack ack);
void pm_release_node(const enum pm_node_id node, const uint32_t latency);
void pm_set_requirement(const enum pm_node_id node,
			const uint32_t capabilities,
			const uint32_t qos,
			const enum pm_request_ack ack);
void pm_set_max_latency(const enum pm_node_id node,
			const uint32_t latency);

/* Miscellaneous API functions */
void pm_get_api_version(void);
void pm_get_node_status(const enum pm_node_id node);
void pm_register_notifier(const enum pm_node_id node,
			const uint32_t event,
			const uint8_t enable);

/**********************************************************
 * System-level registration function declarations
 **********************************************************/
/*void pm_register_callback(void);*/


#endif /* _PM_API_SYS_H_ */
