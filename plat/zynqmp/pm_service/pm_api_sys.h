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
