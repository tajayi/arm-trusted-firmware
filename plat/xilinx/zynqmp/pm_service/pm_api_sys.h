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

#include <stdint.h>
#include "pm_defs.h"

/**********************************************************
 * System-level API function declarations
 **********************************************************/
enum pm_ret_status pm_req_suspend(const enum pm_node_id nid,
				  const enum pm_request_ack ack,
				  const uint32_t latency,
				  const uint8_t state);

enum pm_ret_status pm_self_suspend(const enum pm_node_id nid,
				   const uint32_t latency,
				   const uint8_t state,
				   const uint64_t address);

enum pm_ret_status pm_force_powerdown(const enum pm_node_id nid,
				      const enum pm_request_ack ack);

enum pm_ret_status pm_abort_suspend(const enum pm_abort_reason reason);

enum pm_ret_status pm_req_wakeup(const enum pm_node_id nid,
				 const uint32_t set_address,
				 const uint64_t address,
				 const enum pm_request_ack ack);

enum pm_ret_status pm_set_wakeup_source(const enum pm_node_id target,
					const enum pm_node_id wkup_node,
					const uint8_t enable);

enum pm_ret_status pm_system_shutdown(const uint8_t restart);

enum pm_ret_status pm_init_suspend_cb(const enum pm_suspend_reason reason,
				      const uint32_t latency,
				      const uint32_t state,
				      const uint32_t timeout);

/* API functions for managing PM Slaves */
enum pm_ret_status pm_req_node(const enum pm_node_id nid,
			       const uint32_t capabilities,
			       const uint32_t qos,
			       const enum pm_request_ack ack);
enum pm_ret_status pm_release_node(const enum pm_node_id nid);

enum pm_ret_status pm_set_requirement(const enum pm_node_id nid,
				      const uint32_t capabilities,
				      const uint32_t qos,
				      const enum pm_request_ack ack);
enum pm_ret_status pm_set_max_latency(const enum pm_node_id nid,
				      const uint32_t latency);

/* Miscellaneous API functions */
enum pm_ret_status pm_get_api_version(uint32_t *version);
enum pm_ret_status pm_set_configuration(const uint32_t phys_addr);
enum pm_ret_status pm_get_node_status(const enum pm_node_id node);
enum pm_ret_status pm_register_notifier(const enum pm_node_id nid,
					const uint32_t event,
					const uint8_t wake,
					const uint8_t enable);
enum pm_ret_status pm_get_op_characteristic(const enum pm_node_id nid,
					    const enum pm_opchar_type type);
enum pm_ret_status pm_acknowledge_cb(const enum pm_node_id nid,
				     const enum pm_ret_status status,
				     const uint32_t oppoint);
enum pm_ret_status pm_notify_cb(const enum pm_node_id nid,
				const uint32_t event,
				const uint32_t oppoint);

/* Direct-Control API functions */
enum pm_ret_status pm_reset_assert(const uint32_t reset_id,
				   const uint8_t assert);
enum pm_ret_status pm_reset_get_status(const uint32_t reset_id,
				       uint32_t *reset_status);
enum pm_ret_status pm_mmio_write(const uint32_t address,
				 const uint32_t mask,
				 const uint32_t value);
enum pm_ret_status pm_mmio_read(const uint32_t address, uint32_t *value);
#endif /* _PM_API_SYS_H_ */
