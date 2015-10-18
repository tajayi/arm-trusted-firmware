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

/*
 * Contains APU specific macros and macros to be defined depending on
 * the execution enviroment.
 */

#ifndef _PM_CLIENT_H_
#define _PM_CLIENT_H_

#include <arch_helpers.h>
#include <arm_gic.h> /* plat_gic when becomes available */
#include <bakery_lock.h>
#include <debug.h>
#include <platform.h>
#include <mmio.h>

#include "pm_defs.h"
#include "pm_common.h"

#define OCM_BANK_0	0xFFFC0000
#define OCM_BANK_1	(OCM_BANK_0 + 0x10000)
#define OCM_BANK_2	(OCM_BANK_1 + 0x10000)
#define OCM_BANK_3	(OCM_BANK_2 + 0x10000)

#define IPI_APU_MASK		1U

#define IPI_TRIG_OFFSET		0
#define IPI_OBS_OFFSET		4

#define UNDEFINED_CPUID		(~0)

/* Macros to allow pm_api_sys.c to remain PU independent */
#define pm_read(addr)		mmio_read_32(addr)
#define pm_write(addr, value)	mmio_write_32(addr, value)
#define pm_print		INFO
#define pm_this_cpuid()		platform_get_core_pos(read_mpidr_el1())

/* Conditional debugging prints */
#ifdef DEBUG_MODE
	#define pm_dbg(MSG, ...)	pm_print(MSG,##__VA_ARGS__)
#else
	#define pm_dbg(MSG, ...)	{}
#endif

bakery_lock_t pm_secure_lock;

/* Functions to be implemented by each PU */
enum pm_ret_status pm_ipi_send(const struct pm_proc *const proc,
				      uint32_t payload[PAYLOAD_ARG_CNT]);
enum pm_ret_status pm_ipi_send_sync(const struct pm_proc *const proc,
				    uint32_t payload[PAYLOAD_ARG_CNT],
				    uint32_t *val);
void pm_client_suspend(const struct pm_proc *const proc);
void pm_client_abort_suspend(void);
void pm_client_wakeup(const struct pm_proc *const proc);
enum pm_ret_status set_ocm_retention(void);

/* Global variables to be set in pm_client.c */
extern const enum pm_node_id subsystem_node;
extern const struct pm_proc *primary_proc;

/* Declaration of linker defined symbol */
extern unsigned long __BL31_END__;

#endif /* _PM_CLIENT_H_ */
