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
 * SMC function IDs for SiP Service queries, SiP Service Calls version numbers,
 * and specific PM Identifiers
 */

#ifndef _SIP_SVC_H_
#define _SIP_SVC_H_

/* SMC function IDs for SiP Service queries */
#define ZYNQMP_SIP_SVC_CALL_COUNT		0x8200ff00
#define ZYNQMP_SIP_SVC_UID			0x8200ff01
#define ZYNQMP_SIP_SVC_VERSION			0x8200ff03

/* SiP Service Calls version numbers */
#define SIP_SVC_VERSION_MAJOR		0x0
#define SIP_SVC_VERSION_MINOR		0x1

/* These macros are used to identify PM calls from the SMC function ID */
#define PM_FID_MASK			0xf000u
#define PM_FID_VALUE			0u
#define is_pm_fid(_fid) \
	(((_fid) & PM_FID_MASK) == PM_FID_VALUE)

#endif /* _SIP_SVC_H_*/
