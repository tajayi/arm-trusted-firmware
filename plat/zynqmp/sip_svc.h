/*********************************************************************
* 2014 Aggios, Inc.
*
* Written by Anes Hadziahmetagic <anes.hadziahmetagic@aggios.com>
*
* CONTENT
* Contains SMC function IDs for Sip Service queries,
* SIP Service Calls version numbers, and specific PM Identifiers
*********************************************************************/

#ifndef _SIP_SVC_H_
#define _SIP_SVC_H_

/* SMC function IDs for Sip Service queries */
#define ZYNQMP_SIP_SVC_CALL_COUNT		0x8200ff00
#define ZYNQMP_SIP_SVC_UID			0x8200ff01
#define ZYNQMP_SIP_SVC_VERSION			0x8200ff03

/* SIP Service Calls version numbers */
#define SIP_SVC_VERSION_MAJOR		0x0
#define SIP_SVC_VERSION_MINOR		0x1

/* The macros below are used to identify PM calls from the SMC function ID */
#define PM_FID_MASK			0xf000u
#define PM_FID_VALUE			0u
#define is_pm_fid(_fid) \
	(((_fid) & PM_FID_MASK) == PM_FID_VALUE)

#endif /* _SIP_SVC_H_*/
