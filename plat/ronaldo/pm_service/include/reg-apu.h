/**
 * (c) 2013 Xilinx Inc.
 * XREGDB v0.76
 * XREGCHDR v0.15
 *
 * Generated on: 2014-09-02
 *
 * @file: apu.h
 *
 *
 * This file contains confidential and proprietary information
 * of Xilinx, Inc. and is protected under U.S. and
 * international copyright and other intellectual property
 * laws.
 *
 * DISCLAIMER
 * This disclaimer is not a license and does not grant any
 * rights to the materials distributed herewith. Except as
 * otherwise provided in a valid license issued to you by
 * Xilinx, and to the maximum extent permitted by applicable
 * law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
 * WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
 * AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
 * BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
 * INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
 * (2) Xilinx shall not be liable (whether in contract or tort,
 * including negligence, or under any other theory of
 * liability) for any loss or damage of any kind or nature
 * related to, arising under or in connection with these
 * materials, including for any direct, or any indirect,
 * special, incidental, or consequential loss or damage
 * (including loss of data, profits, goodwill, or any type of
 * loss or damage suffered as a result of any action brought
 * by a third party) even if such damage or loss was
 * reasonably foreseeable or Xilinx had been advised of the
 * possibility of the same.
 *
 * CRITICAL APPLICATIONS
 * Xilinx products are not designed or intended to be fail-
 * safe, or for use in any application requiring fail-safe
 * performance, such as life-support or safety devices or
 * systems, Class III medical devices, nuclear facilities,
 * applications related to the deployment of airbags, or any
 * other applications that could lead to death, personal
 * injury, or severe property or environmental damage
 * (individually and collectively, "Critical
 * Applications"). Customer assumes the sole risk and
 * liability of any use of Xilinx products in Critical
 * Applications, subject only to applicable laws and
 * regulations governing limitations on product liability.
 *
 * THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
 * PART OF THIS FILE AT ALL TIMES.
 *
 * Naming Convention: <MODULE>_<REGISTER>[_<FIELD>[_<DESC>]]
 *     <MODULE>       Module name (e.g. can or usb)
 *     <REGISTER>     Register within the current module
 *     [_<FIELD>]     Bit field within a register
 *     [_<DESC>]      Type of bit field define:
 *         SHIFT:     Least significant bit for the field
 *         WIDTH:     Size of field in bites
 *         MASK:      A masking over a range of bits or a bit to
 *                    be used for setting or clearing
 *
 */

#ifndef _APU_H_
#define _APU_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * APU Base Address
 */
#define APU_BASEADDR      0X00000000

/**
 * Register: APU_ERR_CTRL
 */
#define APU_ERR_CTRL    ( ( APU_BASEADDR ) + 0X00000000 )

#define APU_ERR_CTRL_PSLVERR_SHIFT   0
#define APU_ERR_CTRL_PSLVERR_WIDTH   1
#define APU_ERR_CTRL_PSLVERR_MASK    0X00000001

/**
 * Register: APU_ISR
 */
#define APU_ISR    ( ( APU_BASEADDR ) + 0X00000010 )

#define APU_ISR_INV_APB_SHIFT   0
#define APU_ISR_INV_APB_WIDTH   1
#define APU_ISR_INV_APB_MASK    0X00000001

/**
 * Register: APU_IMR
 */
#define APU_IMR    ( ( APU_BASEADDR ) + 0X00000014 )

#define APU_IMR_INV_APB_SHIFT   0
#define APU_IMR_INV_APB_WIDTH   1
#define APU_IMR_INV_APB_MASK    0X00000001

/**
 * Register: APU_IEN
 */
#define APU_IEN    ( ( APU_BASEADDR ) + 0X00000018 )

#define APU_IEN_INV_APB_SHIFT   0
#define APU_IEN_INV_APB_WIDTH   1
#define APU_IEN_INV_APB_MASK    0X00000001

/**
 * Register: APU_IDS
 */
#define APU_IDS    ( ( APU_BASEADDR ) + 0X0000001C )

#define APU_IDS_INV_APB_SHIFT   0
#define APU_IDS_INV_APB_WIDTH   1
#define APU_IDS_INV_APB_MASK    0X00000001

/**
 * Register: APU_CONFIG_0
 */
#define APU_CONFIG_0    ( ( APU_BASEADDR ) + 0X00000020 )

#define APU_CONFIG_0_CFGTE_SHIFT   24
#define APU_CONFIG_0_CFGTE_WIDTH   4
#define APU_CONFIG_0_CFGTE_MASK    0X0F000000

#define APU_CONFIG_0_CFGEND_SHIFT   16
#define APU_CONFIG_0_CFGEND_WIDTH   4
#define APU_CONFIG_0_CFGEND_MASK    0X000F0000

#define APU_CONFIG_0_VINITHI_SHIFT   8
#define APU_CONFIG_0_VINITHI_WIDTH   4
#define APU_CONFIG_0_VINITHI_MASK    0X00000F00

#define APU_CONFIG_0_AA64NAA32_SHIFT   0
#define APU_CONFIG_0_AA64NAA32_WIDTH   4
#define APU_CONFIG_0_AA64NAA32_MASK    0X0000000F

/**
 * Register: APU_CONFIG_1
 */
#define APU_CONFIG_1    ( ( APU_BASEADDR ) + 0X00000024 )

#define APU_CONFIG_1_L2RSTDISABLE_SHIFT   29
#define APU_CONFIG_1_L2RSTDISABLE_WIDTH   1
#define APU_CONFIG_1_L2RSTDISABLE_MASK    0X20000000

#define APU_CONFIG_1_L1RSTDISABLE_SHIFT   28
#define APU_CONFIG_1_L1RSTDISABLE_WIDTH   1
#define APU_CONFIG_1_L1RSTDISABLE_MASK    0X10000000

#define APU_CONFIG_1_CP15DISABLE_SHIFT   0
#define APU_CONFIG_1_CP15DISABLE_WIDTH   4
#define APU_CONFIG_1_CP15DISABLE_MASK    0X0000000F

/**
 * Register: APU_CONFIG_2
 */
#define APU_CONFIG_2    ( ( APU_BASEADDR ) + 0X00000028 )

#define APU_CONFIG_2_BRDC_OUTER_SHIFT   3
#define APU_CONFIG_2_BRDC_OUTER_WIDTH   1
#define APU_CONFIG_2_BRDC_OUTER_MASK    0X00000008

#define APU_CONFIG_2_BRDC_INNER_SHIFT   2
#define APU_CONFIG_2_BRDC_INNER_WIDTH   1
#define APU_CONFIG_2_BRDC_INNER_MASK    0X00000004

#define APU_CONFIG_2_BRDC_CMNT_SHIFT   1
#define APU_CONFIG_2_BRDC_CMNT_WIDTH   1
#define APU_CONFIG_2_BRDC_CMNT_MASK    0X00000002

#define APU_CONFIG_2_BRDC_BARRIER_SHIFT   0
#define APU_CONFIG_2_BRDC_BARRIER_WIDTH   1
#define APU_CONFIG_2_BRDC_BARRIER_MASK    0X00000001

/**
 * Register: APU_RVBARADDR0L
 */
#define APU_RVBARADDR0L    ( ( APU_BASEADDR ) + 0X00000040 )

#define APU_RVBARADDR0L_ADDR_SHIFT   2
#define APU_RVBARADDR0L_ADDR_WIDTH   30
#define APU_RVBARADDR0L_ADDR_MASK    0XFFFFFFFC

/**
 * Register: APU_RVBARADDR0H
 */
#define APU_RVBARADDR0H    ( ( APU_BASEADDR ) + 0X00000044 )

#define APU_RVBARADDR0H_ADDR_SHIFT   0
#define APU_RVBARADDR0H_ADDR_WIDTH   8
#define APU_RVBARADDR0H_ADDR_MASK    0X000000FF

/**
 * Register: APU_RVBARADDR1L
 */
#define APU_RVBARADDR1L    ( ( APU_BASEADDR ) + 0X00000048 )

#define APU_RVBARADDR1L_ADDR_SHIFT   2
#define APU_RVBARADDR1L_ADDR_WIDTH   30
#define APU_RVBARADDR1L_ADDR_MASK    0XFFFFFFFC

/**
 * Register: APU_RVBARADDR1H
 */
#define APU_RVBARADDR1H    ( ( APU_BASEADDR ) + 0X0000004C )

#define APU_RVBARADDR1H_ADDR_SHIFT   0
#define APU_RVBARADDR1H_ADDR_WIDTH   8
#define APU_RVBARADDR1H_ADDR_MASK    0X000000FF

/**
 * Register: APU_RVBARADDR2L
 */
#define APU_RVBARADDR2L    ( ( APU_BASEADDR ) + 0X00000050 )

#define APU_RVBARADDR2L_ADDR_SHIFT   2
#define APU_RVBARADDR2L_ADDR_WIDTH   30
#define APU_RVBARADDR2L_ADDR_MASK    0XFFFFFFFC

/**
 * Register: APU_RVBARADDR2H
 */
#define APU_RVBARADDR2H    ( ( APU_BASEADDR ) + 0X00000054 )

#define APU_RVBARADDR2H_ADDR_SHIFT   0
#define APU_RVBARADDR2H_ADDR_WIDTH   8
#define APU_RVBARADDR2H_ADDR_MASK    0X000000FF

/**
 * Register: APU_RVBARADDR3L
 */
#define APU_RVBARADDR3L    ( ( APU_BASEADDR ) + 0X00000058 )

#define APU_RVBARADDR3L_ADDR_SHIFT   2
#define APU_RVBARADDR3L_ADDR_WIDTH   30
#define APU_RVBARADDR3L_ADDR_MASK    0XFFFFFFFC

/**
 * Register: APU_RVBARADDR3H
 */
#define APU_RVBARADDR3H    ( ( APU_BASEADDR ) + 0X0000005C )

#define APU_RVBARADDR3H_ADDR_SHIFT   0
#define APU_RVBARADDR3H_ADDR_WIDTH   8
#define APU_RVBARADDR3H_ADDR_MASK    0X000000FF

/**
 * Register: APU_SNOOP_CTRL
 */
#define APU_SNOOP_CTRL    ( ( APU_BASEADDR ) + 0X00000080 )

#define APU_SNOOP_CTRL_ACE_INACT_SHIFT   4
#define APU_SNOOP_CTRL_ACE_INACT_WIDTH   1
#define APU_SNOOP_CTRL_ACE_INACT_MASK    0X00000010

#define APU_SNOOP_CTRL_ACP_INACT_SHIFT   0
#define APU_SNOOP_CTRL_ACP_INACT_WIDTH   1
#define APU_SNOOP_CTRL_ACP_INACT_MASK    0X00000001

/**
 * Register: APU_PWRCTL
 */
#define APU_PWRCTL    ( ( APU_BASEADDR ) + 0X00000090 )

#define APU_PWRCTL_CLREXMONREQ_SHIFT   17
#define APU_PWRCTL_CLREXMONREQ_WIDTH   1
#define APU_PWRCTL_CLREXMONREQ_MASK    0X00020000

#define APU_PWRCTL_L2FLUSHREQ_SHIFT   16
#define APU_PWRCTL_L2FLUSHREQ_WIDTH   1
#define APU_PWRCTL_L2FLUSHREQ_MASK    0X00010000

#define APU_PWRCTL_CPUPWRDWNREQ_SHIFT   0
#define APU_PWRCTL_CPUPWRDWNREQ_WIDTH   4
#define APU_PWRCTL_CPUPWRDWNREQ_MASK    0X0000000F

/**
 * Register: APU_PWRSTAT
 */
#define APU_PWRSTAT    ( ( APU_BASEADDR ) + 0X00000094 )

#define APU_PWRSTAT_CLREXMONACK_SHIFT   17
#define APU_PWRSTAT_CLREXMONACK_WIDTH   1
#define APU_PWRSTAT_CLREXMONACK_MASK    0X00020000

#define APU_PWRSTAT_L2FLUSHDONE_SHIFT   16
#define APU_PWRSTAT_L2FLUSHDONE_WIDTH   1
#define APU_PWRSTAT_L2FLUSHDONE_MASK    0X00010000

#define APU_PWRSTAT_DBGNOPWRDWN_SHIFT   0
#define APU_PWRSTAT_DBGNOPWRDWN_WIDTH   4
#define APU_PWRSTAT_DBGNOPWRDWN_MASK    0X0000000F

/**
 * Register: APU_ECO
 */
#define APU_ECO    ( ( APU_BASEADDR ) + 0X000000EC )

#define APU_ECO_SPARE_SHIFT   0
#define APU_ECO_SPARE_WIDTH   32
#define APU_ECO_SPARE_MASK    0XFFFFFFFF

/**
 * Register: APU_RAM_ADJ_0
 */
#define APU_RAM_ADJ_0    ( ( APU_BASEADDR ) + 0X000000F0 )

#define APU_RAM_ADJ_0_L1_ITAG_EMAS_SHIFT   29
#define APU_RAM_ADJ_0_L1_ITAG_EMAS_WIDTH   1
#define APU_RAM_ADJ_0_L1_ITAG_EMAS_MASK    0X20000000

#define APU_RAM_ADJ_0_L1_ITAG_EMAW_SHIFT   27
#define APU_RAM_ADJ_0_L1_ITAG_EMAW_WIDTH   2
#define APU_RAM_ADJ_0_L1_ITAG_EMAW_MASK    0X18000000

#define APU_RAM_ADJ_0_L1_ITAG_EMA_SHIFT   24
#define APU_RAM_ADJ_0_L1_ITAG_EMA_WIDTH   3
#define APU_RAM_ADJ_0_L1_ITAG_EMA_MASK    0X07000000

#define APU_RAM_ADJ_0_L1_IDATA_EMAS_SHIFT   21
#define APU_RAM_ADJ_0_L1_IDATA_EMAS_WIDTH   1
#define APU_RAM_ADJ_0_L1_IDATA_EMAS_MASK    0X00200000

#define APU_RAM_ADJ_0_L1_IDATA_EMAW_SHIFT   19
#define APU_RAM_ADJ_0_L1_IDATA_EMAW_WIDTH   2
#define APU_RAM_ADJ_0_L1_IDATA_EMAW_MASK    0X00180000

#define APU_RAM_ADJ_0_L1_IDATA_EMA_SHIFT   16
#define APU_RAM_ADJ_0_L1_IDATA_EMA_WIDTH   3
#define APU_RAM_ADJ_0_L1_IDATA_EMA_MASK    0X00070000

#define APU_RAM_ADJ_0_L1_DTAG_EMAS_SHIFT   13
#define APU_RAM_ADJ_0_L1_DTAG_EMAS_WIDTH   1
#define APU_RAM_ADJ_0_L1_DTAG_EMAS_MASK    0X00002000

#define APU_RAM_ADJ_0_L1_DTAG_EMAW_SHIFT   11
#define APU_RAM_ADJ_0_L1_DTAG_EMAW_WIDTH   2
#define APU_RAM_ADJ_0_L1_DTAG_EMAW_MASK    0X00001800

#define APU_RAM_ADJ_0_L1_DTAG_EMA_SHIFT   8
#define APU_RAM_ADJ_0_L1_DTAG_EMA_WIDTH   3
#define APU_RAM_ADJ_0_L1_DTAG_EMA_MASK    0X00000700

#define APU_RAM_ADJ_0_L1_DDATA_EMAS_SHIFT   5
#define APU_RAM_ADJ_0_L1_DDATA_EMAS_WIDTH   1
#define APU_RAM_ADJ_0_L1_DDATA_EMAS_MASK    0X00000020

#define APU_RAM_ADJ_0_L1_DDATA_EMAW_SHIFT   3
#define APU_RAM_ADJ_0_L1_DDATA_EMAW_WIDTH   2
#define APU_RAM_ADJ_0_L1_DDATA_EMAW_MASK    0X00000018

#define APU_RAM_ADJ_0_L1_DDATA_EMA_SHIFT   0
#define APU_RAM_ADJ_0_L1_DDATA_EMA_WIDTH   3
#define APU_RAM_ADJ_0_L1_DDATA_EMA_MASK    0X00000007

/**
 * Register: APU_RAM_ADJ_1
 */
#define APU_RAM_ADJ_1    ( ( APU_BASEADDR ) + 0X000000F4 )

#define APU_RAM_ADJ_1_TLB_EMAS_SHIFT   29
#define APU_RAM_ADJ_1_TLB_EMAS_WIDTH   1
#define APU_RAM_ADJ_1_TLB_EMAS_MASK    0X20000000

#define APU_RAM_ADJ_1_TLB_EMAW_SHIFT   27
#define APU_RAM_ADJ_1_TLB_EMAW_WIDTH   2
#define APU_RAM_ADJ_1_TLB_EMAW_MASK    0X18000000

#define APU_RAM_ADJ_1_TLB_EMA_SHIFT   24
#define APU_RAM_ADJ_1_TLB_EMA_WIDTH   3
#define APU_RAM_ADJ_1_TLB_EMA_MASK    0X07000000

#define APU_RAM_ADJ_1_DIRTY_EMAS_SHIFT   21
#define APU_RAM_ADJ_1_DIRTY_EMAS_WIDTH   1
#define APU_RAM_ADJ_1_DIRTY_EMAS_MASK    0X00200000

#define APU_RAM_ADJ_1_DIRTY_EMAW_SHIFT   19
#define APU_RAM_ADJ_1_DIRTY_EMAW_WIDTH   2
#define APU_RAM_ADJ_1_DIRTY_EMAW_MASK    0X00180000

#define APU_RAM_ADJ_1_DIRTY_EMA_SHIFT   16
#define APU_RAM_ADJ_1_DIRTY_EMA_WIDTH   3
#define APU_RAM_ADJ_1_DIRTY_EMA_MASK    0X00070000

#define APU_RAM_ADJ_1_BTAC1_EMAS_SHIFT   13
#define APU_RAM_ADJ_1_BTAC1_EMAS_WIDTH   1
#define APU_RAM_ADJ_1_BTAC1_EMAS_MASK    0X00002000

#define APU_RAM_ADJ_1_BTAC1_EMAW_SHIFT   11
#define APU_RAM_ADJ_1_BTAC1_EMAW_WIDTH   2
#define APU_RAM_ADJ_1_BTAC1_EMAW_MASK    0X00001800

#define APU_RAM_ADJ_1_BTAC1_EMA_SHIFT   8
#define APU_RAM_ADJ_1_BTAC1_EMA_WIDTH   3
#define APU_RAM_ADJ_1_BTAC1_EMA_MASK    0X00000700

#define APU_RAM_ADJ_1_BTAC0_EMAS_SHIFT   5
#define APU_RAM_ADJ_1_BTAC0_EMAS_WIDTH   1
#define APU_RAM_ADJ_1_BTAC0_EMAS_MASK    0X00000020

#define APU_RAM_ADJ_1_BTAC0_EMAW_SHIFT   3
#define APU_RAM_ADJ_1_BTAC0_EMAW_WIDTH   2
#define APU_RAM_ADJ_1_BTAC0_EMAW_MASK    0X00000018

#define APU_RAM_ADJ_1_BTAC0_EMA_SHIFT   0
#define APU_RAM_ADJ_1_BTAC0_EMA_WIDTH   3
#define APU_RAM_ADJ_1_BTAC0_EMA_MASK    0X00000007

/**
 * Register: APU_RAM_ADJ_2
 */
#define APU_RAM_ADJ_2    ( ( APU_BASEADDR ) + 0X000000F8 )

#define APU_RAM_ADJ_2_ETF_EMAS_SHIFT   29
#define APU_RAM_ADJ_2_ETF_EMAS_WIDTH   1
#define APU_RAM_ADJ_2_ETF_EMAS_MASK    0X20000000

#define APU_RAM_ADJ_2_ETF_EMAW_SHIFT   27
#define APU_RAM_ADJ_2_ETF_EMAW_WIDTH   2
#define APU_RAM_ADJ_2_ETF_EMAW_MASK    0X18000000

#define APU_RAM_ADJ_2_ETF_EMA_SHIFT   24
#define APU_RAM_ADJ_2_ETF_EMA_WIDTH   3
#define APU_RAM_ADJ_2_ETF_EMA_MASK    0X07000000

#define APU_RAM_ADJ_2_SCU_TAG_EMAS_SHIFT   13
#define APU_RAM_ADJ_2_SCU_TAG_EMAS_WIDTH   1
#define APU_RAM_ADJ_2_SCU_TAG_EMAS_MASK    0X00002000

#define APU_RAM_ADJ_2_SCU_TAG_EMAW_SHIFT   11
#define APU_RAM_ADJ_2_SCU_TAG_EMAW_WIDTH   2
#define APU_RAM_ADJ_2_SCU_TAG_EMAW_MASK    0X00001800

#define APU_RAM_ADJ_2_SCU_TAG_EMA_SHIFT   8
#define APU_RAM_ADJ_2_SCU_TAG_EMA_WIDTH   3
#define APU_RAM_ADJ_2_SCU_TAG_EMA_MASK    0X00000700

#define APU_RAM_ADJ_2_L2_VICTIM_EMAS_SHIFT   5
#define APU_RAM_ADJ_2_L2_VICTIM_EMAS_WIDTH   1
#define APU_RAM_ADJ_2_L2_VICTIM_EMAS_MASK    0X00000020

#define APU_RAM_ADJ_2_L2_VICTIM_EMAW_SHIFT   3
#define APU_RAM_ADJ_2_L2_VICTIM_EMAW_WIDTH   2
#define APU_RAM_ADJ_2_L2_VICTIM_EMAW_MASK    0X00000018

#define APU_RAM_ADJ_2_L2_VICTIM_EMA_SHIFT   0
#define APU_RAM_ADJ_2_L2_VICTIM_EMA_WIDTH   3
#define APU_RAM_ADJ_2_L2_VICTIM_EMA_MASK    0X00000007

/**
 * Register: APU_RAM_ADJ_3
 */
#define APU_RAM_ADJ_3    ( ( APU_BASEADDR ) + 0X000000FC )

#define APU_RAM_ADJ_3_L2_TAGECC_EMAS_SHIFT   29
#define APU_RAM_ADJ_3_L2_TAGECC_EMAS_WIDTH   1
#define APU_RAM_ADJ_3_L2_TAGECC_EMAS_MASK    0X20000000

#define APU_RAM_ADJ_3_L2_TAGECC_EMAW_SHIFT   27
#define APU_RAM_ADJ_3_L2_TAGECC_EMAW_WIDTH   2
#define APU_RAM_ADJ_3_L2_TAGECC_EMAW_MASK    0X18000000

#define APU_RAM_ADJ_3_L2_TAGECC_EMA_SHIFT   24
#define APU_RAM_ADJ_3_L2_TAGECC_EMA_WIDTH   3
#define APU_RAM_ADJ_3_L2_TAGECC_EMA_MASK    0X07000000

#define APU_RAM_ADJ_3_L2_TAG_EMAS_SHIFT   21
#define APU_RAM_ADJ_3_L2_TAG_EMAS_WIDTH   1
#define APU_RAM_ADJ_3_L2_TAG_EMAS_MASK    0X00200000

#define APU_RAM_ADJ_3_L2_TAG_EMAW_SHIFT   19
#define APU_RAM_ADJ_3_L2_TAG_EMAW_WIDTH   2
#define APU_RAM_ADJ_3_L2_TAG_EMAW_MASK    0X00180000

#define APU_RAM_ADJ_3_L2_TAG_EMA_SHIFT   16
#define APU_RAM_ADJ_3_L2_TAG_EMA_WIDTH   3
#define APU_RAM_ADJ_3_L2_TAG_EMA_MASK    0X00070000

#define APU_RAM_ADJ_3_L2_DATAECC_EMAS_SHIFT   13
#define APU_RAM_ADJ_3_L2_DATAECC_EMAS_WIDTH   1
#define APU_RAM_ADJ_3_L2_DATAECC_EMAS_MASK    0X00002000

#define APU_RAM_ADJ_3_L2_DATAECC_EMAW_SHIFT   11
#define APU_RAM_ADJ_3_L2_DATAECC_EMAW_WIDTH   2
#define APU_RAM_ADJ_3_L2_DATAECC_EMAW_MASK    0X00001800

#define APU_RAM_ADJ_3_L2_DATAECC_EMA_SHIFT   8
#define APU_RAM_ADJ_3_L2_DATAECC_EMA_WIDTH   3
#define APU_RAM_ADJ_3_L2_DATAECC_EMA_MASK    0X00000700

#define APU_RAM_ADJ_3_L2_DATA_EMAS_SHIFT   5
#define APU_RAM_ADJ_3_L2_DATA_EMAS_WIDTH   1
#define APU_RAM_ADJ_3_L2_DATA_EMAS_MASK    0X00000020

#define APU_RAM_ADJ_3_L2_DATA_EMAW_SHIFT   3
#define APU_RAM_ADJ_3_L2_DATA_EMAW_WIDTH   2
#define APU_RAM_ADJ_3_L2_DATA_EMAW_MASK    0X00000018

#define APU_RAM_ADJ_3_L2_DATA_EMA_SHIFT   0
#define APU_RAM_ADJ_3_L2_DATA_EMA_WIDTH   3
#define APU_RAM_ADJ_3_L2_DATA_EMA_MASK    0X00000007

/**
 * Register: APU_XPD_REG0
 */
#define APU_XPD_REG0    ( ( APU_BASEADDR ) + 0X00000600 )

#define APU_XPD_REG0_PRE_LOAD_SHIFT   0
#define APU_XPD_REG0_PRE_LOAD_WIDTH   32
#define APU_XPD_REG0_PRE_LOAD_MASK    0XFFFFFFFF

/**
 * Register: APU_XPD_REG1
 */
#define APU_XPD_REG1    ( ( APU_BASEADDR ) + 0X00000604 )

#define APU_XPD_REG1_EXPECTED_SHIFT   0
#define APU_XPD_REG1_EXPECTED_WIDTH   32
#define APU_XPD_REG1_EXPECTED_MASK    0XFFFFFFFF

/**
 * Register: APU_XPD_CTRL0
 */
#define APU_XPD_CTRL0    ( ( APU_BASEADDR ) + 0X00000608 )

#define APU_XPD_CTRL0_DELAY_SPARE_SHIFT   25
#define APU_XPD_CTRL0_DELAY_SPARE_WIDTH   5
#define APU_XPD_CTRL0_DELAY_SPARE_MASK    0X3E000000

#define APU_XPD_CTRL0_CMP_SEL_SHIFT   24
#define APU_XPD_CTRL0_CMP_SEL_WIDTH   1
#define APU_XPD_CTRL0_CMP_SEL_MASK    0X01000000

#define APU_XPD_CTRL0_DELAY_CELL_TYPE_SHIFT   19
#define APU_XPD_CTRL0_DELAY_CELL_TYPE_WIDTH   5
#define APU_XPD_CTRL0_DELAY_CELL_TYPE_MASK    0X00F80000

#define APU_XPD_CTRL0_DELAY_VT_TYPE_SHIFT   17
#define APU_XPD_CTRL0_DELAY_VT_TYPE_WIDTH   2
#define APU_XPD_CTRL0_DELAY_VT_TYPE_MASK    0X00060000

#define APU_XPD_CTRL0_DELAY_VALUE_SHIFT   6
#define APU_XPD_CTRL0_DELAY_VALUE_WIDTH   11
#define APU_XPD_CTRL0_DELAY_VALUE_MASK    0X0001FFC0

#define APU_XPD_CTRL0_PATH_SEL_SHIFT   0
#define APU_XPD_CTRL0_PATH_SEL_WIDTH   6
#define APU_XPD_CTRL0_PATH_SEL_MASK    0X0000003F

/**
 * Register: APU_XPD_CTRL1
 */
#define APU_XPD_CTRL1    ( ( APU_BASEADDR ) + 0X0000060C )

#define APU_XPD_CTRL1_CLK_SPARE_SHIFT   12
#define APU_XPD_CTRL1_CLK_SPARE_WIDTH   4
#define APU_XPD_CTRL1_CLK_SPARE_MASK    0X0000F000

#define APU_XPD_CTRL1_CLK_PHASE_SEL_SHIFT   10
#define APU_XPD_CTRL1_CLK_PHASE_SEL_WIDTH   2
#define APU_XPD_CTRL1_CLK_PHASE_SEL_MASK    0X00000C00

#define APU_XPD_CTRL1_CLK_VT_TYPE_SHIFT   8
#define APU_XPD_CTRL1_CLK_VT_TYPE_WIDTH   2
#define APU_XPD_CTRL1_CLK_VT_TYPE_MASK    0X00000300

#define APU_XPD_CTRL1_CLK_CELL_TYPE_SHIFT   6
#define APU_XPD_CTRL1_CLK_CELL_TYPE_WIDTH   2
#define APU_XPD_CTRL1_CLK_CELL_TYPE_MASK    0X000000C0

#define APU_XPD_CTRL1_CLK_INSERT_DLY_SHIFT   2
#define APU_XPD_CTRL1_CLK_INSERT_DLY_WIDTH   4
#define APU_XPD_CTRL1_CLK_INSERT_DLY_MASK    0X0000003C

#define APU_XPD_CTRL1_CLK_SEL_SHIFT   0
#define APU_XPD_CTRL1_CLK_SEL_WIDTH   2
#define APU_XPD_CTRL1_CLK_SEL_MASK    0X00000003

/**
 * Register: APU_XPD_CTRL2
 */
#define APU_XPD_CTRL2    ( ( APU_BASEADDR ) + 0X00000614 )

#define APU_XPD_CTRL2_CTRL_SPARE_SHIFT   1
#define APU_XPD_CTRL2_CTRL_SPARE_WIDTH   2
#define APU_XPD_CTRL2_CTRL_SPARE_MASK    0X00000006

#define APU_XPD_CTRL2_ENABLE_SHIFT   0
#define APU_XPD_CTRL2_ENABLE_WIDTH   1
#define APU_XPD_CTRL2_ENABLE_MASK    0X00000001

/**
 * Register: APU_XPD_CTRL3
 */
#define APU_XPD_CTRL3    ( ( APU_BASEADDR ) + 0X00000618 )

#define APU_XPD_CTRL3_DCYCLE_CNT_VALUE_SHIFT   3
#define APU_XPD_CTRL3_DCYCLE_CNT_VALUE_WIDTH   12
#define APU_XPD_CTRL3_DCYCLE_CNT_VALUE_MASK    0X00007FF8

#define APU_XPD_CTRL3_DCYCLE_HIGH_LOW_SHIFT   2
#define APU_XPD_CTRL3_DCYCLE_HIGH_LOW_WIDTH   1
#define APU_XPD_CTRL3_DCYCLE_HIGH_LOW_MASK    0X00000004

#define APU_XPD_CTRL3_DCYCLE_CNT_CLR_SHIFT   1
#define APU_XPD_CTRL3_DCYCLE_CNT_CLR_WIDTH   1
#define APU_XPD_CTRL3_DCYCLE_CNT_CLR_MASK    0X00000002

#define APU_XPD_CTRL3_DCYCLE_START_SHIFT   0
#define APU_XPD_CTRL3_DCYCLE_START_WIDTH   1
#define APU_XPD_CTRL3_DCYCLE_START_MASK    0X00000001

/**
 * Register: APU_XPD_SOFT_RST
 */
#define APU_XPD_SOFT_RST    ( ( APU_BASEADDR ) + 0X0000061C )

#define APU_XPD_SOFT_RST_CLK2_SHIFT   2
#define APU_XPD_SOFT_RST_CLK2_WIDTH   1
#define APU_XPD_SOFT_RST_CLK2_MASK    0X00000004

#define APU_XPD_SOFT_RST_CLK1_SHIFT   1
#define APU_XPD_SOFT_RST_CLK1_WIDTH   1
#define APU_XPD_SOFT_RST_CLK1_MASK    0X00000002

#define APU_XPD_SOFT_RST_CLK0_SHIFT   0
#define APU_XPD_SOFT_RST_CLK0_WIDTH   1
#define APU_XPD_SOFT_RST_CLK0_MASK    0X00000001

/**
 * Register: APU_XPD_STAT
 */
#define APU_XPD_STAT    ( ( APU_BASEADDR ) + 0X00000620 )

#define APU_XPD_STAT_CMP_RESULT_SHIFT   1
#define APU_XPD_STAT_CMP_RESULT_WIDTH   1
#define APU_XPD_STAT_CMP_RESULT_MASK    0X00000002

#define APU_XPD_STAT_CMP_DONE_SHIFT   0
#define APU_XPD_STAT_CMP_DONE_WIDTH   1
#define APU_XPD_STAT_CMP_DONE_MASK    0X00000001

#ifdef __cplusplus
}
#endif


#endif /* _APU_H_ */
