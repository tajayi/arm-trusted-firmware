#
# Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Shared memory may be allocated at the top of Trusted SRAM (tsram) or at the
# base of Trusted SRAM (tdram)
ZYNQMP_SHARED_DATA_LOCATION	?=	tsram
ifeq (${ZYNQMP_SHARED_DATA_LOCATION}, tsram)
  ZYNQMP_SHARED_DATA_LOCATION_ID := ZYNQMP_IN_TRUSTED_SRAM
else ifeq (${ZYNQMP_SHARED_DATA_LOCATION}, tdram)
  ZYNQMP_SHARED_DATA_LOCATION_ID := ZYNQMP_IN_TRUSTED_DRAM
else
  $(error "Unsupported ZYNQMP_SHARED_DATA_LOCATION value")
endif

# On ZYNQMP, the TSP can execute either from Trusted SRAM or Trusted DRAM.
# Trusted SRAM is the default.
ZYNQMP_TSP_RAM_LOCATION	?=	tsram
ifeq (${ZYNQMP_TSP_RAM_LOCATION}, tsram)
  ZYNQMP_TSP_RAM_LOCATION_ID := ZYNQMP_IN_TRUSTED_SRAM
else ifeq (${ZYNQMP_TSP_RAM_LOCATION}, tdram)
  ZYNQMP_TSP_RAM_LOCATION_ID := ZYNQMP_IN_TRUSTED_DRAM
else
  $(error "Unsupported ZYNQMP_TSP_RAM_LOCATION value")
endif

ifeq (${ZYNQMP_SHARED_DATA_LOCATION}, tsram)
  ifeq (${ZYNQMP_TSP_RAM_LOCATION}, tdram)
    $(error Shared data in Trusted SRAM and TSP in Trusted DRAM is not supported)
  endif
endif

# Process flags
$(eval $(call add_define,ZYNQMP_SHARED_DATA_LOCATION_ID))
$(eval $(call add_define,ZYNQMP_TSP_RAM_LOCATION_ID))

PLAT_INCLUDES		:=	-Iinclude/plat/arm/common/			\
				-Iinclude/plat/arm/common/aarch64/		\
				-Iplat/zynqmp/include/				\
				-Iplat/zynqmp/pm_service/			\
				-Iplat/zynqmp/pm_service/include

PLAT_BL_COMMON_SOURCES	:=	drivers/cadence/uart/cdns_console.S		\
				drivers/cadence/uart/cdns_common.c		\
				lib/aarch64/xlat_tables.c			\
				plat/arm/common/aarch64/arm_common.c		\
				plat/arm/common/aarch64/arm_helpers.S           \
				plat/common/aarch64/plat_common.c		\
				plat/zynqmp/aarch64/zynqmp_common.c

BL31_SOURCES		+=	drivers/arm/cci/cci.c				\
				drivers/arm/gic/arm_gic.c			\
				drivers/arm/gic/gic_v2.c			\
				drivers/arm/gic/gic_v3.c			\
				drivers/arm/tzc400/tzc400.c			\
				lib/cpus/aarch64/aem_generic.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				lib/cpus/aarch64/cortex_a57.S			\
				plat/common/plat_gic.c				\
				plat/common/aarch64/platform_mp_stack.S		\
				plat/zynqmp/bl31_zynqmp_setup.c			\
				plat/zynqmp/plat_pm.c				\
				plat/zynqmp/plat_topology.c			\
				plat/zynqmp/sip_svc_setup.c			\
				plat/zynqmp/aarch64/zynqmp_helpers.S		\
				plat/zynqmp/pm_service/pm_svc_main.c		\
				plat/zynqmp/pm_service/pm_api_sys.c		\
				plat/zynqmp/pm_service/pm_client.c

ifneq (${RESET_TO_BL31},1)
  $(error "Using BL3-1 as the reset vector is only one option supported on ZynqMP. \
  Please set RESET_TO_BL31 to 1.")
endif
