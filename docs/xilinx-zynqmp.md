ARM Trusted Firmware for Xilinx ZynqMP
================================

ARM Trusted Firmware implements the EL3 firmware layer for Xilinx ZynqMP.
Platform only uses the runtime part of ATF as ZynqMP already has a
BootROM (BL1), FSBL (BL2).

BL3-1 is ATF.
BL3-2 is an optional Secure Payload.
BL3-3 is the Non-secure world (U-Boot, Linux etc).

To build:
make DEBUG=1 RESET_TO_BL31=1 CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp bl31

To build bl32 TSP you have to rebuild bl31 too:
make DEBUG=1 RESET_TO_BL31=1 CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp SPD=tspd bl31 bl32
