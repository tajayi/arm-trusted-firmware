ARM Trusted Firmware for Ronaldo
================================

1.  Introduction
----------------
ARM Trusted Firmware implements the EL3 firmware layer for Ronaldo.
We will only use the runtime part of ATF as Ronaldo already has a
BootROM (BL1), FSBL (BL2).

BL3-1 is ATF.
BL3-2 is an optional Secure Payload.
BL3-3 is the Non-secure world (Uboot, Linux etc).

To build:
make DEBUG=1 RESET_TO_BL31=1 CROSS_COMPILE=aarch64-none-elf- PLAT=ronaldo bl31

To run on QEMU:
qemu-system-aarch64 -M arm-generic-fdt \
	-hw-dtb dts/3-1/SINGLE_ARCH/ronaldo-arm.dtb \
	-serial mon:stdio -display none \
	-device loader,addr=0xfe500104,data=0x8000000e,data-len=4 \
	-kernel arm-trusted-firmware/build/ronaldo/debug/bl31/bl31.elf

You should see the following:
BL31 Built : 13:03:55, Jul 24 2014
ASSERT: bl31_prepare_next_image_entry <150> : next_image_info
