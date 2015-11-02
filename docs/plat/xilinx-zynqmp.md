ARM Trusted Firmware for Xilinx Zynq MPSoC
================================

ARM Trusted Firmware implements the EL3 firmware layer for Xilinx Zynq MPSoC.
The platform only uses the runtime part of ATF as ZynqMP already has a
BootROM (BL1) and FSBL (BL2).

BL3-1 is ATF.  
BL3-2 is an optional Secure Payload.  
BL3-3 is the non-secure world software (U-Boot, Linux etc).  

To build:
```bash
make DEBUG=1 RESET_TO_BL31=1 CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp bl31
```

To build bl32 TSP you have to rebuild bl31 too:
```bash
make DEBUG=1 RESET_TO_BL31=1 CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp SPD=tspd bl31 bl32
```

# ATF Location
On default the ATF is placed in OCM memory. Alternatively, the ATF can be
placed in DRAM (at 0x30000000). To choose DRAM, add
```
ZYNQMP_ATF_LOCATION=tdram
```
to the make command line.
