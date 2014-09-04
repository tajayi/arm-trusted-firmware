/*
 * Copyright (c) 2014, Xilinx Inc.
 * Written by Edgar E. Iglesias.
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

#ifndef __CADENCE_UART_H__
#define __CADENCE_UART_H__

#include <mmio.h>

/* This is very minimalistic and will only work in QEMU.  */

/* CADENCE Registers */
#define R_UART_CR    (0x00)
#define R_UART_IER   (0x08)
#define R_UART_IDR   (0x0C)
#define R_UART_IMR   (0x10)
#define R_UART_CISR  (0x14)
#define R_UART_RTRIG (0x20)
#define R_UART_SR    (0x2C)
#define UART_SR_INTR_RTRIG     0x00000001
#define UART_SR_INTR_REMPTY    0x00000002
#define UART_SR_INTR_TEMPTY    0x00000008
#define UART_SR_INTR_TFUL      0x00000010

#define R_UART_TX  (0x30)
#define R_UART_RX  (0x30)

static inline void cadence_uart_tx(unsigned long base, unsigned int val)
{
	uint32_t status;

	/* Wait for an empty slot.  */
	do {
		status = mmio_read_32(base + R_UART_SR);
	} while (status & UART_SR_INTR_TFUL);

        mmio_write_32(base + R_UART_TX, val);
}

static inline unsigned int cadence_uart_rx(unsigned long base)
{
	uint32_t status;

	/* Wait for a data.  */
	do {
		status = mmio_read_32(base + R_UART_SR);
	} while (status & UART_SR_INTR_REMPTY);

        return mmio_read_32(base + R_UART_RX);
}

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

void cadence_uart_setbaudrate(unsigned long base_addr, unsigned int baudrate);

#endif
