/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <console.h>
#include <platform.h>
#include <cadence/uart.h>

static unsigned long uart_base;

/* MS Call from ASM - fix this */
int console_core_init(unsigned long base_addr, unsigned int uart_clk, unsigned int baud_rate) {
	return console_init(base_addr, uart_clk, baud_rate);
}

int console_init(unsigned long base_addr, unsigned int uart_clk, unsigned int baud_rate)
{
	/* TODO: assert() internally calls printf() and will result in
	 * an infinite loop. This needs to be fixed with some kind of
	 * exception  mechanism or early panic support. This also applies
	 * to the other assert() calls below.
	 */
	assert(base_addr);

	/* Initialise internal base address variable */
	uart_base = base_addr;

	/* Baud Rate */
	cadence_uart_setbaudrate(uart_base, baud_rate);
	/* FIXME: enable Tx/Rx clear interrupts etc.  */
	return 0;
}

/* MS call from ASM - fix this */
int console_core_putc(int c)
{
	return console_putc(c);
}

int console_putc(int c)
{
	assert(uart_base);

	if (c == '\n') {
		cadence_uart_tx(uart_base, '\r');
	}

	cadence_uart_tx(uart_base, c);
	return c;
}

int console_getc(void)
{
	assert(uart_base);

	return cadence_uart_rx(uart_base);
}
