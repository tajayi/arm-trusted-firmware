/*
 * Copyright (c) 2015, Xilinx Inc.
 * Written by Michal Simek.
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

#include <mmio.h>
#include <stddef.h>
#include <cadence/cdns_uart.h>

void cadence_serial_setbrg(uint32_t base, uint32_t clock, uint32_t baudrate)
{
	/* Calculation results. */
	uint32_t calc_bauderror, bdiv, bgen;
	uint32_t calc_baud = 0;
	uint32_t baud = baudrate;

	/* Covering case where input clock is so slow */
	if (clock < 1000000 && baudrate > 9600) {
		baud = 9600;
	}

	/*                master clock
	 * Baud rate = ------------------
	 *              bgen * (bdiv + 1)
	 *
	 * Find acceptable values for baud generation.
	 */
	for (bdiv = 4; bdiv < 255; bdiv++) {
		bgen = clock / (baud * (bdiv + 1));
		if (bgen < 2 || bgen > 65535)
			continue;

		calc_baud = clock / (bgen * (bdiv + 1));

		/*
		 * Use first calculated baudrate with
		 * an acceptable (<3%) error
		 */
		if (baud > calc_baud)
			calc_bauderror = baud - calc_baud;
		else
			calc_bauderror = calc_baud - baud;
		if (((calc_bauderror * 100) / baud) < 3)
			break;
	}

	mmio_write_32(base + R_UART_BRD, bdiv);
	mmio_write_32(base + R_UART_BRG, bgen);
}
