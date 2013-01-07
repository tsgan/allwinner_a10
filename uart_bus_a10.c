/*-
 * Copyright (C) 2012 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of MARVELL nor the names of contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>

#include "gpio.h"

static int uart_a10_probe(device_t dev);

static device_method_t uart_a10_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,	uart_a10_probe),
	DEVMETHOD(device_attach, uart_bus_attach),
	DEVMETHOD(device_detach, uart_bus_detach),
	{ 0, 0 }
};

static driver_t uart_a10_driver = {
	uart_driver_name,
	uart_a10_methods,
	sizeof(struct uart_softc),
};

static int
uart_a10_probe(device_t dev)
{
	struct	uart_softc *sc;
	int status;

	/* set gpio config pin for UART0 */

	a10_gpio_set_cfgpin(A10_GPB(22), A10_GPB22_UART0_TX);
	a10_gpio_set_cfgpin(A10_GPB(23), A10_GPB23_UART0_RX);

        volatile uint32_t *ccm_apb1_gating = (uint32_t *) 0xe1c2006c;
        volatile uint32_t *ccm_apb1_clk_div_cfg = (uint32_t *) 0xe1c20058;
        
        /* config apb1 clock */
        *ccm_apb1_clk_div_cfg &= ~(1 << 24);
        *ccm_apb1_clk_div_cfg &= ~(1 << 25);
        
        *ccm_apb1_clk_div_cfg &= ~(1 << 16);
        *ccm_apb1_clk_div_cfg &= ~(1 << 17);
        
        *ccm_apb1_clk_div_cfg &= ~(1 << 0);
        *ccm_apb1_clk_div_cfg &= ~(1 << 1);
        *ccm_apb1_clk_div_cfg &= ~(1 << 2);
        *ccm_apb1_clk_div_cfg &= ~(1 << 3);
        *ccm_apb1_clk_div_cfg &= ~(1 << 4);
          
        /* Gating clock for uart0 */
	*ccm_apb1_gating |= (1 << 16);  /* clock gate uart0 */

	sc = device_get_softc(dev);
	sc->sc_class = &uart_ns8250_class;
	status = uart_bus_probe(dev, 2, 24000000, 0, 0);
	return (status);
}


DRIVER_MODULE(uart, simplebus, uart_a10_driver, uart_devclass, 0, 0);
