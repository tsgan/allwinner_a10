/*-
 * Copyright (c) 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/arm/allwinner/a20_wdog.c 246707 2013-02-12 07:27:40Z gonzo $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/watchdog.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/machdep.h>
#include <machine/fdt.h>

#include <arm/allwinner/a20/a20_wdog.h>

#define	READ(_sc, _r) bus_read_4((_sc)->res, (_r))
#define	WRITE(_sc, _r, _v) bus_write_4((_sc)->res, (_r), (_v))

#define	WDOG_CTRL		0x00
#define		WDOG_CTRL_RESTART	(1 << 0)
#define	WDOG_MODE		0x04
#define		WDOG_MODE_INTVL_SHIFT	3
#define		WDOG_MODE_RST_EN	(1 << 1)
#define		WDOG_MODE_EN		(1 << 0)

struct a20wd_interval {
	uint64_t	milliseconds;
	unsigned int	value;
};

struct a20wd_interval wd_intervals[] = {
	{   500,	 0 },
	{  1000,	 1 },
	{  2000,	 2 },
	{  3000,	 3 },
	{  4000,	 4 },
	{  5000,	 5 },
	{  6000,	 6 },
	{  8000,	 7 },
	{ 10000,	 8 },
	{ 12000,	 9 },
	{ 14000,	10 },
	{ 16000,	11 },
	{ 0,		 0 } /* sentinel */
};

static struct a20wd_softc *a20wd_sc = NULL;

struct a20wd_softc {
	device_t		dev;
	struct resource *	res;
	struct mtx		mtx;
};

static void a20wd_watchdog_fn(void *private, u_int cmd, int *error);

static int
a20wd_probe(device_t dev)
{

	if (ofw_bus_is_compatible(dev, "allwinner,sun7i-wdt")) {
		device_set_desc(dev, "Allwinner A20 Watchdog");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
a20wd_attach(device_t dev)
{
	struct a20wd_softc *sc;
	int rid;

	if (a20wd_sc != NULL)
		return (ENXIO);

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	a20wd_sc = sc;
	mtx_init(&sc->mtx, "A20 Watchdog", "a20wd", MTX_DEF);
	EVENTHANDLER_REGISTER(watchdog_list, a20wd_watchdog_fn, sc, 0);

	return (0);
}

static void
a20wd_watchdog_fn(void *private, u_int cmd, int *error)
{
	struct a20wd_softc *sc;
	uint64_t ms;
	int i;

	sc = private;
	mtx_lock(&sc->mtx);

	cmd &= WD_INTERVAL;

	if (cmd > 0) {
		ms = ((uint64_t)1 << (cmd & WD_INTERVAL)) / 1000000;
		i = 0;
		while (wd_intervals[i].milliseconds && 
		    (ms > wd_intervals[i].milliseconds))
			i++;
		if (wd_intervals[i].milliseconds) {
			WRITE(sc, WDOG_MODE, 
			    (wd_intervals[i].value << WDOG_MODE_INTVL_SHIFT) |
			    WDOG_MODE_EN | WDOG_MODE_RST_EN);
			WRITE(sc, WDOG_CTRL, WDOG_CTRL_RESTART);
		}
	}
	else
		WRITE(sc, WDOG_MODE, 0);

	mtx_unlock(&sc->mtx);
}

void
a20wd_watchdog_reset()
{

	if (a20wd_sc == NULL) {
		printf("Reset: watchdog device has not been initialized\n");
		return;
	}

	WRITE(a20wd_sc, WDOG_MODE, 
	    (wd_intervals[0].value << WDOG_MODE_INTVL_SHIFT) |
	    WDOG_MODE_EN | WDOG_MODE_RST_EN);

	while(1)
		;

}

static device_method_t a20wd_methods[] = {
	DEVMETHOD(device_probe, a20wd_probe),
	DEVMETHOD(device_attach, a20wd_attach),

	DEVMETHOD_END
};

static driver_t a20wd_driver = {
	"a20wd",
	a20wd_methods,
	sizeof(struct a20wd_softc),
};
static devclass_t a20wd_devclass;

DRIVER_MODULE(a20wd, simplebus, a20wd_driver, a20wd_devclass, 0, 0);
