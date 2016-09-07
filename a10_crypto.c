/*-
 * Copyright (c) 2016 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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

/*
 * Allwinner A10/A20 Security System/Crypto accelerator
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/endian.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <machine/bus.h>
#include <sys/random.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/extres/clk/clk.h>

#include <arm/allwinner/a10_crypto.h>

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun4i-a10-crypto",		1},
	{ NULL,					0 }
};

struct a10_crypto_softc {
	device_t		dev;
	struct resource		*res;
	struct callout		co;
	int			ticks;
	uint32_t		sc_seed[A10_SS_FIFO_WORDS];
	uint32_t		sc_buf[A10_SS_FIFO_WORDS];
};

static struct resource_spec a10_crypto_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

#define	RD4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static void
a10_rng_start(struct a10_crypto_softc *sc)
{
	uint32_t prng_mode;
	int i;

	prng_mode = A10_SS_OP_PRNG | A10_SS_PRNG_CONTINUE | A10_SS_ENABLED;

	/* Write PRNG mode */
	WR4(sc, A10_SS_CTL, prng_mode);

	/* Write the seed */
	for (i = 0; i < A10_SS_FIFO_WORDS; i++)
		WR4(sc, A10_SS_KEY0 + i * 4, sc->sc_seed[i]);

	/* Start PRNG */
	WR4(sc, A10_SS_CTL, prng_mode | A10_SS_PRNG_START);
}

static void
a10_rng_stop(struct a10_crypto_softc *sc)
{
	uint32_t ctrl;

	/* Disable PRNG */
	ctrl = RD4(sc, A10_SS_CTL);
	ctrl &= ~A10_SS_PRNG_START;
	WR4(sc, A10_SS_CTL, ctrl);

	/* Disable and flush FIFO */
	WR4(sc, A10_SS_CTL, A10_SS_DISABLED);
}

static void
a10_rng_harvest(void *arg)
{
	struct a10_crypto_softc *sc = arg;
	uint32_t val;
	int i;

	/* Start PRNG */
	a10_rng_start(sc);

	/* Read random data */
	bus_read_multi_4(sc->res, A10_SS_TXFIFO, sc->sc_buf,
	    A10_SS_DATA_LEN / sizeof(uint32_t));

	/* Update the seed */
	for (i = 0; i < A10_SS_FIFO_WORDS; i++) {
		val = RD4(sc, A10_SS_KEY0 + i * 4);
		sc->sc_seed[i] = val;
	}

	/* Disable and flush FIFO */
	WR4(sc, A10_SS_CTL, A10_SS_DISABLED);

	random_harvest_queue(sc->sc_buf, sizeof(sc->sc_buf),
	    sizeof(sc->sc_buf) * NBBY / 2, RANDOM_PURE_ALLWINNER);

	callout_reset(&sc->co, sc->ticks, a10_rng_harvest, sc);
}

static int
a10_crypto_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "A10/A20 Security System/Crypto accelerator");
	return (BUS_PROBE_DEFAULT);
}

static int
a10_crypto_attach(device_t dev)
{
	struct a10_crypto_softc *sc;
	clk_t clk_ss, clk_gate;
	int err, i;
	uint32_t val = 0;

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, a10_crypto_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	/* Get clocks and enable them */
	err = clk_get_by_ofw_name(dev, 0, "ahb", &clk_gate);
	if (err != 0) {
		device_printf(dev, "Cannot get gate clock\n");
		return (ENXIO);
	}
	err = clk_get_by_ofw_name(dev, 0, "mod", &clk_ss);
	if (err != 0) {
		device_printf(dev, "Cannot get SS clock\n");
		return (ENXIO);
	}
	err = clk_enable(clk_gate);
	if (err != 0) {
		device_printf(dev, "Cannot enable clk gate\n");
		goto error;
	}
	err = clk_enable(clk_ss);
	if (err != 0) {
		device_printf(dev, "Cannot enable SS clock\n");
		goto error;
	}
	/*
	 * "Die Bonding ID"
	 */
	WR4(sc, A10_SS_CTL, A10_SS_ENABLED);
	val = RD4(sc, A10_SS_CTL);
	val >>= DIE_ID_START_BIT;
	val &= DIE_ID_MASK;
	device_printf(dev, "Die ID %d\n", val);
	WR4(sc, A10_SS_CTL, 0);

	/* Install a periodic collector for the RNG */
	if (hz > 100)
		sc->ticks = hz / 100;
	else
		sc->ticks = 1;

	/* Generate the seed fist */
	for (i = 0; i < A10_SS_FIFO_WORDS; i++)
		sc->sc_seed[i] = arc4random();

	callout_init(&sc->co, 1);
	callout_reset(&sc->co, sc->ticks, a10_rng_harvest, sc);

	return (0);

error:
	if (clk_gate != NULL)
		clk_release(clk_gate);
	if (clk_ss != NULL)
		clk_release(clk_ss);
	bus_release_resources(dev, a10_crypto_spec, &sc->res);
	return (ENXIO);
}

static int
a10_crypto_detach(device_t dev)
{
	struct a10_crypto_softc *sc = device_get_softc(dev);

	a10_rng_stop(sc);
	callout_drain(&sc->co);

	bus_release_resources(dev, a10_crypto_spec, &sc->res);

	return (0);
}

static device_method_t a10_crypto_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a10_crypto_probe),
	DEVMETHOD(device_attach,	a10_crypto_attach),
	DEVMETHOD(device_detach,	a10_crypto_detach),

	DEVMETHOD_END
};

static driver_t a10_crypto_driver = {
	"a10_crypto",
	a10_crypto_methods,
	sizeof(struct a10_crypto_softc),
};

static devclass_t a10_crypto_devclass;

DRIVER_MODULE(a10_crypto, simplebus, a10_crypto_driver, a10_crypto_devclass, 0, 0);
MODULE_VERSION(a10_crypto, 1);
MODULE_DEPEND(a10_crypto, crypto, 1, 1, 1);
MODULE_DEPEND(a10_crypto, randomdev, 1, 1, 1);
