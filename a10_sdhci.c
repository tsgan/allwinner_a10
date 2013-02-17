/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * Copyright (c) 2008 Alexander Motin <mav@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Allwinner A10 SDHCI driver.
 * Derived mainly from sdhci_pci.c
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/sdhci/sdhci.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

#define MAX_SLOTS	1

struct a10_sdhci_softc {
	device_t	dev;		/* Controller device */
	u_int		quirks;		/* Chip specific quirks */
	u_int		caps;		/* If we override SDHCI_CAPABILITIES */
	struct resource *irq_res;	/* IRQ resource */
	void 		*intrhand;	/* Interrupt handle */

	int		num_slots;	/* Number of slots on this controller*/
	struct sdhci_slot slots[MAX_SLOTS];
	struct resource	*mem_res[MAX_SLOTS];	/* Memory resource */
};

static uint8_t
a10_sdhci_read_1(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	return (bus_read_1(sc->mem_res[slot->num], off));
}

static void
a10_sdhci_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint8_t val)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	bus_write_1(sc->mem_res[slot->num], off, val);
}

static uint16_t
a10_sdhci_read_2(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	return (bus_read_2(sc->mem_res[slot->num], off));
}

static void
a10_sdhci_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint16_t val)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	bus_write_2(sc->mem_res[slot->num], off, val);
}

static uint32_t
a10_sdhci_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	return (bus_read_4(sc->mem_res[slot->num], off));
}

static void
a10_sdhci_write_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t val)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	bus_write_4(sc->mem_res[slot->num], off, val);
}

static void
a10_sdhci_read_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	bus_read_multi_4(sc->mem_res[slot->num], off, data, count);
}

static void
a10_sdhci_write_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	bus_write_multi_4(sc->mem_res[slot->num], off, data, count);
}

static void
a10_sdhci_intr(void *arg)
{
	struct a10_sdhci_softc *sc = (struct a10_sdhci_softc *)arg;
	int i;

	for (i = 0; i < sc->num_slots; i++) {
		struct sdhci_slot *slot = &sc->slots[i];
		sdhci_generic_intr(slot);
	}
}

static int
a10_sdhci_probe(device_t dev)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	phandle_t node;
	pcell_t cid;

	sc->quirks = 0;
	sc->num_slots = 1;

	if (ofw_bus_is_compatible(dev, "allwinner,sun4i-sdhci"))
		device_set_desc(dev, "Allwinner A10 SDHCI controller");
	else
		return (ENXIO);

	node = ofw_bus_get_node(dev);

	/* Allow dts to patch quirks and slots. */
	if ((OF_getprop(node, "quirks", &cid, sizeof(cid))) > 0)
		sc->quirks = fdt32_to_cpu(cid);
	if ((OF_getprop(node, "num-slots", &cid, sizeof(cid))) > 0)
		sc->num_slots = fdt32_to_cpu(cid);
		
	return (0);
}

static int
a10_sdhci_attach(device_t dev)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	int err, slots, rid, i;
	
	sc->dev = dev;

	/* Allocate IRQ. */
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Can't allocate IRQ\n");
		return (ENOMEM);
	}

	sc->caps = SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_180;
	sc->caps |= SDHCI_CAN_DO_HISPD;
	sc->quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK
	    | SDHCI_QUIRK_BROKEN_TIMEOUT_VAL
	    | SDHCI_QUIRK_MISSING_CAPS;

	/* Scan all slots. */
	slots = sc->num_slots;	/* number of slots determined in probe(). */
	sc->num_slots = 0;
	for (i = 0; i < slots; i++) {
		struct sdhci_slot *slot = &sc->slots[sc->num_slots];

		/* Allocate memory. */
		rid = 0;
		sc->mem_res[i] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &rid, RF_ACTIVE);
		if (sc->mem_res[i] == NULL) {
			device_printf(dev, "Can't allocate memory for "
			    "slot %d\n", i);
			continue;
		}

		slot->quirks = sc->quirks;
		slot->caps = sc->caps;

		if (sdhci_init_slot(dev, slot, i) != 0)
			continue;

		sc->num_slots++;
	}
	device_printf(dev, "%d slot(s) allocated\n", sc->num_slots);

	/* Activate the interrupt */
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, a10_sdhci_intr, sc, &sc->intrhand);
	if (err) {
		device_printf(dev, "Cannot setup IRQ\n");
		return (err);
	}

	/* Process cards detection. */
	for (i = 0; i < sc->num_slots; i++) {
		struct sdhci_slot *slot = &sc->slots[i];
		sdhci_start_slot(slot);
	}
		
	return (0);
}

static int
a10_sdhci_detach(device_t dev)
{
	struct a10_sdhci_softc *sc = device_get_softc(dev);
	int i;

	bus_teardown_intr(dev, sc->irq_res, sc->intrhand);
	bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq_res),
	    sc->irq_res);

	for (i = 0; i < sc->num_slots; i++) {
		struct sdhci_slot *slot = &sc->slots[i];

		sdhci_cleanup_slot(slot);
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res[i]), sc->mem_res[i]);
	}

	return (0);
}

static device_method_t a10_sdhci_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, 	a10_sdhci_probe),
	DEVMETHOD(device_attach, 	a10_sdhci_attach),
	DEVMETHOD(device_detach, 	a10_sdhci_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,	sdhci_generic_write_ivar),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_update_ios, 	sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_request, 	sdhci_generic_request),
	DEVMETHOD(mmcbr_get_ro, 	sdhci_generic_get_ro),
	DEVMETHOD(mmcbr_acquire_host, 	sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host, 	sdhci_generic_release_host),

	/* SDHCI registers accessors */
	DEVMETHOD(sdhci_read_1,		a10_sdhci_read_1),
	DEVMETHOD(sdhci_read_2,		a10_sdhci_read_2),
	DEVMETHOD(sdhci_read_4,		a10_sdhci_read_4),
	DEVMETHOD(sdhci_read_multi_4,	a10_sdhci_read_multi_4),
	DEVMETHOD(sdhci_write_1,	a10_sdhci_write_1),
	DEVMETHOD(sdhci_write_2,	a10_sdhci_write_2),
	DEVMETHOD(sdhci_write_4,	a10_sdhci_write_4),
	DEVMETHOD(sdhci_write_multi_4,	a10_sdhci_write_multi_4),

	DEVMETHOD_END
};

static driver_t a10_sdhci_driver = {
	"a10_sdhci",
	a10_sdhci_methods,
	sizeof(struct a10_sdhci_softc),
};
static devclass_t a10_sdhci_devclass;

DRIVER_MODULE(a10_sdhci, simplebus, a10_sdhci_driver, a10_sdhci_devclass, 0,0);
MODULE_DEPEND(a10_sdhci, sdhci, 1, 1, 1);
