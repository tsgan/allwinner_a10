/*-
 * Copyright (c) 2012 Ganbold Tsagaankhuu <ganbold@gmail.com>
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
__FBSDID("$FreeBSD$");

#include "opt_bus.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/resource.h>
#include <sys/systm.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/endian.h>
#include <sys/sema.h>
#include <sys/taskqueue.h>
#include <vm/uma.h>
#include <sys/gpio.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_sim.h>
#include <cam/cam_xpt_sim.h>
#include <cam/cam_debug.h>

#include <sys/ata.h>
#include <dev/ata/ata-all.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio_if.h"

#include "a10_clk.h"

#include "ata_if.h"

#define SATA_CHAN_NUM			1

struct a10_ahci_softc {
	device_t		sc_dev;
	unsigned int		sc_version;
	struct resource		*sc_mem_res;
	bus_space_tag_t		sc_mem_res_bustag;
	bus_space_handle_t	sc_mem_res_bushdl;
	struct resource		*sc_irq_res;
	void			*sc_irq_cookiep;
        struct {
		void	(*function)(void *);
		void	*argument;
	} sc_interrupt[SATA_CHAN_NUM];
};

#define AHCI_READ_4(sc, reg)             \
        bus_space_read_4((sc)->sc_mem_res_bustag, (sc)->sc_mem_res_bushdl, reg)

#define AHCI_WRITE_4(sc, reg, data)      \
        bus_space_write_4((sc)->sc_mem_res_bustag, (sc)->sc_mem_res_bushdl, reg, data)

#define SW_AHCI_BISTAFR		0x00A0
#define SW_AHCI_BISTCR		0x00A4
#define SW_AHCI_BISTFCTR	0x00A8
#define SW_AHCI_BISTSR		0x00AC
#define SW_AHCI_BISTDECR	0x00B0
#define SW_AHCI_DIAGNR		0x00B4
#define SW_AHCI_DIAGNR1		0x00B8
#define SW_AHCI_OOBR		0x00BC
#define SW_AHCI_PHYCS0R		0x00C0
#define SW_AHCI_PHYCS1R		0x00C4
#define SW_AHCI_PHYCS2R		0x00C8
#define SW_AHCI_TIMER1MS	0x00E0
#define SW_AHCI_GPARAM1R	0x00E8
#define SW_AHCI_GPARAM2R	0x00EC
#define SW_AHCI_PPARAMR		0x00F0
#define SW_AHCI_TESTR		0x00F4
#define SW_AHCI_VERSIONR	0x00F8
#define SW_AHCI_IDR		0x00FC
#define SW_AHCI_RWCR		0x00FC

#define SW_AHCI_P0DMACR		0x0170
#define SW_AHCI_P0PHYCR		0x0178
#define SW_AHCI_P0PHYSR		0x017C

#define GPIO_AHCI_PWR		40

/* Per-port registers */
#define AHCI_P_OFFSET(port)	(0x80 * (port))
#define SW_AHCI_P_DMA(p)	(0x170 + AHCI_P_OFFSET(p))

static int	a10_ahci_probe(device_t dev);
static int	a10_ahci_attach(device_t dev);
static int	a10_ahci_detach(device_t dev);
static void	a10_ahci_intr(void*);
static void 	a10_ahci_phy_init(device_t dev);
static struct resource * a10_ahci_alloc_resource(device_t dev, device_t child,
    int type, int *rid, u_long start, u_long end, u_long count, u_int flags);
static int	a10_ahci_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r);
static int	a10_ahci_setup_intr(device_t dev, device_t child,
    struct resource *irq, int flags, driver_filter_t *filt,
    driver_intr_t *function, void *argument, void **cookiep);
static int	a10_ahci_teardown_intr(device_t dev, device_t child,
    struct resource *irq, void *cookie);

static int
a10_ahci_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "allwinner,ahci"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner Integrated AHCI Controller");
	return (0);
}

static void 
a10_ahci_phy_init(device_t dev)
{
	struct a10_ahci_softc *sc;
	uint32_t reg_val;
	uint32_t timeout;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	DELAY(5000);
	AHCI_WRITE_4(sc, SW_AHCI_RWCR, 0);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS1R);
	reg_val |= (1 << 19);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS1R, reg_val);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS0R);
	reg_val |= (1 << 18);
	reg_val |= (1 << 23);
	reg_val |= (1 << 24);
	reg_val |= (1 << 26);
	reg_val &= ~(1 << 25);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS0R, reg_val);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS1R);
	reg_val |= (1 << 7);
	reg_val |= (1 << 9);
	reg_val |= (1 << 10);
	reg_val |= (1 << 17);

	reg_val &= ~(1 << 6);
	reg_val &= ~(1 << 8);
	reg_val &= ~(1 << 11);
	reg_val &= ~(1 << 12);
	reg_val &= ~(1 << 16);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS1R, reg_val);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS1R);
	reg_val |= (1 << 15);
	reg_val |= (1 << 28);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS1R, reg_val);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS1R);
	reg_val &= ~(1 << 19);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS1R, reg_val);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS0R);
	reg_val |= (1 << 20);
	reg_val |= (1 << 21);
	reg_val &= ~(1 << 22);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS0R, reg_val);
	DELAY(10);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS2R);
	reg_val |= (1 << 5);
	reg_val |= (1 << 8);
	reg_val |= (1 << 9);
	reg_val &= ~(1 << 6);
	reg_val &= ~(1 << 7);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS2R, reg_val);
	DELAY(10);

	reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS0R);
	reg_val |= (1 << 19);
	AHCI_WRITE_4(sc, SW_AHCI_PHYCS0R, reg_val);

	timeout = 1000;
	do {
		DELAY(1);
		reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS0R);
	} while(--timeout && 
		SHIFTOUT(reg_val, ((1 << 28) | (1 << 30)) != 2));

	if(!timeout)
		device_printf(dev, "SATA AHCI Phy Power Failed!!\n");
	else {
		reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS2R);
		reg_val |= (1 << 24);
		AHCI_WRITE_4(sc, SW_AHCI_PHYCS2R, reg_val);
		timeout = 1000;
		do {
			DELAY(10);
			reg_val = AHCI_READ_4(sc, SW_AHCI_PHYCS2R);
		} while(--timeout && (reg_val & (1 << 24)));
		if(!timeout)
			device_printf(dev, "SATA AHCI Phy calibration Failed!!\n");
	}
	DELAY(10);
	AHCI_WRITE_4(sc, SW_AHCI_RWCR, 0x07);
}

static int
a10_ahci_attach(device_t dev)
{
	struct a10_ahci_softc *sc;
//	struct ata_channel *ch;
	int mem_id, irq_id, error, i;
	device_t child;
        device_t sc_gpio_dev;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	/* Allocate resources */
	mem_id = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &mem_id, RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "could not allocate memory.\n");
                error = ENOMEM;
                goto err;
	}

	sc->sc_mem_res_bustag = rman_get_bustag(sc->sc_mem_res);
	sc->sc_mem_res_bushdl = rman_get_bushandle(sc->sc_mem_res);
	KASSERT(sc->sc_mem_res_bustag && sc->sc_mem_res_bushdl,
	    ("cannot get bus handle or tag."));

	irq_id = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_id,
	    RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "could not allocate IRQ.\n");
		error = ENOMEM;
		goto err;
	}

	/* Enable SATA Clock in SATA PLL */
	a10_clk_sata_activate();

	/* Init phy */
	a10_ahci_phy_init(dev);

        /* Get the GPIO device, we need this to give power to USB */
        sc_gpio_dev = devclass_get_device(devclass_find("gpio"), 0);
        if (sc_gpio_dev == NULL) {
                device_printf(dev, "Error: failed to get the GPIO device\n");
                error = ENXIO;
                goto err;
        }

        /* Give power to SATA */
        GPIO_PIN_SETFLAGS(sc_gpio_dev, GPIO_AHCI_PWR, GPIO_PIN_OUTPUT);
        GPIO_PIN_SET(sc_gpio_dev, GPIO_AHCI_PWR, GPIO_PIN_HIGH);

	error = bus_setup_intr(dev, sc->sc_irq_res,
	    INTR_TYPE_BIO | INTR_MPSAFE | INTR_ENTROPY,
	    NULL, a10_ahci_intr, sc, &sc->sc_irq_cookiep);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt.\n");
		goto err;
	}

	/* Attach channels */
	for (i = 0; i < SATA_CHAN_NUM; i++) {
		child = device_add_child(dev, "ata", -1);
		if (!child) {
			device_printf(dev, "cannot add channel %d.\n", i);
			error = ENOMEM;
			goto err;
		}
	}

	bus_generic_attach(dev);
	return (0);
err:
	a10_ahci_detach(dev);
	return (error);
}

static int
a10_ahci_detach(device_t dev)
{
	struct a10_ahci_softc *sc;

	sc = device_get_softc(dev);

	if (device_is_attached(dev))
		bus_generic_detach(dev);

	if (sc->sc_mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->sc_mem_res), sc->sc_mem_res);
		sc->sc_mem_res = NULL;
	}

	if (sc->sc_irq_res != NULL) {
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_cookiep);
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(sc->sc_irq_res), sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}

	return (0);
}

static void
a10_ahci_intr(void *xsc)
{
	struct a10_ahci_softc *sc;
	int unit;

	sc = xsc;

	/*
	 * Behave like ata_generic_intr() for PCI controllers.
	 * Simply invoke ISRs on all channels.
	 */
	for (unit = 0; unit < SATA_CHAN_NUM; unit++)
		if (sc->sc_interrupt[unit].function != NULL)
			sc->sc_interrupt[unit].function(
				sc->sc_interrupt[unit].argument);
}

static struct resource *
a10_ahci_alloc_resource(device_t dev, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct a10_ahci_softc *sc;

	sc = device_get_softc(dev);

	KASSERT(type == SYS_RES_IRQ && *rid == ATA_IRQ_RID,
	    ("illegal resource request (type %u, rid %u).",
	    type, *rid));

	return (sc->sc_irq_res);
}

static int
a10_ahci_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{

	KASSERT(type == SYS_RES_IRQ && rid == ATA_IRQ_RID,
	    ("strange type %u and/or rid %u while releasing resource.", type,
	    rid));

	return (0);
}

static int
a10_ahci_setup_intr(device_t dev, device_t child, struct resource *irq, int flags,
    driver_filter_t *filt, driver_intr_t *function, void *argument,
    void **cookiep)
{
	struct a10_ahci_softc *sc;
	struct ata_channel *ch;

	sc = device_get_softc(dev);
	ch = device_get_softc(child);

	if (filt != NULL) {
		device_printf(dev, "filter interrupts are not supported.\n");
		return (EINVAL);
	}

	sc->sc_interrupt[ch->unit].function = function;
	sc->sc_interrupt[ch->unit].argument = argument;
	*cookiep = sc;

	return (0);
}

static int
a10_ahci_teardown_intr(device_t dev, device_t child, struct resource *irq,
    void *cookie)
{
	struct a10_ahci_softc *sc;
	struct ata_channel *ch;

	sc = device_get_softc(dev);
	ch = device_get_softc(child);

	sc->sc_interrupt[ch->unit].function = NULL;
	sc->sc_interrupt[ch->unit].argument = NULL;

	return (0);
}

static device_method_t a10_ahci_methods[] = {
	/* Device method */
	DEVMETHOD(device_probe,		a10_ahci_probe),
	DEVMETHOD(device_attach,	a10_ahci_attach),
	DEVMETHOD(device_detach,	a10_ahci_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),

	DEVMETHOD(bus_alloc_resource,		a10_ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,		a10_ahci_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,		a10_ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,		a10_ahci_teardown_intr),

	DEVMETHOD_END
};

static driver_t a10_ahci_driver = {
	"a10_ahci",
	a10_ahci_methods,
	sizeof(struct a10_ahci_softc),
};

devclass_t a10_ahci_devclass;

DRIVER_MODULE(a10_ahci, simplebus, a10_ahci_driver, a10_ahci_devclass, 0, 0);
MODULE_VERSION(a10_ahci, 1);
MODULE_DEPEND(a10_ahci, ata, 1, 1, 1);

static void
ahci_channel_start(device_t dev, struct ata_channel *ch)
{
        struct a10_ahci_softc *sc = device_get_softc(device_get_parent(dev));
	uint32_t dma_reg, dma;

	printf("---------- dma_reg first ------------\n");
	dma_reg = SW_AHCI_P_DMA(ch->unit);

	printf("---------- dma_reg %x ------------\n", dma_reg);

	dma = AHCI_READ_4(sc, dma_reg);
	dma &= ~0xff00;
	dma |= 0x4400;
	AHCI_WRITE_4(sc, dma_reg, dma);
	printf("---------- dma_reg last ------------\n");
}

static int
ahci_channel_probe(device_t dev)
{
        struct ata_channel *ch = device_get_softc(dev);

        ch->unit = 0;
        ch->flags |= ATA_USE_16BIT | ATA_NO_SLAVE;
        device_set_desc_copy(dev, "Allwinner Integrated SATA channel 0");

        return ata_probe(dev);
}

static int
ahci_channel_attach(device_t dev)
{
        struct a10_ahci_softc *sc = device_get_softc(device_get_parent(dev));
        struct ata_channel *ch = device_get_softc(dev);
        int i;

        if (ch->attached)
                return (0);

        ch->dev = dev;
//        ch->unit = device_get_unit(dev);
        ch->flags |= ATA_USE_16BIT | ATA_NO_SLAVE | ATA_SATA;

        for (i = 0; i < ATA_MAX_RES; i++)
                ch->r_io[i].res = sc->sc_mem_res;

        ch->r_io[ATA_ALTSTAT].res = sc->sc_mem_res;
        ch->r_io[ATA_ALTSTAT].offset = ATA_ALTSTAT;
        ch->r_io[ATA_CONTROL].res = sc->sc_mem_res;
        ch->r_io[ATA_CONTROL].offset = ATA_CONTROL;

        ch->r_io[ATA_DATA].offset = ATA_DATA;
        ch->r_io[ATA_FEATURE].offset = ATA_FEATURE;
        ch->r_io[ATA_COUNT].offset = ATA_COUNT;
        ch->r_io[ATA_SECTOR].offset = ATA_SECTOR;
        ch->r_io[ATA_CYL_LSB].offset = ATA_CYL_LSB;
        ch->r_io[ATA_CYL_MSB].offset = ATA_CYL_MSB;
        ch->r_io[ATA_DRIVE].offset = ATA_DRIVE;
        ch->r_io[ATA_COMMAND].offset = ATA_COMMAND;
        ch->r_io[ATA_ERROR].offset = ATA_FEATURE;
        /* NB: should be used only for ATAPI devices */
        ch->r_io[ATA_IREASON].offset = ATA_COUNT;
        ch->r_io[ATA_STATUS].offset = ATA_COMMAND;

        ch->r_io[ATA_IDX_ADDR].offset = 0;
	ata_default_registers(dev);

        /* Set SATA resources. */
        ch->r_io[ATA_SSTATUS].res = sc->sc_mem_res;
        ch->r_io[ATA_SSTATUS].offset = ATA_SSTATUS;
        ch->r_io[ATA_SERROR].res = sc->sc_mem_res;
        ch->r_io[ATA_SERROR].offset = ATA_SERROR;
        ch->r_io[ATA_SCONTROL].res = sc->sc_mem_res;
        ch->r_io[ATA_SCONTROL].offset = ATA_SCONTROL;

	printf("---------- 14.1 ------------\n");
	ahci_channel_start(dev, ch);
	printf("---------- 14.2 ------------\n");

        ata_generic_hw(dev);
        ch->attached = 1;

        return ata_attach(dev);
}

static device_method_t ahci_channel_methods[] = {
        /* device interface */
        DEVMETHOD(device_probe,     ahci_channel_probe),
        DEVMETHOD(device_attach,    ahci_channel_attach),
        DEVMETHOD(device_detach,    ata_detach),
        DEVMETHOD(device_shutdown,  bus_generic_shutdown),
        DEVMETHOD(device_suspend,   ata_suspend),
        DEVMETHOD(device_resume,    ata_resume),

        { 0, 0 }
};

driver_t ahci_channel_driver = {
        "ata",
        ahci_channel_methods,
        sizeof(struct ata_channel),
};
DRIVER_MODULE(ata, a10_ahci, ahci_channel_driver, ata_devclass, 0, 0);

