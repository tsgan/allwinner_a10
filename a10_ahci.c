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

#include "ata_if.h"

#define SATA_CHAN_NUM                   1

/* Identification section. */
struct a10_ahci_softc {
	device_t		sc_dev;
	unsigned int		sc_version;
	struct resource		*sc_mem_res;
	bus_space_tag_t		sc_mem_res_bustag;
	bus_space_handle_t	sc_mem_res_bushdl;
	struct resource		*sc_irq_res;
	void			*sc_irq_cookiep;
        struct {
                void    (*function)(void *);
                void    *argument;
        } sc_interrupt[SATA_CHAN_NUM];
};


#define	ahci_readl(base,offset) 	(*((volatile uint32_t *)((base)+(offset))))
#define ahci_writel(base,offset,val)	(*((volatile uint32_t *)((base)+(offset))) = (val))

#define SW_AHCI_BASE			0xe1c18000

#define SW_AHCI_BISTAFR_OFFSET		0x00A0
#define SW_AHCI_BISTCR_OFFSET		0x00A4
#define SW_AHCI_BISTFCTR_OFFSET		0x00A8
#define SW_AHCI_BISTSR_OFFSET		0x00AC
#define SW_AHCI_BISTDECR_OFFSET		0x00B0
#define SW_AHCI_DIAGNR_OFFSET		0x00B4
#define SW_AHCI_DIAGNR1_OFFSET		0x00B8
#define SW_AHCI_OOBR_OFFSET		0x00BC
#define SW_AHCI_PHYCS0R_OFFSET		0x00C0
#define SW_AHCI_PHYCS1R_OFFSET		0x00C4
#define SW_AHCI_PHYCS2R_OFFSET		0x00C8
#define SW_AHCI_TIMER1MS_OFFSET		0x00E0
#define SW_AHCI_GPARAM1R_OFFSET		0x00E8
#define SW_AHCI_GPARAM2R_OFFSET		0x00EC
#define SW_AHCI_PPARAMR_OFFSET		0x00F0
#define SW_AHCI_TESTR_OFFSET		0x00F4
#define SW_AHCI_VERSIONR_OFFSET		0x00F8
#define SW_AHCI_IDR_OFFSET		0x00FC
#define SW_AHCI_RWCR_OFFSET		0x00FC

#define SW_AHCI_P0DMACR_OFFSET		0x0170
#define SW_AHCI_P0PHYCR_OFFSET		0x0178
#define SW_AHCI_P0PHYSR_OFFSET		0x017C

#define SW_AHCI_ACCESS_LOCK(base,x)	(*((volatile uint32_t *)((base)+SW_AHCI_RWCR_OFFSET)) = (x))

#define INTC_IRQNO_AHCI			56

#define CCMU_PLL6_VBASE			0xe1c20028


/* Controller functions */
static int	a10_ahci_probe(device_t dev);
static int	a10_ahci_attach(device_t dev);
static int	a10_ahci_detach(device_t dev);
static void     a10_ahci_intr(void*);
static struct resource * a10_ahci_alloc_resource(device_t dev, device_t child,
    int type, int *rid, u_long start, u_long end, u_long count, u_int flags);
static int      a10_ahci_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r);
static int      a10_ahci_setup_intr(device_t dev, device_t child,
    struct resource *irq, int flags, driver_filter_t *filt,
    driver_intr_t *function, void *argument, void **cookiep);
static int      a10_ahci_teardown_intr(device_t dev, device_t child,
    struct resource *irq, void *cookie);


static int
a10_ahci_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "a10,ahci"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner Integrated AHCI Controller");
	return (0);
}

static int
a10_ahci_attach(device_t dev)
{
	struct a10_ahci_softc *sc;
	int mem_id, irq_id, error, i;
	device_t child;
	uint32_t tmp;
	uint32_t timeout_val = 0x100000;
	uint32_t timeout = timeout_val;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	mem_id = 0;
	irq_id = 0;


	printf("---------- 10 ------------\n");

	/*Enable SATA Clock in SATA PLL*/
	ahci_writel(CCMU_PLL6_VBASE, 0, ahci_readl(CCMU_PLL6_VBASE, 0)|(0x1<<14));

	for(tmp=0; tmp<0x1000; tmp++);

	/* phy init */

	printf("---------- 10 next ------------\n");

//	volatile uint32_t *base = (uint32_t *) 0xe1c180fc;
//	*base = 0;

	SW_AHCI_ACCESS_LOCK(SW_AHCI_BASE, 0);

	printf("---------- 100 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET);
	tmp |= (0x1<<19);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET, tmp);

	printf("---------- 1001 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET);
	tmp |= 0x1<<23;
	tmp |= 0x1<<18;
	tmp &= ~(0x7<<24);
	tmp |= 0x5<<24;
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET, tmp);

	printf("---------- 1002 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET);
	tmp &= ~(0x3<<16);
	tmp |= (0x2<<16);
	tmp &= ~(0x1f<<8);
	tmp |= (6<<8);
	tmp &= ~(0x3<<6);
	tmp |= (2<<6);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET, tmp);

	printf("---------- 1003 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET);
	tmp |= (0x1<<28);
	tmp |= (0x1<<15);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET, tmp);

	printf("---------- 1004 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET);
	tmp &= ~(0x1<<19);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS1R_OFFSET, tmp);

	printf("---------- 1005 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET);
	tmp &= ~(0x7<<20);
	tmp |= (0x03<<20);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET, tmp);

	printf("---------- 1006 ------------\n");

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS2R_OFFSET);
	tmp &= ~(0x1f<<5);
	tmp |= (0x19<<5);
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS2R_OFFSET, tmp);

	printf("---------- 110 XXX ------------\n");

	for(tmp=0; tmp<0x1000; tmp++);

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET);
	tmp |= 0x1<<19;
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET, tmp);

	printf("---------- 110 ------------\n");

	timeout = timeout_val;
	do{
		tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS0R_OFFSET);
		timeout --;
		if(!timeout) break;
	}while((tmp&(0x7<<28))!=(0x02<<28));

	printf("---------- 1100 ------------\n");

	if(!timeout)
	{
		device_printf(dev, "SATA AHCI Phy Power Failed!!\n");
	}

	tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS2R_OFFSET);
	tmp |= 0x1<<24;
	ahci_writel(SW_AHCI_BASE, SW_AHCI_PHYCS2R_OFFSET, tmp);

	printf("---------- 11000 ------------\n");

	timeout = timeout_val;
	do{
		tmp = ahci_readl(SW_AHCI_BASE, SW_AHCI_PHYCS2R_OFFSET);
		timeout --;
		if(!timeout) break;
	}while(tmp&(0x1<<24));

	printf("---------- 111 ------------\n");

	if(!timeout)
	{
		device_printf(dev, "SATA AHCI Phy Calibration Failed!!\n");
	}
	for(tmp=0; tmp<0x3000; tmp++);

	SW_AHCI_ACCESS_LOCK(SW_AHCI_BASE, 0x07);

	printf("---------- 11 ------------\n");

	/* Allocate resources */
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &mem_id, RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "could not allocate memory.\n");
		return (ENOMEM);
	}

	printf("---------- 12 ------------\n");

	sc->sc_mem_res_bustag = rman_get_bustag(sc->sc_mem_res);
	sc->sc_mem_res_bushdl = rman_get_bushandle(sc->sc_mem_res);
	KASSERT(sc->sc_mem_res_bustag && sc->sc_mem_res_bushdl,
	    ("cannot get bus handle or tag."));

	printf("---------- 13 ------------\n");

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_id,
	    RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "could not allocate IRQ.\n");
		error = ENOMEM;
		goto err;
	}

	printf("---------- 0 ------------\n");

	error = bus_setup_intr(dev, sc->sc_irq_res,
	    INTR_TYPE_BIO | INTR_MPSAFE | INTR_ENTROPY,
	    NULL, a10_ahci_intr, sc, &sc->sc_irq_cookiep);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt.\n");
		goto err;
	}

	printf("---------- 1 ------------\n");

        /* Attach channels */
        for (i = 0; i < SATA_CHAN_NUM; i++) {
                child = device_add_child(dev, "ahcich", -1);

                if (!child) {
                        device_printf(dev, "cannot add channel %d.\n", i);
                        error = ENOMEM;
                        goto err;
                }
        }

	printf("---------- 2 ------------\n");


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

        DEVMETHOD(bus_alloc_resource,           a10_ahci_alloc_resource),
        DEVMETHOD(bus_release_resource,         a10_ahci_release_resource),
        DEVMETHOD(bus_activate_resource,        bus_generic_activate_resource),
        DEVMETHOD(bus_deactivate_resource,      bus_generic_deactivate_resource),
        DEVMETHOD(bus_setup_intr,               a10_ahci_setup_intr),
        DEVMETHOD(bus_teardown_intr,            a10_ahci_teardown_intr),

	DEVMETHOD_END
};

static driver_t a10_ahci_driver = {
	"a10_ahci",
	a10_ahci_methods,
	sizeof(struct a10_ahci_softc),
};

devclass_t a10_ahci_devclass;

DRIVER_MODULE(ahci, simplebus, a10_ahci_driver, a10_ahci_devclass, 0, 0);
MODULE_VERSION(ahci, 1);
MODULE_DEPEND(ahci, cam, 1, 1, 1);

