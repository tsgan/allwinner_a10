/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * Copyright (c) 2012 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * Copyright (c) 2012 Luiz Otavio O Souza.
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
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/arm/allwinner/a20_gpio.c 249449 2013-04-13 21:21:13Z dim $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/fdt.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio_if.h"

/*
 * A20 have 9 banks of gpio.
 * 32 pins per bank:
 * PA0 - PA17 | PB0 - PB23 | PC0 - PC24
 * PD0 - PD27 | PE0 - PE31 | PF0 - PF5
 * PG0 - PG9 | PH0 - PH27 | PI0 - PI12
 */

#define	A20_GPIO_PINS		288
#define	A20_GPIO_DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |	\
    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)

#define A20_GPIO_NONE		0
#define A20_GPIO_PULLUP		1
#define A20_GPIO_PULLDOWN	2

#define A20_GPIO_INPUT		0
#define A20_GPIO_OUTPUT		1

struct a20_gpio_softc {
	device_t		sc_dev;
	struct mtx		sc_mtx;
	struct resource *	sc_mem_res;
	struct resource *	sc_irq_res;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	void *			sc_intrhand;
	int			sc_gpio_npins;
	struct gpio_pin		sc_gpio_pins[A20_GPIO_PINS];
};

#define	A20_GPIO_LOCK(_sc)		mtx_lock(&_sc->sc_mtx)
#define	A20_GPIO_UNLOCK(_sc)		mtx_unlock(&_sc->sc_mtx)
#define	A20_GPIO_LOCK_ASSERT(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED)

#define	A20_GPIO_GP_CFG(_bank, _pin)	0x00 + ((_bank) * 0x24) + ((_pin)<<2)
#define	A20_GPIO_GP_DAT(_bank)		0x10 + ((_bank) * 0x24)
#define	A20_GPIO_GP_DRV(_bank, _pin)	0x14 + ((_bank) * 0x24) + ((_pin)<<2)
#define	A20_GPIO_GP_PUL(_bank, _pin)	0x1c + ((_bank) * 0x24) + ((_pin)<<2)

#define	A20_GPIO_GP_INT_CFG0		0x200
#define	A20_GPIO_GP_INT_CFG1		0x204
#define	A20_GPIO_GP_INT_CFG2		0x208
#define	A20_GPIO_GP_INT_CFG3		0x20c

#define	A20_GPIO_GP_INT_CTL		0x210
#define	A20_GPIO_GP_INT_STA		0x214
#define	A20_GPIO_GP_INT_DEB		0x218

#define	A20_GPIO_WRITE(_sc, _off, _val)		\
    bus_space_write_4(_sc->sc_bst, _sc->sc_bsh, _off, _val)
#define	A20_GPIO_READ(_sc, _off)		\
    bus_space_read_4(_sc->sc_bst, _sc->sc_bsh, _off)

static uint32_t
a20_gpio_get_function(struct a20_gpio_softc *sc, uint32_t pin)
{
	uint32_t bank, func, offset;

	bank = pin / 32;
	pin = pin - 32 * bank;
	func = pin >> 3;
	offset = ((pin & 0x07) << 2);

	A20_GPIO_LOCK(sc);
	func = (A20_GPIO_READ(sc, A20_GPIO_GP_CFG(bank, func)) >> offset) & 7;
	A20_GPIO_UNLOCK(sc);

	return (func);
}

static uint32_t
a20_gpio_func_flag(uint32_t nfunc)
{

	switch (nfunc) {
	case A20_GPIO_INPUT:
		return (GPIO_PIN_INPUT);
	case A20_GPIO_OUTPUT:
		return (GPIO_PIN_OUTPUT);
	}
	return (0);
}

static void
a20_gpio_set_function(struct a20_gpio_softc *sc, uint32_t pin, uint32_t f)
{
	uint32_t bank, func, data, offset;

	/* Must be called with lock held. */
	A20_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin - 32 * bank;
	func = pin >> 3;
	offset = ((pin & 0x07) << 2);

	data = A20_GPIO_READ(sc, A20_GPIO_GP_CFG(bank, func));
	data &= ~(7 << offset);
	data |= (f << offset);
	A20_GPIO_WRITE(sc, A20_GPIO_GP_CFG(bank, func), data);
}

static void
a20_gpio_set_pud(struct a20_gpio_softc *sc, uint32_t pin, uint32_t state)
{
	uint32_t bank, offset, pull, val;

	/* Must be called with lock held. */
	A20_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin - 32 * bank;
	pull = pin >> 4;
	offset = ((pin & 0x0f) << 1);

	val = A20_GPIO_READ(sc, A20_GPIO_GP_PUL(bank, pull));
	val &= ~(0x03 << offset);
	val |= (state << offset);
	A20_GPIO_WRITE(sc, A20_GPIO_GP_PUL(bank, pull), val);
}

static void
a20_gpio_pin_configure(struct a20_gpio_softc *sc, struct gpio_pin *pin,
    unsigned int flags)
{

	A20_GPIO_LOCK(sc);

	/*
	 * Manage input/output.
	 */
	if (flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) {
		pin->gp_flags &= ~(GPIO_PIN_INPUT|GPIO_PIN_OUTPUT);
		if (flags & GPIO_PIN_OUTPUT) {
			pin->gp_flags |= GPIO_PIN_OUTPUT;
			a20_gpio_set_function(sc, pin->gp_pin,
			    A20_GPIO_OUTPUT);
		} else {
			pin->gp_flags |= GPIO_PIN_INPUT;
			a20_gpio_set_function(sc, pin->gp_pin,
			    A20_GPIO_INPUT);
		}
	}

	/* Manage Pull-up/pull-down. */
	pin->gp_flags &= ~(GPIO_PIN_PULLUP|GPIO_PIN_PULLDOWN);
	if (flags & (GPIO_PIN_PULLUP|GPIO_PIN_PULLDOWN)) {
		if (flags & GPIO_PIN_PULLUP) {
			pin->gp_flags |= GPIO_PIN_PULLUP;
			a20_gpio_set_pud(sc, pin->gp_pin, A20_GPIO_PULLUP);
		} else {
			pin->gp_flags |= GPIO_PIN_PULLDOWN;
			a20_gpio_set_pud(sc, pin->gp_pin, A20_GPIO_PULLDOWN);
		}
	} else 
		a20_gpio_set_pud(sc, pin->gp_pin, A20_GPIO_NONE);

	A20_GPIO_UNLOCK(sc);
}

static int
a20_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = A20_GPIO_PINS - 1;
	return (0);
}

static int
a20_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	A20_GPIO_LOCK(sc);
	*caps = sc->sc_gpio_pins[i].gp_caps;
	A20_GPIO_UNLOCK(sc);

	return (0);
}

static int
a20_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	A20_GPIO_LOCK(sc);
	*flags = sc->sc_gpio_pins[i].gp_flags;
	A20_GPIO_UNLOCK(sc);

	return (0);
}

static int
a20_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	A20_GPIO_LOCK(sc);
	memcpy(name, sc->sc_gpio_pins[i].gp_name, GPIOMAXNAME);
	A20_GPIO_UNLOCK(sc);

	return (0);
}

static int
a20_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	/* Check for unwanted flags. */
	if ((flags & sc->sc_gpio_pins[i].gp_caps) != flags)
		return (EINVAL);

	/* Can't mix input/output together. */
	if ((flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) ==
	    (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT))
		return (EINVAL);

	/* Can't mix pull-up/pull-down together. */
	if ((flags & (GPIO_PIN_PULLUP|GPIO_PIN_PULLDOWN)) ==
	    (GPIO_PIN_PULLUP|GPIO_PIN_PULLDOWN))
		return (EINVAL);

	a20_gpio_pin_configure(sc, &sc->sc_gpio_pins[i], flags);

	return (0);
}

static int
a20_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	uint32_t bank, offset, data;
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	bank = pin / 32;
	pin = pin - 32 * bank;
	offset = pin & 0x1f;

	A20_GPIO_LOCK(sc);
	data = A20_GPIO_READ(sc, A20_GPIO_GP_DAT(bank));
	if (value)
		data |= (1 << offset);
	else
		data &= ~(1 << offset);
	A20_GPIO_WRITE(sc, A20_GPIO_GP_DAT(bank), data);
	A20_GPIO_UNLOCK(sc);

	return (0);
}

static int
a20_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	uint32_t bank, offset, reg_data;
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	bank = pin / 32;
	pin = pin - 32 * bank;
	offset = pin & 0x1f;

	A20_GPIO_LOCK(sc);
	reg_data = A20_GPIO_READ(sc, A20_GPIO_GP_DAT(bank));
	A20_GPIO_UNLOCK(sc);
	*val = (reg_data & (1 << offset)) ? 1 : 0;

	return (0);
}

static int
a20_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	uint32_t bank, data, offset;
	int i;

	for (i = 0; i < sc->sc_gpio_npins; i++) {
		if (sc->sc_gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->sc_gpio_npins)
		return (EINVAL);

	bank = pin / 32;
	pin = pin - 32 * bank;
	offset = pin & 0x1f;

	A20_GPIO_LOCK(sc);
	data = A20_GPIO_READ(sc, A20_GPIO_GP_DAT(bank));
	if (data & (1 << offset))
		data &= ~(1 << offset);
	else
		data |= (1 << offset);
	A20_GPIO_WRITE(sc, A20_GPIO_GP_DAT(bank), data);
	A20_GPIO_UNLOCK(sc);

	return (0);
}

static int
a20_gpio_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "allwinner,sun7i-gpio"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
a20_gpio_attach(device_t dev)
{
	struct a20_gpio_softc *sc = device_get_softc(dev);
	uint32_t func;
	int i, rid;
	phandle_t gpio;

	sc->sc_dev = dev;

	mtx_init(&sc->sc_mtx, "a20 gpio", "gpio", MTX_DEF);

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->sc_bst = rman_get_bustag(sc->sc_mem_res);
	sc->sc_bsh = rman_get_bushandle(sc->sc_mem_res);

	rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->sc_irq_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

	/* Find our node. */
	gpio = ofw_bus_get_node(sc->sc_dev);

	if (!OF_hasprop(gpio, "gpio-controller"))
		/* Node is not a GPIO controller. */
		goto fail;

	/* Initialize the software controlled pins. */
	for (i = 0; i < A20_GPIO_PINS; i++) {
		snprintf(sc->sc_gpio_pins[i].gp_name, GPIOMAXNAME,
		    "pin %d", i);
		func = a20_gpio_get_function(sc, i);
		sc->sc_gpio_pins[i].gp_pin = i;
		sc->sc_gpio_pins[i].gp_caps = A20_GPIO_DEFAULT_CAPS;
		sc->sc_gpio_pins[i].gp_flags = a20_gpio_func_flag(func);
	}
	sc->sc_gpio_npins = i;

	device_add_child(dev, "gpioc", device_get_unit(dev));
	device_add_child(dev, "gpiobus", device_get_unit(dev));
	return (bus_generic_attach(dev));

fail:
	if (sc->sc_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_irq_res);
	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	return (ENXIO);
}

static int
a20_gpio_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t a20_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a20_gpio_probe),
	DEVMETHOD(device_attach,	a20_gpio_attach),
	DEVMETHOD(device_detach,	a20_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_pin_max,		a20_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	a20_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	a20_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	a20_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	a20_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		a20_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		a20_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	a20_gpio_pin_toggle),

	DEVMETHOD_END
};

static devclass_t a20_gpio_devclass;

static driver_t a20_gpio_driver = {
	"gpio",
	a20_gpio_methods,
	sizeof(struct a20_gpio_softc),
};

DRIVER_MODULE(a20_gpio, simplebus, a20_gpio_driver, a20_gpio_devclass, 0, 0);
