/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio.h"

#define GPIO_READL(reg)			(*(volatile uint32_t *)(reg))
#define GPIO_WRITEL(val,reg)		(*(volatile uint32_t *)(reg) = (val))

int a10_gpio_set_cfgpin(uint32_t pin, uint32_t val) {

	uint32_t cfg;
	uint32_t bank = GPIO_BANK(pin);
	uint32_t index = GPIO_CFG_INDEX(pin);
	uint32_t offset = GPIO_CFG_OFFSET(pin);

	struct a10_gpio *pio =
		&((struct a10_gpio_reg *)A10_PIO_BASE)->gpio_bank[bank];

	cfg = GPIO_READL(&pio->cfg[0] + index);
	cfg &= ~(0xf << offset);
	cfg |= val << offset;

	GPIO_WRITEL(cfg, &pio->cfg[0] + index);

	return 0;
}

int a10_gpio_get_cfgpin(uint32_t pin) {

	uint32_t cfg;
	uint32_t bank = GPIO_BANK(pin);
	uint32_t index = GPIO_CFG_INDEX(pin);
	uint32_t offset = GPIO_CFG_OFFSET(pin);

	struct a10_gpio *pio =
		&((struct a10_gpio_reg *)A10_PIO_BASE)->gpio_bank[bank];

	cfg = GPIO_READL(&pio->cfg[0] + index);
	cfg >>= offset;

	return (cfg & 0xf);
}

int a10_gpio_output(uint32_t pin, uint32_t val) {

	uint32_t dat;
	uint32_t bank = GPIO_BANK(pin);
	uint32_t num = GPIO_NUM(pin);

	struct a10_gpio *pio =
		&((struct a10_gpio_reg *)A10_PIO_BASE)->gpio_bank[bank];

	dat = GPIO_READL(&pio->dat);
	if(val)
		dat |= 1 << num;
	else
		dat &= ~(1 << num);

	GPIO_WRITEL(dat, &pio->dat);

	return 0;
}

int a10_gpio_input(uint32_t pin) {

	uint32_t dat;
	uint32_t bank = GPIO_BANK(pin);
	uint32_t num = GPIO_NUM(pin);

	struct a10_gpio *pio =
		&((struct a10_gpio_reg *)A10_PIO_BASE)->gpio_bank[bank];

	dat = GPIO_READL(&pio->dat);
	dat >>= num;

	return (dat & 0x1);
}

int a10_gpio_request(unsigned gpio, const char *label) {

	return 0;
}

int a10_gpio_free(unsigned gpio) {

	return 0;
}

int a10_gpio_direction_input(unsigned gpio) {

	a10_gpio_set_cfgpin(gpio, A10_GPIO_INPUT);
	return a10_gpio_input(gpio);
}

int a10_gpio_direction_output(unsigned gpio, int value) {

	a10_gpio_set_cfgpin(gpio, A10_GPIO_OUTPUT);
	return a10_gpio_output(gpio, value);
}

int a10_gpio_get_value(unsigned gpio) {

	return a10_gpio_input(gpio);
}

int a10_gpio_set_value(unsigned gpio, int value) {

	return a10_gpio_output(gpio, value);
}

