/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
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
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/* A10/A20 EMAC driver */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_mib.h>
#include <net/ethernet.h>
#include <net/if_vlan_var.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#endif

#include <net/bpf.h>
#include <net/bpfdesc.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/allwinner/if_emacreg.h>

#include "miibus_if.h"

#include "gpio_if.h"

#include "a10_clk.h"
#include "a10_sramc.h"
#include "a10_gpio.h"

struct emac_softc {
	struct ifnet		*emac_ifp;
	device_t		emac_dev;
	device_t		emac_miibus;
	bus_space_handle_t	emac_handle;
	bus_space_tag_t		emac_tag;
	struct resource		*emac_res;
	struct resource		*emac_irq;
	void			*emac_intrhand;
#define EMAC_FLAG_LINK		(1 << 0)
	uint32_t		emac_flags;
	struct mtx		emac_mtx;
	struct callout		emac_tick_ch;
	int			emac_watchdog_timer;
	int			emac_rx_completed_flag;
};

static int emac_probe(device_t);
static int emac_attach(device_t);
static int emac_detach(device_t);

static void emac_sys_setup();
static void emac_reset(struct emac_softc *sc);
static void emac_powerup(struct emac_softc *sc);
static void emac_powerdown(struct emac_softc *sc);

static void emac_init_locked(struct emac_softc *);
static void emac_start_locked(struct ifnet *ifp);
static void emac_init(void *xcs);
static void emac_stop(struct emac_softc *sc);
static void emac_intr(void *arg);
static int emac_ioctl(struct ifnet *ifp, u_long command, caddr_t data);

static void emac_rxeof(struct emac_softc *sc);
static void emac_txeof(struct emac_softc *sc);

static int emac_wait_link(device_t dev);
static int emac_miibus_readreg(device_t dev, int phy, int reg);
static int emac_miibus_writereg(device_t dev, int phy, int reg, int data);
static void emac_miibus_statchg(device_t);

static int emac_ifmedia_upd(struct ifnet *ifp);
static void emac_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr);

#define	emac_read_reg(sc, reg)		\
    bus_space_read_4(sc->emac_tag, sc->emac_handle, reg)
#define	emac_write_reg(sc, reg, val)	\
    bus_space_write_4(sc->emac_tag, sc->emac_handle, reg, val)

static uint8_t eaddr[ETHER_ADDR_LEN];

static void
emac_sys_setup()
{
	int i;

	a10_clk_emac_activate();
	/*
	 * Configure pin mux settings for MII.
	 * Pins PA0 from PA17.
	 */
	for (i = 0; i <= 17; i++)
		a10_emac_gpio_config(i);
	/* Map sram */
	a10_map_to_emac();
}

static void
emac_powerup(struct emac_softc *sc)
{
	device_t dev;
	uint32_t reg_val, rnd;
	int phy_val;

	dev = sc->emac_dev;

	/* Flush RX FIFO */
	reg_val = emac_read_reg(sc, EMAC_RX_CTL);
	reg_val |= EMAC_RX_FLUSH_FIFO;
	emac_write_reg(sc, EMAC_RX_CTL, reg_val);
	DELAY(1);

	/* Soft reset MAC */
	reg_val = emac_read_reg(sc, EMAC_MAC_CTL0);
	reg_val &= (~EMAC_MAC_CTL0_SOFT_RST);
	emac_write_reg(sc, EMAC_MAC_CTL0, reg_val);

	/* Set MII clock */
	reg_val = emac_read_reg(sc, EMAC_MAC_MCFG);
	reg_val &= (~(0xf << 2));
	reg_val |= (0xd << 2);
	emac_write_reg(sc, EMAC_MAC_MCFG, reg_val);

	/* Clear RX counter */
	emac_write_reg(sc, EMAC_RX_FBC, 0);

	/* Disable all interrupt and clear interrupt status */
	emac_write_reg(sc, EMAC_INT_CTL, 0);
	reg_val = emac_read_reg(sc, EMAC_INT_STA);
	emac_write_reg(sc, EMAC_INT_STA, reg_val);
	DELAY(1);

	/* Set up TX */
	reg_val = emac_read_reg(sc, EMAC_TX_MODE);
	reg_val |= EMAC_TX_AB_M;
	reg_val &= EMAC_TX_TM;
	emac_write_reg(sc, EMAC_TX_MODE, reg_val);

	/* Set up RX */
	reg_val = emac_read_reg(sc, EMAC_RX_CTL);
	reg_val |= EMAC_RX_SETUP;
	reg_val &= EMAC_RX_TM;
	emac_write_reg(sc, EMAC_RX_CTL, reg_val);

	/* Set up MAC CTL0. */
	reg_val = emac_read_reg(sc, EMAC_MAC_CTL0);
	reg_val |= EMAC_MAC_CTL0_SETUP;
	emac_write_reg(sc, EMAC_MAC_CTL0, reg_val);

	/* Set up MAC CTL1. */
	reg_val = emac_read_reg(sc, EMAC_MAC_CTL1);
	DELAY(10);
	phy_val = emac_miibus_readreg(dev, 0, 0);
	if (phy_val & EMAC_PHY_DUPLEX)
		reg_val |= EMAC_MAC_CTL1_DUP;
	else
		reg_val &= ~EMAC_MAC_CTL1_DUP;
	reg_val |= EMAC_MAC_CTL1_SETUP;
	emac_write_reg(sc, EMAC_MAC_CTL1, reg_val);

	/* Set up IPGT */
	emac_write_reg(sc, EMAC_MAC_IPGT, EMAC_MAC_IPGT_FD);

	/* Set up IPGR */
	emac_write_reg(sc, EMAC_MAC_IPGR, EMAC_MAC_NBTB_IPG2 | 
	    (EMAC_MAC_NBTB_IPG1 << 8));

	/* Set up Collison window */
	emac_write_reg(sc, EMAC_MAC_CLRT, EMAC_MAC_RM | (EMAC_MAC_CW << 8));

	/* Set up Max Frame Length */
	emac_write_reg(sc, EMAC_MAC_MAXF, EMAC_MAC_MFL);

	/* XXX: Hardcode the ethernet address for now */
	rnd = arc4random() & 0x00ffffff;
	eaddr[0] = 'b';
	eaddr[1] = 's';
	eaddr[2] = 'd';
	eaddr[3] = rnd >> 16;
	eaddr[4] = rnd >>  8;
	eaddr[5] = rnd >>  0;

	/* Write ethernet address to register */
	emac_write_reg(sc, EMAC_MAC_A1, eaddr[0] << 16 | 
	    eaddr[1] << 8 | eaddr[2]);
	emac_write_reg(sc, EMAC_MAC_A0, eaddr[3] << 16 | 
	    eaddr[4] << 8 | eaddr[5]);
}

static void
emac_powerdown(struct emac_softc *sc)
{
	device_t dev;
	int reg_val;

	dev = sc->emac_dev;

	/* Reset phy */
	reg_val = emac_miibus_readreg(dev, 0, 0);
	emac_miibus_writereg(dev, 0, 0, reg_val & EMAC_PHY_RESET);
	DELAY(10);
	/* Power down phy */
	emac_miibus_writereg(dev, 0, 0, reg_val | EMAC_PHY_PWRDOWN);

	/* Disable all interrupt and clear interrupt status */
	emac_write_reg(sc, EMAC_INT_CTL, 0);
	reg_val = emac_read_reg(sc, EMAC_INT_STA);
	emac_write_reg(sc, EMAC_INT_STA, reg_val);

	/* Disable RX/TX */
	reg_val = emac_read_reg(sc, EMAC_CTL);
	reg_val &= ~(EMAC_CTL_RST | EMAC_CTL_TX_EN | EMAC_CTL_RX_EN);
	emac_write_reg(sc, EMAC_CTL, reg_val);
}

static void
emac_reset(struct emac_softc *sc)
{

	emac_write_reg(sc, EMAC_CTL, 0);
	DELAY(200);
	emac_write_reg(sc, EMAC_CTL, 1);
	DELAY(200);
}

static void
emac_txeof(struct emac_softc *sc)
{
	struct ifnet *ifp;

	ifp = sc->emac_ifp;
	ifp->if_opackets++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

        /* Unarm watchdog timer if no TX */
	sc->emac_watchdog_timer = 0;
}

static void
fix_mbuf(struct mbuf *m, int sramc_align)
{
	uint16_t *src, *dst;
	int i;

	src = mtod(m, uint16_t *);
	dst = src - (sramc_align - ETHER_ALIGN) / sizeof(*src);
	for (i = 0; i < (m->m_len / sizeof(uint16_t) + 1); i++)
		*dst++ = *src++;
	m->m_data -= sramc_align - ETHER_ALIGN;
}

static void
emac_rxeof(struct emac_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	uint32_t reg_val, rxcount;
	int16_t len;
	uint16_t status;
	int good_packet;

	ifp = sc->emac_ifp;
	for (;;) {
		/*
		 * Race warning: The first packet might arrive with
		 * the interrupts disabled, but the second will fix
		 */
		rxcount = emac_read_reg(sc, EMAC_RX_FBC);
		if (!rxcount) {
			/* Had one stuck? */
			rxcount = emac_read_reg(sc, EMAC_RX_FBC);
			if (!rxcount) {
				sc->emac_rx_completed_flag = 1;
				return;
			}
		}
		/* Check packet header */
		reg_val = emac_read_reg(sc, EMAC_RX_IO_DATA);
		if (reg_val != EMAC_PACKET_HEADER) {
			/* Packet header is wrong */
			/* Disable RX */
			reg_val = emac_read_reg(sc, EMAC_CTL);
			reg_val &= ~EMAC_CTL_RX_EN;
			emac_write_reg(sc, EMAC_CTL, reg_val);

			/* Flush RX FIFO */
			reg_val = emac_read_reg(sc, EMAC_RX_CTL);
			reg_val |= EMAC_RX_FLUSH_FIFO;
			emac_write_reg(sc, EMAC_RX_CTL, reg_val);
			while (emac_read_reg(sc, EMAC_RX_CTL) &
			    EMAC_RX_FLUSH_FIFO)
				;
			/* Enable RX */
			reg_val = emac_read_reg(sc, EMAC_CTL);
			reg_val |= EMAC_CTL_RX_EN;
			emac_write_reg(sc, EMAC_CTL, reg_val);

			sc->emac_rx_completed_flag = 1;
			return;
		}

		good_packet = 1;

		/* Get packet size and status */
		reg_val = emac_read_reg(sc, EMAC_RX_IO_DATA);
		len = reg_val & 0xffff;
		status = (reg_val >> 16) & 0xffff;

		if (len < 64) {
			good_packet = 0;
			if_printf(ifp, "bad packet: len = %i status = %i\n",
			    len, status);
			ifp->if_oerrors++;
		}
#if 0
		if (status & (EMAC_CRCERR | EMAC_LENERR)) {
			good_packet = 0;
			ifp->if_oerrors++;
			if (status & EMAC_CRCERR)
				if_printf(ifp, "crc error\n");
			if (status & EMAC_LENERR)
				if_printf(ifp, "length error\n");
		}
#endif
		if (good_packet) {
			m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
			if (m == NULL) {
				sc->emac_rx_completed_flag = 1;
				return;
			}
			m->m_len = m->m_pkthdr.len = MCLBYTES;
			/*
			 * sram->emac mapping needs 4 bytes alignment
			 * so reserving 4 bytes on the buffer head.
			 */
			m_adj(m, 4);
			len -= ETHER_CRC_LEN;
			bus_space_read_multi_4(sc->emac_tag, sc->emac_handle,
			    EMAC_RX_IO_DATA, mtod(m, uint32_t *),
			    roundup2(len, 4) / 4);

			m->m_pkthdr.rcvif = ifp;
			m->m_len = m->m_pkthdr.len = len;
			/* align IP header */
			fix_mbuf(m, 4);

			ifp->if_ipackets++;
			EMAC_UNLOCK(sc);
			(*ifp->if_input)(ifp, m);
			EMAC_LOCK(sc);
		}
		sc->emac_rx_completed_flag = 1;
	}
}

static void
emac_watchdog(struct emac_softc *sc)
{
	struct ifnet *ifp;

	EMAC_ASSERT_LOCKED(sc);

	if (sc->emac_watchdog_timer == 0 || --sc->emac_watchdog_timer)
		return;

	ifp = sc->emac_ifp;
	if_printf(sc->emac_ifp, "watchdog timeout -- resetting\n");
	ifp->if_oerrors++;
	ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	emac_init_locked(sc);
	if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
		emac_start_locked(ifp);
}

static void
emac_tick(void *arg)
{
	struct emac_softc *sc;
	struct mii_data *mii;

	sc = (struct emac_softc *)arg;
	mii = device_get_softc(sc->emac_miibus);
	mii_tick(mii);
	if((sc->emac_flags & EMAC_FLAG_LINK) == 0)
		emac_miibus_statchg(sc->emac_dev);

	emac_watchdog(sc);
	callout_reset(&sc->emac_tick_ch, hz, emac_tick, sc);
}

static void
emac_init(void *xcs)
{
	struct emac_softc *sc = xcs;

	EMAC_LOCK(sc);
	emac_init_locked(sc);
	EMAC_UNLOCK(sc);
}

static void
emac_init_locked(struct emac_softc *sc)
{
	struct ifnet *ifp = sc->emac_ifp;
	uint32_t reg_val;
	int phy_reg;
	device_t dev;
	int wait_limit = 4500; /* wait up to 4.5 sec for a link */

	dev = sc->emac_dev;
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
		return;

	/* Power up phy */
	phy_reg = emac_miibus_readreg(dev, 0, 0);
	emac_miibus_writereg(dev, 0, 0, phy_reg & ~EMAC_PHY_PWRDOWN);

	/*
	 * Phy needs some time to boot.
	 * This is needed to avoid situations like
	 * having 10Mbit half-duplex link on 100Mbit network.
	 */
	while (emac_wait_link(dev) == 0 && wait_limit > 0) {
		DELAY(500);
		wait_limit -= 500;
	}

	phy_reg = emac_miibus_readreg(dev, 0, 0);

	/* Set EMAC SPEED, depends on phy */
	reg_val = emac_read_reg(sc, EMAC_MAC_SUPP);
	reg_val &= (~(1 << 8));
	reg_val |= (((phy_reg & (1 << 13)) >> 13) << 8);
	emac_write_reg(sc, EMAC_MAC_SUPP, reg_val);

	/* Enable duplex depends on phy */
	reg_val = emac_read_reg(sc, EMAC_MAC_CTL1);
	reg_val &= ~EMAC_MAC_CTL1_DUP;
	reg_val |= (((phy_reg & (1 << 8)) >> 8) << 0);
	emac_write_reg(sc, EMAC_MAC_CTL1, reg_val);

	/* Enable RX/TX */
	reg_val = emac_read_reg(sc, EMAC_CTL);
	emac_write_reg(sc, EMAC_CTL, reg_val | EMAC_CTL_RST | 
	    EMAC_CTL_TX_EN | EMAC_CTL_RX_EN);

	/* Enable RX/TX0/RX Hlevel interrupt */
	reg_val = emac_read_reg(sc, EMAC_INT_CTL);
	reg_val |= (0xf << 0) | (1 << 8);
	emac_write_reg(sc, EMAC_INT_CTL, reg_val);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	sc->emac_rx_completed_flag = 1;

	callout_reset(&sc->emac_tick_ch, hz, emac_tick, sc);
}


static void
emac_start(struct ifnet *ifp)
{
	struct emac_softc *sc;
	sc = ifp->if_softc;

	EMAC_LOCK(sc);
	emac_start_locked(ifp);
	EMAC_UNLOCK(sc);
}

static void
emac_start_locked(struct ifnet *ifp)
{
	struct emac_softc *sc;
	struct mbuf *m, *m0;
	uint32_t reg_val;

	sc = ifp->if_softc;
	if (ifp->if_drv_flags & IFF_DRV_OACTIVE)
		return;
	if (IFQ_IS_EMPTY(&ifp->if_snd))
		return;

	/* Select channel */
	emac_write_reg(sc, EMAC_TX_INS, 0);

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);

		if (m == NULL)
			break;

		/* Address needs to be 4 bytes aligned */
		m0 = m_defrag(m, M_NOWAIT);
		if (m0 == NULL) {
			if_printf(ifp, "FAILED m_defrag()\n");
			m_freem(m);
			return;
		}
		m = m0;

		/* Write data */
		bus_space_write_multi_4(sc->emac_tag, sc->emac_handle,
		    EMAC_TX_IO_DATA, mtod(m, uint32_t *),
		    roundup2(m->m_len, 4) / 4);

		/* Send the data lengh. */
		emac_write_reg(sc, EMAC_TX_PL0, m->m_len);

		/* Start translate from fifo to phy. */
		reg_val = emac_read_reg(sc, EMAC_TX_CTL0);
		reg_val |= 1;
		emac_write_reg(sc, EMAC_TX_CTL0, reg_val);

		/* Set timeout */
		sc->emac_watchdog_timer = 5;

		ifp->if_drv_flags |= IFF_DRV_OACTIVE;
		BPF_MTAP(ifp, m);
		m_freem(m);
	}
}

static void
emac_stop(struct emac_softc *sc)
{

	EMAC_ASSERT_LOCKED(sc);
	callout_stop(&sc->emac_tick_ch);
}

static void
emac_intr(void *arg)
{
	struct emac_softc *sc = (struct emac_softc *)arg;
	struct ifnet *ifp;
	uint32_t int_status, reg_val;

	ifp = sc->emac_ifp;

	EMAC_LOCK(sc);

	/* Disable all interrupts */
	emac_write_reg(sc, EMAC_INT_CTL, 0);
	/* Get EMAC interrupt status */
	int_status = emac_read_reg(sc, EMAC_INT_STA);
	/* Clear ISR status */
	emac_write_reg(sc, EMAC_INT_STA, int_status); 

	/* Received incoming packet */
	if ((int_status & 0x100) && (sc->emac_rx_completed_flag == 1)) {
		sc->emac_rx_completed_flag = 0;
		emac_rxeof(sc);
	}

	/* Transmit Interrupt check */
	if (int_status & (0x01 | 0x02)){
		emac_txeof(sc);
		if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
			emac_start_locked(ifp);
	}

	/* Re-enable interrupt mask */
	if (sc->emac_rx_completed_flag == 1) {
		reg_val = emac_read_reg(sc, EMAC_INT_CTL);
		reg_val |= (0xf << 0) | (1 << 8);
		emac_write_reg(sc, EMAC_INT_CTL, reg_val);
	}

	EMAC_UNLOCK(sc);
}

static int
emac_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int error = 0;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		/*
		 * Switch interface state between "running" and
		 * "stopped", reflecting the UP flag.
		 */
		EMAC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
				emac_init_locked(sc);
		} else {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
				emac_stop(sc);
		}
		EMAC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = EINVAL;
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		mii = device_get_softc(sc->emac_miibus);
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}
	return (error);
}

static int
emac_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "allwinner,sun4i-emac"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner A10/A20 EMAC");
	return (0);
}

static int
emac_detach(device_t dev)
{
	struct emac_softc *sc;

	sc = device_get_softc(dev);

	emac_powerdown(sc);
	sc->emac_ifp->if_drv_flags &= ~IFF_DRV_RUNNING;

	if (sc->emac_ifp != NULL)
		if_free(sc->emac_ifp);

	callout_drain(&sc->emac_tick_ch);

	if (sc->emac_intrhand != NULL)
		bus_teardown_intr(sc->emac_dev, sc->emac_irq, 
		    sc->emac_intrhand);

	if (sc->emac_miibus != NULL) {
		device_delete_child(sc->emac_dev, sc->emac_miibus);
		bus_generic_detach(sc->emac_dev);
	}

	if (sc->emac_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->emac_res);

	if (sc->emac_irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->emac_irq);

	if (mtx_initialized(&sc->emac_mtx))
		mtx_destroy(&sc->emac_mtx);

	return (0);
}

static int
emac_attach(device_t dev)
{
	struct emac_softc *sc;
	struct ifnet *ifp;
	int error, rid;

	sc = device_get_softc(dev);
	sc->emac_dev = dev;

	error = 0;
	mtx_init(&sc->emac_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->emac_tick_ch, &sc->emac_mtx, 0);

	rid = 0;
	sc->emac_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (sc->emac_res == NULL) {
		device_printf(dev, "unable to map memory\n");
		error = ENXIO;
		goto fail;
	}

	sc->emac_tag = rman_get_bustag(sc->emac_res);
	sc->emac_handle = rman_get_bushandle(sc->emac_res);

	rid = 0;
	sc->emac_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->emac_irq == NULL) {
		device_printf(dev, "cannot allocate IRQ resources.\n");
		error = ENXIO;
		goto fail;
	}

	/* Setup EMAC */
	emac_sys_setup();
	emac_powerup(sc);
	emac_reset(sc);

	ifp = sc->emac_ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "unable to allocate ifp\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = sc;

	/* Setup MII */
	error = mii_attach(dev, &sc->emac_miibus, ifp, emac_ifmedia_upd, 
	    emac_ifmedia_sts, BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error != 0) {
		device_printf(dev, "PHY probe failed\n");
		goto fail;
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = emac_start;
	ifp->if_ioctl = emac_ioctl;
	ifp->if_init = emac_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);

	ether_ifattach(ifp, eaddr);

	/* VLAN capability setup. */
	ifp->if_capabilities |= IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;
	/* Tell the upper layer we support VLAN over-sized frames. */
	ifp->if_hdrlen = sizeof(struct ether_vlan_header);

	error = bus_setup_intr(dev, sc->emac_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, emac_intr, sc, &sc->emac_intrhand);
	if (error != 0) {
		device_printf(dev, "could not set up interrupt handler.\n");
		ether_ifdetach(ifp);
		goto fail;
	}

fail:
	if (error != 0)
		emac_detach(dev);
	return (error);
}


/*
 * The MII bus interface
 */
static int
emac_miibus_readreg(device_t dev, int phy, int reg)
{
	struct emac_softc *sc;
	int rval;

	sc = device_get_softc(dev);

	/* Issue phy address and reg */
	emac_write_reg(sc, EMAC_MAC_MADR, (phy << 8) | reg);
	/* Pull up the phy io line */
	emac_write_reg(sc, EMAC_MAC_MCMD, 0x1);
	/* Wait read complete */
	while(emac_read_reg(sc, EMAC_MAC_MIND) & 0x1)
		;
	/* Push down the phy io line */
	emac_write_reg(sc, EMAC_MAC_MCMD, 0x0);
	/* Read data */
	rval = emac_read_reg(sc, EMAC_MAC_MRDD);

	return (rval);
}

static int
emac_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct emac_softc *sc;

	sc = device_get_softc(dev);

	/* Issue phy address and reg */
	emac_write_reg(sc, EMAC_MAC_MADR, (phy << 8) | reg);
	/* Write data */
	emac_write_reg(sc, EMAC_MAC_MWTD, data);
	/* Pull up the phy io line */
	emac_write_reg(sc, EMAC_MAC_MCMD, 0x1);
	/* Wait read complete */
	while(emac_read_reg(sc, EMAC_MAC_MIND) & 0x1)
		;
	/* Push down the phy io line */
	emac_write_reg(sc, EMAC_MAC_MCMD, 0x0);

	return (0);
}

static void
emac_miibus_statchg(device_t dev)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct ifnet *ifp;

	sc = device_get_softc(dev);

	mii = device_get_softc(sc->emac_miibus);
	ifp = sc->emac_ifp;
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;

	if ((mii->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) ==
	    (IFM_ACTIVE | IFM_AVALID))
		sc->emac_flags |= EMAC_FLAG_LINK;
	else
		sc->emac_flags &= ~EMAC_FLAG_LINK;
}

static int
emac_ifmedia_upd(struct ifnet *ifp)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct mii_softc *miisc;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->emac_miibus);
	LIST_FOREACH(miisc, &mii->mii_phys, mii_list)
		PHY_RESET(miisc);

	EMAC_LOCK(sc);
	mii_mediachg(mii);
	EMAC_UNLOCK(sc);

	return (0);
}

static void
emac_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct emac_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->emac_miibus);

	EMAC_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	EMAC_UNLOCK(sc);
}

static int
emac_wait_link(device_t dev)
{
	int rval;

	rval = emac_miibus_readreg(dev, 0, 1);
	if (rval & 0x4) {
		/* phy is linked */
		return (1);
	} else {
		/* phy link is waiting */
		return (0);
	}
}

static device_method_t emac_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		emac_probe),
	DEVMETHOD(device_attach,	emac_attach),
	DEVMETHOD(device_detach,	emac_detach),

	/* bus interface, for miibus */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	emac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	emac_miibus_writereg),
	DEVMETHOD(miibus_statchg,	emac_miibus_statchg),

	DEVMETHOD_END
};

static driver_t emac_driver = {
	"emac",
	emac_methods,
	sizeof(struct emac_softc)
};

static devclass_t emac_devclass;

DRIVER_MODULE(emac, simplebus, emac_driver, emac_devclass, 0, 0);
DRIVER_MODULE(miibus, emac, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(emac, miibus, 1, 1, 1);
MODULE_DEPEND(emac, ether, 1, 1, 1);

