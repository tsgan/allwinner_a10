/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * Copyright (c) 2013 Ian Lepore <ian@freebsd.org>
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
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#include <net/if_vlan_var.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/allwinner/if_wemacreg.h>
#include <arm/allwinner/if_wemacvar.h>

#include "miibus_if.h"

#include "gpio_if.h"

#include "a10_clk.h"
#include "a10_sramc.h"
#include "a10_gpio.h"

/*
 * Driver data and defines.
 */
#define	RX_DESC_COUNT	64
#define	RX_DESC_SIZE	(sizeof(struct wemac_hwdesc) * RX_DESC_COUNT)
#define	TX_DESC_COUNT	64
#define	TX_DESC_SIZE	(sizeof(struct wemac_hwdesc) * TX_DESC_COUNT)

#define	WATCHDOG_TIMEOUT_SECS	5

struct wemac_bufmap {
	struct mbuf	*mbuf;
	bus_dmamap_t	map;
};

struct wemac_softc {
	device_t		dev;
	device_t		miibus;
	struct mii_data *	mii_softc;
	struct ifnet		*ifp;
	int			if_flags;
	struct mtx		mtx;
	struct resource		*irq_res;
	struct resource		*mem_res;
	void *			intr_cookie;
	struct callout		wemac_callout;
	boolean_t		link_is_up;
	boolean_t		is_attached;
	boolean_t		is_detaching;
	int			tx_watchdog_count;

	bus_dma_tag_t		rxdesc_tag;
	bus_dmamap_t		rxdesc_map;
	struct wemac_hwdesc	*rxdesc_ring;
	bus_addr_t		rxdesc_ring_paddr;
	bus_dma_tag_t		rxbuf_tag;
	struct wemac_bufmap	rxbuf_map[RX_DESC_COUNT];
	uint32_t		rx_idx;
	int			rx_completed_flag;

	bus_dma_tag_t		txdesc_tag;
	bus_dmamap_t		txdesc_map;
	struct wemac_hwdesc	*txdesc_ring;
	bus_addr_t		txdesc_ring_paddr;
	bus_dma_tag_t		txbuf_tag;
	struct wemac_bufmap	txbuf_map[RX_DESC_COUNT];
	uint32_t		tx_idx_head;
	uint32_t		tx_idx_tail;
	int			txcount;
};

#define	WEMAC_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	WEMAC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	WEMAC_LOCK_INIT(sc)		mtx_init(&(sc)->mtx, \
	    device_get_nameunit((sc)->dev), MTX_NETWORK_LOCK, MTX_DEF)
#define	WEMAC_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx);
#define	WEMAC_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->mtx, MA_OWNED);
#define	WEMAC_ASSERT_UNLOCKED(sc)	mtx_assert(&(sc)->mtx, MA_NOTOWNED);

static void wemac_init_locked(struct wemac_softc *sc);
static void wemac_stop_locked(struct wemac_softc *sc);
static void wemac_txstart_locked(struct wemac_softc *sc);
static void wemac_txfinish_locked(struct wemac_softc *sc);

static void wemac_sys_setup();
static void wemac_reset(struct wemac_softc *sc);
static void wemac_powerup(struct wemac_softc *sc);

static uint8_t eaddr[ETHER_ADDR_LEN];

static void
wemac_sys_setup()
{

	a10_clk_emac_activate();
	a10_emac_gpio_config();
	a10_map_to_emac();
}

static inline uint16_t
RD2(struct wemac_softc *sc, bus_size_t off)
{

	return (bus_read_2(sc->mem_res, off));
}

static inline void
WR2(struct wemac_softc *sc, bus_size_t off, uint16_t val)
{

	bus_write_2(sc->mem_res, off, val);
}

static inline uint32_t
RD4(struct wemac_softc *sc, bus_size_t off)
{

	return (bus_read_4(sc->mem_res, off));
}

static inline void
WR4(struct wemac_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->mem_res, off, val);
}

static inline uint32_t
next_rxidx(struct wemac_softc *sc, uint32_t curidx)
{

	return ((curidx == RX_DESC_COUNT - 1) ? 0 : curidx + 1);
}

static inline uint32_t
next_txidx(struct wemac_softc *sc, uint32_t curidx)
{

	return ((curidx == TX_DESC_COUNT - 1) ? 0 : curidx + 1);
}

static void
wemac_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

static void
wemac_reset(struct wemac_softc *sc)
{

	printf("------- resetting wemac...\n");

	WR4(sc, EMAC_CTL, 0);
	DELAY(200);
	WR4(sc, EMAC_CTL, 1);
	DELAY(200);
}

static int
wemac_miibus_readreg(device_t dev, int phy, int reg)
{
	struct wemac_softc *sc;
	int rval;

	//printf("-- readreg %i\n", phy);
	//if (phy != 1)
	//      return (0);

	sc = device_get_softc(dev);

	/* issue the phy address and reg */
	WR4(sc, EMAC_MAC_MADR, (1 << 8) | reg);
	/* pull up the phy io line */
	WR4(sc, EMAC_MAC_MCMD, 0x1);
	/* Wait read complete */
	DELAY(10);
	/* push down the phy io line */
	WR4(sc, EMAC_MAC_MCMD, 0x0);
	/* and read data */
	rval = RD4(sc, EMAC_MAC_MRDD);

	return (rval);
}

static int
wemac_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct wemac_softc *sc;

	//printf("-- writereg %i\n", phy);
	//if (phy != 1)
	//      return (0);

	sc = device_get_softc(dev);

	/* issue the phy address and reg */
	WR4(sc, EMAC_MAC_MADR, (1 << 8) | reg);
	/* pull up the phy io line */
	WR4(sc, EMAC_MAC_MCMD, 0x1);
	/* Wait read complete */
	DELAY(10);
	/* push down the phy io line */
	WR4(sc, EMAC_MAC_MCMD, 0x0);
	/* and write data */
	WR4(sc, EMAC_MAC_MWTD, data);

	return (0);
}

static void
wemac_miibus_statchg(device_t dev)
{
	struct wemac_softc *sc;
	struct mii_data *mii;
	struct ifnet *ifp;

	/*
	 * Called by the MII bus driver when the PHY establishes link to set the
	 * MAC interface registers.
	 */

	sc = device_get_softc(dev);

	WEMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;

	mii = sc->mii_softc;

	if (mii->mii_media_status & IFM_ACTIVE)
		sc->link_is_up = true;
	else
		sc->link_is_up = false;

}

static void
wemac_media_status(struct ifnet * ifp, struct ifmediareq *ifmr)
{
	struct wemac_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = sc->mii_softc;
	WEMAC_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	WEMAC_UNLOCK(sc);
}

static int
wemac_media_change_locked(struct wemac_softc *sc)
{

	return (mii_mediachg(sc->mii_softc));
}

static int
wemac_media_change(struct ifnet * ifp)
{
	struct wemac_softc *sc;
	int error;

	sc = ifp->if_softc;

	WEMAC_LOCK(sc);
	error = wemac_media_change_locked(sc);
	WEMAC_UNLOCK(sc);
	return (error);
}

static void
wemac_powerup(struct wemac_softc *sc)
{
	uint32_t reg_val;
	int phy_val;
	uint32_t duplex_flag;
	device_t dev;

	printf("------- powerup wemac...\n");

	dev = sc->dev;

	/* initial EMAC */
	/* flush RX FIFO */
	reg_val = RD4(sc, EMAC_RX_CTL);
	reg_val |= 0x8;
	WR4(sc, EMAC_RX_CTL, reg_val);
	DELAY(1);

	/* soft reset MAC */
	reg_val = RD4(sc, EMAC_MAC_CTL0);
	reg_val &= (~EMAC_MAC_CTL0_SOFT_RST);
	WR4(sc, EMAC_MAC_CTL0, reg_val);

	/* set MII clock */
	reg_val = RD4(sc, EMAC_MAC_MCFG);
	reg_val &= (~(0xf << 2));
	reg_val |= (0xd << 2);
	WR4(sc, EMAC_MAC_MCFG, reg_val);

	/* clear RX counter */
	WR4(sc, EMAC_RX_FBC, 0);

	/* disable all interrupt and clear interrupt status */
	WR4(sc, EMAC_INT_CTL, 0);
	reg_val = RD4(sc, EMAC_INT_STA);
	WR4(sc, EMAC_INT_STA, reg_val);
	DELAY(1);

	// emac setup

	/* Set up TX */
	reg_val = RD4(sc, EMAC_TX_MODE);
	reg_val |= EMAC_TX_AB_M;
	reg_val &= EMAC_TX_TM;
	WR4(sc, EMAC_TX_MODE, reg_val);

	/* Set up RX */
	reg_val = RD4(sc, EMAC_RX_CTL);
	reg_val |= EMAC_RX_SETUP;
	reg_val &= EMAC_RX_TM;
	WR4(sc, EMAC_RX_CTL, reg_val);

	/* Set up MAC CTL0. */
	reg_val = RD4(sc, EMAC_MAC_CTL0);
	reg_val |= EMAC_MAC_CTL0_SETUP;
	WR4(sc, EMAC_MAC_CTL0, reg_val);

	/* Set up MAC CTL1. */
	reg_val = RD4(sc, EMAC_MAC_CTL1);
	DELAY(10);
	phy_val = wemac_miibus_readreg(dev, 0, 0);
	duplex_flag = !!(phy_val & EMAC_PHY_DUPLEX);
	if (duplex_flag)
		reg_val |= EMAC_MAC_CTL1_DUP;
	else
		reg_val &= (EMAC_MAC_CTL1_DUP);
	reg_val |= EMAC_MAC_CTL1_SETUP;
	WR4(sc, EMAC_MAC_CTL1, reg_val);

	/* Set up IPGT */
	WR4(sc, EMAC_MAC_IPGT, EMAC_MAC_IPGT_FD);

	/* Set up IPGR */
	WR4(sc, EMAC_MAC_IPGR, EMAC_MAC_NBTB_IPG2 | (EMAC_MAC_NBTB_IPG1 << 8));

	/* Set up Collison window */
	WR4(sc, EMAC_MAC_CLRT, EMAC_MAC_RM | (EMAC_MAC_CW << 8));

	/* Set up Max Frame Length */
	WR4(sc, EMAC_MAC_MAXF, EMAC_MAC_MFL);

	/* XXX: Hardcode the ethernet address for now */
	eaddr[0] = 0x4e;
	eaddr[0] &= 0xfe;       /* the 48bit must set 0 */
	eaddr[0] |= 0x02;       /* the 47bit must set 1 */

	eaddr[1] = 0x34;
	eaddr[2] = 0x84;
	eaddr[3] = 0xd3;
	eaddr[4] = 0xd3;
	eaddr[5] = 0xa9;

	/* Write ethernet address to register */
	WR4(sc, EMAC_MAC_A1, eaddr[0] << 16 | eaddr[1] << 8 | eaddr[2]);
	WR4(sc, EMAC_MAC_A0, eaddr[3] << 16 | eaddr[4] << 8 | eaddr[5]);
}

static void
wemac_tick(void *arg)
{
	struct wemac_softc *sc;
	struct ifnet *ifp;
	int link_was_up;

	sc = arg;

	WEMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;

	if (!(ifp->if_drv_flags & IFF_DRV_RUNNING))
	    return;

	/*
	 * Typical tx watchdog.  If this fires it indicates that we enqueued
	 * packets for output and never got a txdone interrupt for them.  Maybe
	 * it's a missed interrupt somehow, just pretend we got one.
	 */
	if (sc->tx_watchdog_count > 0) {
		if (--sc->tx_watchdog_count == 0) {
			wemac_txfinish_locked(sc);
		}
	}

	/* Check the media status. */
	link_was_up = sc->link_is_up;
	mii_tick(sc->mii_softc);
	if (sc->link_is_up && !link_was_up)
		wemac_txstart_locked(sc);

	/* Schedule another check one second from now. */
	callout_reset(&sc->wemac_callout, hz, wemac_tick, sc);
}
/*
static void
fix_mbuf(struct mbuf *m, int sramc_align)
{
	uint16_t *src, *dst;
	int i;

	src = mtod(m, uint16_t *);
	dst = src - (sramc_align - ETHER_ALIGN) / sizeof *src;
	for (i = 0; i < (m->m_len / sizeof(uint16_t) + 1); i++)
		*dst++ = *src++;
	m->m_data -= sramc_align - ETHER_ALIGN;
}
*/
inline static uint32_t
wemac_setup_txdesc(struct wemac_softc *sc, int idx, bus_addr_t paddr, 
    uint32_t len)
{
	uint32_t nidx;

	nidx = next_txidx(sc, idx);

	/* Addr/len 0 means we're clearing the descriptor after xmit done. */
	if (paddr == 0 || len == 0) {
		--sc->txcount;
	} else {
		++sc->txcount;
	}

	/*
	 * The hardware requires 32-bit physical addresses.  We set up the dma
	 * tag to indicate that, so the cast to uint32_t should never lose
	 * significant bits.
	 */
	sc->txdesc_ring[idx].buf_paddr = (uint32_t)paddr;

	return (nidx);
}

static int
wemac_setup_txbuf(struct wemac_softc *sc, int idx, struct mbuf **mp)
{
	struct mbuf * m;
	int error, nsegs;
	struct bus_dma_segment seg;

	if ((m = m_defrag(*mp, M_NOWAIT)) == NULL)
		return (ENOMEM);
	*mp = m;

	error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag, sc->txbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (ENOMEM);
	}
	bus_dmamap_sync(sc->txbuf_tag, sc->txbuf_map[idx].map, 
	    BUS_DMASYNC_PREWRITE);

	sc->txbuf_map[idx].mbuf = m;
	wemac_setup_txdesc(sc, idx, seg.ds_addr, seg.ds_len);

	return (0);

}

static void
wemac_txstart_locked(struct wemac_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	int enqueued;
	uint32_t reg_val;

	WEMAC_ASSERT_LOCKED(sc);

	if (!sc->link_is_up)
		return;

	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_OACTIVE)
		return;

	enqueued = 0;

	/* Select channel */
	WR4(sc, EMAC_TX_INS, 0); // TODO: use multiple buffers

	for (;;) {
		if (sc->txcount == (TX_DESC_COUNT-1)) {
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
		if (wemac_setup_txbuf(sc, sc->tx_idx_head, &m) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m);
			break;
		}
		/* Send the data lengh. */
		WR4(sc, EMAC_TX_PL0, m->m_len);

		/* Start translate from fifo to phy. */
		reg_val = RD4(sc, EMAC_TX_CTL0);
		reg_val |= 1;
		WR4(sc, EMAC_TX_CTL0, reg_val);

		printf("------- sending %i bytes\n", m->m_len);

		BPF_MTAP(ifp, m);
		sc->tx_idx_head = next_txidx(sc, sc->tx_idx_head);
		++enqueued;
	}

	if (enqueued != 0) {
		sc->tx_watchdog_count = WATCHDOG_TIMEOUT_SECS;
	}
}

static void
wemac_txstart(struct ifnet *ifp)
{
	struct wemac_softc *sc = ifp->if_softc;

	WEMAC_LOCK(sc);
	wemac_txstart_locked(sc);
	WEMAC_UNLOCK(sc);
}

static void
wemac_txfinish_locked(struct wemac_softc *sc)
{
	struct ifnet *ifp;
	struct wemac_hwdesc *desc;
	struct wemac_bufmap *bmap;
	boolean_t retired_buffer;

	WEMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	retired_buffer = false;
	while (sc->tx_idx_tail != sc->tx_idx_head) {
		desc = &sc->txdesc_ring[sc->tx_idx_tail];
		retired_buffer = true;
		bmap = &sc->txbuf_map[sc->tx_idx_tail];
		bus_dmamap_sync(sc->txbuf_tag, bmap->map, 
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->txbuf_tag, bmap->map);
		m_freem(bmap->mbuf);
		bmap->mbuf = NULL;
		wemac_setup_txdesc(sc, sc->tx_idx_tail, 0, 0);
		sc->tx_idx_tail = next_txidx(sc, sc->tx_idx_tail);
	}

	/*
	 * If we retired any buffers, there will be open tx slots available in
	 * the descriptor ring, go try to start some new output.
	 */
	if (retired_buffer) {
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
		wemac_txstart_locked(sc);
	}

	/* If there are no buffers outstanding, muzzle the watchdog. */
	if (sc->tx_idx_tail == sc->tx_idx_head) {
		sc->tx_watchdog_count = 0;
	}
}

inline static uint32_t
wemac_setup_rxdesc(struct wemac_softc *sc, int idx, bus_addr_t paddr)
{
	uint32_t nidx;

	/*
	 * The hardware requires 32-bit physical addresses.  We set up the dma
	 * tag to indicate that, so the cast to uint32_t should never lose
	 * significant bits.
	 */
	nidx = next_rxidx(sc, idx);
	sc->rxdesc_ring[idx].buf_paddr = (uint32_t)paddr;

	return (nidx);
}

static int
wemac_setup_rxbuf(struct wemac_softc *sc, int idx, struct mbuf * m)
{
	int error, nsegs;
	struct bus_dma_segment seg;

	/* hardware required 4 byte alignment */
	m_adj(m, 4);

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (error);
	}

	bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf_map[idx].map, 
	    BUS_DMASYNC_PREREAD);

	sc->rxbuf_map[idx].mbuf = m;
	wemac_setup_rxdesc(sc, idx, seg.ds_addr);
	
	return (0);
}

static struct mbuf *
wemac_alloc_mbufcl(struct wemac_softc *sc)
{
	struct mbuf *m;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;

	return (m);
}

static void
wemac_rxfinish_onebuf(struct wemac_softc *sc, int len)
{
	struct mbuf *m, *newmbuf;
	struct wemac_bufmap *bmap;
//	uint8_t *dst, *src;
	int error;

	/*
	 *  First try to get a new mbuf to plug into this slot in the rx ring.
	 *  If that fails, drop the current packet and recycle the current
	 *  mbuf, which is still mapped and loaded.
	 */
	if ((newmbuf = wemac_alloc_mbufcl(sc)) == NULL) {
		++sc->ifp->if_iqdrops;
		wemac_setup_rxdesc(sc, sc->rx_idx, 
		    sc->rxdesc_ring[sc->rx_idx].buf_paddr);
		return;
	}

	/*
	 *  Unfortunately, the protocol headers need to be aligned on a 32-bit
	 *  boundary for the upper layers.  The hardware requires receive
	 *  buffers to be 4-byte aligned.  The ethernet header is 14 bytes,
	 *  leaving the protocol header unaligned.  We used m_adj() after
	 *  allocating the buffer to leave empty space at the start of the
	 *  buffer, now we'll use the alignment agnostic routine to
	 *  shuffle all the data backwards 2 bytes and adjust m_data.
	 *
	 */

	WEMAC_UNLOCK(sc);

	bmap = &sc->rxbuf_map[sc->rx_idx];
	len -= ETHER_CRC_LEN;
	bus_dmamap_sync(sc->rxbuf_tag, bmap->map, BUS_DMASYNC_POSTREAD);
	bus_dmamap_unload(sc->rxbuf_tag, bmap->map);
	m = bmap->mbuf;
	bmap->mbuf = NULL;
	m->m_len = len;
	m->m_pkthdr.len = len;
	m->m_pkthdr.rcvif = sc->ifp;

//	src = mtod(m, uint8_t*);
//	dst = src - ETHER_ALIGN;
//	bcopy(src, dst, len);
//	m->m_data = dst;

	// align the IP header
//	fix_mbuf(m, 4);

	sc->ifp->if_input(sc->ifp, m);

	WEMAC_LOCK(sc);

	if ((error = wemac_setup_rxbuf(sc, sc->rx_idx, newmbuf)) != 0) {
		device_printf(sc->dev, "wemac_setup_rxbuf error %d\n", error);
		/* XXX Now what?  We've got a hole in the rx ring. */
	}

}

static void
wemac_rxfinish_locked(struct wemac_softc *sc)
{
	struct ifnet *ifp;
	uint32_t reg_val, rxcount;
	int16_t len;
	uint16_t status;
	int good_packet;

	ifp = sc->ifp;

	WEMAC_ASSERT_LOCKED(sc);

	for (;;) {

		/*
		 * Race warning: The first packet might arrive with
		 * the interrupts disabled, but the second will fix
		 */
		rxcount = RD4(sc, EMAC_RX_FBC);
		if (!rxcount) {
			/* Had one stuck? */
			rxcount = RD4(sc, EMAC_RX_FBC);
			if (!rxcount) {
				sc->rx_completed_flag = 1;
				return;
			}
		}

		// packet header
		reg_val = RD4(sc, EMAC_RX_IO_DATA);
		if (reg_val != 0x0143414d) {
			// packet header wrong

			/* Disable RX */
			reg_val = RD4(sc, EMAC_CTL);
			reg_val &= ~EMAC_CTL_RX_EN;
			WR4(sc, EMAC_CTL, reg_val);

			/* Flush RX FIFO */
			reg_val = RD4(sc, EMAC_RX_CTL);
			reg_val |= (1 << 3);
			WR4(sc, EMAC_RX_CTL, reg_val);
			while (RD4(sc, EMAC_RX_CTL) & (1 << 3))
				;

			/* Enable RX */
			reg_val = RD4(sc, EMAC_CTL);
			reg_val |= EMAC_CTL_RX_EN;
			WR4(sc, EMAC_CTL, reg_val);

			sc->rx_completed_flag = 1;
			return;
		}

		good_packet = 1;

		/* get packet size */
		reg_val = RD4(sc, EMAC_RX_IO_DATA);

		len = reg_val & 0xffff;
		status = (reg_val >> 16) & 0xffff;

//		printf("------ len=%i status=%i\n", len, status);

		if (len < 0x40) {
			good_packet = 0;
			printf("------ bad packet: len=%i status=%i< 0x40\n", len, status);
		}

		/* rx_status is identical to RSR register. */
		if (0 & status & (EMAC_CRCERR | EMAC_LENERR)) {
			good_packet = 0;
			printf("------ error\n");
			if (status & EMAC_CRCERR)
				printf("------ crc error\n");
			if (status & EMAC_LENERR)
				printf("------ length error\n");
		}

		if (good_packet) {
			/*
			 *  Normal case: a good frame all in one buffer.
			 */
			wemac_rxfinish_onebuf(sc, len);
		}
		sc->rx_completed_flag = 1;

		sc->rx_idx = next_rxidx(sc, sc->rx_idx);
	}
}

static void
wemac_stop_locked(struct wemac_softc *sc)
{
	struct ifnet *ifp;
	struct wemac_hwdesc *desc;
	struct wemac_bufmap *bmap;
	int idx;

	WEMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	sc->tx_watchdog_count = 0;

	/*
	 * Stop the media-check callout.  Do not use callout_drain() because
	 * we're holding a mutex the callout acquires, and if it's currently
	 * waiting to acquire it, we'd deadlock.  If it is waiting now, the
	 * wemac_tick() routine will return without doing anything when it sees
	 * that IFF_DRV_RUNNING is not set, so avoiding callout_drain() is safe.
	 */
	callout_stop(&sc->wemac_callout);

	/*
	 * Discard all untransmitted buffers.  Each buffer is simply freed;
	 * it's as if the bits were transmitted and then lost on the wire.
	 *
	 * XXX Is this right?  Or should we use IFQ_DRV_PREPEND() to put them
	 * back on the queue for when we get restarted later?
	 */
	idx = sc->tx_idx_tail;
	while (idx != sc->tx_idx_head) {
		desc = &sc->txdesc_ring[idx];
		bmap = &sc->txbuf_map[idx];
		if (desc->buf_paddr != 0) {
			bus_dmamap_unload(sc->txbuf_tag, bmap->map);
			m_freem(bmap->mbuf);
			bmap->mbuf = NULL;
			wemac_setup_txdesc(sc, idx, 0, 0);
		}
		idx = next_txidx(sc, idx);
	}

	/*
	 * Discard all unprocessed receive buffers.  This amounts to just
	 * pretending that nothing ever got received into them.  We reuse the
	 * mbuf already mapped for each desc, simply turning the EMPTY flags
	 * back on so they'll get reused when we start up again.
	 */
	for (idx = 0; idx < RX_DESC_COUNT; ++idx) {
		desc = &sc->rxdesc_ring[idx];
		wemac_setup_rxdesc(sc, idx, desc->buf_paddr);
	}
}

static void
wemac_init_locked(struct wemac_softc *sc)
{
	struct ifnet *ifp = sc->ifp;
	uint32_t reg_val;
	int phy_reg;
	device_t dev;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	printf("------- initializing wemac...\n");

	dev = sc->dev;

	//wemac_reset(sc); // TODO: reset?

	// TODO:
	// need GPIO power
	// DELAY(50);

	/* PHY POWER UP */
	phy_reg = wemac_miibus_readreg(dev, 0, 0);
	wemac_miibus_writereg(dev, 0, 0, phy_reg & (~(1 << 11)));
	DELAY(4500);

	phy_reg = wemac_miibus_readreg(dev, 0, 0);

	/* set EMAC SPEED, depend on PHY */
	reg_val = RD4(sc, EMAC_MAC_SUPP);
	reg_val &= (~(0x1 << 8));
	reg_val |= (((phy_reg & (1 << 13)) >> 13) << 8);
	WR4(sc, EMAC_MAC_SUPP, reg_val);

	/* set duplex depend on phy */
	reg_val = RD4(sc, EMAC_MAC_CTL1);
	reg_val &= (~(0x1 << 0));
	reg_val |= (((phy_reg & (1 << 8)) >> 8) << 0);
	WR4(sc, EMAC_MAC_CTL1, reg_val);

	/* enable RX/TX */
	reg_val = RD4(sc, EMAC_CTL);
	WR4(sc, EMAC_CTL, reg_val | EMAC_CTL_RST | EMAC_CTL_TX_EN | EMAC_CTL_RX_EN);

	/* enable RX/TX0/RX Hlevel interrupt */
	reg_val = RD4(sc, EMAC_INT_CTL);
	reg_val |= (0xf << 0) | (0x01 << 8);
	WR4(sc, EMAC_INT_CTL, reg_val);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	sc->rx_completed_flag = 1;

	/*
	* Call mii_mediachg() which will call back into wemac_miibus_statchg() to
	* set up the remaining config registers based on the current media.
	*/
	mii_mediachg(sc->mii_softc);
	callout_reset(&sc->wemac_callout, hz, wemac_tick, sc);

}

static void
wemac_init(void *if_softc)
{
	struct wemac_softc *sc = if_softc;

	WEMAC_LOCK(sc);
	wemac_init_locked(sc);
	WEMAC_UNLOCK(sc);
}

static void
wemac_intr(void *arg)
{
	struct wemac_softc *sc = (struct wemac_softc *)arg;
	struct ifnet *ifp;
	uint32_t int_status, reg_val;

	ifp = sc->ifp;

	WEMAC_LOCK(sc);

	/* Disable all interrupts */
	WR4(sc, EMAC_INT_CTL, 0);
	/* Get WEMAC interrupt status */
	int_status = RD4(sc, EMAC_INT_STA); /* Got ISR */
	WR4(sc, EMAC_INT_STA, int_status); /* Clear ISR status */

	/* Received the coming packet */
	if ((int_status & 0x100) && (sc->rx_completed_flag == 1)) {
		sc->rx_completed_flag = 0;
		wemac_rxfinish_locked(sc);
	}

	/* Transmit Interrupt check */
	if (int_status & (0x01 | 0x02)){
		wemac_txfinish_locked(sc);
		if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
			wemac_txstart_locked(sc);
	}

	/* Re-enable interrupt mask */
	if (sc->rx_completed_flag == 1) {
		reg_val = RD4(sc, EMAC_INT_CTL);
		reg_val |= (0xf << 0) | (0x01 << 8);
		WR4(sc, EMAC_INT_CTL, reg_val);
	}
	WEMAC_UNLOCK(sc);
}

static int
wemac_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct wemac_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int mask, error;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	error = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		WEMAC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING)==0)
				wemac_init_locked(sc);
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				wemac_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		WEMAC_UNLOCK(sc);
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		mii = sc->mii_softc;
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, cmd);
		break;

	case SIOCSIFCAP:
		mask = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (mask & IFCAP_VLAN_MTU) {
			/* No work to do except acknowledge the change took. */
			ifp->if_capenable ^= IFCAP_VLAN_MTU;
		}
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}       

	return (error);
}

static int
wemac_detach(device_t dev)
{
	struct wemac_softc *sc;
	bus_dmamap_t map;
	int idx;

	/*
	 * NB: This function can be called internally to unwind a failure to
	 * attach. Make sure a resource got allocated/created before destroying.
	 */

	sc = device_get_softc(dev);

	if (sc->is_attached) {
		WEMAC_LOCK(sc);
		sc->is_detaching = true;
		wemac_stop_locked(sc);
		WEMAC_UNLOCK(sc);
		callout_drain(&sc->wemac_callout);
		ether_ifdetach(sc->ifp);
	}

	/* XXX no miibus detach? */

	/* Clean up RX DMA resources and free mbufs. */
	for (idx = 0; idx < RX_DESC_COUNT; ++idx) {
		if ((map = sc->rxbuf_map[idx].map) != NULL) {
			bus_dmamap_unload(sc->rxbuf_tag, map);
			bus_dmamap_destroy(sc->rxbuf_tag, map);
			m_freem(sc->rxbuf_map[idx].mbuf);
		}
	}
	if (sc->rxbuf_tag != NULL)
		bus_dma_tag_destroy(sc->rxbuf_tag);
	if (sc->rxdesc_map != NULL) {
		bus_dmamap_unload(sc->rxdesc_tag, sc->rxdesc_map);
		bus_dmamap_destroy(sc->rxdesc_tag, sc->rxdesc_map);
	}
	if (sc->rxdesc_tag != NULL)
	bus_dma_tag_destroy(sc->rxdesc_tag);

	/* Clean up TX DMA resources. */
	for (idx = 0; idx < TX_DESC_COUNT; ++idx) {
		if ((map = sc->txbuf_map[idx].map) != NULL) {
			/* TX maps are already unloaded. */
			bus_dmamap_destroy(sc->txbuf_tag, map);
		}
	}
	if (sc->txbuf_tag != NULL)
		bus_dma_tag_destroy(sc->txbuf_tag);
	if (sc->txdesc_map != NULL) {
		bus_dmamap_unload(sc->txdesc_tag, sc->txdesc_map);
		bus_dmamap_destroy(sc->txdesc_tag, sc->txdesc_map);
	}
	if (sc->txdesc_tag != NULL)
	bus_dma_tag_destroy(sc->txdesc_tag);

	/* Release bus resources. */
	if (sc->intr_cookie)
		bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);

	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);

	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	WEMAC_LOCK_DESTROY(sc);
	return (0);
}

static int
wemac_attach(device_t dev)
{
	struct wemac_softc *sc;
	struct ifnet *ifp = NULL;
	struct mbuf *m;
	phandle_t ofw_node;
	int error, rid;
	uint32_t idx;

	sc = device_get_softc(dev);
	sc->dev = dev;

	WEMAC_LOCK_INIT(sc);

	/*
	 * We have to be told what kind of electrical connection exists between
	 * the MAC and PHY or we can't operate correctly.
	 */
	if ((ofw_node = ofw_bus_get_node(dev)) == -1) {
		device_printf(dev, "Impossible: Can't find ofw bus node\n");
		error = ENXIO;
		goto out;
	}

	callout_init_mtx(&sc->wemac_callout, &sc->mtx, 0);

	/* Allocate bus resources for accessing the hardware. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not allocate memory resources.\n");
		error = ENOMEM;
		goto out;
	}
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "could not allocate interrupt resources.\n");
		error = ENOMEM;
		goto out;
	}

	/*
	 * Set up TX descriptor ring, descriptors, and dma maps.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),	/* Parent tag. */
	    EMAC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    TX_DESC_SIZE, 1, 		/* maxsize, nsegments */
	    TX_DESC_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	error = bus_dmamem_alloc(sc->txdesc_tag, (void**)&sc->txdesc_ring,
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO, &sc->txdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate TX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->txdesc_tag, sc->txdesc_map, sc->txdesc_ring,
	    TX_DESC_SIZE, wemac_get1paddr, &sc->txdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load TX descriptor ring map.\n");
		goto out;
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),	/* Parent tag. */
	    EMAC_TXBUF_ALIGN, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < TX_DESC_COUNT; ++idx) {
		error = bus_dmamap_create(sc->txbuf_tag, 0, 
		    &sc->txbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create TX buffer DMA map.\n");
			goto out;
		}
		wemac_setup_txdesc(sc, idx, 0, 0);
	}

	/*
	 * Set up RX descriptor ring, descriptors, dma maps, and mbufs.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),	/* Parent tag. */
	    EMAC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    RX_DESC_SIZE, 1, 		/* maxsize, nsegments */
	    RX_DESC_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX ring DMA tag.\n");
		goto out;
	}

	error = bus_dmamem_alloc(sc->rxdesc_tag, (void **)&sc->rxdesc_ring, 
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO, &sc->rxdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate RX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->rxdesc_tag, sc->rxdesc_map, sc->rxdesc_ring,
	    RX_DESC_SIZE, wemac_get1paddr, &sc->rxdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load RX descriptor ring map.\n");
		goto out;
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),	/* Parent tag. */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX buf DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < RX_DESC_COUNT; ++idx) {
		error = bus_dmamap_create(sc->rxbuf_tag, 0, 
		    &sc->rxbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create RX buffer DMA map.\n");
			goto out;
		}
		if ((m = wemac_alloc_mbufcl(sc)) == NULL) {
			device_printf(dev, "Could not alloc mbuf\n");
			error = ENOMEM;
			goto out;
		}
		if ((error = wemac_setup_rxbuf(sc, idx, m)) != 0) {
			device_printf(sc->dev,
			    "could not create new RX buffer.\n");
			goto out;
		}
	}

	wemac_sys_setup();
	wemac_powerup(sc);
	wemac_reset(sc);

	/* Setup interrupt handler. */
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, wemac_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		goto out;
	}

	/* Set up the ethernet interface. */
	sc->ifp = ifp = if_alloc(IFT_ETHER);

	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;
	ifp->if_start = wemac_txstart;
	ifp->if_ioctl = wemac_ioctl;
	ifp->if_init = wemac_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, TX_DESC_COUNT - 1);
	ifp->if_snd.ifq_drv_maxlen = TX_DESC_COUNT - 1;
	IFQ_SET_READY(&ifp->if_snd);
	ifp->if_data.ifi_hdrlen = sizeof(struct ether_vlan_header);


	/* Attach the mii driver. */
	error = mii_attach(dev, &sc->miibus, ifp, wemac_media_change,
	    wemac_media_status, BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error != 0) {
		device_printf(dev, "PHY attach failed\n");
		goto out;
	}
	sc->mii_softc = device_get_softc(sc->miibus);

	/* All ready to run, attach the ethernet interface. */
	ether_ifattach(ifp, eaddr);
	sc->is_attached = true;

	error = 0;
out:

	if (error != 0)
		wemac_detach(dev);

	return (error);
}

static int
wemac_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "allwinner,wemac"))
		return (ENXIO);

        device_set_desc(dev, "Allwinner A10 WEMAC");
	return (0);
}


static device_method_t wemac_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,		wemac_probe),
	DEVMETHOD(device_attach,	wemac_attach),
	DEVMETHOD(device_detach,	wemac_detach),

/*
	DEVMETHOD(device_shutdown,	wemac_shutdown),
	DEVMETHOD(device_suspend,	wemac_suspend),
	DEVMETHOD(device_resume,	wemac_resume),
*/

	/* MII interface. */
	DEVMETHOD(miibus_readreg,	wemac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	wemac_miibus_writereg),
	DEVMETHOD(miibus_statchg,	wemac_miibus_statchg),

	DEVMETHOD_END
};

static driver_t wemac_driver = {
	"wemac",
	wemac_methods,
	sizeof(struct wemac_softc)
};

static devclass_t wemac_devclass;

DRIVER_MODULE(wemac, simplebus, wemac_driver, wemac_devclass, 0, 0);
DRIVER_MODULE(miibus, wemac, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(wemac, ether, 1, 1, 1);
MODULE_DEPEND(wemac, miibus, 1, 1, 1);
