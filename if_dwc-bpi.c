/*-
 * Copyright (c) 2014 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory under DARPA/AFRL contract (FA8750-10-C-0237)
 * ("CTSRD"), as part of the DARPA CRASH research programme.
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
 * Ethernet media access controller (EMAC)
 * Chapter 17, Altera Cyclone V Device Handbook (CV-5V2 2014.07.22)
 *
 * EMAC is an instance of the Synopsys DesignWare 3504-0
 * Universal 10/100/1000 Ethernet MAC (DWC_gmac).
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/dev/dwc/if_dwc.c 272226 2014-09-27 20:43:01Z glebius $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/endian.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#include <net/if_vlan_var.h>

#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miibus_if.h"

#define	READ4(_sc, _reg) \
	bus_read_4((_sc)->res[0], _reg)
#define	WRITE4(_sc, _reg, _val) \
	bus_write_4((_sc)->res[0], _reg, _val)

#define	READ4_SID(_sc, _reg) \
	bus_read_4((_sc)->res[1], _reg)
#define	WRITE4_SID(_sc, _reg, _val) \
	bus_write_4((_sc)->res[1], _reg, _val)

#define	WATCHDOG_TIMEOUT_SECS	5
#define	STATS_HARVEST_INTERVAL	2
#define	MII_CLK_VAL		2

#include <arm/allwinner/if_dwc-bpi.h>
#include <arm/allwinner/a10_clk.h>
#include <arm/allwinner/a10_gpio.h>

#define	DWC_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	DWC_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	DWC_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->mtx, MA_OWNED);
#define	DWC_ASSERT_UNLOCKED(sc)		mtx_assert(&(sc)->mtx, MA_NOTOWNED);

/* tx/rx status definitions */

/* tx status bits definitions */
#define	DDESC_TDES0_OWN			(1 << 31)
#define	DDESC_TDES0_MSK		        (0x1ffff << 0)

#define DDESC_TDES0_TXINT               (1 << 30)
#define DDESC_TDES0_TXLAST              (1 << 29)
#define DDESC_TDES0_TXFIRST             (1 << 28)
#define DDESC_TDES0_TXCRCDIS            (1 << 27)
#define DDESC_TDES0_TXRINGEND           (1 << 21)
#define DDESC_TDES0_TXCHAIN             (1 << 20)

/* rx status bits definitions */
#define	DDESC_RDES0_OWN			(1 << 31)
#define	DDESC_RDES0_FL_MASK		0x3fff
#define	DDESC_RDES0_FL_SHIFT		16	/* Frame Length */
#define	DDESC_RDES1_CHAINED		(1 << 14)

#define DDESC_RXSTS_DAFILTERFAIL        (1 << 30)
#define DDESC_RXSTS_FRMLENMSK           (0x3FFF << 16)
#define DDESC_RXSTS_FRMLENSHFT          (16)
#define DDESC_RXSTS_ERROR               (1 << 15)
#define DDESC_RXSTS_RXTRUNCATED         (1 << 14)
#define DDESC_RXSTS_SAFILTERFAIL        (1 << 13)
#define DDESC_RXSTS_RXIPC_GIANTFRAME    (1 << 12)
#define DDESC_RXSTS_RXDAMAGED           (1 << 11)
#define DDESC_RXSTS_RXVLANTAG           (1 << 10)
#define DDESC_RXSTS_RXFIRST             (1 << 9)
#define DDESC_RXSTS_RXLAST              (1 << 8)
#define DDESC_RXSTS_RXIPC_GIANT         (1 << 7)
#define DDESC_RXSTS_RXCOLLISION         (1 << 6)
#define DDESC_RXSTS_RXFRAMEETHER        (1 << 5)
#define DDESC_RXSTS_RXWATCHDOG          (1 << 4)
#define DDESC_RXSTS_RXMIIERROR          (1 << 3)
#define DDESC_RXSTS_RXDRIBBLING         (1 << 2)
#define DDESC_RXSTS_RXCRC               (1 << 1)

/* tx control bits definitions */
#define DDESC_TXCTRL_TXINT               (1 << 31)
#define DDESC_TXCTRL_TXLAST              (1 << 30)
#define DDESC_TXCTRL_TXFIRST             (1 << 29)
#define DDESC_TXCTRL_TXCHECKINSCTRL      (3 << 27)
#define DDESC_TXCTRL_TXCRCDIS            (1 << 26)
#define DDESC_TXCTRL_TXRINGEND           (1 << 25)
#define DDESC_TXCTRL_TXCHAIN             (1 << 24)
#define DDESC_TXCTRL_SIZE1MASK           (0x7FF << 0)
#define DDESC_TXCTRL_SIZE1SHFT           (0)
#define DDESC_TXCTRL_SIZE2MASK           (0x7FF << 11)
#define DDESC_TXCTRL_SIZE2SHFT           (11)

/* rx control bits definitions */
#define DDESC_RXCTRL_RXINTDIS            (1 << 31)
#define DDESC_RXCTRL_RXRINGEND           (1 << 25)
#define DDESC_RXCTRL_RXCHAIN             (1 << 24)
#define DDESC_RXCTRL_SIZE1MASK           (0x7FF << 0)
#define DDESC_RXCTRL_SIZE1SHFT           (0)
#define DDESC_RXCTRL_SIZE2MASK           (0x7FF << 11)
#define DDESC_RXCTRL_SIZE2SHFT           (11)

struct dwc_bufmap {
	bus_dmamap_t	map;
	struct mbuf	*mbuf;
};

/*
 * A hardware buffer descriptor.  Rx and Tx buffers have the same descriptor
 * layout, but the bits in the flags field have different meanings.
 */
struct dwc_hwdesc
{
	uint32_t txrx_status;
	uint32_t dmamac_cntl;
	uint32_t addr;		/* pointer to buffer data */
	uint32_t addr_next;	/* link to next descriptor */
};

/*
 * Driver data and defines.
 */
#define	RX_DESC_COUNT	16
#define	RX_DESC_SIZE	(sizeof(struct dwc_hwdesc) * RX_DESC_COUNT)
#define	TX_DESC_COUNT	16
#define	TX_DESC_SIZE	(sizeof(struct dwc_hwdesc) * TX_DESC_COUNT)

/*
 * The hardware imposes alignment restrictions on various objects involved in
 * DMA transfers.  These values are expressed in bytes (not bits).
 */
#define	DWC_DESC_RING_ALIGN		2048

struct dwc_softc {
	struct resource		*res[3];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	bus_space_tag_t		sid_bst;
	bus_space_handle_t	sid_bsh;
	device_t		dev;
	int			mii_clk;
	device_t		miibus;
	struct mii_data *	mii_softc;
	struct ifnet		*ifp;
	int			if_flags;
	struct mtx		mtx;
	void *			intr_cookie;
	struct callout		dwc_callout;
	uint8_t			phy_conn_type;
	uint8_t			mactype;
	boolean_t		link_is_up;
	boolean_t		is_attached;
	boolean_t		is_detaching;
	int			tx_watchdog_count;
	int			stats_harvest_count;

	/* RX */
	bus_dma_tag_t		rxdesc_tag;
	bus_dmamap_t		rxdesc_map;
	struct dwc_hwdesc	*rxdesc_ring;
	bus_addr_t		rxdesc_ring_paddr;
	bus_dma_tag_t		rxbuf_tag;
	struct dwc_bufmap	rxbuf_map[RX_DESC_COUNT];
	uint32_t		rx_idx;

	/* TX */
	bus_dma_tag_t		txdesc_tag;
	bus_dmamap_t		txdesc_map;
	struct dwc_hwdesc	*txdesc_ring;
	bus_addr_t		txdesc_ring_paddr;
	bus_dma_tag_t		txbuf_tag;
	struct dwc_bufmap	txbuf_map[RX_DESC_COUNT];
	uint32_t		tx_idx_head;
	uint32_t		tx_idx_tail;
	int			txcount;
};

static struct resource_spec dwc_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
        { SYS_RES_MEMORY,       1,      RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static void dwc_txfinish_locked(struct dwc_softc *sc);
static void dwc_rxfinish_locked(struct dwc_softc *sc);
static void dwc_stop_locked(struct dwc_softc *sc);
static void dwc_setup_rxfilter(struct dwc_softc *sc);

static inline uint32_t
next_rxidx(struct dwc_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % RX_DESC_COUNT);
}

static inline uint32_t
next_txidx(struct dwc_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % TX_DESC_COUNT);
}

static void
dwc_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

inline static uint32_t
dwc_setup_txdesc(struct dwc_softc *sc, int idx, bus_addr_t paddr,
    uint32_t len)
{
	uint32_t flags;
	uint32_t nidx;

	nidx = next_txidx(sc, idx);

	/* Addr/len 0 means we're clearing the descriptor after xmit done. */
	if (paddr == 0 || len == 0) {
		flags = 0;
		--sc->txcount;
	} else {
		flags = ((len << DDESC_TXCTRL_SIZE1SHFT) & \
		    DDESC_TXCTRL_SIZE1MASK) | DDESC_TXCTRL_TXLAST | \
		    DDESC_TXCTRL_TXFIRST;
		++sc->txcount;
	}

	sc->txdesc_ring[idx].addr = (uint32_t)(paddr);
	sc->txdesc_ring[idx].txrx_status = flags;
	sc->txdesc_ring[idx].dmamac_cntl = len;

	if (paddr && len) {
		wmb();
		sc->txdesc_ring[idx].txrx_status |= DDESC_TDES0_OWN;
		wmb();
	}

	return (nidx);
}

static int
dwc_setup_txbuf(struct dwc_softc *sc, int idx, struct mbuf **mp)
{
	struct bus_dma_segment seg;
	int error, nsegs;
	struct mbuf * m;

	if ((m = m_defrag(*mp, M_NOWAIT)) == NULL)
		return (ENOMEM);
	*mp = m;

	error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag, sc->txbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (ENOMEM);
	}

	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->txbuf_tag, sc->txbuf_map[idx].map,
	    BUS_DMASYNC_PREWRITE);

	sc->txbuf_map[idx].mbuf = m;

	dwc_setup_txdesc(sc, idx, seg.ds_addr, seg.ds_len);

	return (0);
}

static void
dwc_txstart_locked(struct dwc_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	int enqueued;

	DWC_ASSERT_LOCKED(sc);

	if (!sc->link_is_up)
		return;

	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_OACTIVE) {
		return;
	}

	enqueued = 0;

	for (;;) {
		if (sc->txcount == (TX_DESC_COUNT-1)) {
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
		if (dwc_setup_txbuf(sc, sc->tx_idx_head, &m) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m);
			break;
		}
		BPF_MTAP(ifp, m);
		sc->tx_idx_head = next_txidx(sc, sc->tx_idx_head);
		++enqueued;
	}

	if (enqueued != 0) {
		WRITE4(sc, TRANSMIT_POLL_DEMAND, 0x1);
//		WRITE4(sc, TRANSMIT_POLL_DEMAND, 0xffffffff);
		sc->tx_watchdog_count = WATCHDOG_TIMEOUT_SECS;
	}
}

static void
dwc_txstart(struct ifnet *ifp)
{
	struct dwc_softc *sc = ifp->if_softc;

	DWC_LOCK(sc);
	dwc_txstart_locked(sc);
	DWC_UNLOCK(sc);
}

static void
dwc_stop_locked(struct dwc_softc *sc)
{
	struct ifnet *ifp;
	int reg;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	sc->tx_watchdog_count = 0;
	sc->stats_harvest_count = 0;

	callout_stop(&sc->dwc_callout);

	/* Stop DMA TX */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_ST);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Flush TX */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_FTF);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Stop transmitters */
	reg = READ4(sc, MAC_CONFIGURATION);
	reg &= ~(CONF_TE | CONF_RE);
	WRITE4(sc, MAC_CONFIGURATION, reg);

	/* Stop DMA RX */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);
}

static void dwc_clear_stats(struct dwc_softc *sc)
{
	int reg;

	reg = READ4(sc, MMC_CONTROL);
	reg |= (MMC_CONTROL_CNTRST);
	WRITE4(sc, MMC_CONTROL, reg);
}

static void
dwc_harvest_stats(struct dwc_softc *sc)
{
	struct ifnet *ifp;

	/* We don't need to harvest too often. */
	if (++sc->stats_harvest_count < STATS_HARVEST_INTERVAL)
		return;

	sc->stats_harvest_count = 0;
	ifp = sc->ifp;

	if_inc_counter(ifp, IFCOUNTER_IPACKETS, READ4(sc, RXFRAMECOUNT_GB));
	if_inc_counter(ifp, IFCOUNTER_IMCASTS, READ4(sc, RXMULTICASTFRAMES_G));
	if_inc_counter(ifp, IFCOUNTER_IERRORS,
	    READ4(sc, RXOVERSIZE_G) + READ4(sc, RXUNDERSIZE_G) +
	    READ4(sc, RXCRCERROR) + READ4(sc, RXALIGNMENTERROR) +
	    READ4(sc, RXRUNTERROR) + READ4(sc, RXJABBERERROR) +
	    READ4(sc, RXLENGTHERROR));

	if_inc_counter(ifp, IFCOUNTER_OPACKETS, READ4(sc, TXFRAMECOUNT_G));
	if_inc_counter(ifp, IFCOUNTER_OMCASTS, READ4(sc, TXMULTICASTFRAMES_G));
	if_inc_counter(ifp, IFCOUNTER_OERRORS,
	    READ4(sc, TXOVERSIZE_G) + READ4(sc, TXEXCESSDEF) +
	    READ4(sc, TXCARRIERERR) + READ4(sc, TXUNDERFLOWERROR));

	if_inc_counter(ifp, IFCOUNTER_COLLISIONS,
	    READ4(sc, TXEXESSCOL) + READ4(sc, TXLATECOL));

	dwc_clear_stats(sc);
}

static void
dwc_tick(void *arg)
{
	struct dwc_softc *sc;
	struct ifnet *ifp;
	int link_was_up;

	sc = arg;

	DWC_ASSERT_LOCKED(sc);

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
			dwc_txfinish_locked(sc);
		}
	}

	/* Gather stats from hardware counters. */
	dwc_harvest_stats(sc);

	/* Check the media status. */
	link_was_up = sc->link_is_up;
	mii_tick(sc->mii_softc);
	if (sc->link_is_up && !link_was_up)
		dwc_txstart_locked(sc);

	/* Schedule another check one second from now. */
	callout_reset(&sc->dwc_callout, hz, dwc_tick, sc);
}

static void
dwc_init_locked(struct dwc_softc *sc)
{
	struct ifnet *ifp = sc->ifp;
	int reg;

	DWC_ASSERT_LOCKED(sc);

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	dwc_setup_rxfilter(sc);

	/* Initialize DMA and enable transmitters */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_TSF | MODE_OSF | MODE_FUF);
	reg &= ~(MODE_RSF);
	reg |= (MODE_RTC_LEV32 << MODE_RTC_SHIFT);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Flush TX */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_FTF);
	WRITE4(sc, OPERATION_MODE, reg);

       	WRITE4(sc, INTERRUPT_ENABLE, INT_EN_DEFAULT);

	/* Start DMA */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_ST | MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Enable transmitters */
	reg = READ4(sc, MAC_CONFIGURATION);
	reg |= (CONF_JD | CONF_ACS | CONF_BE);
	reg |= (CONF_TE | CONF_RE);
	WRITE4(sc, MAC_CONFIGURATION, reg);

	/* Mask GMAC interrupts */
	WRITE4(sc, INTERRUPT_MASK, 0x207);

	/*
	 * Call mii_mediachg() which will call back into dwc_miibus_statchg()
	 * to set up the remaining config registers based on current media.
	 */
	mii_mediachg(sc->mii_softc);
	callout_reset(&sc->dwc_callout, hz, dwc_tick, sc);
}

static void
dwc_init(void *if_softc)
{
	struct dwc_softc *sc = if_softc;

	DWC_LOCK(sc);
	dwc_init_locked(sc);
	DWC_UNLOCK(sc);
}

inline static uint32_t
dwc_setup_rxdesc(struct dwc_softc *sc, int idx, bus_addr_t paddr)
{
	uint32_t nidx;

	sc->rxdesc_ring[idx].addr = (uint32_t)paddr;
	nidx = next_rxidx(sc, idx);
	sc->rxdesc_ring[idx].addr_next = sc->rxdesc_ring_paddr +	\
	    (nidx * sizeof(struct dwc_hwdesc));
	sc->rxdesc_ring[idx].dmamac_cntl = (MCLBYTES & DDESC_RXCTRL_SIZE1MASK) | \
	    DDESC_RXCTRL_RXCHAIN;

	wmb();
	sc->rxdesc_ring[idx].txrx_status = DDESC_RDES0_OWN;
	wmb();

	return (nidx);
}

static int
dwc_setup_rxbuf(struct dwc_softc *sc, int idx, struct mbuf *m)
{
	struct bus_dma_segment seg;
	int error, nsegs;

	m_adj(m, ETHER_ALIGN);

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (error);
	}

	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
	    BUS_DMASYNC_PREREAD);

	sc->rxbuf_map[idx].mbuf = m;
	dwc_setup_rxdesc(sc, idx, seg.ds_addr);

	return (0);
}

static struct mbuf *
dwc_alloc_mbufcl(struct dwc_softc *sc)
{
	struct mbuf *m;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;

	return (m);
}

static void
dwc_media_status(struct ifnet * ifp, struct ifmediareq *ifmr)
{
	struct dwc_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = sc->mii_softc;
	DWC_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	DWC_UNLOCK(sc);
}

static int
dwc_media_change_locked(struct dwc_softc *sc)
{

	return (mii_mediachg(sc->mii_softc));
}

static int
dwc_media_change(struct ifnet * ifp)
{
	struct dwc_softc *sc;
	int error;

	sc = ifp->if_softc;

	DWC_LOCK(sc);
	error = dwc_media_change_locked(sc);
	DWC_UNLOCK(sc);
	return (error);
}

static const uint8_t nibbletab[] = {
	/* 0x0 0000 -> 0000 */  0x0,
	/* 0x1 0001 -> 1000 */  0x8,
	/* 0x2 0010 -> 0100 */  0x4,
	/* 0x3 0011 -> 1100 */  0xc,
	/* 0x4 0100 -> 0010 */  0x2,
	/* 0x5 0101 -> 1010 */  0xa,
	/* 0x6 0110 -> 0110 */  0x6,
	/* 0x7 0111 -> 1110 */  0xe,
	/* 0x8 1000 -> 0001 */  0x1,
	/* 0x9 1001 -> 1001 */  0x9,
	/* 0xa 1010 -> 0101 */  0x5,
	/* 0xb 1011 -> 1101 */  0xd,
	/* 0xc 1100 -> 0011 */  0x3,
	/* 0xd 1101 -> 1011 */  0xb,
	/* 0xe 1110 -> 0111 */  0x7,
	/* 0xf 1111 -> 1111 */  0xf, };

static uint8_t
bitreverse(uint8_t x)
{

	return (nibbletab[x & 0xf] << 4) | nibbletab[x >> 4];
}

static void
dwc_setup_rxfilter(struct dwc_softc *sc)
{
	struct ifmultiaddr *ifma;
	struct ifnet *ifp;
	uint8_t *eaddr;
	uint32_t crc, hashes[2];
//	uint8_t val;
//	int hashbit;
//	int hashreg;
	int ffval;
//	int reg;
	int lo;
	int hi;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;

	/*
	 * Set the multicast (group) filter hash.
	 */
	if ((ifp->if_flags & IFF_ALLMULTI)) {
		ffval = (FRAME_FILTER_PM);
                hashes[0] = 0xffffffff;
                hashes[1] = 0xffffffff;
	} else {
		ffval = (FRAME_FILTER_HMC);
		if_maddr_rlock(ifp);
		TAILQ_FOREACH(ifma, &sc->ifp->if_multiaddrs, ifma_link) {
			if (ifma->ifma_addr->sa_family != AF_LINK)
				continue;
			crc = ether_crc32_le(LLADDR((struct sockaddr_dl *)
				ifma->ifma_addr), ETHER_ADDR_LEN) >> 26;

                        hashes[crc >> 5] |= 1 << (crc & 0x1f);

			/* Take lower 8 bits and reverse it */
			//val = bitreverse(~crc & 0xff);
			//hashreg = (val >> 5);
			//hashbit = (val & 31);

			//reg = READ4(sc, HASH_TABLE_REG(hashreg));
			//reg |= (1 << hashbit);
			//WRITE4(sc, HASH_TABLE_REG(hashreg), reg);

		}
		if_maddr_runlock(ifp);
	}

       	WRITE4(sc, MAC_HASH_LOW, hashes[0]);
       	WRITE4(sc, MAC_HASH_HIGH, hashes[1]);

	/*
	 * Set the individual address filter hash.
	 */
	if (ifp->if_flags & IFF_PROMISC)
		ffval |= (FRAME_FILTER_PR);

	/*
	 * Set the primary address.
	 */
	eaddr = IF_LLADDR(ifp);
	lo = eaddr[0] | (eaddr[1] << 8) | (eaddr[2] << 16) |
	    (eaddr[3] << 24);
	hi = eaddr[4] | (eaddr[5] << 8);
	WRITE4(sc, MAC_ADDRESS_LOW(0), lo);
	WRITE4(sc, MAC_ADDRESS_HIGH(0), hi);

	WRITE4(sc, MAC_FRAME_FILTER, ffval);
}

static int
dwc_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct dwc_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int mask, error;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	error = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		DWC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				if ((ifp->if_flags ^ sc->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI))
					dwc_setup_rxfilter(sc);
			} else {
				if (!sc->is_detaching)
					dwc_init_locked(sc);
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				dwc_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		DWC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			DWC_LOCK(sc);
			dwc_setup_rxfilter(sc);
			DWC_UNLOCK(sc);
		}
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		mii = sc->mii_softc;
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, cmd);
		break;
	case SIOCSIFCAP:
		mask = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (mask & IFCAP_VLAN_MTU) {
			/* No work to do except acknowledge the change took */
			ifp->if_capenable ^= IFCAP_VLAN_MTU;
		}
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static void
dwc_txfinish_locked(struct dwc_softc *sc)
{
	struct dwc_bufmap *bmap;
	struct dwc_hwdesc *desc;
	struct ifnet *ifp;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;

	while (sc->tx_idx_tail != sc->tx_idx_head) {
		desc = &sc->txdesc_ring[sc->tx_idx_tail];
		if ((desc->txrx_status & DDESC_TDES0_OWN) != 0)
			break;

//		desc->dmamac_cntl = ((len << DDESC_TXCTRL_SIZE1SHFT) &	\
//		    DDESC_TXCTRL_SIZE1MASK) | DDESC_TXCTRL_TXLAST |	\
//		    DDESC_TXCTRL_TXFIRST;
		wmb();
		desc->txrx_status |= DDESC_TDES0_OWN;
		wmb();

      		bmap = &sc->txbuf_map[sc->tx_idx_tail];
		bus_dmamap_sync(sc->txbuf_tag, bmap->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->txbuf_tag, bmap->map);
		m_freem(bmap->mbuf);
		bmap->mbuf = NULL;
		dwc_setup_txdesc(sc, sc->tx_idx_tail, 0, 0);
		sc->tx_idx_tail = next_txidx(sc, sc->tx_idx_tail);
	}

	/* If there are no buffers outstanding, muzzle the watchdog. */
	if (sc->tx_idx_tail == sc->tx_idx_head) {
		sc->tx_watchdog_count = 0;
	}
}

static void
dwc_rxfinish_locked(struct dwc_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m0;
	struct mbuf *m;
	int error;
	int rdes0;
	int idx;
	int len;

	ifp = sc->ifp;

	for (;;) {
		idx = sc->rx_idx;

		rdes0 = sc->rxdesc_ring[idx].txrx_status;
		if ((rdes0 & DDESC_RDES0_OWN) != 0)
			break;

		bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->rxbuf_tag, sc->rxbuf_map[idx].map);

		len = (rdes0 >> DDESC_RDES0_FL_SHIFT) & DDESC_RDES0_FL_MASK;
		if (len != 0) {
			m = sc->rxbuf_map[idx].mbuf;
			m->m_pkthdr.rcvif = ifp;
			m->m_pkthdr.len = len;
			m->m_len = len;
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);

			DWC_UNLOCK(sc);
			(*ifp->if_input)(ifp, m);
			DWC_LOCK(sc);

		} else {
			/* XXX Zero-length packet ? */
		}

		if ((m0 = dwc_alloc_mbufcl(sc)) != NULL) {
			if ((error = dwc_setup_rxbuf(sc, idx, m0)) != 0) {
				/*
				 * XXX Now what?
				 * We've got a hole in the rx ring.
				 */
			}
		} else
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);

	        /* make current descriptor valid again */
		sc->rxdesc_ring[idx].txrx_status |= DDESC_RDES0_OWN;

		sc->rx_idx = next_rxidx(sc, sc->rx_idx);
	}
}

static void
dwc_intr(void *arg)
{
	struct dwc_softc *sc;
	uint32_t reg;

	sc = arg;

	DWC_LOCK(sc);

	reg = READ4(sc, INTERRUPT_STATUS);
	if (reg) {
		mii_mediachg(sc->mii_softc);
		READ4(sc, SGMII_RGMII_SMII_CTRL_STATUS);
	}

	reg = READ4(sc, DMA_STATUS);
	if (reg & DMA_STATUS_NIS) {
		if (reg & DMA_STATUS_RI)
			dwc_rxfinish_locked(sc);

		if (reg & DMA_STATUS_TI)
			dwc_txfinish_locked(sc);
	}

	if (reg & DMA_STATUS_AIS) {
		if (reg & DMA_STATUS_FBI) {
			/* Fatal bus error */
			device_printf(sc->dev,
			    "Ethernet DMA error, restarting controller.\n");
			dwc_stop_locked(sc);
			dwc_init_locked(sc);
		}
	}

	WRITE4(sc, DMA_STATUS, reg & DMA_STATUS_INTR_MASK);
	DWC_UNLOCK(sc);
}

static int
setup_dma(struct dwc_softc *sc)
{
	struct mbuf *m;
	int error;
	int nidx;
	int idx;

	/*
	 * Set up TX descriptor ring, descriptors, and dma maps.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    DWC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
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
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->txdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate TX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->txdesc_tag, sc->txdesc_map,
	    sc->txdesc_ring, TX_DESC_SIZE, dwc_get1paddr,
	    &sc->txdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load TX descriptor ring map.\n");
		goto out;
	}

	for (idx = 0; idx < TX_DESC_COUNT; idx++) {
		sc->txdesc_ring[idx].dmamac_cntl = DDESC_TXCTRL_TXCHAIN;
		sc->txdesc_ring[idx].txrx_status = 0;
		nidx = next_txidx(sc, idx);
		sc->txdesc_ring[idx].addr_next = sc->txdesc_ring_paddr + \
		    (nidx * sizeof(struct dwc_hwdesc));
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    1, 0,			/* alignment, boundary */
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

	for (idx = 0; idx < TX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->txbuf_tag, BUS_DMA_COHERENT,
		    &sc->txbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create TX buffer DMA map.\n");
			goto out;
		}
		dwc_setup_txdesc(sc, idx, 0, 0);
	}

	/*
	 * Set up RX descriptor ring, descriptors, dma maps, and mbufs.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    DWC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
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
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->rxdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate RX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->rxdesc_tag, sc->rxdesc_map,
	    sc->rxdesc_ring, RX_DESC_SIZE, dwc_get1paddr,
	    &sc->rxdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load RX descriptor ring map.\n");
		goto out;
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
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

	for (idx = 0; idx < RX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->rxbuf_tag, BUS_DMA_COHERENT,
		    &sc->rxbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create RX buffer DMA map.\n");
			goto out;
		}
		if ((m = dwc_alloc_mbufcl(sc)) == NULL) {
			device_printf(sc->dev, "Could not alloc mbuf\n");
			error = ENOMEM;
			goto out;
		}
		if ((error = dwc_setup_rxbuf(sc, idx, m)) != 0) {
			device_printf(sc->dev,
			    "could not create new RX buffer.\n");
			goto out;
		}
	}

out:
	if (error != 0)
		return (ENXIO);

	return (0);
}

static int
dwc_get_hwaddr(struct dwc_softc *sc, uint8_t *hwaddr)
{
	int rnd;
	int lo;
	int hi;

	/*
	 * Try to recover a MAC address from the running hardware. If there's
	 * something non-zero there, assume the bootloader did the right thing
	 * and just use it.
	 *
	 * Otherwise, set the address to a convenient locally assigned address,
	 * 'bsd' + random 24 low-order bits.  'b' is 0x62, which has the locally
	 * assigned bit set, and the broadcast/multicast bit clear.
	 */
	lo = READ4(sc, MAC_ADDRESS_LOW(0));
	hi = READ4(sc, MAC_ADDRESS_HIGH(0)) & 0xffff;
	if ((lo != 0xffffffff) || (hi != 0xffff)) {
		hwaddr[0] = (lo >>  0) & 0xff;
		hwaddr[1] = (lo >>  8) & 0xff;
		hwaddr[2] = (lo >> 16) & 0xff;
		hwaddr[3] = (lo >> 24) & 0xff;
		hwaddr[4] = (hi >>  0) & 0xff;
		hwaddr[5] = (hi >>  8) & 0xff;
	} else {
		rnd = arc4random() & 0x00ffffff;
		hwaddr[0] = 'b';
		hwaddr[1] = 's';
		hwaddr[2] = 'd';
		hwaddr[3] = rnd >> 16;
		hwaddr[4] = rnd >>  8;
		hwaddr[5] = rnd >>  0;
	}

	return (0);
}

static int
dwc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "snps,dwmac"))
		return (ENXIO);

	device_set_desc(dev, "Gigabit Ethernet Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
dwc_attach(device_t dev)
{
	uint8_t macaddr[ETHER_ADDR_LEN];
	struct dwc_softc *sc;
	struct ifnet *ifp;
	int error;
	int reg;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->mii_clk = MII_CLK_VAL;
	sc->rx_idx = 0;

	sc->txcount = TX_DESC_COUNT;

	if (bus_alloc_resources(dev, dwc_spec, sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Memory interface */
	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	/* SID */
	sc->sid_bst = rman_get_bustag(sc->res[1]);
	sc->sid_bsh = rman_get_bushandle(sc->res[1]);

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev),
	    MTX_NETWORK_LOCK, MTX_DEF);

	callout_init_mtx(&sc->dwc_callout, &sc->mtx, 0);

	/* Setup interrupt handler. */
	error = bus_setup_intr(dev, sc->res[2], INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, dwc_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		return (ENXIO);
	}

	/* Read MAC before reset */
	if (dwc_get_hwaddr(sc, macaddr)) {
		device_printf(sc->dev, "can't get mac\n");
		return (ENXIO);
	}

	/* Activate clock */
	a10_clk_gmac_activate();

	/* Configure pin mux settings for GMAC */
//	for (i = 0; i <= 16; i++) {

		/* skip unused pins in RGMII mode */
//		if (i == 9 || i == 14)
//			continue;

//		a10_gmac_gpio_config(i);
//		sunxi_gpio_set_drv(pin, 3);
//	}
	
	/* Reset */
	reg = READ4(sc, BUS_MODE);
	reg |= (BUS_MODE_SWR);
	WRITE4(sc, BUS_MODE, reg);

	for (i = 0; i < 100; i++) {
		if ((READ4(sc, BUS_MODE) & BUS_MODE_SWR) == 0)
			break;
		DELAY(10);
	}
	if (i == 0) {
		device_printf(sc->dev, "Can't reset DWC.\n");
		return (ENXIO);
	}

	/* Set hw address again */
	reg = READ4_SID(sc, 0x0);//read the chipID
	macaddr[0] = 0x02; /* Non OUI / registered MAC address */
	macaddr[1] = (reg >> 0) & 0xff;
	reg = READ4_SID(sc, 0x0c);
	macaddr[2] = (reg >> 24) & 0xff;
	macaddr[3] = (reg >> 16) & 0xff;
	macaddr[4] = (reg >> 8) & 0xff;
	macaddr[5] = (reg >> 0) & 0xff;

	printf("MAC address: %s\n", ether_sprintf(macaddr));
	
	reg = READ4(sc, BUS_MODE);
	reg |= (BUS_MODE_EIGHTXPBL);
	reg |= (BUS_MODE_PBL_BEATS_8 << BUS_MODE_PBL_SHIFT);
	WRITE4(sc, BUS_MODE, reg);

	/*
	 * DMA must be stop while changing descriptor list addresses.
	 */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_ST | MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);

	if (setup_dma(sc))
	        return (ENXIO);

	/* Mask GMAC interrupts */
	//WRITE4(sc, INTERRUPT_MASK, 0x207);

	/* Setup addresses */
	WRITE4(sc, RX_DESCR_LIST_ADDR, sc->rxdesc_ring_paddr);
	WRITE4(sc, TX_DESCR_LIST_ADDR, sc->txdesc_ring_paddr);

	/* Set up the ethernet interface. */
	sc->ifp = ifp = if_alloc(IFT_ETHER);

	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;
	ifp->if_start = dwc_txstart;
	ifp->if_ioctl = dwc_ioctl;
	ifp->if_init = dwc_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, TX_DESC_COUNT - 1);
	ifp->if_snd.ifq_drv_maxlen = TX_DESC_COUNT - 1;
	IFQ_SET_READY(&ifp->if_snd);
	ifp->if_hdrlen = sizeof(struct ether_vlan_header);

	/* Attach the mii driver. */
	error = mii_attach(dev, &sc->miibus, ifp, dwc_media_change,
	    dwc_media_status, BMSR_DEFCAPMASK, MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);

	if (error != 0) {
		device_printf(dev, "PHY attach failed\n");
		return (ENXIO);
	}
	sc->mii_softc = device_get_softc(sc->miibus);

	/* All ready to run, attach the ethernet interface. */
	ether_ifattach(ifp, macaddr);
	sc->is_attached = true;

	return (0);
}

static int
dwc_miibus_read_reg(device_t dev, int phy, int reg)
{
	struct dwc_softc *sc;
	uint16_t mii;
	size_t cnt;
	int rv = 0;

	sc = device_get_softc(dev);

	mii = ((phy << GMII_ADDRESS_PA_SHIFT) & GMII_ADDRESS_PA_MASK)
	    | ((reg << GMII_ADDRESS_GR_SHIFT) & GMII_ADDRESS_GR_MASK)
	    | (sc->mii_clk << GMII_ADDRESS_CR_SHIFT)
	    | GMII_ADDRESS_GB; /* Busy flag */

	WRITE4(sc, GMII_ADDRESS, mii);

	for (cnt = 0; cnt < 1000; cnt++) {
		if (!(READ4(sc, GMII_ADDRESS) & GMII_ADDRESS_GB)) {
			rv = READ4(sc, GMII_DATA);
			break;
		}
		DELAY(10);
	}

	return rv;
}

static int
dwc_miibus_write_reg(device_t dev, int phy, int reg, int val)
{
	struct dwc_softc *sc;
	uint16_t mii;
	size_t cnt;

	sc = device_get_softc(dev);

	mii = ((phy << GMII_ADDRESS_PA_SHIFT) & GMII_ADDRESS_PA_MASK)
	    | ((reg << GMII_ADDRESS_GR_SHIFT) & GMII_ADDRESS_GR_MASK)
	    | (sc->mii_clk << GMII_ADDRESS_CR_SHIFT)
	    | GMII_ADDRESS_GB | GMII_ADDRESS_GW;

	WRITE4(sc, GMII_DATA, val);
	WRITE4(sc, GMII_ADDRESS, mii);

	for (cnt = 0; cnt < 1000; cnt++) {
		if (!(READ4(sc, GMII_ADDRESS) & GMII_ADDRESS_GB)) {
			break;
                }
		DELAY(10);
	}

	return (0);
}

static void
dwc_miibus_statchg(device_t dev)
{
	struct dwc_softc *sc;
	struct mii_data *mii;
	int reg;

	/*
	 * Called by the MII bus driver when the PHY establishes
	 * link to set the MAC interface registers.
	 */

	sc = device_get_softc(dev);

	DWC_ASSERT_LOCKED(sc);

	mii = sc->mii_softc;

	if (mii->mii_media_status & IFM_ACTIVE)
		sc->link_is_up = true;
	else
		sc->link_is_up = false;

	reg = READ4(sc, MAC_CONFIGURATION);
	switch (IFM_SUBTYPE(mii->mii_media_active)) {
	case IFM_1000_T:
	case IFM_1000_SX:
		reg &= ~(CONF_FES | CONF_PS);
		break;
	case IFM_100_TX:
		reg |= (CONF_FES | CONF_PS);
		break;
	case IFM_10_T:
		reg &= ~(CONF_FES);
		reg |= (CONF_PS);
		break;
	case IFM_NONE:
		sc->link_is_up = false;
		return;
	default:
		sc->link_is_up = false;
		device_printf(dev, "Unsupported media %u\n",
		    IFM_SUBTYPE(mii->mii_media_active));
		return;
	}
	if ((IFM_OPTIONS(mii->mii_media_active) & IFM_FDX) != 0)
		reg |= (CONF_DM);
	else
		reg &= ~(CONF_DM);
	WRITE4(sc, MAC_CONFIGURATION, reg);
}

static device_method_t dwc_methods[] = {
	DEVMETHOD(device_probe,		dwc_probe),
	DEVMETHOD(device_attach,	dwc_attach),

	/* MII Interface */
	DEVMETHOD(miibus_readreg,	dwc_miibus_read_reg),
	DEVMETHOD(miibus_writereg,	dwc_miibus_write_reg),
	DEVMETHOD(miibus_statchg,	dwc_miibus_statchg),

	{ 0, 0 }
};

static driver_t dwc_driver = {
	"dwc",
	dwc_methods,
	sizeof(struct dwc_softc),
};

static devclass_t dwc_devclass;

DRIVER_MODULE(dwc, simplebus, dwc_driver, dwc_devclass, 0, 0);
DRIVER_MODULE(miibus, dwc, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(dwc, ether, 1, 1, 1);
MODULE_DEPEND(dwc, miibus, 1, 1, 1);
