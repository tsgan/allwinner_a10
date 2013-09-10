/*-
 * Copyright (c) 2013 Alexander Fedorov <alexander.fedorov@rtlservice.com>
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
__FBSDID("$FreeBSD: head/sys/arm/lpc/lpc_mmc.c 239278 2012-08-15 05:37:10Z gonzo $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>

#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include <arm/allwinner/a10_clk.h>
#include <arm/allwinner/a10_mmc.h>

struct a10_mmc_softc {
	device_t		a10_dev;
	struct mtx		a10_mtx;
	struct resource *	a10_mem_res;
	struct resource *	a10_irq_res;
	bus_space_tag_t		a10_bst;
	bus_space_handle_t	a10_bsh;
	void *			a10_intrhand;
	struct mmc_host		a10_host;
	struct mmc_request *	a10_req;
	int			a10_bus_busy;
	uint8_t wait;
	uint32_t error;
	enum{
		A10_MMC_ST_UNK = 0,
		A10_MMC_ST_WAIT_CMD_DONE,
		A10_MMC_ST_WAIT_DATA_DONE
	}state;
};

static int a10_mmc_probe(device_t);
static int a10_mmc_attach(device_t);
static int a10_mmc_detach(device_t);
static void a10_mmc_intr(void *);

static int a10_mmc_update_ios(device_t, device_t);
static int a10_mmc_request(device_t, device_t, struct mmc_request *);
static int a10_mmc_get_ro(device_t, device_t);
static int a10_mmc_acquire_host(device_t, device_t);
static int a10_mmc_release_host(device_t, device_t);

#define	a10_mmc_lock(_sc)						\
    mtx_lock(&_sc->a10_mtx);
#define	a10_mmc_unlock(_sc)						\
    mtx_unlock(&_sc->a10_mtx);
#define	a10_mmc_read_4(_sc, _reg)					\
    bus_space_read_4(_sc->a10_bst, _sc->a10_bsh, _reg)
#define	a10_mmc_write_4(_sc, _reg, _value)				\
    bus_space_write_4(_sc->a10_bst, _sc->a10_bsh, _reg, _value)

static int
a10_mmc_reset(struct a10_mmc_softc *sc)
{
    uint32_t rval = a10_mmc_read_4(sc, MMC_GCTRL) | MMC_SOFT_RESET_B | MMC_FIFO_RESET_B | MMC_DMA_RESET_B;
    int time = 0xffff;

	a10_mmc_write_4(sc, MMC_GCTRL, rval);
    while((a10_mmc_read_4(sc, MMC_GCTRL) & (MMC_SOFT_RESET_B | MMC_FIFO_RESET_B | MMC_DMA_RESET_B)) && time--);
    if (time <= 0){
		device_printf(sc->a10_dev, "Reset failed\n");
        return -1;
    }
    return 0;
}

static void
a10_mmc_int_enable(struct a10_mmc_softc *sc)
{
    a10_mmc_write_4(sc, MMC_GCTRL, a10_mmc_read_4(sc, MMC_GCTRL)|MMC_INT_ENABLE_B);
}

static int
a10_mmc_update_clk(struct a10_mmc_softc *sc)
{
	unsigned int cmd;
	unsigned timeout = 0xfffff;

	cmd = MMC_Start | MMC_UPCLKOnly | MMC_WaitPreOver;
  	a10_mmc_write_4(sc, MMC_CMDR, cmd);
	while((a10_mmc_read_4(sc, MMC_CMDR) & MMC_Start) && timeout--);
	if (!timeout)
		return -1;

	return 0;
}

static int
a10_mmc_config_clock(struct a10_mmc_softc *sc, unsigned div)
{
	unsigned rval = a10_mmc_read_4(sc, MMC_CLKCR);

	/*
	 * CLKCREG[7:0]: divider
	 * CLKCREG[16]:  on/off
	 * CLKCREG[17]:  power save
	 */

	/* Disable Clock */
	rval &= ~MMC_CARD_CLK_ON;
	a10_mmc_write_4(sc, MMC_CLKCR, rval);
	if(a10_mmc_update_clk(sc))
		return -1;

	/* Change Divider Factor */
	rval &= ~(0xFF);
	rval |= div;
	a10_mmc_write_4(sc, MMC_CLKCR, rval);
	if(a10_mmc_update_clk(sc))
		return -1;

	/* Enable Clock */
	rval |= MMC_CARD_CLK_ON;
	a10_mmc_write_4(sc, MMC_CLKCR, rval);
	if(a10_mmc_update_clk(sc))
		return -1;

	return 0;
}

static int
a10_mmc_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "allwinner,sun4i-mmc"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner Integrated MMC/SD controller");
	return (BUS_PROBE_DEFAULT);
}

static int
a10_mmc_attach(device_t dev)
{
	struct a10_mmc_softc *sc = device_get_softc(dev);
	device_t child;
	int rid;

	sc->a10_dev = dev;
	sc->a10_req = NULL;

	mtx_init(&sc->a10_mtx, device_get_nameunit(sc->a10_dev), "a10_mmc", MTX_DEF);

	rid = 0;
	sc->a10_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->a10_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->a10_bst = rman_get_bustag(sc->a10_mem_res);
	sc->a10_bsh = rman_get_bushandle(sc->a10_mem_res);

	rid = 0;
	sc->a10_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!sc->a10_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->a10_mem_res);
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->a10_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, a10_mmc_intr, sc, &sc->a10_intrhand))
	{
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->a10_mem_res);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->a10_irq_res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	a10_clk_mmc_activate();

	/* Reset controller */
	a10_mmc_reset(sc);

    /* config DMA/Interrupt Trigger threshold */
  //  a10_mmc_write_4(sc, MMC_FTRGL, 0x70008);

    /* config timeout register */
    a10_mmc_write_4(sc, MMC_TMOUT, 0xffffffff);

    /* clear interrupt flags */
    a10_mmc_write_4(sc, MMC_RINTR, 0xffffffff);

    a10_mmc_write_4(sc, MMC_DBGC, 0xdeb);
    a10_mmc_write_4(sc, MMC_FUNS, 0xceaa0000);

	sc->a10_host.f_min = 400000;
	sc->a10_host.f_max = 52000000;
	sc->a10_host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->a10_host.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_HSPEED;
	sc->a10_host.mode = mode_sd;

	device_set_ivars(dev, &sc->a10_host);

	child = device_add_child(dev, "mmc", 0);
	if (!child) {
		device_printf(dev, "attaching MMC bus failed!\n");
		bus_teardown_intr(dev, sc->a10_irq_res, sc->a10_intrhand);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->a10_mem_res);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->a10_irq_res);
		return (ENXIO);
	}

	device_set_ivars(dev, &sc->a10_host);
	bus_generic_attach(dev);

	return (0);
}

static int
a10_mmc_detach(device_t dev)
{
	return (EBUSY);
}

static int
mmc_trans_data_by_cpu(struct a10_mmc_softc *sc, struct mmc_data *data)
{
	unsigned i;
	unsigned byte_cnt = data->len;
	unsigned *buff;
	unsigned timeout = 0xfffff;

	if (data->flags & MMC_DATA_READ) {
		buff = (unsigned int *)data->data;
		for (i=0; i<(byte_cnt>>2); i++) {
			while(--timeout && (a10_mmc_read_4(sc, MMC_STAS) & MMC_FIFOEmpty));
			if (timeout <= 0) 
				goto out;
			buff[i] = a10_mmc_read_4(sc, MMC_FIFO);
			timeout = 0xfffff;
		}
	} else {
		buff = (unsigned int *)data->data;
		for (i=0; i<(byte_cnt>>2); i++) {
			while(--timeout && (a10_mmc_read_4(sc, MMC_STAS) & MMC_FIFOFull));
			if (timeout <= 0) 
				goto out;
			a10_mmc_write_4(sc, MMC_FIFO, buff[i]);
			timeout = 0xfffff;
		}
	}

out:
	if (timeout <= 0)
		return -1;

	return 0;
}

static void
a10_req_ok(struct a10_mmc_softc *sc)
{
	struct mmc_command *cmd = sc->a10_req->cmd;
	uint32_t resp_status;;

	do{
		resp_status = a10_mmc_read_4(sc, MMC_STAS);
	}while(resp_status & MMC_CardDataBusy);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = a10_mmc_read_4(sc, MMC_RESP3);
		cmd->resp[1] = a10_mmc_read_4(sc, MMC_RESP2);
		cmd->resp[2] = a10_mmc_read_4(sc, MMC_RESP1);
		cmd->resp[3] = a10_mmc_read_4(sc, MMC_RESP0);
	} else {
		cmd->resp[0] = a10_mmc_read_4(sc, MMC_RESP0);
	}

	sc->a10_req->cmd->error = MMC_ERR_NONE;
	sc->a10_req->done(sc->a10_req);
	sc->a10_req = NULL;
}

static void
a10_req_err(struct a10_mmc_softc *sc)
{
	struct mmc_command *cmd = sc->a10_req->cmd;
	device_printf(sc->a10_dev, "req error\n");
	cmd->error = MMC_ERR_TIMEOUT;
	sc->a10_req->done(sc->a10_req);
	sc->a10_req = NULL;
}

static void
a10_mmc_intr(void *arg)
{
	struct a10_mmc_softc *sc = (struct a10_mmc_softc *)arg;
	uint32_t rint = a10_mmc_read_4(sc, MMC_RINTR);
	uint32_t imask = a10_mmc_read_4(sc, MMC_IMASK);

	imask &= ~rint;
	a10_mmc_write_4(sc, MMC_IMASK, imask);
	a10_mmc_write_4(sc, MMC_RINTR, rint);

	if(sc->a10_req == NULL){
		device_printf(sc->a10_dev, "req == NULL, rint: 0x%08X\n", rint);
	}

	struct mmc_command *cmd = sc->a10_req->cmd;

//	if (cmd->data) {
//		if (cmd->data->flags & MMC_DATA_WRITE){
//			device_printf(sc->a10_dev, "rint: 0x%08X, imask: 0x%08X\n", rint, imask);
//		}
//	}

	if(rint & MMC_IntErrBit){
		device_printf(sc->a10_dev, "error rint: 0x%08X\n", rint);
		a10_req_err(sc);
		return;
	}

	if(!cmd->data && (rint & MMC_CmdDone)){
		a10_req_ok(sc);
		return;
	}

	if(cmd->data && (rint & MMC_DataOver)){
		a10_req_ok(sc);
		return;
	}

	if(cmd->data->flags & MMC_DATA_READ){
		int ret = mmc_trans_data_by_cpu(sc, cmd->data);
		if(ret){
			device_printf(sc->a10_dev, "data read error, rint: 0x%08X\n", rint);
			a10_req_err(sc);

		}
	}

	if(cmd->data->flags & MMC_DATA_WRITE){
		if(rint & MMC_TxDataReq){
			int ret = mmc_trans_data_by_cpu(sc, cmd->data);
			if(ret){
				device_printf(sc->a10_dev, "data write error, rint: 0x%08X\n", rint);
				a10_req_err(sc);
			}
		}
	}	
}

static int
a10_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	unsigned int cmdreg = 0x80000000;
	struct a10_mmc_softc *sc = device_get_softc(bus);
	struct mmc_command *cmd = req->cmd;
	uint32_t imask = MMC_CmdDone | MMC_IntErrBit;

	a10_mmc_lock(sc);
	if (sc->a10_req){
		a10_mmc_unlock(sc);
		return (EBUSY);
	}

	sc->a10_req = req;

	if (cmd->opcode == MMC_GO_IDLE_STATE)
		cmdreg |= MMC_SendInitSeq;
	if (cmd->flags & MMC_RSP_PRESENT)
		cmdreg |= MMC_RspExp;
	if (cmd->flags & MMC_RSP_136)
		cmdreg |= MMC_LongRsp;
	if (cmd->flags & MMC_RSP_CRC)
		cmdreg |= MMC_CheckRspCRC;

	if (cmd->data) {
		cmdreg |= MMC_DataExp | MMC_WaitPreOver;
		imask |= MMC_DataOver;
		if (cmd->data->flags & MMC_DATA_WRITE){
			cmdreg |= MMC_Write;
			imask |= MMC_TxDataReq;
		}else{
			imask |= MMC_RxDataReq;
		}

//		if (data->blocks > 1)
//			cmdreg |= MMC_SendAutoStop;

		a10_mmc_write_4(sc, MMC_BLKSZ, cmd->data->len);
		a10_mmc_write_4(sc, MMC_BCNTR, cmd->data->len);

		/* Choose access by AHB */
		a10_mmc_write_4(sc, MMC_GCTRL, a10_mmc_read_4(sc, MMC_GCTRL)|0x80000000);
	}

	if (cmd->flags & MMC_RSP_BUSY) {
		imask |= MMC_DataTimeout;
	}

	/* Enable interrupts and set IMASK */
	a10_mmc_write_4(sc, MMC_IMASK, imask);
	a10_mmc_int_enable(sc);

	a10_mmc_write_4(sc, MMC_CARG, cmd->arg);
	a10_mmc_write_4(sc, MMC_CMDR, cmdreg|cmd->opcode);
	sc->state = A10_MMC_ST_WAIT_CMD_DONE;

	a10_mmc_unlock(sc);
	return 0;
}

static int
a10_mmc_read_ivar(device_t bus, device_t child, int which, 
    uintptr_t *result)
{
	struct a10_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->a10_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->a10_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->a10_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->a10_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->a10_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->a10_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->a10_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->a10_host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->a10_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->a10_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->a10_host.ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->a10_host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = 1;
		break;
	}

	return (0);
}

static int
a10_mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct a10_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->a10_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->a10_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->a10_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->a10_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->a10_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->a10_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->a10_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->a10_host.ios.vdd = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}
	return (0);
}

static int
a10_mmc_update_ios(device_t bus, device_t child)
{
	struct a10_mmc_softc *sc = device_get_softc(bus);
	struct mmc_ios *ios = &sc->a10_host.ios;
	unsigned int clkdiv = 0;

	/* Change clock first */
	clkdiv = (0x04dd1e00 + (ios->clock>>1))/ios->clock/2;
	if (ios->clock) {
		if (a10_mmc_config_clock(sc, clkdiv)) {
			return -1;
		}
	}

	/* Set the bus width */
	switch (ios->bus_width) {
		case bus_width_1:
			a10_mmc_write_4(sc, MMC_WIDTH, MMC_WIDTH1);
			break;
		case bus_width_4:
			a10_mmc_write_4(sc, MMC_WIDTH, MMC_WIDTH4);
			break;
		case bus_width_8:
			a10_mmc_write_4(sc, MMC_WIDTH, MMC_WIDTH8);
			break;
	}

	return (0);
}

static int
a10_mmc_get_ro(device_t bus, device_t child)
{
	return (0);
}

static int
a10_mmc_acquire_host(device_t bus, device_t child)
{
	struct a10_mmc_softc *sc = device_get_softc(bus);
	int error = 0;

	a10_mmc_lock(sc);
	while (sc->a10_bus_busy)
		error = mtx_sleep(sc, &sc->a10_mtx, PZERO, "mmcah", 0);

	sc->a10_bus_busy++;
	a10_mmc_unlock(sc);
	return (error);
}

static int
a10_mmc_release_host(device_t bus, device_t child)
{
	struct a10_mmc_softc *sc = device_get_softc(bus);

	a10_mmc_lock(sc);
	sc->a10_bus_busy--;
	wakeup(sc);
	a10_mmc_unlock(sc);
	return (0);
}

static device_method_t a10_mmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a10_mmc_probe),
	DEVMETHOD(device_attach,	a10_mmc_attach),
	DEVMETHOD(device_detach,	a10_mmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	a10_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	a10_mmc_write_ivar),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	a10_mmc_update_ios),
	DEVMETHOD(mmcbr_request,	a10_mmc_request),
	DEVMETHOD(mmcbr_get_ro,		a10_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	a10_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host,	a10_mmc_release_host),

	{ 0, 0 }
};

static devclass_t a10_mmc_devclass;

static driver_t a10_mmc_driver = {
	"a10_mmc",
	a10_mmc_methods,
	sizeof(struct a10_mmc_softc),
};

DRIVER_MODULE(a10_mmc, simplebus, a10_mmc_driver, a10_mmc_devclass, 0, 0);

