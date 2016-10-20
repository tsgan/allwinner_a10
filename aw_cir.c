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
 * Allwinner sunXi IR controller
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>

#define	READ_1(_sc, _r)			bus_read_1((_sc)->res[0], (_r))
#define	READ(_sc, _r)			bus_read_4((_sc)->res[0], (_r))
#define	WRITE(_sc, _r, _v)		bus_write_4((_sc)->res[0], (_r), (_v))

/* IR Control Reg */
#define	IR_CTL				0x00
/* Global Enable bit */
#define	 IR_CTL_GEN			(1 << 0)
/* RX block enable bit */
#define	 IR_CTL_RXEN			(1 << 1)
/* CIR mode bits */
#define	 IR_CTL_MD			(1 << 4) | (1 << 5)

/* RX Config Reg */
#define	IR_RXCTL			0x10
/* Pulse Polarity Invert flag */
#define	 IR_RXCTL_RPPI			(1 << 2)

/* RX Data Reg */
#define	IR_RXFIFO			0x20

/* RX Interrupt Control Reg */
#define	IR_RXINT			0x2C
/* RX FIFO Overflow bit */
#define	 IR_RXINT_ROI_EN		(1 << 0)
/* RX Packet End bit */
#define	 IR_RXINT_RPEI_EN		(1 << 1)
/* RX FIFO Data Available bit */
#define	 IR_RXINT_RAI_EN		(1 << 4)
/* RX FIFO available received byte level */
#define	 IR_RXINT_RAL(val)		((val) << 8)

/* RX Interrupt Status Reg */
#define	IR_RXSTA			0x30
/* RX FIFO Get Available Counter */
#define	 IR_RXSTA_COUNTER(val)		(((val) >> 8) & (sc->fifo_size * 2 - 1))
/* Clear all interrupt status */
#define	 IR_RXSTA_CLEARALL		0xff

/* IR Sample Configure Reg */
#define	IR_CIR				0x34
/* Filter Threshold = 8*42.7 = ~341us	< 500us */
#define	 IR_RXFILT_VAL			(((8) & 0x3f) << 2)
/* Idle Threshold = (2+1)*128*42.7 = ~16.4ms > 9ms */
#define	 IR_RXIDLE_VAL			(((2) & 0xff) << 8)

/* Required frequency for IR0 or IR1 clock in CIR mode */
#define	IR_BASE_CLK			8000000
/* Frequency after IR internal divider  */
#define	IR_CLK				(IR_BASE_CLK / 64)
/* Sample period in ns */
#define IR_SAMPLE			(1000000000ul / IR_CLK)
/* Noise threshold in samples  */
#define	IR_RXNOISE			1
/* Idle threshold in samples */
#define	IR_RXIDLE			20
/* Time after which device stops sending data in ms */
#define	IR_TIMEOUT			120

/* Active threshold */
#define	IR_ACTIVE_T			((0 & 0xff) << 16)
#define	IR_ACTIVE_T_C			((1 & 0xff) << 23)
/* 80*42.7 = ~3.4ms, Lead1(4.5ms) > IR_L1_MIN */
#define	IR_L1_MIN			(80)
/* 40*42.7 = ~1.7ms, Lead0(4.5ms) Lead0R(2.25ms)> IR_L0_MIN */
#define	IR_L0_MIN			(40)
/* 26*42.7 = ~1109us ~= 561*2, Pluse < IR_PMAX */
#define	IR_PMAX				(26)
/* 26*42.7 = ~1109us ~= 561*2, D1 > IR_DMID, D0 =< IR_DMID */
#define	IR_DMID				(26)
/* 53*42.7 = ~2263us ~= 561*4, D < IR_DMAX */
#define	IR_DMAX				(53)

#define	IR_ERROR_CODE			(0xffffffff)
#define	IR_REPEAT_CODE			(0x00000000)

#define	A10_IR				1
#define	A13_IR				2

struct aw_ir_softc {
	device_t		dev;
	struct resource		*res[2];
	void *			intrhand;
	struct callout		co;
	int			ticks;
	int			fifo_size;
	struct aw_ir_raw_buffer *rawbuf;
	unsigned long		code;
	unsigned long		dcnt;	/*Packet Count*/
#define	IR_RAW_BUF_SIZE		128
	unsigned char		buf[IR_RAW_BUF_SIZE];
	int			timer_used;
	struct evdev_dev	*sc_evdev;
};

static struct resource_spec aw_ir_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun4i-a10-ir",	A10_IR },
	{ "allwinner,sun5i-a13-ir",	A13_IR },
	{ NULL,				0 }
};

static void
aw_ir_reset_rawbuffer(struct aw_ir_softc *sc)
{

	sc->dcnt = 0;
}

static void
aw_ir_write_rawbuffer(struct aw_ir_softc *sc, unsigned char data)
{

	if (sc->dcnt < IR_RAW_BUF_SIZE)
		sc->buf[sc->dcnt++] = data;
	else
		device_printf(sc->dev, "aw_ir_write_rawbuffer: IR RX Buffer Full!\n");
}

static unsigned char
aw_ir_read_rawbuffer(struct aw_ir_softc *sc)
{
	unsigned char data = 0x00;

	if (sc->dcnt > 0)
		data = sc->buf[--sc->dcnt];

	return data;
}

static int
aw_ir_rawbuffer_empty(struct aw_ir_softc *sc)
{

	return (sc->dcnt == 0);
}

static int
aw_ir_rawbuffer_full(struct aw_ir_softc *sc)
{

	return (sc->dcnt >= IR_RAW_BUF_SIZE);
}

static void
print_err_code(struct aw_ir_softc *sc)
{
	unsigned long i = 0;

	device_printf(sc->dev, "error code:\n");
	for (i = 0; i < sc->dcnt; i++) {
		device_printf(sc->dev, "%d:%d  ",
		    ((sc->buf[i] & 0x80) >> 7), (sc->buf[i] & 0x7f));
		if((i + 1) % 6 == 0)
			device_printf(sc->dev, "\n");
	}
}

static unsigned long
aw_ir_packet_handler(struct aw_ir_softc *sc)
{
	unsigned long len;
	unsigned char val = 0x00;
	unsigned char last = 0x00;
	unsigned long code = 0;
	int bitCnt = 0;
	unsigned long i=0;
	unsigned int active_delay = 0;

	device_printf(sc->dev, "dcnt = %d \n", (int)sc->dcnt);

	/* Find Lead '1' */
	active_delay = (IR_ACTIVE_T + 1) * (IR_ACTIVE_T_C ? 128 : 1);
	device_printf(sc->dev, "%d active_delay = %d\n", __LINE__, active_delay);
	len = 0;
	len += (active_delay >> 1);
	for (i = 0; i < sc->dcnt; i++) {
		val = sc->buf[i];
		if (val & 0x80)
			len += val & 0x7f;
		else {
			if (len > IR_L1_MIN)
				break;
			len = 0;
		}
	}

	device_printf(sc->dev, "%d len = %ld\n", __LINE__, len);

	if ((val & 0x80) || (len <=IR_L1_MIN)){
		device_printf(sc->dev, "start 1 error code\n" );
		goto error_code; /* Invalid Code */
	}

	/* Find Lead '0' */
	len = 0;
	for (; i < sc->dcnt; i++) {
		val = sc->buf[i];
		if (val & 0x80) {
			if(len > IR_L0_MIN)
				break;
			len = 0;
		} else
			len += val & 0x7f;
	}

	if ((!(val & 0x80)) || (len <= IR_L0_MIN)){
		device_printf(sc->dev, "start 0 error code\n");
		goto error_code; /* Invalid Code */
	}

	/* go decoding */
	code = 0;  /* 0 for Repeat Code */
	bitCnt = 0;
	last = 1;
	len = 0;
	for (; i < sc->dcnt; i++) {
		val = sc->buf[i];
		if (last) {
			if (val & 0x80)
				len += val & 0x7f;
			else {
				if (len > IR_PMAX) {
					/* Error Pulse */
					device_printf(sc->dev,
					    "len > IR_PMAX\n");
					goto error_code;
				}
				last = 0;
				len = val & 0x7f;
			}
		} else {
			if (val & 0x80) {
				if (len > IR_DMAX) {
					/* Error Distant */
					device_printf(sc->dev,
					    "len > IR_DMAX\n");
					goto error_code;
				} else {
					if (len > IR_DMID)  {
						/* data '1'*/
						code |= 1 << bitCnt;
					}
					bitCnt++;
					if (bitCnt == 32)
						break;  /* decode over */
				}
				last = 1;
				len = val & 0x7f;
			} else
				len += val & 0x7f;
		}
	}
	return (code);

error_code:
	print_err_code(sc);

	return (IR_ERROR_CODE);
}

static int
aw_ir_code_valid(unsigned long code)
{
	unsigned long v1, v2;

	v1 = code & 0x00ff00ff;
	v2 = (code & 0xff00ff00) >> 8;

	return (((v1 ^ v2) & 0x00ff0000) == 0x00ff0000);
}

static unsigned char
aw_ir_get_data(struct aw_ir_softc *sc)
{

    return (unsigned char)(READ_1(sc, IR_RXFIFO));
}

static void aw_ir_timer_handle(void *arg)
{
    struct aw_ir_softc *sc = arg;

    sc->timer_used = 0;

    /* Time Out, means that the key is up */
    evdev_push_event(sc->sc_evdev, EV_KEY, (sc->code >> 16) & 0xff, 0);
    evdev_sync(sc->sc_evdev);

    device_printf(sc->dev, "IR KEY TIMER OUT UP\n");

    sc->dcnt = 0;

    device_printf(sc->dev, "aw_ir_timer_handle: timeout\n");
}

static void
aw_ir_intr(void *arg)
{
	struct aw_ir_softc *sc;
	uint32_t val;
	int i, dcnt;
	unsigned long code;
	int code_valid;

	sc = (struct aw_ir_softc *)arg;

	/* Read RX interrupt status */
	val = READ(sc, IR_RXSTA);

	/* Clean all pending interrupt statuses */
	WRITE(sc, IR_RXSTA, val | IR_RXSTA_CLEARALL);

	/* When Rx FIFO Data available or Packet end */
	if (val & (IR_RXINT_RAI_EN | IR_RXINT_RPEI_EN)) {
		/* Get available message count in fifo */
		dcnt  = IR_RXSTA_COUNTER(val);
		/* When there is a data, read FIFO */
		for (i = 0; i < dcnt; i++) {
			if (aw_ir_rawbuffer_full(sc))
				aw_ir_get_data(sc);
			else
				aw_ir_write_rawbuffer(sc, aw_ir_get_data(sc));
		}
	}

	if (val & IR_RXINT_ROI_EN) {
		/* When there is RX FIFO overflow */
		/* flush raw buffer */
		aw_ir_reset_rawbuffer(sc);
	} else if (val & IR_RXINT_RPEI_EN) {
		/* When there is RX Packet end */

		if (aw_ir_rawbuffer_full(sc)) {
			device_printf(sc->dev, "Raw Buffer Full!\n");
			sc->dcnt = 0;
			return;
		}

		code = aw_ir_packet_handler(sc);
		sc->dcnt = 0;
		code_valid = aw_ir_code_valid(code);

		device_printf(sc->dev, "IR code = 0x%lx\n", code);

		if (sc->timer_used) {
			if (code_valid) {  /* the pre-key is released */
				evdev_push_event(sc->sc_evdev,
				    EV_KEY, (sc->code >> 16) & 0xff, 0);
				evdev_sync(sc->sc_evdev);

				device_printf(sc->dev, "IR KEY UP\n");

				sc->dcnt = 0;
			}
			if ((code == IR_REPEAT_CODE) || (code_valid)) {
				/* Error, may interfere from other sources */
				callout_reset(&sc->co, sc->ticks,
				    aw_ir_timer_handle, sc);
				// mod_timer(s_timer, jiffies + (HZ / 5));
			}
		} else {
			if (code_valid) {
				callout_reset(&sc->co, sc->ticks,
				    aw_ir_timer_handle, sc);
				// mod_timer(s_timer, jiffies + (HZ / 5));
				sc->timer_used = 1;
			}
		}

		if (sc->timer_used) {
			sc->dcnt++;
			if (sc->dcnt == 1) {
				if (code_valid) {
					/* update saved code */
					sc->code = code;
				}
				device_printf(sc->dev, "IR RAW CODE : %lu\n",
				    (sc->code >> 16) & 0xff);
				evdev_push_event(sc->sc_evdev,
				    EV_KEY, (sc->code >> 16) & 0xff, 1);

				device_printf(sc->dev, "IR CODE : %lu\n",
				    (sc->code >> 16) & 0xff);
				evdev_sync(sc->sc_evdev);

				device_printf(sc->dev, "IR KEY VALE %lu\n",
				    (sc->code >> 16) & 0xff);
			}
		}
		device_printf(sc->dev,
		    "RX Packet End, code=0x%x, ir_code=0x%x, timer_used=%d\n",
		    (int)code, (int)sc->code, sc->timer_used);
	}
}

static int
aw_ir_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner CIR controller");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_ir_attach(device_t dev)
{
	struct aw_ir_softc *sc;
	hwreset_t rst_apb;
	clk_t clk_ir, clk_gate;
	int err;
	uint32_t val = 0;

	clk_ir = clk_gate = NULL;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, aw_ir_spec, sc->res) != 0) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	switch (ofw_bus_search_compatible(dev, compat_data)->ocd_data) {
	case A10_IR:
		sc->fifo_size = 16;
		break;
	case A13_IR:
		sc->fifo_size = 64;
		break;
	}

	/* De-assert reset */
	if (hwreset_get_by_ofw_name(dev, 0, "apb", &rst_apb) == 0) {
		err = hwreset_deassert(rst_apb);
		if (err != 0) {
			device_printf(dev, "cannot de-assert reset\n");
			goto error;
		}
	}

	sc->dcnt = 0;
	sc->code = 0;
	sc->timer_used = 0;

	/* Reset buffer */
	aw_ir_reset_rawbuffer(sc);

	/* Get clocks and enable them */
	err = clk_get_by_ofw_name(dev, 0, "apb", &clk_gate);
	if (err != 0) {
		device_printf(dev, "Cannot get gate clock\n");
		goto error;
	}
	err = clk_get_by_ofw_name(dev, 0, "ir", &clk_ir);
	if (err != 0) {
		device_printf(dev, "Cannot get IR clock\n");
		goto error;
	}
	/* Set clock rate */
	err = clk_set_freq(clk_ir, IR_BASE_CLK, 0);
	if (err != 0) {
		device_printf(dev, "cannot set IR clock rate\n");
		goto error;
	}
	/* Enable clocks */
	err = clk_enable(clk_gate);
	if (err != 0) {
		device_printf(dev, "Cannot enable clk gate\n");
		goto error;
	}
	err = clk_enable(clk_ir);
	if (err != 0) {
		device_printf(dev, "Cannot enable IR clock\n");
		goto error;
	}

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, aw_ir_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, aw_ir_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	/* Enable CIR Mode */
	WRITE(sc, IR_CTL, IR_CTL_MD);

	/* Set noise and idle threshold */
	WRITE(sc, IR_CIR, IR_RXFILT_VAL | IR_RXIDLE_VAL);

	/* Invert Input Signal */
	WRITE(sc, IR_RXCTL, IR_RXCTL_RPPI);

	/* Clear All RX Interrupt Statuses */
	WRITE(sc, IR_RXSTA, IR_RXSTA_CLEARALL);

	/*
	 * Enable interrupt in case of overflow, packet end
	 * and FIFO available with trigger level
	 */
	WRITE(sc, IR_RXINT, IR_RXINT_ROI_EN | IR_RXINT_RPEI_EN |
	    IR_RXINT_RAI_EN | IR_RXINT_RAL(sc->fifo_size / 2 - 1));

	/* Enable IR Module */
	val = READ(sc, IR_CTL);
	WRITE(sc, IR_CTL, val | IR_CTL_GEN | IR_CTL_RXEN);

	sc->sc_evdev = evdev_alloc();
	evdev_set_name(sc->sc_evdev, device_get_desc(sc->dev));
	evdev_set_phys(sc->sc_evdev, device_get_nameunit(sc->dev));
	evdev_set_id(sc->sc_evdev, BUS_HOST, 0, 0, 0);
	evdev_support_prop(sc->sc_evdev, INPUT_PROP_DIRECT);
	evdev_support_event(sc->sc_evdev, EV_SYN);
	evdev_support_event(sc->sc_evdev, EV_KEY);
	evdev_support_event(sc->sc_evdev, EV_MSC);

	evdev_support_msc(sc->sc_evdev, MSC_SCAN);

	err = evdev_register(sc->sc_evdev);
	if (err) {
		device_printf(dev,
		    "failed to register evdev: error=%d\n", err);
		goto error;
	}
	if (hz > 100)
		sc->ticks = hz / 100;
	else
		sc->ticks = 1;

	callout_init(&sc->co, 1);

	return (0);
error:
	if (clk_gate != NULL)
		clk_release(clk_gate);
	if (clk_ir != NULL)
		clk_release(clk_ir);
	if (rst_apb != NULL)
		hwreset_release(rst_apb);
	evdev_free(sc->sc_evdev);
	sc->sc_evdev = NULL;    /* Avoid double free */

	bus_release_resources(dev, aw_ir_spec, sc->res);
	return (ENXIO);
}

static device_method_t aw_ir_methods[] = {
	DEVMETHOD(device_probe, aw_ir_probe),
	DEVMETHOD(device_attach, aw_ir_attach),

	DEVMETHOD_END
};

static driver_t aw_ir_driver = {
	"aw_ir",
	aw_ir_methods,
	sizeof(struct aw_ir_softc),
};
static devclass_t aw_ir_devclass;

DRIVER_MODULE(aw_ir, simplebus, aw_ir_driver, aw_ir_devclass, 0, 0);
MODULE_DEPEND(aw_ir, evdev, 1, 1, 1);
