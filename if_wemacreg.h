/*
 * Copyright (C) 2013 Ganbold Tsagaankhuu
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

#ifndef __IF_WEMACREG_H__
#define __IF_WEMACREG_H__

/*
 * WEMAC register definitions
 */
#define EMAC_CTL		0x00
#define  CTL_RST		(1<<0)

#define EMAC_TX_MODE		0x04
#define EMAC_TX_FLOW		0x08
#define EMAC_TX_CTL0		0x0C
#define EMAC_TX_CTL1		0x10
#define EMAC_TX_INS		0x14
#define EMAC_TX_PL0		0x18
#define EMAC_TX_PL1		0x1C
#define EMAC_TX_STA		0x20
#define EMAC_TX_IO_DATA		0x24
#define EMAC_TX_IO_DATA1	0x28
#define EMAC_TX_TSVL0		0x2C
#define EMAC_TX_TSVH0		0x30
#define EMAC_TX_TSVL1		0x34
#define EMAC_TX_TSVH1		0x38

#define EMAC_RX_CTL		0x3C
#define EMAC_RX_HASH0		0x40
#define EMAC_RX_HASH1		0x44
#define EMAC_RX_STA		0x48
#define EMAC_RX_IO_DATA		0x4C
#define EMAC_RX_FBC		0x50

#define EMAC_INT_CTL		0x54
#define EMAC_INT_STA		0x58

#define EMAC_MAC_CTL0		0x5C
#define EMAC_MAC_CTL1		0x60
#define EMAC_MAC_IPGT		0x64
#define EMAC_MAC_IPGR		0x68
#define EMAC_MAC_CLRT		0x6C
#define EMAC_MAC_MAXF		0x70
#define EMAC_MAC_SUPP		0x74
#define EMAC_MAC_TEST		0x78
#define EMAC_MAC_MCFG		0x7C
#define EMAC_MAC_MCMD		0x80
#define EMAC_MAC_MADR		0x84
#define EMAC_MAC_MWTD		0x88
#define EMAC_MAC_MRDD		0x8C
#define EMAC_MAC_MIND		0x90
#define EMAC_MAC_SSRR		0x94
#define EMAC_MAC_A0		0x98
#define EMAC_MAC_A1		0x9C
#define EMAC_MAC_A2		0xA0

#define EMAC_SAFX_L0		0xA4
#define EMAC_SAFX_H0		0xA8
#define EMAC_SAFX_L1		0xAC
#define EMAC_SAFX_H1		0xB0
#define EMAC_SAFX_L2		0xB4
#define EMAC_SAFX_H2		0xB8
#define EMAC_SAFX_L3		0xBC
#define EMAC_SAFX_H3		0xC0



/* 0: Disable, 1: Aborted frame enable(default) */
#define EMAC_TX_AB_M		(0x1 << 0)

/* 0: CPU, 1: DMA(default) */
#define EMAC_TX_TM		(0)

#define EMAC_TX_SETUP		(0)

/* 0: DRQ asserted, 1: DRQ automatically(default) */
#define EMAC_RX_DRQ_MODE	(0x1 << 1)

/* 0: CPU, 1: DMA(default) */
#define EMAC_RX_TM		(0x1 << 2)

/* 0: Normal(default), 1: Pass all Frames */
#define EMAC_RX_PA		(0x1 << 4)

/* 0: Normal(default), 1: Pass Control Frames */
#define EMAC_RX_PCF		(0x1 << 5)

/* 0: Normal(default), 1: Pass Frames with CRC Error */
#define EMAC_RX_PCRCE		(0x1 << 6)

/* 0: Normal(default), 1: Pass Frames with Length Error */
#define EMAC_RX_PLE		(0x1 << 7)

/* 0: Normal, 1: Pass Frames length out of range(default) */
#define EMAC_RX_POR		(0x1 << 8)

/* 0: Not accept, 1: Accept unicast Packets(default) */
#define EMAC_RX_UCAD		(0x1 << 16)

/* 0: Normal(default), 1: DA Filtering */
#define EMAC_RX_DAF		(0x1 << 17)

/* 0: Not accept, 1: Accept multicast Packets(default) */
#define EMAC_RX_MCO		(0x1 << 20)

/* 0: Disable(default), 1: Enable Hash filter */
#define EMAC_RX_MHF		(0x1 << 21)

/* 0: Not accept, 1: Accept Broadcast Packets(default) */
#define EMAC_RX_BCO		(0x1 << 22)

/* 0: Disable(default), 1: Enable SA Filtering */
#define EMAC_RX_SAF		(0x1 << 24)

/* 0: Normal(default), 1: Inverse Filtering */
#define EMAC_RX_SAIF		(0x1 << 25)

#define EMAC_RX_SETUP		(EMAC_RX_POR | EMAC_RX_UCAD | EMAC_RX_DAF | \
				    EMAC_RX_MCO | EMAC_RX_BCO)

/* 0: Disable, 1: Enable Receive Flow Control(default) */
#define EMAC_MAC_CTL0_RFC	(0x1 << 2)

/* 0: Disable, 1: Enable Transmit Flow Control(default) */
#define EMAC_MAC_CTL0_TFC	(0x1 << 3)

#define EMAC_MAC_CTL0_SETUP	(EMAC_MAC_CTL0_RFC | EMAC_MAC_CTL0_TFC)

/* 0: Disable, 1: Enable MAC Frame Length Checking(default) */
#define EMAC_MAC_CTL1_FLC	(0x1 << 1)

/* 0: Disable(default), 1: Enable Huge Frame */
#define EMAC_MAC_CTL1_HF	(0x1 << 2)

/* 0: Disable(default), 1: Enable MAC Delayed CRC */
#define EMAC_MAC_CTL1_DCRC	(0x1 << 3)

/* 0: Disable, 1: Enable MAC CRC(default) */
#define EMAC_MAC_CTL1_CRC	(0x1 << 4)

/* 0: Disable, 1: Enable MAC PAD Short frames(default) */
#define EMAC_MAC_CTL1_PC	(0x1 << 5)

/* 0: Disable(default), 1: Enable MAC PAD Short frames and append CRC */
#define EMAC_MAC_CTL1_VC	(0x1 << 6)

/* 0: Disable(default), 1: Enable MAC auto detect Short frames */
#define EMAC_MAC_CTL1_ADP	(0x1 << 7)

/* 0: Disable(default), 1: Enable */
#define EMAC_MAC_CTL1_PRE	(0x1 << 8)

/* 0: Disable(default), 1: Enable */
#define EMAC_MAC_CTL1_LPE	(0x1 << 9)

/* 0: Disable(default), 1: Enable no back off */
#define EMAC_MAC_CTL1_NB	(0x1 << 12)

/* 0: Disable(default), 1: Enable */
#define EMAC_MAC_CTL1_BNB	(0x1 << 13)

/* 0: Disable(default), 1: Enable */
#define EMAC_MAC_CTL1_ED	(0x1 << 14)

#define EMAC_MAC_CTL1_SETUP	(EMAC_MAC_CTL1_FLC | EMAC_MAC_CTL1_CRC | \
				    EMAC_MAC_CTL1_PC)
#define EMAC_MAC_IPGT_VAL	0x15

#define EMAC_MAC_NBTB_IPG1	0xC
#define EMAC_MAC_NBTB_IPG2	0x12

#define EMAC_MAC_CW		0x37
#define EMAC_MAC_RM		0xF

#define EMAC_MAC_MFL		0x0600

/* Receive status */
#define EMAC_CRCERR		(1 << 4)
#define EMAC_LENERR		(3 << 5)

#endif
