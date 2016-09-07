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

#ifndef	A10_CRYPTO_H_
#define	A10_CRYPTO_H_

#define	A10_SS_PHYSBASE			0x01c15000
#define	A10_SS_SIZE			0x1000
#define	A10_SS_CTL			0x00
#define	A10_SS_KEY0			0x04
#define	A10_SS_KEY1			0x08
#define	A10_SS_KEY2			0x0C
#define	A10_SS_KEY3			0x10
#define	A10_SS_KEY4			0x14
#define	A10_SS_KEY5			0x18
#define	A10_SS_KEY6			0x1C
#define	A10_SS_KEY7			0x20

#define	A10_SS_IV0			0x24
#define	A10_SS_IV1			0x28
#define	A10_SS_IV2			0x2C
#define	A10_SS_IV3			0x30

#define	A10_SS_FCSR			0x44

#define	A10_SS_MD0			0x4C
#define	A10_SS_MD1			0x50
#define	A10_SS_MD2			0x54
#define	A10_SS_MD3			0x58
#define	A10_SS_MD4			0x5C

#define	A10_SS_RXFIFO			0x200
#define	A10_SS_TXFIFO			0x204

/* A10_SS_CTL values */

/* PRNG generator mode - bit 15 */
#define	A10_SS_PRNG_ONESHOT		(0 << 15)
#define	A10_SS_PRNG_CONTINUE		(1 << 15)

/* IV mode for hash */
#define	A10_SS_IV_ARBITRARY		(1 << 14)

/* SS operation mode - bits 12-13 */
#define	A10_SS_ECB			(0 << 12)
#define	A10_SS_CBC			(1 << 12)
#define	A10_SS_CTS			(3 << 12)

/* Counter width for CNT mode - bits 10-11 */
#define	A10_SS_CNT_16BITS		(0 << 10)
#define	A10_SS_CNT_32BITS		(1 << 10)
#define	A10_SS_CNT_64BITS		(2 << 10)

/* Key size for AES - bits 8-9 */
#define	A10_SS_AES_128BITS		(0 << 8)
#define	A10_SS_AES_192BITS		(1 << 8)
#define	A10_SS_AES_256BITS		(2 << 8)

/* Operation direction - bit 7 */
#define	A10_SS_ENCRYPTION		(0 << 7)
#define	A10_SS_DECRYPTION		(1 << 7)

/* SS Method - bits 4-6 */
#define	A10_SS_OP_AES			(0 << 4)
#define	A10_SS_OP_DES			(1 << 4)
#define	A10_SS_OP_3DES			(2 << 4)
#define	A10_SS_OP_SHA1			(3 << 4)
#define	A10_SS_OP_MD5			(4 << 4)
#define	A10_SS_OP_PRNG			(5 << 4)

/* Data end bit - bit 2 */
#define	A10_SS_DATA_END			(1 << 2)

/* PRNG start bit - bit 1 */
#define	A10_SS_PRNG_START		(1 << 1)

/* SS Enable bit - bit 0 */
#define	A10_SS_DISABLED			(0 << 0)
#define	A10_SS_ENABLED			(1 << 0)

/* A10_SS_FCSR configuration values */
/* RX FIFO status - bit 30 */
#define	A10_SS_RXFIFO_FREE		(1 << 30)

/* RX FIFO empty spaces - bits 24-29 */
#define	A10_SS_RXFIFO_SPACES(val)	(((val) >> 24) & 0x3f)

/* TX FIFO status - bit 22 */
#define	A10_SS_TXFIFO_AVAILABLE		(1 << 22)

/* TX FIFO available spaces - bits 16-21 */
#define	A10_SS_TXFIFO_SPACES(val)	(((val) >> 16) & 0x3f)

#define	A10_SS_RX_MAX			32
#define	A10_SS_RX_DEFAULT		A10_SS_RX_MAX
#define	A10_SS_TX_MAX			33

#define	A10_SS_RXFIFO_EMP_INT_PENDING	(1 << 10)
#define	A10_SS_TXFIFO_AVA_INT_PENDING	(1 << 8)
#define	A10_SS_RXFIFO_EMP_INT_ENABLE	(1 << 2)
#define	A10_SS_TXFIFO_AVA_INT_ENABLE	(1 << 0)

#define	A10_SS_SEED_LEN_BITS		192
#define	A10_SS_SEED_LEN			(A10_SS_SEED_LEN_BITS / 8)

#define	A10_SS_DATA_LEN_BITS		160
#define	A10_SS_DATA_LEN			(A10_SS_DATA_LEN_BITS / 8)

#define	A10_SS_FIFO_WORDS		(A10_SS_SEED_LEN / sizeof(uint32_t))

#define	DIE_ID_START_BIT		16
#define	DIE_ID_MASK			0x07

#endif
