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

#ifndef _A10_MMC_H_
#define _A10_MMC_H_

#define MMC_GCTRL              0x00              // SMC Global Control Register
#define MMC_CLKCR              0x04              // SMC Clock Control Register
#define MMC_TMOUT              0x08              // SMC Time Out Register
#define MMC_WIDTH              0x0C              // SMC Bus Width Register
#define MMC_BLKSZ              0x10              // SMC Block Size Register
#define MMC_BCNTR              0x14              // SMC Byte Count Register
#define MMC_CMDR               0x18              // SMC Command Register
#define MMC_CARG               0x1C              // SMC Argument Register
#define MMC_RESP0              0x20              // SMC Response Register 0
#define MMC_RESP1              0x24              // SMC Response Register 1
#define MMC_RESP2              0x28              // SMC Response Register 2
#define MMC_RESP3              0x2C              // SMC Response Register 3
#define MMC_IMASK              0x30              // SMC Interrupt Mask Register
#define MMC_MISTA              0x34              // SMC Masked Interrupt Status Register
#define MMC_RINTR              0x38              // SMC Raw Interrupt Status Register
#define MMC_STAS               0x3C              // SMC Status Register
#define MMC_FTRGL              0x40              // SMC FIFO Threshold Watermark Register
#define MMC_FUNS               0x44              // SMC Function Select Register
#define MMC_CBCR               0x48              // SMC CIU Byte Count Register
#define MMC_BBCR               0x4C              // SMC BIU Byte Count Register
#define MMC_DBGC               0x50              // SMC Debug Enable Register
#define MMC_DMAC               0x80              // SMC IDMAC Control Register
#define MMC_DLBA               0x84              // SMC IDMAC Descriptor List Base Address Register
#define MMC_IDST               0x88              // SMC IDMAC Status Register
#define MMC_IDIE               0x8C              // SMC IDMAC Interrupt Enable Register
#define MMC_CHDA               0x90
#define MMC_CBDA               0x94
#define MMC_FIFO               0x100             // SMC FIFO Access Address

/* MMC_GCTRL */
#define MMC_SOFT_RESET_B                (1 <<  0)
#define MMC_FIFO_RESET_B                (1 <<  1)
#define MMC_DMA_RESET_B                 (1 <<  2)
#define MMC_INT_ENABLE_B                   (1 << 4)
#define MMC_DMA_ENABLE_B                   (1 << 5)
#define MMC_DEBOUNCE_ENABLE_B              (1 << 8)
#define MMC_PosedgeLatchData         (1 << 9)
#define MMC_NegedgeLatchData         (0 << 9)
#define MMC_DDR_MODE                 (1 << 10)
#define MMC_ACCESS_BY_AHB            (1 << 31)
#define MMC_ACCESS_BY_DMA            (0 << 31)

/* CLKCR */
#define MMC_CARD_CLK_ON                (1 << 16)
#define MMC_LOW_POWER_ON               (1 << 17)

/* MMC_WIDTH */
#define MMC_WIDTH1                   (0)
#define MMC_WIDTH4                   (1)
#define MMC_WIDTH8                   (2)

/* MMC_CMDR */
#define MMC_RspExp                   (1 << 6)  //0x40
#define MMC_LongRsp                  (1 << 7)  //0x80
#define MMC_CheckRspCRC              (1 << 8)  //0x100
#define MMC_DataExp                  (1 << 9)  //0x200
#define MMC_Read                     (0 << 10) //0x000
#define MMC_Write                    (1 << 10) //0x400
#define MMC_Blockmod                 (0 << 11) //0x000
#define MMC_Seqmod                   (1 << 11) //0x800
#define MMC_SendAutoStop             (1 << 12) //0x1000
#define MMC_WaitPreOver              (1 << 13) //0x2000
#define MMC_StopAbortCMD             (1 << 14) //0x4000
#define MMC_SendInitSeq              (1 << 15) //0x8000
#define MMC_UPCLKOnly                (1 << 21) //0x200000
#define MMC_RdCEATADev               (1 << 22) //0x400000
#define MMC_CCSExp                   (1 << 23) //0x800000
#define MMC_EnbBoot                  (1 << 24) //0x1000000
#define MMC_AltBootOpt               (1 << 25) //0x2000000
#define MMC_MandBootOpt              (0 << 25) //0x0000000
#define MMC_BootACKExp               (1 << 26) //0x4000000
#define MMC_DisableBoot              (1 << 27) //0x8000000
#define MMC_VolSwitch                (1 << 28) //0x10000000
#define MMC_Start                    (1 << 31) //0x80000000

/* Struct for Intrrrupt Information */
#define MMC_RespErr                  (1 << 1)  //0x2
#define MMC_CmdDone                  (1 << 2)  //0x4
#define MMC_DataOver                 (1 << 3)  //0x8
#define MMC_TxDataReq                (1 << 4)  //0x10
#define MMC_RxDataReq                (1 << 5)  //0x20
#define MMC_RespCRCErr               (1 << 6)  //0x40
#define MMC_DataCRCErr               (1 << 7)  //0x80
#define MMC_RespTimeout              (1 << 8)  //0x100
#define MMC_ACKRcv                   (1 << 8)  //0x100
#define MMC_DataTimeout              (1 << 9)  //0x200
#define MMC_BootStart                (1 << 9)  //0x200
#define MMC_DataStarve               (1 << 10) //0x400
#define MMC_VolChgDone               (1 << 10) //0x400
#define MMC_FIFORunErr               (1 << 11) //0x800
#define MMC_HardWLocked              (1 << 12) //0x1000
#define MMC_StartBitErr              (1 << 13) //0x2000
#define MMC_AutoCMDDone              (1 << 14) //0x4000
#define MMC_EndBitErr                (1 << 15) //0x8000
#define MMC_SDIOInt                  (1 << 16) //0x10000
#define MMC_CardInsert               (1 << 30) //0x40000000
#define MMC_CardRemove               (1 << 31) //0x80000000
#define MMC_IntErrBit                (MMC_RespErr | MMC_RespCRCErr | MMC_DataCRCErr | MMC_RespTimeout | MMC_DataTimeout  \
                                        | MMC_FIFORunErr | MMC_HardWLocked | MMC_StartBitErr | MMC_EndBitErr)  //0xbfc2
/* status */
#define MMC_RXWLFlag                 (1 << 0)
#define MMC_TXWLFlag                 (1 << 1)
#define MMC_FIFOEmpty                (1 << 2)
#define MMC_FIFOFull                 (1 << 3)
#define MMC_CardPresent              (1 << 8)
#define MMC_CardDataBusy             (1 << 9)
#define MMC_DataFSMBusy              (1 << 10)
#define MMC_DMAReq                   (1 << 31)
#define MMC_FIFO_SIZE                (16)
/* Function select */
#define MMC_CEATAOn                  (0xceaaU<< 16)
#define MMC_SendIrqRsp               (1 << 0)
#define MMC_SDIORdWait               (1 << 1)
#define MMC_AbtRdData                (1 << 2)
#define MMC_SendCCSD                 (1 << 8)
#define MMC_SendAutoStopCCSD         (1 << 9)
#define MMC_CEATADevIntEnb           (1 << 10)
/* IDMA controller bus mod bit field */
#define MMC_IDMACSoftRST             (1 << 0)
#define MMC_IDMACFixBurst            (1 << 1)
#define MMC_IDMACIDMAOn              (1 << 7)
#define MMC_IDMACRefetchDES          (1 << 31)
/* IDMA status bit field */
#define MMC_IDMACTransmitInt         (1 << 0)
#define MMC_IDMACReceiveInt          (1 << 1)
#define MMC_IDMACFatalBusErr         (1 << 2)
#define MMC_IDMACDesInvalid          (1 << 4)
#define MMC_IDMACCardErrSum          (1 << 5)
#define MMC_IDMACNormalIntSum        (1 << 8)
#define MMC_IDMACAbnormalIntSum      (1 << 9)
#define MMC_IDMACHostAbtInTx         (1 << 10)
#define MMC_IDMACHostAbtInRx         (1 << 10)
#define MMC_IDMACIdle                (0 << 13)
#define MMC_IDMACSuspend             (1 << 13)
#define MMC_IDMACDESCRd              (0x2U<< 13)
#define MMC_IDMACDESCCheck           (0x3U<< 13)
#define MMC_IDMACRdReqWait           (0x4U<< 13)
#define MMC_IDMACWrReqWait           (0x5U<< 13)
#define MMC_IDMACRd                  (0x6U<< 13)
#define MMC_IDMACWr                  (0x7U<< 13)
#define MMC_IDMACDESCClose           (0x8U<< 13)

#define MMC_IDMA_OVER       (MMC_IDMACTransmitInt|MMC_IDMACReceiveInt|MMC_IDMACNormalIntSum)
#define MMC_IDMA_ERR        (MMC_IDMACFatalBusErr|MMC_IDMACDesInvalid|MMC_IDMACCardErrSum|MMC_IDMACAbnormalIntSum)

#endif /* _A10_MMC_H_ */

