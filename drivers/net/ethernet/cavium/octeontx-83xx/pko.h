// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef PKO_H
#define PKO_H

#include <linux/pci.h>
#include "octeontx.h"

/* PCI DEV IDs */
#define PCI_DEVICE_ID_OCTEONTX_PKO_PF	0xA048
#define PCI_DEVICE_ID_OCTEONTX_PKO_VF	0xA049

#define PKO_MAX_VF			32

#define PCI_PKO_PF_CFG_BAR		0
#define PCI_PKO_PF_MSIX_BAR		4
#define PKO_MSIX_COUNT			25

#define PCI_PKO_VF_CFG_BAR		0

#define PKO_VF_OFFSET(x)		(0x400000000 | (0x100000 * (x)))
#define PKO_VF_CFG_SIZE			0x100000

/* PF registers */
#define PKO_PF_L1_SQX_SCHEDULE(x)		(0x8ULL | ((x) << 9))
#define PKO_PF_L1_SQX_SHAPE(x)			(0x10ULL | ((x) << 9))
#define PKO_PF_L1_SQX_CIR(x)			(0x18ULL | ((x) << 9))
#define PKO_PF_L1_SQX_SHAPE_STATE(x)		(0x30ULL | ((x) << 9))
#define PKO_PF_L1_SQX_LINK(x)			(0x38ULL | ((x) << 9))
#define PKO_PF_L1_SQX_DROPPED_PACKETS(x)	(0x80ULL | ((x) << 9))
#define PKO_PF_L1_SQX_DROPPED_BYTES(x)		(0x88ULL | ((x) << 9))
#define PKO_PF_L1_SQX_RED_PACKETS(x)		(0x90ULL | ((x) << 9))
#define PKO_PF_L1_SQX_RED_BYTES(x)		(0x98ULL | ((x) << 9))
#define PKO_PF_L1_SQX_YELLOW_PACKETS(x)		(0xA0ULL | ((x) << 9))
#define PKO_PF_L1_SQX_YELLOW_BYTES(x)		(0xA8ULL | ((x) << 9))
#define PKO_PF_L1_SQX_GREEN_PACKETS(x)		(0xB0ULL | ((x) << 9))
#define PKO_PF_L1_SQX_GREEN_BYTES(x)		(0xB8ULL | ((x) << 9))
#define PKO_PF_DQX_DROPPED_PACKETS(x)		(0xC0ULL | ((x) << 9))
#define PKO_PF_DQX_DROPPED_BYTES(x)		(0xC8ULL | ((x) << 9))
#define PKO_PF_DQX_PACKETS(x)			(0xD0ULL | ((x) << 9))
#define PKO_PF_DQX_BYTES(x)			(0xD8ULL | ((x) << 9))
#define PKO_PF_L1_SQX_SW_XOFF(x)		(0xE0ULL | ((x) << 9))
#define PKO_PF_PSE_PQ_ECC_CTL0			0x100
#define PKO_PF_PSE_PQ_BIST_STATUS		0x138
#define PKO_PF_PQ_DRAIN_W1C			0x140
#define PKO_PF_PQ_DRAIN_W1S			0x148
#define PKO_PF_PQ_DRAIN_INT_ENA_W1C		0x150
#define PKO_PF_PQ_DRAIN_INT_ENA_W1S		0x158
#define PKO_PF_PQ_ECC_SBE_W1C			0x180
#define PKO_PF_PQ_ECC_SBE_W1S			0x188
#define PKO_PF_PQ_ECC_SBE_INT_ENA_W1C		0x190
#define PKO_PF_PQ_ECC_SBE_INT_ENA_W1S		0x198
#define PKO_PF_PQ_ECC_DBE_W1C			0x1A0
#define PKO_PF_PQ_ECC_DBE_W1S			0x1A8
#define PKO_PF_PQ_ECC_DBE_INT_ENA_W1C		0x1B0
#define PKO_PF_PQ_ECC_DBE_INT_ENA_W1S		0x1B8
#define PKO_PF_L1_SQX_TOPOLOGY(x)		(0x80000 | ((x) << 9))
#define PKO_PF_L2_SQX_SCHEDULE(x)		(0x80008 | ((x) << 9))
#define PKO_PF_L2_SQX_SHAPE(x)			(0x80010 | ((x) << 9))
#define PKO_PF_L2_SQX_CIR(x)			(0x80018 | ((x) << 9))
#define PKO_PF_L2_SQX_PIR(x)			(0x80020 | ((x) << 9))
#define PKO_PF_L2_SQX_SCHED_STATE(x)		(0x80028 | ((x) << 9))
#define PKO_PF_L2_SQX_SHAPE_STATE(x)		(0x80030 | ((x) << 9))
#define PKO_PF_L3_L2_SQX_CHANNEL(x)		(0x80038 | ((x) << 9))
#define PKO_PF_L1_SQX_PICK(x)			(0x80070 | ((x) << 9))
#define PKO_PF_L2_SQX_SW_XOFF(x)		(0x800E0 | ((x) << 9))
#define PKO_PF_CHANNEL_LEVEL			0x800F0
#define PKO_PF_SHAPER_CFG			0x800F8
#define PKO_PF_PSE_SQ1_ECC_CTL0			0x80100
#define PKO_PF_PSE_SQ1_BIST_STATUS		0x80138
#define PKO_PF_L1_ECC_SBE_W1C			0x80180
#define PKO_PF_L1_ECC_SBE_W1S			0x80188
#define PKO_PF_L1_ECC_SBE_INT_ENA_W1C		0x80190
#define PKO_PF_L1_ECC_SBE_INT_ENA_W1S		0x80198
#define PKO_PF_L1_ECC_DBE_W1C			0x801A0
#define PKO_PF_L1_ECC_DBE_W1S			0x801A8
#define PKO_PF_L1_ECC_DBE_INT_ENA_W1C		0x801B0
#define PKO_PF_L1_ECC_DBE_INT_ENA_W1S		0x801B8
#define PKO_PF_L2_SQX_TOPOLOGY(x)		(0x100000 | ((x) << 9))
#define PKO_PF_L3_SQX_SCHEDULE(x)		(0x100008 | ((x) << 9))
#define PKO_PF_L3_SQX_SHAPE(x)			(0x100010 | ((x) << 9))
#define PKO_PF_L3_SQX_CIR(x)			(0x100018 | ((x) << 9))
#define PKO_PF_L3_SQX_PIR(x)			(0x100020 | ((x) << 9))
#define PKO_PF_L3_SQX_SCHED_STATE(x)		(0x100028 | ((x) << 9))
#define PKO_PF_L3_SQX_SHAPE_STATE(x)		(0x100030 | ((x) << 9))
#define PKO_PF_L2_SQX_PICK(x)			(0x100070 | ((x) << 9))
#define PKO_PF_L3_SQX_SW_XOFF(x)		(0x1000E0 | ((x) << 9))
#define PKO_PF_L2_ECC_SBE_W1C			0x100180
#define PKO_PF_L2_ECC_SBE_W1S			0x100188
#define PKO_PF_L2_ECC_SBE_INT_ENA_W1C		0x100190
#define PKO_PF_L2_ECC_SBE_INT_ENA_W1S		0x100198
#define PKO_PF_L2_ECC_DBE_W1C			0x1001A0
#define PKO_PF_L2_ECC_DBE_W1S			0x1001A8
#define PKO_PF_L2_ECC_DBE_INT_ENA_W1C		0x1001B0
#define PKO_PF_L2_ECC_DBE_INT_ENA_W1S		0x1001B8
#define PKO_PF_L3_SQX_TOPOLOGY(x)		(0x180000 | ((x) << 9))
#define PKO_PF_L3_SQX_PICK(x)			(0x180070 | ((x) << 9))
#define PKO_PF_PSE_SQ3_ECC_CTL0			0x180100
#define PKO_PF_PSE_SQ3_BIST_STATUS		0x180138
#define PKO_PF_L3_ECC_SBE_W1C			0x180180
#define PKO_PF_L3_ECC_SBE_W1S			0x180188
#define PKO_PF_L3_ECC_SBE_INT_ENA_W1C		0x180190
#define PKO_PF_L3_ECC_SBE_INT_ENA_W1S		0x180198
#define PKO_PF_L3_ECC_DBE_W1C			0x1801A0
#define PKO_PF_L3_ECC_DBE_W1S			0x1801A8
#define PKO_PF_L3_ECC_DBE_INT_ENA_W1C		0x1801B0
#define PKO_PF_L3_ECC_DBE_INT_ENA_W1S		0x1801B8
#define PKO_PF_DQX_SCHEDULE(x)			(0x280008 | ((x) << 9))
#define PKO_PF_DQX_SHAPE(x)			(0x280010 | ((x) << 9))
#define PKO_PF_DQX_CIR(x)			(0x280018 | ((x) << 9))
#define PKO_PF_DQX_PIR(x)			(0x280020 | ((x) << 9))
#define PKO_PF_DQX_SCHED_STATE(x)		(0x280028 | ((x) << 9))
#define PKO_PF_DQX_SHAPE_STATE(x)		(0x280030 | ((x) << 9))
#define PKO_PF_DQX_TOPOLOGY(x)			(0x300000 | ((x) << 9))
#define PKO_PF_DQX_PICK(x)			(0x300070 | ((x) << 9))
#define PKO_PF_PSE_DQ_ECC_CTL0			0x300100
#define PKO_PF_PSE_DQ_BIST_STATUS		0x300138
#define PKO_PF_DQ_ECC_SBE_W1C			0x300180
#define PKO_PF_DQ_ECC_SBE_W1S			0x300188
#define PKO_PF_DQ_ECC_SBE_INT_ENA_W1C		0x300190
#define PKO_PF_DQ_ECC_SBE_INT_ENA_W1S		0x300198
#define PKO_PF_DQ_ECC_DBE_W1C			0x3001A0
#define PKO_PF_DQ_ECC_DBE_W1S			0x3001A8
#define PKO_PF_DQ_ECC_DBE_INT_ENA_W1C		0x3001B0
#define PKO_PF_DQ_ECC_DBE_INT_ENA_W1S		0x3001B8
#define PKO_PF_PDM_CFG				0x800000
#define PKO_PF_PDM_STS_W1C			0x800008
#define PKO_PF_PDM_MEM_DATA			0x800010
#define PKO_PF_PDM_MEM_ADDR			0x800018
#define PKO_PF_PDM_MEM_RW_CTL			0x800020
#define PKO_PF_PDM_MEM_RW_STS			0x800028
#define PKO_PF_PDM_STS_W1S			0x800030
#define PKO_PF_PDM_STS_INT_ENA_W1C		0x800038
#define PKO_PF_PDM_STS_INT_ENA_W1S		0x800040
#define PKO_PF_PDM_STS_INFO			0x800048
#define PKO_PF_PDM_DQX_MINPAD(x)		(0x8F0000 | ((x) << 3))
#define PKO_PF_PDM_BIST_STATUS			0x8FFF00
#define PKO_PF_PDM_ECC_SBE_W1C			0x8FFF80
#define PKO_PF_PDM_ECC_SBE_W1S			0X8FFF88
#define PKO_PF_PDM_ECC_SBE_INT_ENA_W1C		0x8FFF90
#define PKO_PF_PDM_ECC_SBE_INT_ENA_W1S		0x8FFF98
#define PKO_PF_PDM_ECC_DBE_W1C			0x8FFFA0
#define PKO_PF_PDM_ECC_DBE_W1S			0x8FFFA8
#define PKO_PF_PDM_ECC_DBE_INT_ENA_W1C		0x8FFFB0
#define PKO_PF_PDM_ECC_DBE_INT_ENA_W1S		0x8FFFB8
#define PKO_PF_PDM_ECC_CTL0			0x8FFFD0
#define PKO_PF_PDM_ECC_CTL1			0x8FFFD8
#define PKO_PF_MACX_CFG(x)			(0x900000 | ((x) << 3))
#define PKO_PF_PTFX_STATUS(x)			(0x900100 | ((x) << 3))
#define PKO_PF_PTGFX_CFG(x)			(0x900200 | ((x) << 3))
#define PKO_PF_PTF_IOBP_CFG			0x900300
#define PKO_PF_PEB_NCB_CFG			0x900308
#define PKO_PF_PEB_TSO_CFG			0x900310
#define PKO_PF_FORMATX_CTL(x)			(0x900800 | ((x) << 3))
#define PKO_PF_PEB_ERR_INT_W1C			0x900C00
#define PKO_PF_PEB_JUMP_DEF_ERR_INFO		0x900C10
#define PKO_PF_PEB_FCS_SOP_ERR_INFO		0x900C18
#define PKO_PF_PEB_PSE_FIFO_ERR_INFO		0x900C20
#define PKO_PF_PEB_PAD_ERR_INFO			0x900C28
#define PKO_PF_PEB_TRUNC_ERR_INFO		0x900C30
#define PKO_PF_PEB_SUBD_ADDR_ERR_INFO		0x900C38
#define PKO_PF_PEB_SUBD_SIZE_ERR_INFO		0x900C40
#define PKO_PF_PEB_MAX_LINK_ERR_INFO		0x900C48
#define PKO_PF_PEB_MACX_CFG_WR_ERR_INFO		0x900C50
#define PKO_PF_PEB_ERR_INT_W1S			0x900C80
#define PKO_PF_PEB_ERR_INT_ENA_W1C		0x900C88
#define PKO_PF_PEB_ERR_INT_ENA_W1S		0x900C90
#define PKO_PF_PEB_BIST_STATUS			0x900D00
#define PKO_PF_TXFX_PKT_CNT_RD(x)		(0x900E00 | ((x) << 3))
#define PKO_PF_PEB_ECC_SBE_W1C			0x9FFF80
#define PKO_PF_PEB_ECC_SBE_W1S			0x9FFF88
#define PKO_PF_PEB_ECC_SBE_INT_ENA_W1C		0x9FFF90
#define PKO_PF_PEB_ECC_SBE_INT_ENA_W1S		0x9FFF98
#define PKO_PF_PEB_ECC_CTL1			0x9FFFA8
#define PKO_PF_PEB_ECC_DBE_W1C			0x9FFFB0
#define PKO_PF_PEB_ECC_DBE_W1S			0x9FFFB8
#define PKO_PF_PEB_ECC_DBE_INT_ENA_W1C		0x9FFFC0
#define PKO_PF_PEB_ECC_DBE_INT_ENA_W1S		0x9FFFC8
#define PKO_PF_PEB_ECC_CTL0			0x9FFFD0
#define PKO_PF_MCI1_MAX_CREDX(x)		(0xA80000 | ((x) << 3))
#define PKO_PF_MCI1_CREDX_CNT(x)		(0xA80100 | ((x) << 3))
#define PKO_PF_LUTX(x)				(0xB00000 | ((x) << 3))
#define PKO_PF_LUT_BIST_STATUS			0xB08000
#define PKO_PF_LUT_ECC_DBE_W1C			0xBFFF60
#define PKO_PF_LUT_ECC_DBE_W1S			0xBFFF68
#define PKO_PF_LUT_ECC_DBE_INT_ENA_W1C		0xBFFF70
#define PKO_PF_LUT_ECC_DBE_INT_ENA_W1S		0xBFFF78
#define PKO_PF_LUT_ECC_SBE_W1C			0xBFFF80
#define PKO_PF_LUT_ECC_SBE_W1S			0xBFFF88
#define PKO_PF_LUT_ECC_SBE_INT_ENA_W1C		0xBFFF90
#define PKO_PF_LUT_ECC_SBE_INT_ENA_W1S		0xBFFF98
#define PKO_PF_LUT_ECC_CTL0			0xBFFFD0
#define PKO_PF_DPFI_STATUS			0xC00000
#define PKO_PF_DPFI_FLUSH			0xC00008
#define PKO_PF_DPFI_FPA_AURA			0xC00010
#define PKO_PF_DPFI_ENA				0xC00018
#define PKO_PF_DPFI_GMCTL			0xC00020
#define PKO_PF_STATUS				0xD00000
#define PKO_PF_ENABLE				0xD00008
#define PKO_PF_CONST				0xD00010
#define PKO_PF_CONST1				0xD00018
#define PKO_PF_L1_CONST				0xD00100
#define PKO_PF_L2_CONST				0xD00108
#define PKO_PF_L3_CONST				0xD00110
#define PKO_PF_L4_CONST				0xD00118
#define PKO_PF_L5_CONST				0xD00120
#define PKO_PF_DQ_CONST				0xD00138
#define PKO_PF_PDM_NCB_TX_ERR_WORD		0xE00000
#define PKO_PF_PDM_NCB_TX_ERR_INFO		0xE00008
#define PKO_PF_PDM_NCB_INT_W1C			0xE00010
#define PKO_PF_PDM_NCB_INT_W1S			0xE00018
#define PKO_PF_PDM_NCB_INT_ENA_W1C		0xE00020
#define PKO_PF_PDM_NCB_INT_ENA_W1S		0xE00028
#define PKO_PF_PDM_NCB_BIST_STATUS		0xEFFF00
#define PKO_PF_PDM_NCB_ECC_SBE_W1C		0xEFFF80
#define PKO_PF_PDM_NCB_ECC_SBE_W1S		0xEFFF88
#define PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1C	0xEFFF90
#define PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1S	0xEFFF98
#define PKO_PF_PDM_NCB_ECC_DBE_W1C		0xEFFFA0
#define PKO_PF_PDM_NCB_ECC_DBE_W1S		0xEFFFA8
#define PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1C	0xEFFFB0
#define PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1S	0xEFFFB8
#define PKO_PF_PDM_NCB_ECC_CTL0			0xEFFFD0
#define PKO_PF_PDM_NCB_CFG			0x1800050
#define PKO_PF_PDM_NCB_MEM_FAULT		0x1800058
#define PKO_PF_PEB_NCB_MEM_FAULT		0x1800060
#define PKO_PF_PEB_NCB_INT_W1C			0x1E00010
#define PKO_PF_PEB_NCB_INT_W1S			0x1E00018
#define PKO_PF_PEB_NCB_INT_ENA_W1C		0x1E00020
#define PKO_PF_PEB_NCB_INT_ENA_W1S		0x1E00028
#define PKO_PF_PEB_NCB_BIST_STATUS		0x1EFFF00
#define PKO_PF_PEB_NCB_ECC_SBE_W1C		0x1EFFF80
#define PKO_PF_PEB_NCB_ECC_SBE_W1S		0x1EFFF88
#define PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1C	0x1EFFF90
#define PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1S	0x1EFFF98
#define PKO_PF_PEB_NCB_ECC_DBE_W1C		0x1EFFFA0
#define PKO_PF_PEB_NCB_ECC_DBE_W1S		0x1EFFFA8
#define PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1C	0x1EFFFB0
#define PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1S	0x1EFFFB8
#define PKO_PF_PEB_NCB_ECC_CTL0			0x1EFFFD0
#define PKO_PF_VFX_GMCTL(x)			(0x40001000ULL | ((x) << 20))
#define PKO_PF_VFX_DQX_MP_STATEX(x, y, z)	(0x7001FE00ULL | ((x) << 20) \
						| ((y) << 17) | ((z) << 3))
#define PKO_PF_VFX_DQX_PD_STATEX(x, y, z)	(0x7001FF00ULL | ((x) << 20) \
						| ((y) << 17) | ((z) << 3))

/* VF Regs */
#define PKO_VF_DQX_SW_XOFF(x)			(0x100 | ((x) << 17))
#define PKO_VF_DQX_WM_CTL(x)			(0x130 | ((x) << 17))
#define PKO_VF_DQX_WM_CNT(x)			(0x150 | ((x) << 17))
#define PKO_VF_DQ_FC_CONFIG			0x160
#define PKO_VF_DQX_FC_STATUS			(0x168 | ((x) << 17))
#define PKO_VF_DQX_OP_SENDX(x, y)		(0x1000ULL | ((x) << 17) | \
						((y) << 3))
#define PKO_VF_DQX_OP_OPEN(x)			(0x1100ULL | ((x) << 17))
#define PKO_VF_DQX_OP_CLOSE(x)			(0x1200ULL | ((x) << 17))
#define PKO_VF_DQX_OP_QUERY(x)			(0x1300ULL | ((x) << 17))

#define PKO_CONST_GET_LEVELS(x)			((x) & 0xf)
#define PKO_CONST_GET_PTGFS(x)			(((x) >> 4) & 0xf)
#define PKO_CONST_GET_FORMATS(x)		(((x) >> 8) & 0xff)
#define PKO_CONST_GET_PDM_BUF_SIZE(x)		(((x) >> 16) & 0xffff)
#define PKO_CONST_GET_DQS_PER_VM(x)		((((x) | 0ULL) >> 32) & 0x3f)

#define PKO_PAD_MINLEN				0x3C
#define PKO_PDM_CFG_SET_PAD_MINLEN(x)		(((x) & 0x7F) << 3)
#define PKO_PDM_CFG_SET_DQ_FC_SKID(x)		(((x) & 0x3FF) << 16)
#define PKO_PDM_CFG_SET_EN(x)			(((x) & 0x1) << 28)

#define PKO_PF_VFX_GMCTL_GET_BE(x)		(((x) >> 24) & 0x1)
#define PKO_PF_VFX_GMCTL_BE(x)			(((x) & 0x1) << 24)
#define PKO_PF_VFX_GMCTL_GMID(x)		(((x) & 0xFFFF))
#define PKO_PF_VFX_GMCTL_STRM(x)		(((x) & 0xFF) << 16)

#define PKO_PF_TOPOLOGY_LINK_MASK		0x1F
#define PKO_PF_TOPOLOGY_LINK_SHIFT		16

#define PKO_PF_PICK_ADJUST_MASK			0x1FF
#define PKO_PF_PICK_ADJUST_SHIFT		20
#define PKO_PF_VALID_META			0x100

#define PKO_VF_DQ_OP_DQSTATUS_MASK              0xF
#define PKO_VF_DQ_OP_DQSTATUS_SHIFT             60
#define PKO_VF_DQ_STATUS_OK			0x0

#define PKO_PF_CC_WORD_CNT_MASK			0xFFFFF
#define PKO_PF_CC_WORD_CNT_SHIFT		12

#define NULL_FIFO				0x13
#define DQS_PER_VF				0x08

struct pkopf_vf {
	struct octeontx_pf_vf	domain;
	int			chan;
	int			mac_num;
	int			tx_fifo_sz;
};

struct pkopf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	int			id;
	struct msix_entry	*msix_entries;
	struct list_head	list;

	int			total_vfs;
	int			vfs_in_use;
#define PKO_SRIOV_ENABLED	0x1
	u32			flags;

	int			max_levels;
	int			max_ptgfs;
	int			max_formats;
	int			pdm_buf_size;
	int			dqs_per_vf;

	struct pkopf_vf		vf[PKO_MAX_VF];
};

struct pkopf_com_s {
	u64 (*create_domain)(u32 id, u16 domain_id, u32 pko_vf_count,
			     struct octtx_bgx_port *bgx_port, int bgx_count,
			     struct octtx_lbk_port *lbk_port, int lbk_count,
			     struct octtx_sdp_port *sdp_port, int sdp_count,
			     void *master, void *master_data,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp,
			       void *mdata);
	int (*get_vf_count)(u32 id);
};

extern struct pkopf_com_s pkopf_com;

struct pkovf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	struct msix_entry	*msix_entries;
	struct list_head	list;

	bool			setup_done;
	u16			domain_id;
	u16			subdomain_id;

	struct octeontx_master_com_t	*master;
	void			*master_data;
};

struct pkovf_com_s {
	struct pkovf* (*get)(u16 domain_id, u16 subdomain_id,
			     struct octeontx_master_com_t *master,
			     void *master_data);
	int (*setup)(struct pkovf *pko);
	void (*close)(struct pkovf *pko);
};

extern struct pkovf_com_s pkovf_com;

#endif
