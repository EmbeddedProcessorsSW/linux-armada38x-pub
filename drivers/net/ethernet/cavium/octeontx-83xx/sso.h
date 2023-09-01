// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef SSO_H
#define SSO_H

#include <linux/pci.h>
#include "octeontx.h"

/* PCI DEV IDs */
#define PCI_DEVICE_ID_OCTEONTX_SSO_PF	0xA04A
#define PCI_DEVICE_ID_OCTEONTX_SSO_VF	0xA04B
#define PCI_DEVICE_ID_OCTEONTX_SSOW_PF	0xA04C
#define PCI_DEVICE_ID_OCTEONTX_SSOW_VF	0xA04D

#define SSO_MAX_VF			64
#define SSOW_MAX_VF			32

#define PCI_SSO_PF_CFG_BAR		0
#define PCI_SSO_PF_MSIX_BAR		4
#define SSO_PF_MSIX_COUNT		4

#define PCI_SSO_VF_CFG_BAR		0
#define PCI_SSO_VF_ADD_WORK_BAR		2
#define PCI_SSO_VF_MSIX_BAR		4
#define SSO_VF_MSIX_COUNT		1

#define PCI_SSOW_VF_CFG_BAR		0
#define PCI_SSOW_VF_LMT_BAR		2
#define PCI_SSOW_VF_MBOX_BAR		4

#define SSO_VF_OFFSET(x)		(0x800000000 | (0x100000 * (x)))
#define SSO_VF_CFG_SIZE			0x100000

#define SSOW_VF_BASE(x)			(0x861800000000ULL | (0x100000 * (x)))
#define SSOW_VF_SIZE			0x100000

/* SSO PF register offsets */
#define SSO_PF_CONST			0x1000
#define SSO_PF_CONST1			0x1008
#define SSO_PF_WQ_INT_PC		0x1020
#define SSO_PF_NW_TIM			0x1028
#define	SSO_PF_NOS_CNT			0x1040
#define SSO_PF_AW_WE			0x1080
#define SSO_PF_WS_CFG			0x1088
#define SSO_PF_PAGE_CNT			0x1090
#define	SSO_PF_GWE_CFG			0x1098
#define SSO_PF_GWE_RANDOM		0x10B0
#define SSO_PF_AW_STATUS		0x10E0
#define SSO_PF_AW_CFG			0x10F0
#define SSO_PF_RESET			0x10F8
#define SSO_PF_ACTIVE_CYCLES0		0x1100
#define SSO_PF_ACTIVE_CYCLES1		0x1108
#define SSO_PF_ACTIVE_CYCLES2		0x1110
#define SSO_PF_BIST_STATUS0		0x1200
#define SSO_PF_BIST_STATUS1		0x1208
#define SSO_PF_BIST_STATUS2		0x1210
#define SSO_PF_ERR0			0x1220
#define SSO_PF_ERR0_W1S			0x1228
#define SSO_PF_ERR0_ENA_W1C		0x1230
#define SSO_PF_ERR0_ENA_W1S		0x1238
#define SSO_PF_ERR1			0x1240
#define SSO_PF_ERR1_W1S			0x1248
#define SSO_PF_ERR1_ENA_W1C		0x1250
#define SSO_PF_ERR1_ENA_W1S		0x1258
#define SSO_PF_ERR2			0x1260
#define SSO_PF_ERR2_W1S			0x1268
#define SSO_PF_ERR2_ENA_W1C		0x1270
#define SSO_PF_ERR2_ENA_W1S		0x1278
#define SSO_PF_UNMAP_INFO		0x12f0
#define SSO_PF_UNMAP_INFO2		0x1300
#define SSO_PF_MBOX_INT			0x1400
#define SSO_PF_MBOX_INT_W1S		0x1440
#define SSO_PF_MBOX_ENA_W1C		0x1480
#define SSO_PF_MBOX_ENA_W1S		0x14C0
#define SSO_PF_AW_INP_CTL		0x2070
#define SSO_PF_AW_ADD			0x2080
#define SSO_PF_AW_READ_ARB		0x2090
#define SSO_PF_AW_TAG_REQ_PC		0x20A0
#define SSO_PF_AW_TAG_LATENCY_PC	0x20A8
#define SSO_PF_XAQ_REQ_PC		0x20B0
#define SSO_PF_XAQ_LATENCY_PC		0x20B8
#define SSO_PF_TAQ_CNT			0x20C0
#define SSO_PF_TAQ_ADD			0x20E0
#define SSO_PF_XAQ_AURA			0x2100
#define SSO_PF_XAQ_GMCTL		0x2110
#define SSO_PF_MAPX(x)			(0x4000 | ((x) << 3))
#define SSO_PF_XAQX_HEAD_PTR(x)		(0x80000 | ((x) << 3))
#define SSO_PF_XAQX_TAIL_PTR(x)		(0x90000 | ((x) << 3))
#define SSO_PF_XAQX_HEAD_NEXT(x)	(0xA0000 | ((x) << 3))
#define SSO_PF_XAQX_TAIL_NEXT(x)	(0xB0000 | ((x) << 3))
#define SSO_PF_TIAQX_STATUS(x)		(0xC0000 | ((x) << 3))
#define SSO_PF_TOAQX_STATUS(x)		(0xD0000 | ((x) << 3))
#define SSO_PF_GRPX_IAQ_THR(x)		(0x20000000 | ((x) << 20))
#define SSO_PF_GRPX_TAQ_THR(x)		(0x20000100 | ((x) << 20))
#define SSO_PF_GRPX_PRI(x)		(0x20000200 | ((x) << 20))
#define SSO_PF_GRPX_XAQ_LIMIT(x)	(0x20000220 | ((x) << 20))
#define SSO_PF_VHGRPX_MBOX(x, y)	(0x20000400 | ((x) << 20) | \
					 ((y) << 3))
#define SSO_PF_GRPX_WS_PC(x)		(0x20001000 | ((x) << 20))
#define SSO_PF_GRPX_EXT_PC(x)		(0x20001100 | ((x) << 20))
#define SSO_PF_GRPX_WA_PC(x)		(0x20001200 | ((x) << 20))
#define SSO_PF_GRPX_TS_PC(x)		(0x20001300 | ((x) << 20))
#define SSO_PF_GRPX_DS_PC(x)		(0x20001400 | ((x) << 20))
#define SSO_PF_HWSX_ARB(x)		(0x40000100 | ((x) << 20))
#define SSO_PF_HWSX_GMCTL(x)		(0x40000200 | ((x) << 20))
#define SSO_PF_HWSX_SX_GRPMASK(x, y)	(0x40001000 | ((x) << 20) | \
					 (((y) & 1) << 5))
#define SSO_PF_IPL_FREEX(x)		(0x80000000 | ((x) << 3))
#define SSO_PF_IPL_IAQ(x)		(0x80040000 | ((x) << 3))
#define SSO_PF_IPL_DESCHED(x)		(0x80060000 | ((x) << 3))
#define SSO_PF_IPL_CONF(x)		(0x80080000 | ((x) << 3))
#define SSO_PF_IENTX_TAG(x)		(0xA0000000 | ((x) << 3))
#define SSO_PF_IENTX_GRP(x)		(0xA0020000 | ((x) << 3))
#define SSO_PF_IENTX_PENDTAG(x)		(0xA0040000 | ((x) << 3))
#define SSO_PF_IENTX_LINKS(x)		(0xA0060000 | ((x) << 3))
#define SSO_PF_IENTX_QLINKS(x)		(0xA0080000 | ((x) << 3))
#define SSO_PF_IENTX_WQP(x)		(0xA00A0000 | ((x) << 3))
#define SSO_PF_TAQX_LINK(x)		(0xC0000000 | ((x) << 12))
#define SSO_PF_TAQX_WAEX_TAG(x, y)	(0xD0000000 | ((x) << 12) | ((y) << 4))
#define SSO_PF_TAQX_WAEX_WQP(x, y)	(0xD0000008 | ((x) << 12) | ((y) << 4))

/* SSO VF register offsets */
#define SSO_VF_VHGRPX_QCTL(x)		(0x10ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT(x)		(0x100ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT_W1S(x)	(0x108ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT_ENA_W1S(x)	(0x110ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT_ENA_W1C(x)	(0x118ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT_THR(x)	(0x140ULL | ((x) << 20))
#define SSO_VF_VHGRPX_INT_CNT(x)	(0x180ULL | ((x) << 20))
#define SSO_VF_VHGRPX_XAQ_CNT(x)	(0x1B0ULL | ((x) << 20))
#define SSO_VF_VHGRPX_AQ_CNT(x)		(0x1C0ULL | ((x) << 20))
#define SSO_VF_VHGRPX_AQ_THR(x)		(0x1E0ULL | ((x) << 20))
#define SSO_VF_VHGRPX_PF_MBOXX(x, y)	(0x200ULL | ((x) << 20) | ((y) << 3))

/* bar2 */
#define SSO_VF_VHGRPX_OP_ADD_WORK0(x)	(0x00ULL | ((x) << 20))
#define SSO_VF_VHGRPX_OP_ADD_WORK1(x)	(0x08ULL | ((x) << 20))

/* SSOW VF register offsets */
#define SSOW_VF_VHWSX_GRPMSK_CHGX(x, y) (0x80ULL | ((x) << 20) | ((y) << 3))
#define SSOW_VF_VHWSX_TAG(x)		(0x300ULL | ((x) << 20))
#define SSOW_VF_VHWSX_WQP(x)		(0x308ULL | ((x) << 20))
#define SSOW_VF_VHWSX_LINKS(x)		(0x310ULL | ((x) << 20))
#define SSOW_VF_VHWSX_PENDTAG(x)	(0x340ULL | ((x) << 20))
#define SSOW_VF_VHWSX_PENDWQP(x)	(0x348ULL | ((x) << 20))
#define SSOW_VF_VHWSX_SWTP(x)		(0x400ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_ALLOC_WE(x)	(0x410ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_UPD_WQP_GRP0(x)	(0x440ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_UPD_WQP_GRP1(x)	(0x448ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_UNTAG(x)	(0x490ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_CLR(x)	(0x820ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_DESCHED(x)	(0x860ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_DESCHED_NOSCH(x)	(0x870ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_DESCHED(x)	(0x8C0ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_NOSCHED(x)	(0x8D0ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTP_SET(x)	(0xC20ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_NORM(x)	(0xC80ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_FULL0(x)	(0xCA0ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_SWTAG_FULL1(x)	(0xCA8ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_CLR_NSCHED(x)	(0x10000ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_GET_WORK0(x)	(0x80000ULL | ((x) << 20))
#define SSOW_VF_VHWSX_OP_GET_WORK1(x)	(0x80008ULL | ((x) << 20))

#define SSO_CONST_GRP_SHIFT		0
#define SSO_CONST_GRP_MASK		0xffff
#define SSO_CONST_IUE_SHIFT		16
#define SSO_CONST_IUE_MASK		0xffff
#define SSO_CONST_HWS_SHIFT		56
#define SSO_CONST_HWS_MASK		0xff

#define SSO_CONST1_XAQ_BUF_SIZE_SHIFT	0
#define SSO_CONST1_XAQ_BUF_SIZE_MASK	0xffff
#define SSO_CONST1_XAE_WAES_SHIFT	16
#define SSO_CONST1_XAE_WAES_MASK	0xffff
#define SSO_CONST1_MAPS_SHIFT		32
#define SSO_CONST1_MAPS_MASK		0xfff

#define SSO_AW_WE_FREE_CNT_SHIFT        0
#define SSO_AW_WE_FREE_CNT_MASK         0x1fff
#define SSO_AW_WE_RSVD_CNT_SHIFT        16
#define SSO_AW_WE_RSVD_CNT_MASK         0x1fff

#define SSO_TAQ_CNT_FREE_CNT_SHIFT      0
#define SSO_TAQ_CNT_FREE_CNT_MASK       0x7ff
#define SSO_TAQ_CNT_RSVD_CNT_SHIFT      16
#define SSO_TAQ_CNT_RSVD_CNT_MASK       0x7ff

#define SSO_GRP_IAQ_THR_RSVD_SHIFT      0
#define SSO_GRP_IAQ_THR_RSVD_MASK       0x1fff
#define SSO_GRP_IAQ_THR_MAX_SHIFT       32
#define SSO_GRP_IAQ_THR_MAX_MASK        0x1fff

#define SSO_AW_ADD_RSVD_SHIFT           16
#define SSO_AW_ADD_RSVD_MASK            0x3fff

#define SSO_GRP_TAQ_THR_RSVD_SHIFT      0
#define SSO_GRP_TAQ_THR_RSVD_MASK       0x7ff
#define SSO_GRP_TAQ_THR_MAX_SHIFT       32
#define SSO_GRP_TAQ_THR_MAX_MASK        0x7ff

#define SSO_TAQ_ADD_RSVD_SHIFT          16
#define SSO_TAQ_ADD_RSVD_MASK           0x1fff

#define SSO_IENT_GRP_GRP_SHIFT		48
#define SSO_IENT_GRP_GRP_MASK		0xff

#define SSO_IENT_MAX			1024

#define SSO_ERR0			((0xffffffULL << 32) | 0xfff)
#define SSO_ERR1			0xffff
#define SSO_ERR2		((0xffffULL << 32) | (0x7ULL << 28) | 0x3fff)
#define SSO_MBOX			0xffffffffffffffffULL

#define SSO_MAP_GMID(x)			(((x) & 0xffff) << 0)
#define SSO_MAP_GGRP(x)			(((x) & 0x3ff) << 16)
#define SSO_MAP_VHGRP(x)		(((0ull | (x)) & 0x3fULL) << 32)
#define SSO_MAP_VALID(x)		(((0ull | (x)) & 0x1ULL) << 63)

#define SSO_VF_INT			0x100000000000000fULL
#define SSOW_RAM_MBOX_SIZE		0x10000
#define SSOW_RAM_MBOX(x)		(0x1400000 | ((x) << 16))

struct ssopf_vf {
	struct octeontx_pf_vf	domain;
	struct mbox		mbox;
	u64			grp_mask;
};

struct ssopf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	int			id;
	struct msix_entry	*msix_entries;
	struct list_head	list;

	int			total_vfs;
	int			vfs_in_use;
#define SSO_SRIOV_ENABLED	0x1
	u32			flags;

	u32			xaq_buf_size;
	u32			num_iue;
	struct ssopf_vf		vf[SSO_MAX_VF];
	struct work_struct	mbox_work;
};

struct ssopf_com_s {
	u64 (*create_domain)(u32 id, u16 domain_id, u32 num_grps,
			     void *master, void *master_data,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*send_message)(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			    union mbox_data *, union mbox_data *);
	int (*set_mbox_ram)(u32 node, u16 domain_id,
			    void *mbox_addr, u64 mbox_size);
	int (*get_vf_count)(u32 id);
};

extern struct ssopf_com_s ssopf_com;

struct ssowpf_vf {
	struct octeontx_pf_vf	domain;
	void			*ram_mbox_addr;
};

struct ssowpf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	int			id;
	struct list_head	list;

	int			total_vfs;
	int			vfs_in_use;
#define SSOW_SRIOV_ENABLED	0x1
	u32			flags;

	struct ssowpf_vf	vf[SSOW_MAX_VF];
};

struct ssowpf_com_s {
	u64 (*create_domain)(u32 id, u16 domain_id, u32 num_grps,
			     void *master, void *master_data,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id, u64 grp_mask);
	int (*receive_message)(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp);
	int (*get_vf_count)(u32 id);
	int (*get_ram_mbox_addr)(u32 node, u16 domain_id,
				 void **ram_mbox_addr);
};

extern struct ssowpf_com_s ssowpf_com;

int sso_pf_set_value(u32 id, u64 offset, u64 val);
int sso_pf_get_value(u32 id, u64 offset, u64 *val);
int sso_vf_get_value(u32 id, int vf_id, u64 offset, u64 *val);

#endif
