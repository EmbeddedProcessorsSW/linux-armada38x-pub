// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef FPA_H
#define FPA_H

#include <linux/pci.h>
#include <linux/types.h>

#include "octeontx.h"

/* PCI DEV IDs */
#define PCI_DEVICE_ID_OCTEONTX_FPA_PF	0xA052
#define PCI_DEVICE_ID_OCTEONTX_FPA_VF	0xA053

#define PCI_FPA_PF_CFG_BAR		0
#define PCI_FPA_PF_MSIX_BAR		4

#define PCI_FPA_VF_CFG_BAR		0
#define PCI_FPA_VF_MSIX_BAR		4

#define FPA_PF_MSIX_COUNT		2
#define FPA_VF_MSIX_COUNT		1
#define FPA_MAX_VF			32

/* FPA PF register offsets */
#define FPA_PF_SFT_RST			0x0
#define FPA_PF_CONST			0x10
#define FPA_PF_CONST1			0x18
#define FPA_PF_GEN_CFG			0x50
#define FPA_PF_ECC_CTL			0x58
#define FPA_PF_ECC_INT			0x68
#define FPA_PF_ECC_INT_W1S		0x70
#define FPA_PF_ECC_INT_ENA_W1S		0x78
#define FPA_PF_ECC_INT_ENA_W1C		0x80
#define FPA_PF_STATUS			0xC0
#define FPA_PF_INP_CTL			0xD0
#define FPA_PF_CLK_COUNT		0xF0
#define FPA_PF_RED_DELAY		0x100
#define FPA_PF_GEN_INT			0x140
#define FPA_PF_GEN_INT_W1S		0x148
#define FPA_PF_GEN_INT_ENA_W1S		0x150
#define FPA_PF_GEN_INT_ENA_W1C		0x158
#define FPA_PF_ADDR_RANGE_ERROR		0x458
#define FPA_PF_UNMAP_INFO		0x460
#define FPA_PF_MAPX(x)			(0x1000 | ((x) << 3))
#define FPA_PF_VFX_GMCTL(x)		(0x40001000 | ((x) << 20))
#define FPA_PF_POOLX_CFG(x)		(0x40004100 | ((x) << 20))
#define FPA_PF_POOLX_FPF_MARKS(x)	(0x40004110 | ((x) << 20))
#define FPA_PF_POOLX_STACK_BASE(x)	(0x40004220 | ((x) << 20))
#define FPA_PF_POOLX_STACK_END(x)	(0x40004230 | ((x) << 20))
#define FPA_PF_POOLX_STACK_ADDR(x)	(0x40004240 | ((x) << 20))
#define FPA_PF_POOLX_OP_PC(x)		(0x40004280 | ((x) << 20))
#define FPA_PF_AURAX_POOL(x)		(0x40008100 | ((x) << 16))
#define FPA_PF_AURAX_CFG(x)		(0x40008110 | ((x) << 16))
#define FPA_PF_AURAX_POOL_LEVELS(x)	(0x40008300 | ((x) << 16))
#define FPA_PF_AURAX_CNT_LEVELS(x)	(0x40008310 | ((x) << 16))

#define FPA_VF_OFFSET(x)		(0x400000000 | (0x400000 * (x)))
#define FPA_VF_CFG_SIZE			0x400000

/* FPA VF register offsets */
#define FPA_VF_INT(x)			(0x200ULL | ((x) << 22))
#define FPA_VF_INT_W1S(x)		(0x210ULL | ((x) << 22))
#define FPA_VF_INT_ENA_W1S(x)		(0x220ULL | ((x) << 22))
#define FPA_VF_INT_ENA_W1C(x)		(0x230ULL | ((x) << 22))
#define FPA_VF_VHPOOL_AVAILABLE(x)	(0x4150ULL | ((x) << 22))
#define FPA_VF_VHPOOL_THRESHOLD(x)	(0x4160ULL | ((x) << 22))
#define FPA_VF_VHPOOL_START_ADDR(x)	(0x4200ULL | ((x) << 22))
#define FPA_VF_VHPOOL_END_ADDR(x)	(0x4210ULL | ((x) << 22))
#define FPA_VF_VHAURA_CNT(x)		(0x20120ULL | ((x) << 18))
#define FPA_VF_VHAURA_CNT_ADD(x)	(0x20128ULL | ((x) << 18))
#define FPA_VF_VHAURA_CNT_LIMIT(x)	(0x20130ULL | ((x) << 18))
#define FPA_VF_VHAURA_CNT_THRESHOLD(x)	(0x20140ULL | ((x) << 18))
#define FPA_VF_VHAURA_OP_ALLOC(x)	(0x30000ULL | ((x) << 18))
#define FPA_VF_VHAURA_OP_FREE(x)	(0x38000ULL | ((x) << 18))

#define GEN_CFG_CLK_OVERRIDE_ENABLE	(0x1 << 0)
#define GEN_CFG_CLK_OVERRIDE_DISABLE	(0x0 << 0)
#define GEN_CFG_AVG_EN_ENABLE		(0x1 << 1)
#define GEN_CFG_AVG_EN_DISABLE		(0x0 << 1)
#define GEN_CFG_FPA_POOL_16		(0x2 << 2)
#define GEN_CFG_FPA_POOL_32		(0x1 << 2)
#define GEN_CFG_LVL_DLY(x)		(((x) & 0x3f) << 4)
#define GEN_CFG_OCLA_BP_ENABLE		(0x1 << 10)
#define GEN_CFG_OCLA_BP_DISABLE		(0x0 << 10)
#define GEN_CFG_HALFRATE_ENABLE		(0x1 << 11)
#define GEN_CFG_HALFRATE_DISABLE	(0x0 << 11)
#define	GEN_CFG_DWBQ(x)			(((x) & 0x3f) << 12)

/* PF_GEN_CFG falgs default values */
#define DEF_GEN_CFG_FLAGS	(GEN_CFG_CLK_OVERRIDE_DISABLE |	\
	GEN_CFG_AVG_EN_DISABLE | GEN_CFG_FPA_POOL_32 |		\
	GEN_CFG_LVL_DLY(0x3) | GEN_CFG_OCLA_BP_DISABLE |	\
	GEN_CFG_DWBQ(0x3f))

#define FPA_CONST_POOLS_SHIFT		0
#define FPA_CONST_POOLS_MASK		0xffff
#define FPA_CONST_AURAS_SHIFT		16
#define FPA_CONST_AURAS_MASK		0xffff
#define FPA_CONST_STACK_LN_PTRS_SHIFT	48
#define FPA_CONST_STACK_LN_PTRS_MASK	0xff
#define FPA_CONST1_MAPS_SHIFT		0
#define FPA_CONST1_MAPS_MASK		0xfff

#define FPA_ECC_RAM_SBE_SHIFT		0
#define FPA_ECC_RAM_SBE_MASK		0x1fffff
#define FPA_ECC_RAM_DBE_SHIFT		32
#define FPA_ECC_RAM_DBE_MASK		0x1fffff

#define FPA_GEN_INT_GMID0_MASK		0x1
#define FPA_GEN_INT_GMID_UNMAP_MASK	0x2
#define FPA_GEN_INT_GMID_MULTI_MASK	0x4
#define FPA_GEN_INT_FREE_DIS_MASK	0x8
#define FPA_GEN_INT_ALLOC_DIS_MASK	0x10

#define FPA_UNMAP_INFO_GMID_SHIFT	0
#define FPA_UNMAP_INFO_GMID_MASK	0xffff
#define FPA_UNMAP_INFO_GAURA_SHIFT	16
#define FPA_UNMAP_INFO_GAURA_MASK	0xffff

#define POOL_ENA			(0x1 << 0)
#define POOL_DIS			(0x0 << 0)
#define POOL_SET_NAT_ALIGN		(0x1 << 1)
#define POOL_DIS_NAT_ALIGN		(0x0 << 1)
#define POOL_STYPE(x)			(((x) & 0x1) << 2)
#define POOL_LTYPE(x)			(((x) & 0x3) << 3)
#define POOL_BUF_OFFSET(x)		(((x) & 0x7fffULL) << 16)
#define POOL_BUF_SIZE(x)		(((x) & 0x7ffULL) << 32)

#define FPA_MAP_GMID(x)			(((x) & 0xffffULL) << 0)
#define FPA_MAP_GAURASET(x)		(((x) & 0xffULL) << 16)
#define FPA_MAP_VHAURASET(x)		(((x) & 0x1fULL) << 32)
#define FPA_MAP_VALID(x)		(((x) & 0x1ULL) << 63)

#define FPA_FREE_ADDRS_S(x, y)		((x) | (((y) & 0x1ff) << 3))

#define FPA_LN_SIZE			128
#define FPA_AURA_SET_SIZE		16

#define get_pool(vf_id) (vf_id)
#define get_aura_set(vf_id) (vf_id)
#define FPA_SSO_XAQ_GMID		0x2
#define FPA_SSO_XAQ_AURA		0x0
#define FPA_PKO_DPFI_GMID		0x3
#define FPA_PKO_DPFI_AURA		0x0

struct fpapf_vf {
	struct octeontx_pf_vf	domain;

	u32			hardware_pool;
	u32			hardware_aura_set;
	void			*stack_base_iova;
	u64			stack_size;
	u64			buf_size;
};

struct fpapf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	int			id;
	struct msix_entry	*msix_entries;
	struct list_head	list;

	int			stack_ln_ptrs;
	int			total_vfs;
	int			vfs_in_use;
#define FPA_SRIOV_ENABLED	0x1
	u32			flags;

	struct fpapf_vf		vf[FPA_MAX_VF];
};

/*fpapf_com_s will be used by users
 * to communicate with fpapf, user can create/ remove domains.
 * create_domain: nodeid, domain_id, num_vfs
 * free_domain: nodeid, domain_id
 */
struct fpapf_com_s {
	u64 (*create_domain)(u32 id, u16 domain_id, u32 num_vfs,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp,
			       void *add_data);
	int (*get_vf_count)(u32 node_id);
};

extern struct fpapf_com_s fpapf_com;

struct memvec {
	void			*addr;
	dma_addr_t		iova;
	u32			size;
	bool			in_use;
};

struct fpavf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	struct msix_entry	*msix_entries;
	struct list_head	list;
	u32			ref_count;

	bool			setup_done;
	u16			domain_id;
	u16			subdomain_id;
	u64			num_buffers;
	u64			alloc_thold;

	/* VA of pool memory */
	u64                     vhpool_memvec_size;
	struct memvec           *vhpool_memvec;
	struct device           *vhpool_owner;
	atomic_t		alloc_count;
	u32			stack_ln_ptrs;
	void			*pool_addr;
	dma_addr_t		pool_iova;
	u64			pool_size;
	u64			buf_len;
	struct iommu_domain	*iommu_domain;

	struct octeontx_master_com_t *master;
	void			*master_data;
};

struct fpavf_com_s {
	struct fpavf* (*get)(u16 domain_id, u16 subdomain_id,
			     struct octeontx_master_com_t *master,
			     void *master_data);
	int (*setup)(struct fpavf *, u64 num_buffers, u32 buf_len,
		     struct device *owner);
	void (*free)(struct fpavf*, u32 aura, u64 addr, u32 dwb_count);
	u64 (*alloc)(struct fpavf*, u32 aura);
	void (*add_alloc)(struct fpavf *fpa, int count);
	int (*teardown)(struct fpavf *fpa);
	void (*put)(struct fpavf *fpa);
};

extern struct fpavf_com_s fpavf_com;

#endif
