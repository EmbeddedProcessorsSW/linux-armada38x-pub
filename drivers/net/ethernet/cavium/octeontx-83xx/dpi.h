// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef DPI_H
#define	DPI_H

#include <linux/pci.h>
#include "octeontx.h"

#define DPI_DMA_CMD_SIZE  32

/* PCI device IDs */
#define	PCI_DEVICE_ID_OCTEONTX_DPI_PF	 0xA057
#define PCI_DEVICE_ID_OCTEONTX_DPI_VF	 0xA058

#define DPI_MAX_ENGINES     6
#define DPI_MAX_VFS	    8
#define DPI_MAX_REQQ_INT    8
#define DPI_MAX_CC_INT	    64

#define DPI_DMA_DBE_INT   0x4A
#define DPI_DMA_SBE_INT   0x49
#define DPI_DMA_INT_REG   0x48
#define DPI_DMA_REQQ_INT   0x40

/* PCI BAR nos */
#define	PCI_DPI_PF_CFG_BAR	  0
#define	PCI_DPI_PF_MSIX_BAR   4
#define	PCI_DPI_VF_CFG_BAR	  0
#define	PCI_DPI_VF_MSIX_BAR   4
#define DPI_VF_CFG_SIZE		  0x100000
#define DPI_VF_OFFSET(x)	  (0x20000000 | 0x100000 * (x))

/* MSI-X interrupts */
#define	DPI_PF_MSIX_COUNT		75
#define	DPI_VF_MSIX_COUNT		1

/* TODO: Need to define proper values. */
#define INST_AURA 1
#define INST_STRM 1
#define DMA_STRM 1

#define DPI_INT  0x0
#define DPI_SBE  0x0
#define DPI_DBE  0x0
#define DPI_REQQ  0x0
#define DPI_DMA_CC	0x0

/****************  Macros for register modification ************/
#define DPI_DMA_IBUFF_CSIZE_CSIZE(x)	((x) & 0x1fff)
#define DPI_DMA_IBUFF_CSIZE_GET_CSIZE(x) ((x) & 0x1fff)

#define DPI_DMA_IDS_INST_AURA(x)	((uint64_t)((x) & 0xfff) << 48)
#define DPI_DMA_IDS_GET_INST_AURA(x)	(((x) >> 48) & 0xfff)

#define DPI_DMA_IDS_INST_STRM(x)	((uint64_t)((x) & 0xff) << 40)
#define DPI_DMA_IDS_GET_INST_STRM(x)	(((x) >> 40) & 0xff)

#define DPI_DMA_IDS_DMA_STRM(x)		((uint64_t)((x) & 0xff) << 32)
#define DPI_DMA_IDS_GET_DMA_STRM(x)	(((x) >> 32) & 0xff)

#define DPI_DMA_IDS_GMID(x)		((x) & 0xffff)
#define DPI_DMA_IDS_GET_GMID(x)		((x) & 0xffff)

#define DPI_ENG_BUF_BLKS(x)		((x) & 0x1fULL)
#define DPI_ENG_BUF_GET_BLKS(x)		((x) & 0x1fULL)

#define DPI_ENG_BUF_BASE(x)		(((x) & 0x3fULL) << 16)
#define DPI_ENG_BUF_GET_BASE(x)		(((x) >> 16) & 0x3fULL)

#define DPI_DMA_ENG_EN_QEN(x)		((x) & 0xffULL)
#define DPI_DMA_ENG_EN_GET_QEN(x)	((x) & 0xffULL)

#define DPI_DMA_ENG_EN_MOLR(x)		(((x) & 0x7fULL) << 32)
#define DPI_DMA_ENG_EN_GET_MOLR(x)	(((x) >> 32) & 0x7fULL)

#define DPI_DMA_CONTROL_DMA_ENB(x)	(((x) & 0x3fULL) << 48)
#define DPI_DMA_CONTROL_GET_DMA_ENB(x)	(((x) >> 48) & 0x3fULL)

#define DPI_DMA_CONTROL_O_MODE		(0x1ULL << 14)
#define DPI_DMA_CONTROL_O_NS		(0x1ULL << 17)
#define DPI_DMA_CONTROL_O_RO		(0x1ULL << 18)
#define DPI_DMA_CONTROL_O_ADD1		(0x1ULL << 19)
#define DPI_DMA_CONTROL_LDWB		(0x1ULL << 32)
#define DPI_DMA_CONTROL_NCB_TAG_DIS	(0x1ULL << 34)
#define DPI_DMA_CONTROL_ZBWCSEN		(0x1ULL << 39)
#define DPI_DMA_CONTROL_WQECSDIS	(0x1ULL << 47)
#define DPI_DMA_CONTROL_UIO_DIS		(0x1ULL << 55)
#define DPI_DMA_CONTROL_PKT_EN		(0x1ULL << 56)
#define DPI_DMA_CONTROL_FFP_DIS		(0x1ULL << 59)

#define DPI_CTL_EN			 (0x1ULL)
/******************** macros for Interrupts ************************/
#define DPI_INT_REG_NFOVR				 (0x1ULL << 1)
#define DPI_INT_REG_NDERR				 (0x1ULL)
#define DPI_SBE_INT_RDB_SBE				 (0x1ULL)
#define DPI_DBE_INT_RDB_DBE				 (0x1ULL)
#define DPI_DMA_CC_INT					 (0x1ULL)

#define DPI_REQQ_INT_INSTRFLT			 (0x1ULL)
#define DPI_REQQ_INT_RDFLT				 (0x1ULL << 1)
#define DPI_REQQ_INT_WRFLT				 (0x1ULL << 2)
#define DPI_REQQ_INT_CSFLT				 (0x1ULL << 3)
#define DPI_REQQ_INT_INST_DBO			 (0x1ULL << 4)
#define DPI_REQQ_INT_INST_ADDR_NULL		 (0x1ULL << 5)
#define DPI_REQQ_INT_INST_FILL_INVAL	 (0x1ULL << 6)

#define DPI_REQQ_INT \
	(DPI_REQQ_INT_INSTRFLT		  | \
	DPI_REQQ_INT_RDFLT			 | \
	DPI_REQQ_INT_WRFLT			 | \
	DPI_REQQ_INT_CSFLT			 | \
	DPI_REQQ_INT_INST_DBO		 | \
	DPI_REQQ_INT_INST_ADDR_NULL  | \
	DPI_REQQ_INT_INST_FILL_INVAL)

/***************** Registers ******************/
#define DPI_DMAX_IBUFF_CSIZE(x)		  (0x0ULL | ((x) << 11))
#define DPI_DMAX_REQBANK0(x)		  (0x8ULL | ((x) << 11))
#define DPI_DMAX_REQBANK1(x)		  (0x10ULL | ((x) << 11))
#define DPI_DMAX_IDS(x)				  (0x18ULL | ((x) << 11))
#define DPI_DMAX_IFLIGHT(x)			  (0x20ULL | ((x) << 11))
#define DPI_DMAX_QRST(x)			  (0x28ULL | ((x) << 11))
#define DPI_DMAX_ERR_RSP_STATUS(x)	  (0x30ULL | ((x) << 11))

#define DPI_BIST_STATUS			   (0x4000ULL)
#define DPI_ECC_CTL			   (0x4008ULL)
#define DPI_CTL			   (0x4010ULL)
#define DPI_DMA_CONTROL			   (0x4018ULL)
#define DPI_DMA_ENGX_EN(x)			  (0x4040ULL | ((x) << 3))
#define DPI_REQ_ERR_RSP			   (0x4078ULL)
#define DPI_REQ_ERR_RESP_EN			   (0x4088ULL)
#define DPI_PKT_ERR_RSP			   (0x4098ULL)
#define DPI_NCBX_CFG(x)			   (0x40A0ULL | ((x) << 3))
#define DPI_ENGX_BUF(x)			   (0x40C0ULL | ((x) << 3))
#define DPI_SLI_PRTX_CFG(x)			   (0x4100ULL | ((x) << 3))
#define DPI_SLI_PRTX_ERR(x)			   (0x4120ULL | ((x) << 3))
#define DPI_SLI_PRTX_ERR_INFO(x)			(0x4140ULL | ((x) << 3))
#define DPI_INFO_REG			(0x4160ULL)
#define DPI_INT_REG			   (0x4168ULL)
#define DPI_INT_REG_W1S			   (0x4170ULL)
#define DPI_INT_ENA_W1C			   (0x4178ULL)
#define DPI_INT_ENA_W1S			   (0x4180ULL)
#define DPI_SBE_INT			   (0x4188ULL)
#define DPI_SBE_INT_W1S			   (0x4190ULL)
#define DPI_SBE_INT_ENA_W1C			   (0x4198ULL)
#define DPI_SBE_INT_ENA_W1S			   (0x41A0ULL)
#define DPI_DBE_INT			   (0x41A8ULL)
#define DPI_DBE_INT_W1S			   (0x41B0ULL)
#define DPI_DBE_INT_ENA_W1C			   (0x41B8ULL)
#define DPI_DBE_INT_ENA_W1S		   (0x41C0ULL)

#define DPI_DMA_CCX_INT(x)			  (0x5000ULL | ((x) << 3))
#define DPI_DMA_CCX_INT_W1S(x)			  (0x5400ULL | ((x) << 3))
#define DPI_DMA_CCX_INT_ENA_W1C(x)		  (0x5800ULL | ((x) << 3))
#define DPI_DMA_CCX_INT_ENA_W1S(x)		  (0x5C00ULL | ((x) << 3))
#define DPI_DMA_CCX_CNT(x)			  (0x6000ULL | ((x) << 3))
#define DPI_REQQX_INT(x)			  (0x6600ULL | ((x) << 3))
#define DPI_REQQX_INT_W1S(x)			  (0x6640ULL | ((x) << 3))
#define DPI_REQQX_INT_ENA_W1C(x)		  (0x6680ULL | ((x) << 3))
#define DPI_REQQX_INT_ENA_W1S(x)		  (0x66C0ULL | ((x) << 3))

#define DPI_SLI_MRRS_MIN		128
#define DPI_SLI_MRRS_MAX		1024
#define DPI_SLI_MPS_MIN			128
#define DPI_SLI_MPS_MAX			512
#define DPI_SLI_MAX_PORTS		4
#define DPI_SLI_PRTX_CFG_MPS(x)		(((x) & 0x7) << 4)
#define DPI_SLI_PRTX_CFG_MRRS(x)	(((x) & 0x7) << 0)

/* VF Registers: */
#define DPI_VDMA_EN		(0X0ULL)
#define DPI_VDMA_REQQ_CTL	  (0X8ULL)
#define DPI_VDMA_DBELL	   (0X10ULL)
#define DPI_VDMA_SADDR	   (0X18ULL)
#define DPI_VDMA_COUNTS		(0X20ULL)
#define DPI_VDMA_NADDR	   (0X28ULL)
#define DPI_VDMA_IWBUSY		(0X30ULL)
#define DPI_VDMA_CNT	 (0X38ULL)
#define DPI_VF_INT	   (0X100ULL)
#define DPI_VF_INT_W1S	   (0X108ULL)
#define DPI_VF_INT_ENA_W1C	   (0X110ULL)
#define DPI_VF_INT_ENA_W1S	   (0X118ULL)

/***************** Structures *****************/
struct dpipf_vf {
	struct octeontx_pf_vf domain;
};

struct dpipf {
	struct pci_dev *pdev;
	void __iomem *reg_base;	/* Register start address */
	int id;
	struct msix_entry *msix_entries;
	struct list_head list;

	int total_vfs;
	int vfs_in_use;
#define DPI_SRIOV_ENABLED 1
	unsigned int flags;

	/*TODO:
	 * Add any members specific to DPI if required.
	 */
	struct dpipf_vf vf[DPI_MAX_VFS];
};

struct dpipf_com_s {
	u64 (*create_domain)(u32 id, u16 domain_id, u32 num_vfs,
			     void *master, void *master_data,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr, union mbox_data *req,
			       union mbox_data *resp, void *add_data);
	int (*get_vf_count)(u32 id);
};

extern struct dpipf_com_s dpipf_com;

struct dpivf {
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

struct dpivf_com_s {
	struct dpivf* (*get)(u16 domain_id, u16 subdomain_id,
			     struct octeontx_master_com_t *master,
			     void *master_data);
	int (*setup)(struct dpivf *dpivf);
	void (*close)(struct dpivf *dpivf);
};

extern struct dpivf_com_s dpivf_com;

#endif /* DPI_H */
