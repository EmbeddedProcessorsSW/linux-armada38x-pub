// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef __SLI_H__
#define __SLI_H__

#include <linux/pci.h>
#include <linux/types.h>
#include "octeontx.h"

/* PCI DEV IDs */
#define PCI_DEVICE_ID_OCTEONTX_SLI_PF	0xA038

#define PCI_SLI_PF_CFG_BAR		0
#define PCI_SLI_PF_MSIX_BAR		4

#define SLI_LMAC_MAX_PFS		1

#define SLI_SCRATCH1			(0x1000ULL)
#define SLI_SCRATCH2			(0x1010ULL)
#define SDP_SCRATCHX(x)			(0x80020180ULL | ((x) << 23))
#define SLI_EPFX_SCRATCH(x)		(0x80028100ULL | ((x) << 23))

#define SDP_CONST			(0x880300ULL)
#define SDP_CONST			(0x880300ULL)
#define SLI_LMAC_CONST0X(x)		(0x1004000ULL | ((x) << 4))
#define SLI_LMAC_CONST1X(x)		(0x1004008ULL | ((x) << 4))

#define SDP_OUT_WMARK			(0x880000ULL)
#define SDP_GBL_CONTROL			(0x880200ULL)
#define SDP_GBL_CONTROL_BPKIND_MASK     0x3f
#define SDP_GBL_CONTROL_BPKIND_SHIFT	8
#define SDP_OUT_BP_ENx_W1C(x)		(0x880220ULL | ((x) << 4))
#define SDP_OUT_BP_ENx_W1S(x)		(0x880240ULL | ((x) << 4))
#define SDP_PKIND_VALID			(0x880210ULL)

#define SDP_CHANNEL_START		0x400
#define SDP_HOST_LOADED			0xDEADBEEFULL
#define SDP_GET_HOST_INFO		0xBEEFDEEDULL
#define SDP_HOST_INFO_RECEIVED		0xDEADDEULL
#define SDP_HANDSHAKE_COMPLETED		0xDEEDDEEDULL

#define SDP_HOST_EPF_APP_BASE		0x1
#define SDP_HOST_EPF_APP_NIC		0x2

struct sli_epf {
	int hs_done;
	int app_mode;
	int pf_srn;
	int rppf;
	int num_vfs;
	int vf_srn;
	int rpvf;
};

struct slipf {
	struct pci_dev *pdev;
	void __iomem *reg_base;
	int id;
	struct msix_entry *msix_entries;
	struct list_head    list; /* List of SLI devices */
	int sli_idx; /* CPU-local SLI device index.*/
	int port_count;
	int node; /* CPU node */
	u32 flags;
	struct sli_epf epf[SLI_LMAC_MAX_PFS];
	u64 ticks_per_us;
};

struct slipf_com_s {
	int (*create_domain)(u32 id, u16 domain_id,
			     struct octtx_sdp_port *port_tbl, int ports,
			     struct octeontx_master_com_t *com, void *domain,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			       union mbox_data *req, union mbox_data *resp,
				void *mdata);
	int (*get_num_ports)(int node);
	bool (*get_link_status)(int node, int sdp, int lmac);
	int (*set_pkind)(u32 id, u16 domain_id, int port, int pkind);
};

extern struct slipf_com_s slipf_com;

static inline void set_sdp_field(u64 *ptr, u64 field_mask,
				 u8 field_shift, u64 val)
{
	*ptr &= ~(field_mask << field_shift);
	*ptr |= (val & field_mask) << field_shift;
}

#endif /* __SLI_H__ */
