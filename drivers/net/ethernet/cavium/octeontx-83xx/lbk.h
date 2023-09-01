// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef __LBK_H__
#define __LBK_H__

#include <linux/pci.h>
#include <linux/types.h>
#include "octeontx.h"

struct lbk_com_s {
	int (*create_domain)(u32 id, u16 domain_id,
			     struct octtx_lbk_port *port_tbl, int ports,
			     struct octeontx_master_com_t *com, void *domain,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			       union mbox_data *req, union mbox_data *resp,
			       void *mdata);
	int (*get_num_ports)(int node);
	struct octtx_lbk_port* (*get_port_by_chan)(int node, u16 domain_id,
						   int chan);
};

extern struct lbk_com_s lbk_com;

#endif /* __LBK_H__ */

