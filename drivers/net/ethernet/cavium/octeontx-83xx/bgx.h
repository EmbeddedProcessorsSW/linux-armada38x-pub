// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef __BGX_H__
#define __BGX_H__

#include <linux/pci.h>
#include <linux/types.h>
#include "octeontx.h"

struct bgx_com_s {
	int (*create_domain)(u32 id, u16 domain_id,
			     struct octtx_bgx_port *port_tbl, int ports,
			     struct octeontx_master_com_t *com, void *domain,
			     struct kobject *kobj);
	int (*destroy_domain)(u32 id, u16 domain_id, struct kobject *kobj);
	int (*reset_domain)(u32 id, u16 domain_id);
	int (*receive_message)(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			       union mbox_data *req, union mbox_data *resp,
			       void *mdata);
	int (*get_num_ports)(int node);
	int (*get_link_status)(int node, int bgx, int lmac);
	struct octtx_bgx_port* (*get_port_by_chan)(int node, u16 domain_id,
						   int chan);
	int (*set_pkind)(u32 id, u16 domain_id, int port, int pkind);
	int (*get_port_stats)(struct octtx_bgx_port *port);
};

struct bgx_com_s *bgx_octeontx_init(void);

#endif /* __BGX_H__ */

