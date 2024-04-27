// SPDX-License-Identifier: GPL-2.0
/* Marvell RVU Admin Function driver
 *
 * Copyright (C) 2024 Marvell.
 *
 */

#include <linux/interrupt.h>
#include <linux/irq.h>

#include "rvu_trace.h"
#include "mbox.h"
#include "reg.h"
#include "api.h"

int cn20k_rvu_get_mbox_regions(struct rvu *rvu, void **mbox_addr,
			       int num, int type, unsigned long *pf_bmap)
{
	struct page *page;
	int region;

	for (region = 0; region < num; region++) {
		if (!test_bit(region, pf_bmap))
			continue;

		page = phys_to_page(rvu_read64(rvu, BLKADDR_MBOX,
					       RVU_MBOX_AF_PFX_ADDR(region)));

		mbox_addr[region] = (void *)page_to_virt(page);

		if (!mbox_addr[region])
			goto error;
	}
	return 0;

error:
	return -ENOMEM;
}

int cn20k_rvu_mbox_init(struct rvu *rvu, int type, int ndevs)
{
	int dev;

	if (!is_cn20k(rvu->pdev))
		return 0;

	if (type == TYPE_AFPF) {
		for (dev = 0; dev < ndevs; dev++)
			rvu_write64(rvu, BLKADDR_MBOX,
				    RVU_MBOX_AF_PFX_CFG(dev), ilog2(MBOX_SIZE));
	}

	return 0;
}
