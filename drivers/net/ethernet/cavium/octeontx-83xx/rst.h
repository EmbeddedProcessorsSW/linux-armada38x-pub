// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef RST_H
#define RST_H

#define PCI_DEVICE_ID_OCTEONTX_RST_PF	0xA00E

#define PCI_RST_PF_CFG_BAR	0

/* reg offset */
#define RST_BOOT	0x1600

#define PLL_REF_CLK	(50 * 1000 * 1000)

struct rst_com_s {
	u64 (*get_sclk_freq)(int id);
	u64 (*get_rclk_freq)(int id);
};

extern struct rst_com_s rst_com;

#endif
