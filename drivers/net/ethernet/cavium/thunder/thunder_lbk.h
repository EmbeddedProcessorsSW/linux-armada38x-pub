// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#ifndef THUNDER_LBK_H
#define THUNDER_LBK_H

struct thunder_lbk_com_s {
	int (*port_start)(void);
	void (*port_stop)(void);
	int (*get_port_pkind)(void);
};

extern struct thunder_lbk_com_s thunder_lbk_com;

#endif /* THUNDER_LBK_H */
