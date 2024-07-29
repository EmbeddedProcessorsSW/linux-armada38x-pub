/* SPDX-License-Identifier: GPL-2.0 */
/* Marvell RVU Admin Function driver
 *
 * Copyright (C) 2024 Marvell.
 *
 */

#ifndef RVUM_REG_H
#define RVUM_REG_H

/* CN20K specific registers */
#define RVU_PF_DISC			(0x0)
#define RVU_PRIV_PFX_DISC(a)		(0x8000208 | (a) << 16)
#define RVU_PRIV_HWVFX_DISC(a)		(0xD000000 | (a) << 12)
#define RVU_CN20K_PRIV_HWVFX_INT_CFG(a)		(0xC000000 | (a) << 12)

#endif /* RVU_REG_H */
