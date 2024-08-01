/* SPDX-License-Identifier: GPL-2.0 */
/* Marvell RVU Ethernet driver
 *
 * Copyright (C) 2024 Marvell.
 *
 */

#ifndef CN20K_H
#define CN20K_H

#include "otx2_common.h"

int cn20k_init(struct otx2_nic *pfvf);
int cn20k_check_pf_usable(struct otx2_nic *nic);
#endif /* CN20K_H */
