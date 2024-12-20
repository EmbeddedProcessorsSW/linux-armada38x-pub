/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (C) 2021 Marvell.
 */

#ifndef __OTX2_CPT_DEVLINK_H
#define __OTX2_CPT_DEVLINK_H

#include "otx2_cpt_common.h"
#include "otx2_cptpf.h"

#define RES_META_OFFSET_MASK GENMASK(36, 32)

struct otx2_cpt_devlink {
	struct devlink *dl;
	struct otx2_cptpf_dev *cptpf;
	u8 uc_compcode;
};

enum otx2_cpt_dl_param_id {
	OTX2_CPT_DEVLINK_PARAM_ID_BASE = DEVLINK_PARAM_GENERIC_ID_MAX,
	OTX2_CPT_DEVLINK_PARAM_ID_EGRP_CREATE,
	OTX2_CPT_DEVLINK_PARAM_ID_EGRP_DELETE,
	OTX2_CPT_DEVLINK_PARAM_ID_MAX_RXC_ICB_CNT,
	OTX2_CPT_DEVLINK_PARAM_ID_T106_MODE,
	CN20K_CPT_DEVLINK_PARAM_ID_RES_META_OFFSET,
	CN20K_CPT_DEVLINK_PARAM_ID_UC_COMPLETION_CODE_INDEX,
	CN20K_CPT_DEVLINK_PARAM_ID_COMPLETION_CODE_TO_CQ,
};

/* Devlink APIs */
int otx2_cpt_register_dl(struct otx2_cptpf_dev *cptpf);
void otx2_cpt_unregister_dl(struct otx2_cptpf_dev *cptpf);

#endif /* __OTX2_CPT_DEVLINK_H */
