/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (C) 2020 Marvell.
 */

#ifndef __OTX2_CPTPF_H
#define __OTX2_CPTPF_H

#include "otx2_cpt_common.h"
#include "otx2_cptpf_ucode.h"
#include "otx2_cptlf.h"

#define CPT_CN20K_PFAF_MBOX_BASE	0x80000
#define CPT_CN20K_PFVF_MBOX_IRQS	4
#define CPT_CN20K_PFVF_MBOX_IRQ_NAME	32

struct otx2_cptpf_dev;
struct otx2_cptvf_info {
	struct otx2_cptpf_dev *cptpf;	/* PF pointer this VF belongs to */
	struct work_struct vfpf_mbox_work;
	struct pci_dev *vf_dev;
	int vf_id;
	int intr_idx;
};

struct cptpf_flr_work {
	struct work_struct work;
	struct otx2_cptpf_dev *pf;
};

struct cptpf_irq_data {
	u64 intr_status;
	struct otx2_cptpf_dev *pf;
	char irq_name[CPT_CN20K_PFVF_MBOX_IRQ_NAME];
	int vec_num;
	int start;
	int mdevs;
};

struct otx2_cptpf_dev {
	void __iomem *reg_base;		/* CPT PF registers start address */
	void __iomem *afpf_mbox_base;	/* PF-AF mbox start address */
	void __iomem *vfpf_mbox_base;   /* VF-PF mbox start address */
	struct pci_dev *pdev;		/* PCI device handle */
	struct otx2_cptvf_info vf[OTX2_CPT_MAX_VFS_NUM];
	struct otx2_cpt_eng_grps eng_grps;/* Engine groups information */
	struct otx2_cptlfs_info lfs;      /* CPT LFs attached to this PF */
	struct otx2_cptlfs_info cpt1_lfs; /* CPT1 LFs attached to this PF */
	/* HW capabilities for each engine type */
	union otx2_cpt_eng_caps eng_caps[OTX2_CPT_MAX_ENG_TYPES];
	bool is_eng_caps_discovered;

	/* AF <=> PF mbox */
	struct otx2_mbox	afpf_mbox;
	struct work_struct	afpf_mbox_work;
	struct workqueue_struct *afpf_mbox_wq;

	struct otx2_mbox	afpf_mbox_up;
	struct work_struct	afpf_mbox_up_work;
	void *afpf_bbuf_base;		/* Bounce buffer for AF <=> PF mbox */

	/* VF <=> PF mbox */
	struct otx2_mbox	vfpf_mbox;
	struct workqueue_struct *vfpf_mbox_wq;
	/* CN20K PF<->VF mailbox IRQ vector data */
	struct cptpf_irq_data	irq_data[CPT_CN20K_PFVF_MBOX_IRQS];
	struct qmem		*mbox_qmem;

	struct workqueue_struct	*flr_wq;
	struct cptpf_flr_work   *flr_work;
	struct mutex            lock;   /* serialize mailbox access */

	unsigned long cap_flag;
	u8 pf_id;               /* RVU PF number */
	u8 max_vfs;		/* Maximum number of VFs supported by CPT */
	u8 enabled_vfs;		/* Number of enabled VFs */
	u8 sso_pf_func_ovrd;	/* SSO PF_FUNC override bit */
	u8 kvf_limits;		/* Kernel crypto limits */
	bool has_cpt1;
	u8 rsrc_req_blkaddr;

	/* Devlink */
	struct devlink *dl;
};

irqreturn_t otx2_cptpf_afpf_mbox_intr(int irq, void *arg);
int otx2_cptpf_mbox_bbuf_init(struct otx2_cptpf_dev *cptpf,
			      struct pci_dev *pdev);
irqreturn_t cptpf_cn20k_afpf_mbox_intr(int irq, void *arg);
void otx2_cptpf_afpf_mbox_handler(struct work_struct *work);
void otx2_cptpf_afpf_mbox_up_handler(struct work_struct *work);
irqreturn_t otx2_cptpf_vfpf_mbox_intr(int irq, void *arg);
irqreturn_t cptpf_cn20k_vfpf_mbox_intr(int irq, void *arg);
void otx2_cptpf_vfpf_mbox_handler(struct work_struct *work);

int otx2_inline_cptlf_setup(struct otx2_cptpf_dev *cptpf,
			    struct otx2_cptlfs_info *lfs, u8 egrp, int num_lfs);
void otx2_inline_cptlf_cleanup(struct otx2_cptlfs_info *lfs);

#endif /* __OTX2_CPTPF_H */
