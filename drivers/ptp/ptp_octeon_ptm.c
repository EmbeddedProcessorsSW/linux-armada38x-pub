// SPDX-License-Identifier: GPL-2.0
/* Marvell PTP PHC clock driver for PCIe PTM (Precision Time Measurement) EP
 *
 * Copyright (c) 2023 Marvell.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/ptp_clock_kernel.h>

#include "ptp_private.h"

#define PEMX_PFX_CSX_PFCFGX(pem, pf, offset)      ((0x8e0000008000 | (u64)pem << 36 \
						| pf << 18 \
						| ((offset >> 16) & 1) << 16 \
						| (offset >> 3) << 3) \
						+ (((offset >> 2) & 1) << 2))


#define PEMX_CFG_WR(a)			(0x8E0000000018ull | (u64)(a) << 36)
#define PEMX_CFG_RD(a)			(0x8E0000000020ull | (u64)(a) << 36)

/* Octeon CSRs   */
#define PEMX_CFG                        0x8e00000000d8ULL
#define PEMX_PTM_CTL			0x8e0000000098ULL
#define PEMX_PTM_CTL_CAP		BIT_ULL(10)
#define PEMX_PTM_LCL_TIME		0x8e00000000a0ULL /* PTM time */
#define PEMX_PTM_MAS_TIME		0x8e00000000a8ULL /* PTP time */
#define PTM_DEBUG			0

struct oct_ptp_clock {
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;
	bool cn10k_variant;
};

static struct oct_ptp_clock oct_ptp_clock;
u64 *ptm_ctl_addr;
u64 *ptm_lcl_addr;

/* Config space registers   */
#define PCIEEPX_PTM_REQ_STAT		(oct_ptp_clock.cn10k_variant ? 0x3a8 : 0x474)
#define PCIEEPX_PTM_REQ_T1L		(oct_ptp_clock.cn10k_variant ? 0x3b4 : 0x480)
#define PCIEEPX_PTM_REQ_T1M		(oct_ptp_clock.cn10k_variant ? 0x3b8 : 0x484)
#define PCIEEPX_PTM_REQ_T4L		(oct_ptp_clock.cn10k_variant ? 0x3c4 : 0x490)
#define PCIEEPX_PTM_REQ_T4M		(oct_ptp_clock.cn10k_variant ? 0x3c8 : 0x494)

#define PCI_VENDOR_ID_CAVIUM			0x177d
#define PCI_DEVID_OCTEONTX2_PTP			0xA00C
#define PCI_SUBSYS_DEVID_95XX			0xB300
#define PCI_SUBSYS_DEVID_95XXN			0xB400
#define PCI_SUBSYS_DEVID_95XXMM			0xB500
#define PCI_SUBSYS_DEVID_96XX			0xB200
#define PCI_SUBSYS_DEVID_98XX			0xB100
#define PCI_SUBSYS_DEVID_CN10K_A		0xB900
#define PCI_SUBSYS_DEVID_CN10K_B		0xBD00
#define PCI_SUBSYS_DEVID_CNF10K_A		0xBA00
#define PCI_SUBSYS_DEVID_CNF10K_B		0xBC00

static bool is_otx2_support_ptm(struct pci_dev *pdev)
{
	return (pdev->subsystem_device == PCI_SUBSYS_DEVID_96XX ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_95XX ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_95XXN ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_98XX ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_95XXMM);
}

static bool is_cn10k_support_ptm(struct pci_dev *pdev)
{
	return (pdev->subsystem_device == PCI_SUBSYS_DEVID_CN10K_A ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_CNF10K_A ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_CN10K_B ||
		pdev->subsystem_device == PCI_SUBSYS_DEVID_CNF10K_B);
}

static int ptp_oct_ptm_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	return -EOPNOTSUPP;
}

static int ptp_oct_ptm_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	return -EOPNOTSUPP;
}

static int ptp_oct_ptm_settime(struct ptp_clock_info *ptp,
			       const struct timespec64 *ts)
{
	return -EOPNOTSUPP;
}

static u32 read_pcie_config32(int ep_pem, int cfg_addr)
{
	void __iomem *addr;
	u64 val;

	if (oct_ptp_clock.cn10k_variant) {
		addr  = ioremap(PEMX_PFX_CSX_PFCFGX(ep_pem, 0, cfg_addr), 8);
		if (!addr) {
			pr_err("PTM_EP: Failed to ioremap Octeon CSR space\n");
			return -1U;
		}
		val = readl(addr);
		iounmap(addr);
	} else {
		addr  = ioremap(PEMX_CFG_RD(ep_pem), 8);
		if (!addr) {
			pr_err("PTM_EP: Failed to ioremap Octeon CSR space\n");
			return -1U;
		}
		val = ((1 << 15) | (cfg_addr & 0xfff));
		writeq(val, addr);
		val = readq(addr) >> 32;
		iounmap(addr);
	}
	return (val & 0xffffffff);
}

static uint64_t octeon_csr_read(u64 csr_addr)
{
	u64 val;
	u64 *addr;

	addr = ioremap(csr_addr, 8);
	if (!addr) {
		pr_err("PTM_EP: Failed to ioremap CSR space\n");
		return -1UL;
	}
	val = READ_ONCE(*addr);
	iounmap(addr);
	return val;
}

static int ptp_oct_ptm_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ptm_time, ptp_time, val64;
	u32 val32;

	/* Check for valid PTM context */
	val32 = read_pcie_config32(0, PCIEEPX_PTM_REQ_STAT);
	if (!(val32 & 0x1)) {
		pr_err("PTM_EP: ERROR: PTM context not valid: 0x%x\n", val32);
		ptm_time = 0;
		ptp_time = 0;

		ts->tv_sec = 0;
		ts->tv_nsec = 0;

		return -EINVAL;
	}

	/* Trigger PTM/PTP capture */
	val64 = READ_ONCE(*ptm_ctl_addr);
	val64 |= PEMX_PTM_CTL_CAP;
	WRITE_ONCE(*ptm_ctl_addr, val64);
	/* Read PTM/PTP clocks  */
	ptp_time = READ_ONCE(*ptm_lcl_addr);
	ts->tv_sec = ptp_time / NSEC_PER_SEC;
	ts->tv_nsec = ptp_time % NSEC_PER_SEC;

#if PTM_DEBUG
	ptm_time = octeon_csr_read(PEMX_PTM_MAS_TIME);
	pr_info("PTM_EP: system %lld ptm time: %lld\n", ptp_time, ptm_time);
#endif

	return 0;
}

static int ptp_oct_ptm_enable(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static const struct ptp_clock_info ptp_oct_caps = {
	.owner		= THIS_MODULE,
	.name		= "OCTEON PTM PHC",
	.max_adj	= 0,
	.n_ext_ts	= 0,
	.n_pins		= 0,
	.pps		= 0,
	.adjfreq	= ptp_oct_ptm_adjfreq,
	.adjtime	= ptp_oct_ptm_adjtime,
	.gettime64	= ptp_oct_ptm_gettime,
	.settime64	= ptp_oct_ptm_settime,
	.enable		= ptp_oct_ptm_enable,
};

static void __exit ptp_oct_ptm_exit(void)
{
	iounmap(ptm_ctl_addr);
	iounmap(ptm_lcl_addr);
	ptp_clock_unregister(oct_ptp_clock.ptp_clock);
}

static int __init ptp_oct_ptm_init(void)
{
	struct pci_dev *pdev = NULL;

	if (octeon_csr_read(PEMX_CFG) & 0x1ULL) {
		pr_err("PEM0 is configured as RC\n");
		return 0;
	}

	pdev = pci_get_device(PCI_VENDOR_ID_CAVIUM,
			      PCI_DEVID_OCTEONTX2_PTP, pdev);
	if (!pdev)
		return 0;

	if (is_otx2_support_ptm(pdev)) {
		oct_ptp_clock.cn10k_variant = 0;
	} else if (is_cn10k_support_ptm(pdev)) {
		oct_ptp_clock.cn10k_variant = 1;
	} else {
		/* PTM_EP: unsupported processor */
		return 0;
	}

	ptm_ctl_addr = ioremap(PEMX_PTM_CTL, 8);
	if (!ptm_ctl_addr) {
		pr_err("PTM_EP: Failed to ioremap CSR space\n");
		return 0;
	}

	ptm_lcl_addr = ioremap(PEMX_PTM_LCL_TIME, 8);
	if (!ptm_lcl_addr) {
		pr_err("PTM_EP: Failed to ioremap CSR space\n");
		return 0;
	}

	oct_ptp_clock.caps = ptp_oct_caps;

	oct_ptp_clock.ptp_clock = ptp_clock_register(&oct_ptp_clock.caps, NULL);

	pr_info("PTP device index for PTM clock:%d\n", oct_ptp_clock.ptp_clock->index);
	pr_info("cn10k_variant %d\n", oct_ptp_clock.cn10k_variant);

	return PTR_ERR_OR_ZERO(oct_ptp_clock.ptp_clock);
}

module_init(ptp_oct_ptm_init);
module_exit(ptp_oct_ptm_exit);

MODULE_AUTHOR("Marvell Inc.");
MODULE_DESCRIPTION("PTP PHC clock using PTM");
MODULE_LICENSE("GPL");
