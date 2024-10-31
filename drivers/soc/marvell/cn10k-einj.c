// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Marvell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <linux/sys_soc.h>

static const struct soc_device_attribute cn10kb_socinfo[] = {
	/* cn10kb */
	{.soc_id = "jep106:0369:00bd",},
	{},
};

#define PLAT_OCTEONTX_INJECT_ERROR	(0xc2000b10)

#define PLAT_OCTEONTX_EINJ_DSS		(0xd)

#define CN10K_DSS_EINJ_CRC	(0x40000000)	// CRC
#define EINJ_MAX_PARAMS 7

static int einj_setup(const char *val, const struct kernel_param *kp);

static const struct kernel_param_ops einj_ops = {
		.set = einj_setup,
		.get = param_get_ullong,
};

static u64 params[EINJ_MAX_PARAMS];
module_param_cb(smc, &einj_ops, &params, 0644);
MODULE_PARM_DESC(smc, "Setup error injection parameters\n"
		"		0xd: Injecting error to DSS controller\n"
		"		address: Physical Address to corrupt\n"
		"		flags for ECC injection:\n"
		"			[0:7] bit position to corrupt\n"
		"			[8] error type 0 = DED (double), 1 = SEC (single)\n"
		"		echo \"0xd,0x3fffff000,0x101\" > /sys/module/cn10k_einj/parameters/smc\n"
		"\n"
		"		flags for CRC injection:\n"
		"			[30] must be set to 1 to differentiate from ECC\n"
		"			[20:16] CRC poison times (0 means 1 time poison)\n"
		"			[12:8] CRC poison nibble (0 to 7)\n"
		"			[1] 0 = write, 1 = read\n"
		"			[0] must be set to 1. Causes injection to happen\n"
		"\n"
		"		1 CRC Read error at nibble 0:\n"
		"		echo \"0xd,0x2ffff0000,0x40000003\" > /sys/module/cn10k_einj/parameters/smc\n"
		"		5 CRC Read errors at nibble 1:\n"
		"		echo \"0xd,0x2ffff0000,0x40040103\" > /sys/module/cn10k_einj/parameters/smc\n"
		"\n"
		"		1 CRC Write error at nibble 0:\n"
		"		echo \"0xd,0x2ffff0000,0x40000001\" > /sys/module/cn10k_einj/parameters/smc\n"
		"		2 CRC Write errors at nibble 6:\n"
		"		echo \"0xd,0x2ffff0000,0x40010601\" > /sys/module/cn10k_einj/parameters/smc");

static int einj_setup(const char *val, const struct kernel_param *kp)
{
	struct arm_smccc_res res;
	char *str = (char *) val;
	int rc = 0;
	int i = 0;

	if (!str)
		return -EINVAL;

	for (i = 0; i < EINJ_MAX_PARAMS; i++)
		params[i] = 0;

	for (i = 0; i < EINJ_MAX_PARAMS && *str; i++) {

		int len = strcspn(str, ",");
		char *nxt = len ? str + len + 1 : "";

		if (len)
			str[len] = '\0';

		rc = kstrtoull(str, 0, &params[i]);

		pr_debug("%s: (%s/%s) smc_params[%d]=%llx e?%d\n", __func__, str, nxt,
				i, params[i], rc);
		if (len)
			str[len] = ',';
		str = nxt;
	}

	switch (params[0]) {
	case PLAT_OCTEONTX_EINJ_DSS:
		if (params[2] & CN10K_DSS_EINJ_CRC) {
			if (!soc_device_match(cn10kb_socinfo))	// Only cn10kb supports CRC
				return -EINVAL;

			// params[2] provides full error type and let ATF handle it.
			params[3] = params[1];	// Address to ATF in params[3] instead of params[1]
			params[1] = 0; // Prevent NOT updated ATF from injecting ECC when CRC

		} else {
			params[3] = params[2];
			params[2] >>= 8;
			params[2] &= 1;
			params[3] &= 0xFF;
		}
		break;
	default:
		return -EINVAL;
	}

	pr_debug("%s %llx %llx %llx %llx %llx %llx %llx\n", __func__, params[0],
			params[1], params[2], params[3], params[4], params[5], params[6]);

	arm_smccc_smc(PLAT_OCTEONTX_INJECT_ERROR, params[0], params[1], params[2],
			params[3], params[4], params[5], params[6], &res);

	if (kp)
		WRITE_ONCE(*(ulong *)kp->arg, res.a0);

	return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell CN10K error Injector");
