/* SPDX-License-Identifier: GPL-2.0 */
/* Marvell RVU Admin Function driver
 *
 * Copyright (C) 2024 Marvell.
 *
 */

#ifndef RVU_MBOX_REG_H
#define RVU_MBOX_REG_H

#define PF_TO_REGIDX(pf)	((pf) >= 64 ? 1 : 0)
#define PF_BITMAX		64
static inline u64 pf_to_bitoff(u8 pf)
{
	return (pf >= 64 ? pf - 64 : pf);
}

/* RVUM block registers */
#define RVU_PF_DISC				(0x0)
#define RVU_PRIV_PFX_DISC(a)			(0x8000208 | (a) << 16)
#define RVU_PRIV_HWVFX_DISC(a)			(0xD000000 | (a) << 12)

/* Mbox Registers */
/* RVU AF BAR0 Mbox registers for AF => PFx */
#define RVU_MBOX_AF_PFX_ADDR(a)			(0x5000 | (a) << 4)
#define RVU_MBOX_AF_PFX_CFG(a)			(0x6000 | (a) << 4)
#define RVU_MBOX_AF_AFPFX_TRIGX(a)		(0x9000 | (a) << 3)
#define RVU_MBOX_AF_PFAF_INT(a)			(0x2980 | (a) << 6)
#define RVU_MBOX_AF_PFAF_INT_W1S(a)		(0x2988 | (a) << 6)
#define RVU_MBOX_AF_PFAF_INT_ENA_W1S(a)		(0x2990 | (a) << 6)
#define RVU_MBOX_AF_PFAF_INT_ENA_W1C(a)		(0x2998 | (a) << 6)
#define RVU_MBOX_AF_PFAF1_INT(a)		(0x29A0 | (a) << 6)
#define RVU_MBOX_AF_PFAF1_INT_W1S(a)		(0x29A8 | (a) << 6)
#define RVU_MBOX_AF_PFAF1_INT_ENA_W1S(a)	(0x29B0 | (a) << 6)
#define RVU_MBOX_AF_PFAF1_INT_ENA_W1C(a)	(0x29B8 | (a) << 6)

#define RVU_AF_PFFLR_INTX(a)                    (0x27a0 + 0x40 * (a))
#define RVU_AF_PFFLR_INT_W1SX(a)                (0x27a8 + 0x40 * (a))
#define RVU_AF_PFFLR_INT_ENA_W1SX(a)            (0x27b0 + 0x40 * (a))
#define RVU_AF_PFFLR_INT_ENA_W1CX(a)            (0x27b8 + 0x40 * (a))
#define RVU_AF_PFME_INTX(a)                     (0x28c0 + 0x20 * (a))
#define RVU_AF_PFME_INT_W1SX(a)                 (0x28c8 + 0x20 * (a))
#define RVU_AF_PFME_INT_ENA_W1SX(a)             (0x28d0 + 0x20 * (a))
#define RVU_AF_PFME_INT_ENA_W1CX(a)             (0x28d8 + 0x20 * (a))
#define RVU_AF_PFTRPENDX(a)                     (0x2810 + 0x8 * (a))

/* RVU PF => AF mbox registers */
#define RVU_MBOX_PF_PFAF_TRIGX(a)		(0xC00 | (a) << 3)
#define RVU_MBOX_PF_INT				(0xC20)
#define RVU_MBOX_PF_INT_W1S			(0xC28)
#define RVU_MBOX_PF_INT_ENA_W1S			(0xC30)
#define RVU_MBOX_PF_INT_ENA_W1C			(0xC38)

#define RVU_MBOX_AF_VFAF_INT(a)			(0x3000 | (a) << 6)
#define RVU_MBOX_AF_VFAF_INT_W1S(a)		(0x3008 | (a) << 6)
#define RVU_MBOX_AF_VFAF_INT_ENA_W1S(a)		(0x3010 | (a) << 6)
#define RVU_MBOX_AF_VFAF_INT_ENA_W1C(a)		(0x3018 | (a) << 6)
#define RVU_MBOX_AF_VFAF_INT_ENA_W1C(a)		(0x3018 | (a) << 6)
#define RVU_MBOX_AF_VFAF1_INT(a)		(0x3020 | (a) << 6)
#define RVU_MBOX_AF_VFAF1_INT_W1S(a)		(0x3028 | (a) << 6)
#define RVU_MBOX_AF_VFAF1_IN_ENA_W1S(a)		(0x3030 | (a) << 6)
#define RVU_MBOX_AF_VFAF1_IN_ENA_W1C(a)		(0x3038 | (a) << 6)

#define RVU_MBOX_AF_AFVFX_TRIG(a, b)		(0x10000 | (a) << 4 | (b) << 3)
#define RVU_MBOX_AF_VFX_ADDR(a)			(0x20000 | (a) << 4)
#define RVU_MBOX_AF_VFX_CFG(a)			(0x28000 | (a) << 4)

#define RVU_MBOX_PF_VFX_PFVF_TRIGX(a)		(0x2000 | (a) << 3)

#define RVU_MBOX_PF_VFPF_INTX(a)		(0x1000 | (a) << 3)
#define RVU_MBOX_PF_VFPF_INT_W1SX(a)		(0x1020 | (a) << 3)
#define RVU_MBOX_PF_VFPF_INT_ENA_W1SX(a)	(0x1040 | (a) << 3)
#define RVU_MBOX_PF_VFPF_INT_ENA_W1CX(a)	(0x1060 | (a) << 3)

#define RVU_MBOX_PF_VFPF1_INTX(a)		(0x1080 | (a) << 3)
#define RVU_MBOX_PF_VFPF1_INT_W1SX(a)		(0x10a0 | (a) << 3)
#define RVU_MBOX_PF_VFPF1_INT_ENA_W1SX(a)	(0x10c0 | (a) << 3)
#define RVU_MBOX_PF_VFPF1_INT_ENA_W1CX(a)	(0x10e0 | (a) << 3)

#define RVU_MBOX_PF_VF_ADDR			(0xC40)
#define RVU_MBOX_PF_LMTLINE_ADDR		(0xC48)
#define RVU_MBOX_PF_VF_CFG			(0xC60)

#define RVU_MBOX_VF_VFPF_TRIGX(a)		(0x3000 | (a) << 3)
#define RVU_MBOX_VF_INT				(0x20)
#define RVU_MBOX_VF_INT_W1S			(0x28)
#define RVU_MBOX_VF_INT_ENA_W1S			(0x30)
#define RVU_MBOX_VF_INT_ENA_W1C			(0x38)

#define RVU_MBOX_VF_VFAF_TRIGX(a)		(0x2000 | (a) << 3)

/* NPC registers */
#define NPC_AF_INTFX_EXTRACTORX_CFG(a, b) \
	(0x908000ull | (a) << 10 | (b) << 3)
#define NPC_AF_INTFX_EXTRACTORX_LTX_CFG(a, b, c) \
	(0x900000ull | (a) << 13 | (b) << 8  | (c) << 3)
#define NPC_AF_KPMX_ENTRYX_CAMX(a, b, c) \
	(0x100000ull | (a) << 14 | (b) << 6 | (c) << 3)
#define NPC_AF_KPMX_ENTRYX_ACTION0(a, b) \
	(0x100020ull | (a) << 14 | (b) << 6)
#define NPC_AF_KPMX_ENTRYX_ACTION1(a, b) \
	(0x100028ull | (a) << 14 | (b) << 6)
#define NPC_AF_KPMX_ENTRY_DISX(a, b)	(0x180000ull | (a) << 6 | (b) << 3)
#define NPC_AF_KPM_PASS2_CFG	0x580
#define NPC_AF_KPMX_PASS2_OFFSET(a)	(0x190000ull | (a) << 3)
#define NPC_AF_MCAM_SECTIONX_CFG_EXT(a)	(0xC000000ull | (a) << 3)

#define NPC_AF_CN20K_MCAMEX_BANKX_CAMX_INTF_EXT(a, b, c) ({		\
	u64 offset;							\
	offset = (0x8000000ull | (a) << 8 | (b) << 22 | (c) << 3);	\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_CAMX_W0_EXT(a, b, c) ({		\
	u64 offset;							\
	offset = (0x8000010ull | (a) << 8 | (b) << 22 | (c) << 3);	\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_CAMX_W1_EXT(a, b, c) ({		\
	u64 offset;							\
	offset = (0x8000020ull | (a) << 8 | (b) << 22 | (c) << 3);	\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_CAMX_W2_EXT(a, b, c) ({		\
	u64 offset;							\
	offset = (0x8000030ull | (a) << 8 | (b) << 22 | (c) << 3);	\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_CAMX_W3_EXT(a, b, c) ({		\
	u64 offset;							\
	offset = (0x8000040ull | (a) << 8 | (b) << 22 | (c) << 3);	\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_CFG_EXT(a, b) ({		\
	u64 offset;						\
	offset = (0x8000050ull | (a) << 8 | (b) << 22);		\
	offset; })

#define NPC_AF_CN20K_MCAMEX_BANKX_ACTIONX_EXT(a, b, c) ({		   \
	u64 offset;							   \
									   \
	offset = (0x8000060ull | (a) << 8 | (b) << 22 | (c) << 3);	   \
	offset; })

#define NPC_AF_INTFX_MISS_ACTX(a, b)	(0x1a00000 | (a) << 6 | (b) << 4)

#endif /* RVU_MBOX_REG_H */
