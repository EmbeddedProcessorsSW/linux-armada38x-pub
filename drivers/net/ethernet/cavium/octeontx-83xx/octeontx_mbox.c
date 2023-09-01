// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "sso.h"
#include "octeontx_mbox.h"

/* Mbox operation timeout in milliseconds */
#define MBOX_WAIT_TIME 100

/* MBOX state */
typedef enum {
	MBOX_CHAN_STATE_REQ = 1,
	MBOX_CHAN_STATE_RES = 0
} mbox_chan_state_t;

/* enum for channel specification */
typedef enum {
	MBOX_CHAN_OWN = 0, /* channel which we control */
	MBOX_CHAN_PARTY = 1, /* channel which other party control */
} mbox_chan_t;

/* macro return proper channel index depending on which channel we control and
 * if we TX/RX
 */
#define MBOX_CHAN_IDX(_mbox, _own) \
	(((((_mbox)->chan_own) & 0x1) + ((_own) & 0x1)) & 0x1)

/* macro return the channell MBOX register address depending if we RX or TX */
#define MBOX_REGISTER(_mbox, _own) \
	((_mbox)->mbox_base + (MBOX_CHAN_IDX((_mbox), (_own)) * sizeof(uint64_t)))

/* macro return the RAM MBOX address depending if we RX or TX */
#define MBOX_RAM_ADDRESS(_mbox, _own) ( \
	((_mbox)->ram_base + \
	 (MBOX_CHAN_IDX(_mbox, _own) ? 0 : ((_mbox)->ram_size / 2))))

void mbox_init(struct mbox *mbox, void *mbox_base, void *ram_base,
	       size_t ram_size, mbox_side_t side)
{
	atomic64_t *ram_hdr_addr;
	struct mbox_ram_hdr old_hdr;
	struct mbox_ram_hdr new_hdr;

	mbox->mbox_base = mbox_base;
	mbox->ram_base = ram_base;
	mbox->ram_size = ram_size;
	mbox->hdr_party.val = 0;
	mbox->chan_own =
		(side == MBOX_SIDE_PF) ? 0 : 1;

	/* skip remain setup if RAMBOX is missing */
	if (!ram_base)
		return;
	ram_hdr_addr = MBOX_RAM_ADDRESS(mbox, MBOX_CHAN_OWN);

	/* initialize the channel with tag left by last setup
	 * the value of tag does not mather. What mathers is that new tag value
	 * must be +1 so we notify that previous transactions are invalid
	 */
	old_hdr.val = atomic64_read(ram_hdr_addr);
	mbox->tag_own = (old_hdr.tag + 2) & (~0x1ul); /* next even number */
	new_hdr.val = 0;
	new_hdr.tag = mbox->tag_own;
	atomic64_set(ram_hdr_addr, new_hdr.val);
}

int mbox_send(struct mbox *mbox, struct mbox_hdr *hdr, const void *txmsg,
	      size_t txsize, void *rxmsg, size_t rxsize)
{
	atomic64_t *ram_hdr_addr = MBOX_RAM_ADDRESS(mbox, MBOX_CHAN_OWN);
	 /* body is right after hdr */
	void *ram_body_addr =
	    (void *)((uint8_t *)ram_hdr_addr + sizeof(struct mbox_ram_hdr));
	void *mbox_reg = MBOX_REGISTER(mbox, MBOX_CHAN_OWN);
	struct mbox_ram_hdr ram_hdr;
	size_t wait;
	size_t len;

	/* cannot send msg if RAMBOX is missing */
	if (!mbox->ram_base)
		return -1;

	if (txsize > mbox->ram_size)
		return -1;

	/* \TODO we should check the channel state before overwriting the
	 * message - full sequence will came in next commit
	 */

	/* copy msg body first */
	if (txmsg)
		memcpy(ram_body_addr, txmsg, txsize);

	/* prepare new ram_hdr */
	ram_hdr.val = 0;
	ram_hdr.chan_state = MBOX_CHAN_STATE_REQ;
	ram_hdr.coproc = hdr->coproc;
	ram_hdr.msg = hdr->msg;
	ram_hdr.vfid = hdr->vfid;
	ram_hdr.tag = ++(mbox->tag_own);
	ram_hdr.len = txsize;

	/* write the msg header and at the same time change the channel state */
	atomic64_set(ram_hdr_addr, ram_hdr.val);
	/* notify about new msg - write to MBOX reg will cause IRQ generation */
	writeq_relaxed(MBOX_TRIGGER_NORMAL, mbox_reg);

	/* wait for response */
	wait = MBOX_WAIT_TIME;
	while (wait) {
		usleep_range(10000, 20000);
		ram_hdr.val = atomic64_read(ram_hdr_addr);
		if (ram_hdr.chan_state == MBOX_CHAN_STATE_RES)
			break;
		wait -= 10;
	}
	if (!wait)
		return -1; /* timeout */
	if ((u16)(mbox->tag_own + 1) != ram_hdr.tag)
		return -1; /* tag mismatch */
	(mbox->tag_own)++;

	len = min_t(size_t, (size_t)ram_hdr.len, rxsize);
	memcpy(rxmsg, ram_body_addr, len);
	hdr->res_code = ram_hdr.res_code;

	return len;
}

int mbox_receive(struct mbox *mbox, struct mbox_hdr *hdr, void *rxmsg,
		 size_t rxsize)
{
	atomic64_t *ram_hdr_addr = MBOX_RAM_ADDRESS(mbox, MBOX_CHAN_PARTY);
	/* body is right after hdr */
	void *ram_body_addr =
		(void *)((uint8_t *)ram_hdr_addr + sizeof(struct mbox_ram_hdr));
	void *mbox_reg = MBOX_REGISTER(mbox, MBOX_CHAN_PARTY);
	struct mbox_ram_hdr ram_hdr;
	u64 trg_val;
	size_t len;

	/* clear the mbox_hdr fields */
	memset(hdr, 0, sizeof(*hdr));

	/* check if this is normal msg delivery of out of band request */
	trg_val = readq_relaxed(mbox_reg);
	if (trg_val != MBOX_TRIGGER_NORMAL) {
		if (trg_val & MBOX_TRIGGER_OOB_RES)
			return -1; /* no msg nor OOB */

		mbox->oob = trg_val;
		hdr->oob = trg_val;
		hdr->res_code = MBOX_RET_SUCCESS;
		return 0; /* return only OOB info, no msg or its body */
	}
	mbox->oob = MBOX_TRIGGER_NORMAL; /* no OOB */

	/* Non-OOB messages require RAMBOX */
	if (!mbox->ram_base)
		return -1;

	/* read the header to see if there is a message for us */
	ram_hdr.val = atomic64_read(ram_hdr_addr);
	if (ram_hdr.chan_state != MBOX_CHAN_STATE_REQ)
		return -1;

	/* store the received header for reply */
	mbox->hdr_party = ram_hdr;

	/* also update the hdr so application can use it */
	hdr->vfid = ram_hdr.vfid;
	hdr->coproc = ram_hdr.coproc;
	hdr->msg = ram_hdr.msg;
	hdr->oob = MBOX_TRIGGER_NORMAL; /* no OOB */
	hdr->res_code = MBOX_RET_SUCCESS;

	/* copy the msg body */
	len = min_t(size_t, (size_t)ram_hdr.len, rxsize);
	memcpy(rxmsg, ram_body_addr, len);
	return len;
}

int mbox_reply(struct mbox *mbox, uint8_t res_code, const void *txmsg,
	       size_t txsize)
{
	atomic64_t *ram_hdr_addr = MBOX_RAM_ADDRESS(mbox, MBOX_CHAN_PARTY);
	/* body is right after hdr */
	void *ram_body_addr =
		(void *)((uint8_t *)ram_hdr_addr + sizeof(struct mbox_ram_hdr));
	void *mbox_reg = MBOX_REGISTER(mbox, MBOX_CHAN_PARTY);
	struct mbox_ram_hdr ram_hdr;

	/* \TODO we should check the channel state before overwriting the
	 * message - full sequence will came in next commit
	 */

	/* check if last msg was OOB */
	if (mbox->oob != MBOX_TRIGGER_NORMAL) {
		/* only OOB use mbox register for reply so there is no race */
		writeq_relaxed(mbox->oob | MBOX_TRIGGER_OOB_RES, mbox_reg);
		mbox->oob = MBOX_TRIGGER_NORMAL;
		return 0; /* return with success */
	}

	/* Non-OOB messages require RAMBOX */
	if (!mbox->ram_base)
		return -1;

	/* copy msg body first to reply RAM channel
	 * truncate the message if it is too big
	 */
	if (txmsg) {
		txsize = min(txsize, mbox->ram_size);
		memcpy(ram_body_addr, txmsg, txsize);
	} else {
		txsize = 0;
	}

	/* prepare new hdr by copy of most fields and update some of them */
	ram_hdr = mbox->hdr_party;
	ram_hdr.chan_state = MBOX_CHAN_STATE_RES;
	ram_hdr.tag = mbox->hdr_party.tag + 1;
	ram_hdr.len = txsize;
	ram_hdr.res_code = res_code;

	/* change the channel state and notify about new msg - write to MBOX
	 * register is just for IRQ generation, the value written there is
	 * not so important
	 */
	atomic64_set(ram_hdr_addr, ram_hdr.val);

	return 0;
}
