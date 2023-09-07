/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2013 ARM Ltd.
 * Copyright (C) 2013 Linaro.
 */

/*
 * Copy a buffer from src to dest (alignment handled by the hardware)
 *
 * Parameters:
 *	x0 - dest
 *	x1 - src
 *	x2 - n
 * Returns:
 *	x0 - dest
 */
dstin	.req	x0
src	.req	x1
count	.req	x2
tmp1	.req	x3
tmp1w	.req	w3
tmp2	.req	x4
tmp2w	.req	w4
dst	.req	x6

A_l	.req	x7
A_h	.req	x8
B_l	.req	x9
B_h	.req	x10
C_l	.req	x11
C_h	.req	x12
D_l	.req	x13
D_h	.req	x14

cvmtmp		.req	x15
cvmctl		.req	x16
cvmmemctl	.req	x17

	mrs	cvmctl, S3_0_C11_C0_0  // AP_CVMCTL_EL1
	mov	cvmtmp, cvmctl
	bfi     cvmtmp, xzr, #40, #4   // clear [43:40]
	msr	S3_0_C11_C0_0, cvmtmp

	mrs	cvmmemctl, S3_0_C11_C0_4  // AP_CVMMEMCTL0_EL1
	mov	cvmtmp, cvmmemctl
	bfi     cvmtmp, xzr, #35, #1   // Bit 35 - prefetch disable/enable
	msr	S3_0_C11_C0_4, cvmtmp
	dmb	ld

	mov	dst, dstin
	cmp	count, #16
	/*When memory length is less than 16, the accessed are not aligned.*/
	b.lo	.Ltiny15_nops

	neg	tmp2, src
	ands	tmp2, tmp2, #15/* Bytes to reach alignment. */
	b.eq	.LSrcAligned_nops
	sub	count, count, tmp2
	/*
	* Copy the leading memory data from src to dst in an increasing
	* address order.By this way,the risk of overwriting the source
	* memory data is eliminated when the distance between src and
	* dst is less than 16. The memory accesses here are alignment.
	*/
	tbz	tmp2, #0, 1f
	ldrb1	tmp1w, src, #1
	ins_nops
	strb1	tmp1w, dst, #1
	ins_nops
1:
	tbz	tmp2, #1, 2f
	ldrh1	tmp1w, src, #2
	ins_nops
	strh1	tmp1w, dst, #2
	ins_nops
2:
	tbz	tmp2, #2, 3f
	ldr1	tmp1w, src, #4
	ins_nops
	str1	tmp1w, dst, #4
	ins_nops
3:
	tbz	tmp2, #3, .LSrcAligned_nops
	ldr1	tmp1, src, #8
	ins_nops
	str1	tmp1, dst, #8
	ins_nops

.LSrcAligned_nops:
	cmp	count, #64
	b.ge	.Lcpy_over64_nops
	/*
	* Deal with small copies quickly by dropping straight into the
	* exit block.
	*/
.Ltail63_nops:
	/*
	* Copy up to 48 bytes of data. At this point we only need the
	* bottom 6 bits of count to be accurate.
	*/
	ands	tmp1, count, #0x30
	b.eq	.Ltiny15_nops
	cmp	tmp1w, #0x20
	b.eq	1f
	b.lt	2f
	ldp1	A_l, A_h, src, #16
	ins_nops
	stp1	A_l, A_h, dst, #16
	ins_nops
1:
	ldp1	A_l, A_h, src, #16
	ins_nops
	stp1	A_l, A_h, dst, #16
	ins_nops
2:
	ldp1	A_l, A_h, src, #16
	ins_nops
	stp1	A_l, A_h, dst, #16
	ins_nops
.Ltiny15_nops:
	/*
	* Prefer to break one ldp/stp into several load/store to access
	* memory in an increasing address order,rather than to load/store 16
	* bytes from (src-16) to (dst-16) and to backward the src to aligned
	* address,which way is used in original cortex memcpy. If keeping
	* the original memcpy process here, memmove need to satisfy the
	* precondition that src address is at least 16 bytes bigger than dst
	* address,otherwise some source data will be overwritten when memove
	* call memcpy directly. To make memmove simpler and decouple the
	* memcpy's dependency on memmove, withdrew the original process.
	*/
	tbz	count, #3, 1f
	ldr1	tmp1, src, #8
	ins_nops
	str1	tmp1, dst, #8
	ins_nops
1:
	tbz	count, #2, 2f
	ldr1	tmp1w, src, #4
	ins_nops
	str1	tmp1w, dst, #4
	ins_nops
2:
	tbz	count, #1, 3f
	ldrh1	tmp1w, src, #2
	ins_nops
	strh1	tmp1w, dst, #2
	ins_nops
3:
	tbz	count, #0, .Lexitfunc_nops
	ldrb1	tmp1w, src, #1
	ins_nops
	strb1	tmp1w, dst, #1
	ins_nops

	b	.Lexitfunc_nops

.Lcpy_over64_nops:
	subs	count, count, #128
	b.ge	.Lcpy_body_large_nops
	/*
	* Less than 128 bytes to copy, so handle 64 here and then jump
	* to the tail.
	*/
	ldp1	A_l, A_h, src, #16
	ins_nops
	stp1	A_l, A_h, dst, #16
	ins_nops
	ldp1	B_l, B_h, src, #16
	ins_nops
	ldp1	C_l, C_h, src, #16
	ins_nops
	stp1	B_l, B_h, dst, #16
	ins_nops
	stp1	C_l, C_h, dst, #16
	ins_nops
	ldp1	D_l, D_h, src, #16
	ins_nops
	stp1	D_l, D_h, dst, #16
	ins_nops

	tst	count, #0x3f
	b.ne	.Ltail63_nops
	b	.Lexitfunc_nops

	/*
	* Critical loop.  Start at a new cache line boundary.  Assuming
	* 64 bytes per line this ensures the entire loop is in one line.
	*/
	.p2align	L1_CACHE_SHIFT
.Lcpy_body_large_nops:
	/* pre-get 64 bytes data. */
	ldp1	A_l, A_h, src, #16
	ldp1	B_l, B_h, src, #16
	ldp1	C_l, C_h, src, #16
	ldp1	D_l, D_h, src, #16
	ins_nops
1:
	/*
	* interlace the load of next 64 bytes data block with store of the last
	* loaded 64 bytes data.
	*/
	stp1	A_l, A_h, dst, #16
	ins_nops
	ldp1	A_l, A_h, src, #16
	ins_nops
	stp1	B_l, B_h, dst, #16
	ins_nops
	ldp1	B_l, B_h, src, #16
	ins_nops
	stp1	C_l, C_h, dst, #16
	ins_nops
	ldp1	C_l, C_h, src, #16
	ins_nops
	stp1	D_l, D_h, dst, #16
	ins_nops
	ldp1	D_l, D_h, src, #16
	ins_nops
	subs	count, count, #64
	b.ge	1b
	stp1	A_l, A_h, dst, #16
	ins_nops
	stp1	B_l, B_h, dst, #16
	ins_nops
	stp1	C_l, C_h, dst, #16
	ins_nops
	stp1	D_l, D_h, dst, #16
	ins_nops

	tst	count, #0x3f
	b.ne	.Ltail63_nops
.Lexitfunc_nops:
	msr	S3_0_C11_C0_0, cvmctl
	msr	S3_0_C11_C0_4, cvmmemctl
