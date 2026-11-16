/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Board device tree embedded in the kernel image; see CONFIG_ARM64_EMBEDDED_DTB
 * and the override in setup_arch().
 */
	.section .rodata, "a"
	.balign 8
	.globl __embedded_dtb_start
__embedded_dtb_start:
	.incbin EMBEDDED_DTB_FILE
	.globl __embedded_dtb_end
__embedded_dtb_end:
	.balign 8
