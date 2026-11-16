// SPDX-License-Identifier: GPL-2.0
/*
 * MT6897 (Dimensity 8300) SPM power domains.
 *
 *   compatible = "mediatek,mt6897-scpsys"   (vendor DT: power-controller@1c001000)
 *
 * Domain table ported from the vendor kernel_device_modules-6.1
 * drivers/soc/mediatek/mtk-scpsys-mt6897.c (Copyright (c) 2023 MediaTek Inc.,
 * Benjamin Chao), translated to this tree's scpsys core the same way
 * mtk-scpsys-mt6895.c was.  Domain indices are the vendor
 * <dt-bindings/power/mt6897-power.h> ones, so stock
 * `power-domains = <&scpsys N>` references stay valid (conn is 1, not 0).
 *
 * ============================================================================
 * 反推来源（全部离线、可复现；见 reports/20260918-wifi-bt-bringup-status.md §8）
 * ============================================================================
 *   build/modules/ko/mtk-scpsys-mt6897.ko          纯数据模块（.text 仅 248 B）
 *   build/vendor-modules-.../mtk-scpsys.ko          core（init_scp / 注册）
 *   tools/vendor-forensics/dump_scpsys_domains.py   解析器
 *   ref/mt6897-scpsys-domains.json                  33 + 12 个域的完整数据
 *
 *  1) ★★ bp 的 `type` 字段**不是**主线的 enum regmap_type，而是
 *     `init_scp()` 第 5 个参数 `bus_list[]` 的**下标**（6 项，下标 0 保留）：
 *         [0] NULL     [1] gpu-eb-rpc(0x13f91000)  [2] ifr-bus(0x1002c000)
 *         [3] vlpcfg   [4] nemi-bus                [5] semi-bus
 *     映射回本树 struct bus_prot 的 type：
 *         1 -> MFGRPC_TYPE   2 -> IFR_TYPE   3 -> VLP_TYPE
 *         4 -> NEMI_TYPE     5 -> SEMI_TYPE
 *     The vendor SPM table only uses types 2 (ifr-bus -> IFR_TYPE, DT
 *     property "infracfg") and 3 (vlpcfg -> VLP_TYPE, DT property "vlpcfg").
 *
 *  2) conn：sta_mask = 0、ctl_offs = 0xE04、无 SRAM 位、无时钟、无子域，
 *     caps = BYPASS_INIT_ON | IS_PWR_CON_ON（= 0x180，与厂商一致）。
 *     Kept exactly as the previous (field-tested) port had it.  NOTE: the
 *     vendor core waits on PWR_ACK/PWR_ACK_2ND (PWR_CON bits 30/31) for
 *     IS_PWR_CON_ON domains, which in this core is expressed as
 *     sta_mask = GENMASK(31, 30) (as done for every other domain below).
 *     With sta_mask = 0 the power-on ack wait is a no-op and the power-off
 *     ack wait can never succeed (times out after 1 s).
 *
 *  3) SPM 描述符：base 0x1C001000、pwr_sta_offs = 0xFB0、pwr_sta2nd_offs = 0xFB4。
 *
 * Translation notes (vendor -> this core):
 *  - Vendor IS_PWR_CON_ON domains carry no sta_mask; the vendor core polls
 *    PWR_CON[30] / PWR_CON[31].  Here: sta_mask = GENMASK(31, 30).
 *  - MTK_SCPD_PROFILE (power-on/off timing printk only) does not exist in
 *    this core and is dropped.
 *  - MTK_SCPD_REMOVE_MD_RSTB (md) does not exist in this core; its md ops
 *    always toggle RST_B.  The md domain is therefore registered with power
 *    transitions refused (see mt6897_pd_refuse()).
 *  - cam_vcore has 4 vendor basic clocks but this core has MAX_CLKS = 3:
 *    "ccu_ahb" is carried as a basic_lp_clk (enabled around the power-on /
 *    power-off sequences, i.e. while bus protection is changed, but not held
 *    while the domain is on).  The cam_ccu child domain holds ccu_ahb itself.
 *  - HWV (hardware voter) domains (mm_infra, vde0/1, dis0/1, ovl0/1) need the
 *    "hw-voter-regmap" syscon.
 *
 * Registration policy (differs from mtk_register_power_domains() on purpose):
 *  - Nothing is powered on or off and bus protection is not touched.
 *  - BYPASS_INIT_ON domains (md, conn, apu) are registered OFF without any
 *    register access (vendor semantics).
 *  - SPM PWR_CON domains are registered ON iff PWR_CON has both ack bits
 *    (30/31) and RST_B set and ISO clear; otherwise OFF.  Read-only access.
 *  - HWV domains are registered ON iff the voter reports done && en && no
 *    pending set (the vendor "enable done" predicate).  Read-only access.
 *  - A parent of an ON child is forced ON.
 *  - Every domain registered ON gets GENPD_FLAG_ALWAYS_ON (same bring-up
 *    policy as mtk_register_power_domains()), so neither late_init, genpd
 *    sync_state nor system suspend will power it off.
 *  - Basic clocks not named in the node's clock-names are skipped instead of
 *    failing probe.
 */

#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "scpsys.h"
#include "mtk-scpsys.h"

#include <dt-bindings/power/mt6897-power.h>

/* Bus protection masks, verbatim from the vendor mtk-scpsys-mt6897.c */
#define MT6897_TOP_AXI_PROT_EN_INFRASYS1_MD	(BIT(12))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_MD	(BIT(11))
#define MT6897_TOP_AXI_PROT_EN_EMISYS0_MD	(BIT(17) | BIT(18))
#define MT6897_TOP_AXI_PROT_EN_EMISYS1_MD	(BIT(17) | BIT(18))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_CONN	(BIT(25))
#define MT6897_TOP_AXI_PROT_EN_CONNSYS0_CONN	(BIT(1))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_CONN_2ND	(BIT(26))
#define MT6897_TOP_AXI_PROT_EN_CONNSYS0_CONN_2ND	(BIT(0))
#define MT6897_VLP_AXI_PROT_EN1_PEXTP_MAC0	(BIT(10))
#define MT6897_VLP_AXI_PROT_EN1_PEXTP_PHY0	(BIT(12))
#define MT6897_TOP_AXI_PROT_EN_PERISYS0_AUDIO	(BIT(6))
#define MT6897_VLP_AXI_PROT_EN_ADSP_TOP	(BIT(14) | BIT(15) |  \
			BIT(16) | BIT(17) |  \
			BIT(18) | BIT(23))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS1_ADSP_TOP	(BIT(14))
#define MT6897_VLP_AXI_PROT_EN_ADSP_TOP_2ND	(BIT(21) | BIT(22) |  \
			BIT(25))
#define MT6897_VLP_AXI_PROT_EN1_ADSP_TOP	(BIT(13) | BIT(14) |  \
			BIT(15) | BIT(16) |  \
			BIT(17) | BIT(18) |  \
			BIT(19) | BIT(20) |  \
			BIT(21) | BIT(22) |  \
			BIT(24))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_ADSP_TOP	(BIT(29) | BIT(30))
#define MT6897_VLP_AXI_PROT_EN1_ADSP_AO	(BIT(22) | BIT(23) |  \
			BIT(24))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS1_ADSP_AO	(BIT(14))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_ADSP_AO	(BIT(29) | BIT(30))
#define MT6897_AXI_PROT_EN_ADSP_AO	(BIT(15))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_MAIN	(BIT(24))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_MAIN_2ND	(BIT(25))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_MAIN	(BIT(0))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_MAIN_2ND	(BIT(1))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_DIP1	(BIT(2))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_DIP1_2ND	(BIT(3))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_VCORE	(BIT(14))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_VCORE_2ND	(BIT(15))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_VCORE	(BIT(18) | BIT(19))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN0	(BIT(12))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN0	(BIT(14))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN0_2ND	(BIT(13))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN0_2ND	(BIT(15))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN1	(BIT(22))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN1	(BIT(16))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN1_2ND	(BIT(23))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN1_2ND	(BIT(17))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_MAIN	(BIT(30))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_CAM_MAIN	(BIT(4))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_MRAW	(BIT(7))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_MRAW_2ND	(BIT(8))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBA	(BIT(1))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBA_2ND	(BIT(3))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBB	(BIT(5))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBB_2ND	(BIT(7))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBC	(BIT(8))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBC_2ND	(BIT(9))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_CAM_VCORE	(BIT(5))
#define MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_VCORE	(BIT(12))
#define MT6897_TOP_AXI_PROT_EN_INFRASYS0_CAM_VCORE	(BIT(27))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_VCORE	(BIT(17) | BIT(27) |  \
			BIT(31))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_VCORE	(BIT(19))
#define MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_CCU	(BIT(8) | BIT(10))
#define MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_CCU_2ND	(BIT(9) | BIT(11))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP0	(BIT(18))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP0	(BIT(20))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP0_2ND	(BIT(19))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP0_2ND	(BIT(21))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP1	(BIT(28))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP1	(BIT(22))
#define MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP1_2ND	(BIT(29))
#define MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP1_2ND	(BIT(23))
#define MT6897_VLP_AXI_PROT_EN_MM_PROC	(BIT(11))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_MM_PROC	(BIT(12))
#define MT6897_VLP_AXI_PROT_EN_MM_PROC_2ND	(BIT(12))
#define MT6897_TOP_AXI_PROT_EN_MMSYS2_MM_PROC_2ND	(BIT(15))

/* PWR_CON bits used for the read-only initial state check */
#define MT6897_PWR_RST_B_BIT		BIT(0)
#define MT6897_PWR_ISO_BIT		BIT(1)
#define MT6897_PWR_ACK_BITS		GENMASK(31, 30)	/* PWR_ACK | PWR_ACK_2ND */

/* Vendor IS_PWR_CON_ON ack bits, expressed as this core's sta_mask */
#define MT6897_STA_MASK			MT6897_PWR_ACK_BITS

/* Common HW voter registers (hw-voter-regmap, 0x10320000) */
#define MT6897_HWV(_shift)				\
		.hwv_set_ofs = 0x0198,			\
		.hwv_clr_ofs = 0x019C,			\
		.hwv_done_ofs = 0x141C,			\
		.hwv_en_ofs = 0x1410,			\
		.hwv_set_sta_ofs = 0x146C,		\
		.hwv_clr_sta_ofs = 0x1470,		\
		.hwv_shift = (_shift)

/* conn bus protection: unchanged from the previous port (== vendor data). */
#define MT6897_CONN_BP_TABLE {						\
		BUS_PROT_IGN(IFR_TYPE, 0x0004, 0x0008, 0x0000, 0x000C,	\
			     MT6897_TOP_AXI_PROT_EN_INFRASYS0_CONN),	\
		BUS_PROT_IGN(IFR_TYPE, 0x01C4, 0x01C8, 0x01C0, 0x01CC,	\
			     MT6897_TOP_AXI_PROT_EN_CONNSYS0_CONN),	\
		BUS_PROT_IGN(IFR_TYPE, 0x0004, 0x0008, 0x0000, 0x000C,	\
			     MT6897_TOP_AXI_PROT_EN_INFRASYS0_CONN_2ND),\
		BUS_PROT_IGN(IFR_TYPE, 0x01C4, 0x01C8, 0x01C0, 0x01CC,	\
			     MT6897_TOP_AXI_PROT_EN_CONNSYS0_CONN_2ND),	\
	}

static const struct scp_domain_data scp_domain_data_mt6897[MT6897_SPM_POWER_DOMAIN_NR] = {
	[MT6897_POWER_DOMAIN_MD] = {
		.name = "md",
		.ctl_offs = 0xE00,
		.extb_iso_offs = 0xFA0,
		.extb_iso_bits = 0x3,
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x024, 0x028, 0x020, 0x02c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS1_MD),
			BUS_PROT_IGN(IFR_TYPE, 0x004, 0x008, 0x000, 0x00c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS0_MD),
			BUS_PROT_IGN(IFR_TYPE, 0x124, 0x128, 0x120, 0x12c,
				MT6897_TOP_AXI_PROT_EN_EMISYS0_MD),
			BUS_PROT_IGN(IFR_TYPE, 0x104, 0x108, 0x100, 0x10c,
				MT6897_TOP_AXI_PROT_EN_EMISYS1_MD),
		},
		/* vendor: | MTK_SCPD_REMOVE_MD_RSTB (not in this core) */
		.caps = MTK_SCPD_IS_PWR_CON_ON | MTK_SCPD_MD_OPS |
			MTK_SCPD_BYPASS_INIT_ON,
	},
	[MT6897_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = 0,		/* previous port; see header note 2) */
		.ctl_offs = 0xE04,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.bp_table = MT6897_CONN_BP_TABLE,
		.caps = MTK_SCPD_BYPASS_INIT_ON | MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_PEXTP_MAC0] = {
		.name = "pextp_mac0",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE18,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "pextp",
		.bp_table = {
			BUS_PROT_IGN(VLP_TYPE, 0x0234, 0x0238, 0x0230, 0x0240,
				MT6897_VLP_AXI_PROT_EN1_PEXTP_MAC0),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_PEXTP_PHY0] = {
		.name = "pextp_phy0",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE20,
		.subsys_clk_prefix = "pextp",
		.bp_table = {
			BUS_PROT_IGN(VLP_TYPE, 0x0234, 0x0238, 0x0230, 0x0240,
				MT6897_VLP_AXI_PROT_EN1_PEXTP_PHY0),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE2C,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"audio"},
		.subsys_clk_prefix = "audio",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x0e4, 0x0e8, 0x0e0, 0x0ec,
				MT6897_TOP_AXI_PROT_EN_PERISYS0_AUDIO),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_ADSP_AO] = {
		.name = "adsp_ao",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE38,
		.basic_clk_name = {"adsp_ao"},
		.bp_table = {
			BUS_PROT_IGN(VLP_TYPE, 0x0234, 0x0238, 0x0230, 0x0240,
				MT6897_VLP_AXI_PROT_EN1_ADSP_AO),
			BUS_PROT_IGN(IFR_TYPE, 0x024, 0x028, 0x020, 0x02c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS1_ADSP_AO),
			BUS_PROT_IGN(IFR_TYPE, 0x004, 0x008, 0x000, 0x00c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS0_ADSP_AO),
			BUS_PROT_IGN(IFR_TYPE, 0x804, 0x808, 0x800, 0x80c,
				MT6897_AXI_PROT_EN_ADSP_AO),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_ADSP_TOP_DORMANT] = {
		.name = "adsp_top_dormant",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE30,
		.sram_slp_bits = GENMASK(9, 9),
		.sram_slp_ack_bits = GENMASK(13, 13),
		.bp_table = {
			BUS_PROT_IGN(VLP_TYPE, 0x0214, 0x0218, 0x0210, 0x0220,
				MT6897_VLP_AXI_PROT_EN_ADSP_TOP),
			BUS_PROT_IGN(IFR_TYPE, 0x024, 0x028, 0x020, 0x02c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS1_ADSP_TOP),
			BUS_PROT_IGN(VLP_TYPE, 0x0214, 0x0218, 0x0210, 0x0220,
				MT6897_VLP_AXI_PROT_EN_ADSP_TOP_2ND),
			BUS_PROT_IGN(VLP_TYPE, 0x0234, 0x0238, 0x0230, 0x0240,
				MT6897_VLP_AXI_PROT_EN1_ADSP_TOP),
			BUS_PROT_IGN(IFR_TYPE, 0x004, 0x008, 0x000, 0x00c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS0_ADSP_TOP),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_SRAM_SLP |
			MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_MM_INFRA] = {
		.name = "mm_infra",
		MT6897_HWV(0),
		.basic_clk_name = {"mm_infra"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_MM_PROC_DORMANT] = {
		.name = "mm_proc_dormant",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xEB0,
		.sram_slp_bits = GENMASK(9, 9),
		.sram_slp_ack_bits = GENMASK(13, 13),
		.basic_clk_name = {"mmup"},
		.bp_table = {
			BUS_PROT_IGN(VLP_TYPE, 0x0214, 0x0218, 0x0210, 0x0220,
				MT6897_VLP_AXI_PROT_EN_MM_PROC),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_MM_PROC),
			BUS_PROT_IGN(VLP_TYPE, 0x0214, 0x0218, 0x0210, 0x0220,
				MT6897_VLP_AXI_PROT_EN_MM_PROC_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_MM_PROC_2ND),
		},
		/* vendor: | MTK_SCPD_PROFILE */
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_SRAM_SLP |
			MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_ISP_MAIN] = {
		/* vendor name; also selects the "isp-main-supply" regulator */
		.name = "isp-main",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE3C,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"isp", "ipe"},
		.subsys_clk_prefix = "isp",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_MAIN),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_MAIN_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_MAIN),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_MAIN_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_ISP_DIP1] = {
		.name = "isp_dip1",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE40,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "dip1",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_DIP1),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_DIP1_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_ISP_VCORE] = {
		.name = "isp_vcore",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE48,
		.subsys_clk_prefix = "isp_vcore",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_VCORE),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_ISP_VCORE_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_ISP_VCORE),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_VDE0] = {
		.name = "vde0",
		MT6897_HWV(1),
		.basic_clk_name = {"vde"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_VDE1] = {
		.name = "vde1",
		MT6897_HWV(2),
		.basic_clk_name = {"vde"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_VEN0] = {
		.name = "ven0",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE5C,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"ven"},
		.subsys_clk_prefix = "ven0",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN0),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN0),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN0_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN0_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_VEN1] = {
		.name = "ven1",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE60,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"ven"},
		.subsys_clk_prefix = "ven1",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN1),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN1),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_VEN1_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_VEN1_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_MAIN] = {
		/* vendor name; also selects the "cam-main-supply" regulator */
		.name = "cam-main",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE68,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "cam_main",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_MAIN),
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_CAM_MAIN),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_MRAW] = {
		.name = "cam_mraw",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE6C,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "cam_mraw",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_MRAW),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_MRAW_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_SUBA] = {
		.name = "cam_suba",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE70,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "cam_suba",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBA),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBA_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_SUBB] = {
		.name = "cam_subb",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE74,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "cam_subb",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBB),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBB_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_SUBC] = {
		.name = "cam_subc",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE78,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.subsys_clk_prefix = "cam_subc",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBC),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_SUBC_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_VCORE] = {
		.name = "cam_vcore",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE84,
		/* vendor: {"cam", "dpe", "ccu", "ccu_ahb"}; MAX_CLKS is 3 here */
		.basic_clk_name = {"cam", "dpe", "ccu"},
		.basic_lp_clk_name = {"ccu_ahb"},
		.subsys_clk_prefix = "cam_vcore",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x224, 0x228, 0x220, 0x22c,
				MT6897_TOP_AXI_PROT_EN_MMSYS2_CAM_VCORE),
			BUS_PROT_IGN(IFR_TYPE, 0x264, 0x268, 0x260, 0x26c,
				MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_VCORE),
			BUS_PROT_IGN(IFR_TYPE, 0x004, 0x008, 0x000, 0x00c,
				MT6897_TOP_AXI_PROT_EN_INFRASYS0_CAM_VCORE),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_CAM_VCORE),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_CAM_VCORE),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_CCU] = {
		.name = "cam_ccu",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE88,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"ccu", "ccu_ahb"},
		.subsys_clk_prefix = "cam_ccu",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x264, 0x268, 0x260, 0x26c,
				MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_CCU),
			BUS_PROT_IGN(IFR_TYPE, 0x264, 0x268, 0x260, 0x26c,
				MT6897_TOP_AXI_PROT_EN_CCUSYS0_CAM_CCU_2ND),
		},
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_CAM_CCU_AO] = {
		.name = "cam_ccu_ao",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE8C,
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_MDP0_SHUTDOWN] = {
		.name = "mdp0_shutdown",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE94,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"mdp"},
		.subsys_clk_prefix = "mdp0",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP0),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP0),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP0_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP0_2ND),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_MDP1_SHUTDOWN] = {
		.name = "mdp1_shutdown",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xE98,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_name = {"mdp1"},
		.subsys_clk_prefix = "mdp1",
		.bp_table = {
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP1),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP1),
			BUS_PROT_IGN(IFR_TYPE, 0x1e4, 0x1e8, 0x1e0, 0x1ec,
				MT6897_TOP_AXI_PROT_EN_MMSYS0_MDP1_2ND),
			BUS_PROT_IGN(IFR_TYPE, 0x204, 0x208, 0x200, 0x20c,
				MT6897_TOP_AXI_PROT_EN_MMSYS1_MDP1_2ND),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_DIS0_SHUTDOWN] = {
		.name = "dis0_shutdown",
		MT6897_HWV(3),
		.basic_clk_name = {"disp"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_DIS1_SHUTDOWN] = {
		.name = "dis1_shutdown",
		MT6897_HWV(4),
		.basic_clk_name = {"disp1"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_OVL0_SHUTDOWN] = {
		.name = "ovl0_shutdown",
		MT6897_HWV(5),
		.basic_clk_name = {"ovl"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_OVL1_SHUTDOWN] = {
		.name = "ovl1_shutdown",
		MT6897_HWV(6),
		.basic_clk_name = {"ovl1"},
		.caps = MTK_SCPD_HWV_OPS,
	},
	[MT6897_POWER_DOMAIN_CSI_RX] = {
		.name = "csi_rx",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xEF4,
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_DP_TX] = {
		.name = "dp_tx",
		.sta_mask = MT6897_STA_MASK,
		.ctl_offs = 0xEB4,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_IS_PWR_CON_ON,
	},
	[MT6897_POWER_DOMAIN_APU] = {
		.name = "apu",
		.caps = MTK_SCPD_APU_OPS | MTK_SCPD_BYPASS_INIT_ON,
	},
};

static const struct scp_subdomain scp_subdomain_mt6897[] = {
	{MT6897_POWER_DOMAIN_PEXTP_MAC0, MT6897_POWER_DOMAIN_PEXTP_PHY0},
	{MT6897_POWER_DOMAIN_ADSP_AO, MT6897_POWER_DOMAIN_ADSP_TOP_DORMANT},
	{MT6897_POWER_DOMAIN_ISP_VCORE, MT6897_POWER_DOMAIN_ISP_MAIN},
	{MT6897_POWER_DOMAIN_ISP_MAIN, MT6897_POWER_DOMAIN_ISP_DIP1},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_ISP_VCORE},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_VDE0},
	{MT6897_POWER_DOMAIN_VDE0, MT6897_POWER_DOMAIN_VDE1},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_VEN0},
	{MT6897_POWER_DOMAIN_VEN0, MT6897_POWER_DOMAIN_VEN1},
	{MT6897_POWER_DOMAIN_CAM_VCORE, MT6897_POWER_DOMAIN_CAM_MAIN},
	{MT6897_POWER_DOMAIN_CAM_MAIN, MT6897_POWER_DOMAIN_CAM_MRAW},
	{MT6897_POWER_DOMAIN_CAM_MAIN, MT6897_POWER_DOMAIN_CAM_SUBA},
	{MT6897_POWER_DOMAIN_CAM_MAIN, MT6897_POWER_DOMAIN_CAM_SUBB},
	{MT6897_POWER_DOMAIN_CAM_MAIN, MT6897_POWER_DOMAIN_CAM_SUBC},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_CAM_VCORE},
	{MT6897_POWER_DOMAIN_CAM_VCORE, MT6897_POWER_DOMAIN_CAM_CCU},
	{MT6897_POWER_DOMAIN_CAM_CCU, MT6897_POWER_DOMAIN_CAM_CCU_AO},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_MDP0_SHUTDOWN},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_MDP1_SHUTDOWN},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_DIS0_SHUTDOWN},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_DIS1_SHUTDOWN},
	{MT6897_POWER_DOMAIN_DIS0_SHUTDOWN, MT6897_POWER_DOMAIN_OVL0_SHUTDOWN},
	{MT6897_POWER_DOMAIN_DIS1_SHUTDOWN, MT6897_POWER_DOMAIN_OVL1_SHUTDOWN},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_MM_PROC_DORMANT},
	{MT6897_POWER_DOMAIN_MM_INFRA, MT6897_POWER_DOMAIN_DP_TX},
};

#define MT6897_NUM_SUBDOMAINS	ARRAY_SIZE(scp_subdomain_mt6897)

static const struct scp_soc_data mt6897_data = {
	.domains = scp_domain_data_mt6897,
	.num_domains = MT6897_SPM_POWER_DOMAIN_NR,
	.subdomains = scp_subdomain_mt6897,
	.num_subdomains = MT6897_NUM_SUBDOMAINS,
	.regs = {
		.pwr_sta_offs = 0xFB0,
		.pwr_sta2nd_offs = 0xFB4,
	},
};

/*
 * Power ops for domains this port cannot drive faithfully (or whose
 * regmaps/clocks are missing from DT).  A consumer attaching to such a domain
 * gets an error instead of an incomplete power sequence.
 */
static int mt6897_pd_refuse(struct generic_pm_domain *genpd)
{
	pr_warn("MT6897-SCPSYS: %s: power transitions not supported by this port\n",
		genpd->name);
	return -EOPNOTSUPP;
}

/* Drop basic clocks that are not named in the node's clock-names. */
static void mt6897_filter_basic_clks(struct device *dev, const char *domain,
				     const char **names)
{
	struct device_node *np = dev->of_node;
	int i, j = 0;

	for (i = 0; i < MAX_CLKS && names[i]; i++) {
		if (of_property_match_string(np, "clock-names", names[i]) >= 0) {
			names[j++] = names[i];
			continue;
		}
		dev_notice(dev, "%s: clock \"%s\" not in DT, not used\n",
			   domain, names[i]);
	}
	for (; j < MAX_CLKS; j++)
		names[j] = NULL;
}

/* Same match rule as init_subsys_clks() in mtk-scpsys.c: "<prefix>-..." */
static int mt6897_count_subsys_clks(struct device_node *np, const char *prefix)
{
	size_t len = strlen(prefix);
	struct property *prop;
	const char *name;
	int n = 0;

	of_property_for_each_string(np, "clock-names", prop, name)
		if (!strncmp(name, prefix, len) && name[len] == '-')
			n++;

	return n;
}

static struct regmap *mt6897_bp_regmap(struct scp *scp, enum regmap_type type)
{
	switch (type) {
	case IFR_TYPE:
		return scp->infracfg;
	case SMI_TYPE:
		return scp->smi_common;
	case VLP_TYPE:
		return scp->vlpcfg;
	case MFGRPC_TYPE:
		return scp->mfgrpc;
	case NEMI_TYPE:
		return scp->nemi;
	case SEMI_TYPE:
		return scp->semi;
	default:
		return NULL;
	}
}

/* true if every bus-protection step of @d has its regmap available */
static bool mt6897_bp_regmaps_ok(struct device *dev, struct scp *scp,
				 const struct scp_domain_data *d)
{
	int i;

	for (i = 0; i < MAX_STEPS; i++) {
		enum regmap_type type = d->bp_table[i].type;

		if (type == INVALID_TYPE)
			break;
		if (!mt6897_bp_regmap(scp, type)) {
			dev_warn(dev, "%s: bus-protection regmap (type %d) missing in DT\n",
				 d->name, type);
			return false;
		}
	}

	return true;
}

/* SPM PWR_CON offsets of the voter-controlled domains, indexed by vote bit. */
static const u16 mt6897_hwv_pwr_con[] = {
	0xEAC,	/* mm_infra */
	0xE4C,	/* vde0 */
	0xE50,	/* vde1 */
	0xE9C,	/* dis0 */
	0xEA0,	/* dis1 */
	0xEA4,	/* ovl0 */
	0xEA8,	/* ovl1 */
};

/*
 * Read-only check of the state the bootloader left a domain in.
 * Never writes a register.
 */
static bool mt6897_pd_hw_is_on(struct device *dev, struct scp *scp,
			       const struct scp_domain_data *d)
{
	u32 val, done, en, set_sta, bit;

	if (d->caps & (MTK_SCPD_BYPASS_INIT_ON | MTK_SCPD_APU_OPS |
		       MTK_SCPD_MD_OPS))
		return false;

	if (d->caps & MTK_SCPD_HWV_OPS) {
		/*
		 * The voter registers only say who has voted. The bootloader
		 * powers the display domains through SPM directly, without a
		 * vote, so a lit panel shows no vote at all. The power state is
		 * in the domain's SPM PWR_CON ack bits, which is also what the
		 * vendor's power-domain checker reads.
		 */
		if (d->hwv_shift >= ARRAY_SIZE(mt6897_hwv_pwr_con))
			return false;
		val = readl(scp->base + mt6897_hwv_pwr_con[d->hwv_shift]);
		done = en = set_sta = 0;
		bit = BIT(d->hwv_shift);
		if (scp->hwv_regmap) {
			regmap_read(scp->hwv_regmap, d->hwv_done_ofs, &done);
			regmap_read(scp->hwv_regmap, d->hwv_en_ofs, &en);
			regmap_read(scp->hwv_regmap, d->hwv_set_sta_ofs, &set_sta);
		}
		dev_info(dev, "%s: PWR_CON(%#x)=%#x, vote done=%d en=%d set_sta=%d\n",
			 d->name, mt6897_hwv_pwr_con[d->hwv_shift], val,
			 !!(done & bit), !!(en & bit), !!(set_sta & bit));
		return (val & MT6897_PWR_ACK_BITS) == MT6897_PWR_ACK_BITS;
	}

	val = readl(scp->base + d->ctl_offs);
	dev_dbg(dev, "%s: PWR_CON(%#x)=%#x\n", d->name, d->ctl_offs, val);

	return (val & MT6897_PWR_ACK_BITS) == MT6897_PWR_ACK_BITS &&
	       (val & MT6897_PWR_RST_B_BIT) && !(val & MT6897_PWR_ISO_BIT);
}

static int mt6897_register_domains(struct platform_device *pdev,
				   struct scp *scp,
				   const struct scp_soc_data *soc)
{
	struct device *dev = &pdev->dev;
	struct genpd_onecell_data *pd_data = &scp->pd_data;
	DECLARE_BITMAP(sd_added, MT6897_NUM_SUBDOMAINS);
	bool on[MT6897_SPM_POWER_DOMAIN_NR];
	const struct scp_subdomain *sd;
	bool changed;
	int i, ret;

	bitmap_zero(sd_added, MT6897_NUM_SUBDOMAINS);

	for (i = 0; i < soc->num_domains; i++)
		on[i] = mt6897_pd_hw_is_on(dev, scp, scp->domains[i].data);

	/* A powered child implies a powered parent. */
	do {
		changed = false;
		for (i = 0, sd = soc->subdomains; i < soc->num_subdomains; i++, sd++) {
			if (on[sd->subdomain] && !on[sd->origin]) {
				dev_warn(dev, "%s reads off but child %s is on; registering it on\n",
					 scp->domains[sd->origin].data->name,
					 scp->domains[sd->subdomain].data->name);
				on[sd->origin] = true;
				changed = true;
			}
		}
	} while (changed);

	for (i = 0; i < soc->num_domains; i++) {
		struct generic_pm_domain *genpd = &scp->domains[i].genpd;

		/* Left on by the bootloader: never let genpd switch it off. */
		if (on[i])
			genpd->flags |= GENPD_FLAG_ALWAYS_ON;

		ret = pm_genpd_init(genpd, NULL, !on[i]);
		if (ret) {
			dev_err(dev, "pm_genpd_init(%s) failed: %d\n",
				genpd->name, ret);
			goto err_remove_genpd;
		}

		dev_info(dev, "domain %2d %-16s %s%s\n", i, genpd->name,
			 on[i] ? "on (always-on)" : "off",
			 genpd->power_on == mt6897_pd_refuse ? " [ops refused]" : "");
	}

	for (i = 0, sd = soc->subdomains; i < soc->num_subdomains; i++, sd++) {
		ret = pm_genpd_add_subdomain(pd_data->domains[sd->origin],
					     pd_data->domains[sd->subdomain]);
		if (ret)
			dev_warn(dev, "add subdomain %s -> %s failed: %d\n",
				 pd_data->domains[sd->origin]->name,
				 pd_data->domains[sd->subdomain]->name, ret);
		else
			__set_bit(i, sd_added);
	}

	ret = of_genpd_add_provider_onecell(dev->of_node, pd_data);
	if (ret) {
		dev_err(dev, "Failed to add OF provider: %d\n", ret);
		goto err_remove_sd;
	}

	return 0;

err_remove_sd:
	for (i = soc->num_subdomains - 1; i >= 0; i--) {
		sd = &soc->subdomains[i];
		if (test_bit(i, sd_added))
			pm_genpd_remove_subdomain(pd_data->domains[sd->origin],
						  pd_data->domains[sd->subdomain]);
	}
	i = soc->num_domains;
err_remove_genpd:
	while (--i >= 0)
		pm_genpd_remove(&scp->domains[i].genpd);

	return ret;
}

static const struct of_device_id of_scpsys_match_tbl[] = {
	{
		.compatible = "mediatek,mt6897-scpsys",
		/* ⚠️ 必须带 .data！否则 of_device_get_match_data() 返回 NULL */
		.data = &mt6897_data,
	}, {
		/* sentinel */
	}
};

static int mt6897_scpsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	DECLARE_BITMAP(refuse, MT6897_SPM_POWER_DOMAIN_NR);
	const struct scp_soc_data *soc;
	struct scp_domain_data *data;
	struct scp *scp;
	int i, n, ret;

	soc = of_device_get_match_data(dev);
	if (!soc) {
		dev_err(dev, "no match data\n");
		return -EINVAL;
	}

	dev_info(dev, "MT6897-SCPSYS: probe enter (num_domains=%d)\n",
		 soc->num_domains);

	bitmap_zero(refuse, MT6897_SPM_POWER_DOMAIN_NR);

	/* Work on a copy so clock lists can be trimmed to what DT provides. */
	data = devm_kmemdup(dev, soc->domains,
			    soc->num_domains * sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < soc->num_domains; i++) {
		struct scp_domain_data *d = &data[i];

		mt6897_filter_basic_clks(dev, d->name, d->basic_clk_name);
		mt6897_filter_basic_clks(dev, d->name, d->basic_lp_clk_name);

		if (d->subsys_clk_prefix) {
			n = mt6897_count_subsys_clks(dev->of_node,
						     d->subsys_clk_prefix);
			if (n > MAX_SUBSYS_CLKS) {
				dev_warn(dev, "%s: %d \"%s-*\" clocks > MAX_SUBSYS_CLKS (%d); domain ops disabled\n",
					 d->name, n, d->subsys_clk_prefix,
					 MAX_SUBSYS_CLKS);
				d->subsys_clk_prefix = NULL;
				__set_bit(i, refuse);
			}
		}

		/* MTK_SCPD_REMOVE_MD_RSTB cannot be expressed in this core. */
		if (i == MT6897_POWER_DOMAIN_MD)
			__set_bit(i, refuse);
	}

	scp = init_scp(pdev, data, soc->num_domains, &soc->regs);
	if (IS_ERR(scp))
		return PTR_ERR(scp);

	for (i = 0; i < soc->num_domains; i++) {
		struct scp_domain *scpd = &scp->domains[i];
		const struct scp_domain_data *d = scpd->data;

		if ((d->caps & MTK_SCPD_HWV_OPS) && !scp->hwv_regmap) {
			dev_warn(dev, "%s: no hw-voter-regmap in DT; domain ops disabled\n",
				 d->name);
			__set_bit(i, refuse);
		}
		if (!mt6897_bp_regmaps_ok(dev, scp, d))
			__set_bit(i, refuse);

		if (test_bit(i, refuse)) {
			scpd->genpd.power_on = mt6897_pd_refuse;
			scpd->genpd.power_off = mt6897_pd_refuse;
		}
	}

	ret = mt6897_register_domains(pdev, scp, soc);
	if (ret)
		return ret;

	dev_info(dev, "MT6897-SCPSYS: registered %d power domains\n",
		 soc->num_domains);

	return 0;
}

static struct platform_driver mt6897_scpsys_drv = {
	.probe = mt6897_scpsys_probe,
	.driver = {
		.name = "mtk-scpsys-mt6897",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_scpsys_match_tbl),
	},
};

/*
 * ★★ arch_initcall —— 不能是 late_initcall（2026-09-18 实测教训）
 *
 * late_initcall 排在 device_initcall 之后：consys 拿不到 conn 域而
 * -EPROBE_DEFER，但 wifi 驱动照常 probe 并访问 0x18000000 的 WLAN CSR，
 * 此时 CONN MTCMOS 未上电 ⇒ AXI 总线停摆 ⇒ 卡 boot logo。
 *
 * NOTE: with the full table, any DT-listed clock whose provider is not yet
 * registered makes this probe return -EPROBE_DEFER (conn included), so only
 * list clocks whose providers exist.
 */
static int __init mt6897_scpsys_init(void)
{
	int ret;

	pr_info("MT6897-SCPSYS: driver init enter\n");
	ret = platform_driver_register(&mt6897_scpsys_drv);
	pr_info("MT6897-SCPSYS: driver init ret=%d\n", ret);
	return ret;
}

static void __exit mt6897_scpsys_exit(void)
{
	platform_driver_unregister(&mt6897_scpsys_drv);
}

arch_initcall(mt6897_scpsys_init);
module_exit(mt6897_scpsys_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6897 SPM power domains");
