// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6897 MFlexGraphics (GPU) power domain, driven through the GPUEB.
 *
 * On MT6897 (Dimensity 8300) the GPUEB co-processor is loaded and started by
 * the bootloader and owns all GPU power sequencing: the VGPU/VSRAM bucks, the
 * MFG MTCMOS through MFG_RPC, MFGPLL/MFGSCPLL and DVFS. The AP only sends it
 * "gpufreq" IPI commands. This driver speaks the same IPI ABI as the MT8196
 * driver in mtk-mfg-pmdomain.c, but it is a separate driver because almost
 * everything around the IPIs differs on MT6897:
 *
 *  - the AP never starts or stops the EB (no GHPM, no eb_on/eb_off, no sleep
 *    IPI): the bootloader already did, and the EB must stay up;
 *  - there are no AP-side clocks or regulators for the GPU at all;
 *  - the shared status layout (struct gpufreq_shared_status) has a different
 *    field set, so every offset after 0xa8 differs from the MT8196 GF_REG_*;
 *  - single buck ("dual-buck" = 0), one OPP table, TARGET_GPU only;
 *  - power state is read from MFG_RPC_PWR_CON_STATUS and SPM XPU_PWR_STATUS.
 *
 * Bring-up is split into stages so each can be tested on its own:
 *
 *  Stage A (probe): map registers and the shared memory, read and log the GPR
 *    block (including the IPI magic) and the power status registers. No write
 *    to any hardware register and no IPI. Register the genpd and clock
 *    providers and the debugfs trigger files.
 *  Stage A1 (debugfs a1_init): what the vendor gpufreq_wrapper probe does in
 *    EB mode: read the IPI magic from GPR13 and clear it, zero the gpufreq
 *    shared memory, send CMD_INIT_SHARED_MEM, then decode the status the EB
 *    wrote there.
 *  Stage B (debugfs b_power_on / b_power_off): CMD_POWER_CONTROL as the
 *    vendor Mali platform code does, then read the MFG power status and, only
 *    if the EB reported success and MFG0/MFG1 are on, the Mali ID registers.
 *  Stage C: a consumer (panthor) attached to this genpd. Init (A1) happens
 *    lazily on attach; power on/off are CMD_POWER_CONTROL. One fixed OPP.
 *
 * This driver never touches a regulator, a PLL or an MFG_RPC control
 * register. All of that is the EB's.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/string_choices.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/* GPR block (vendor "gpueb_gpr_base", 0x13c2fd1c, 25 words) */
#define GPR_NUM			25
#define GPR_IPI_MAGIC		(13 * 4)	/* GPUEB_SRAM_GPR13 */

/* MFG_RPC, mapped from 0x13f91000 (vendor gpufreq_reg_mt6897.h, base + 0x1000) */
#define RPC_MFG1_PWR_CON	0x070
#define RPC_PWR_CON_STATUS	0x200
#define RPC_PWR_CON_2ND_STATUS	0x204
#define   RPC_MFG1_14_MASK	(GENMASK(14, 9) | GENMASK(7, 6) | GENMASK(4, 1))

/* SPM, mapped from 0x1c001fc0 (XPU_PWR_STATUS, XPU_PWR_STATUS_2ND) */
#define SPM_XPU_PWR_STATUS	0x0
#define SPM_XPU_PWR_STATUS_2ND	0x4
#define   SPM_MFG0_PWR_BIT	BIT(1)

/* GPUEB_MBOX_PDCA_STA, 0x13c620a8 */
#define PDCA_STA_BUSY		BIT(16)

/*
 * Read-only clock readback (vendor __gpufreq_get_real_fgpu/fstack and the
 * "[Clk] MFG_PLL ... MFG_SEL" line of __gpufreq_dump_infra_status):
 * MFG_PLL_CON1 0x13fa000c, MFGSC_PLL_CON1 0x13fa0c0c, TOPCK_CLK_CFG_30
 * 0x100001f0. Freq(kHz) = (CON1[21:0] * 26000 >> 14) >> CON1[26:24].
 */
#define PLL_CON1_PCW		GENMASK(21, 0)
#define PLL_CON1_POSDIV		GENMASK(26, 24)
#define TOPCK_MFG_SEL_PLL	BIT(16)		/* MFG_SEL_MFGPLL_MASK */
#define TOPCK_MFGSC_SEL_PLL	BIT(17)		/* MFGSC_SEL_MFGPSCLL_MASK */

/* Mali registers read in stage B (offsets from GPU_CONTROL_BASE = 0) */
#define MALI_GPU_ID		0x000
#define MALI_L2_FEATURES	0x004
#define MALI_CORE_FEATURES	0x008
#define MALI_TILER_FEATURES	0x00c
#define MALI_MEM_FEATURES	0x010
#define MALI_MMU_FEATURES	0x014
#define MALI_AS_PRESENT		0x018
#define MALI_CSF_ID		0x01c
#define MALI_GPU_STATUS		0x034
#define MALI_SHADER_PRESENT_LO	0x100
#define MALI_SHADER_PRESENT_HI	0x104
#define MALI_TILER_PRESENT_LO	0x110
#define MALI_L2_PRESENT_LO	0x120
#define MALI_COHERENCY_FEATURES	0x300

/*
 * struct gpufreq_shared_status offsets for MT6897 (vendor gpufreq_v2.h,
 * verified by compiling the vendor struct; all fields are 32-bit).
 */
#define GF_MAGIC		0x0000
#define GF_CUR_OPPIDX_GPU	0x0004
#define GF_CUR_OPPIDX_STACK	0x0008
#define GF_OPP_NUM_GPU		0x000c
#define GF_OPP_NUM_STACK	0x0010
#define GF_SIGNED_OPP_NUM_GPU	0x0014
#define GF_POWER_COUNT		0x001c
#define GF_BUCK_COUNT		0x0020
#define GF_MTCMOS_COUNT		0x0024
#define GF_CG_COUNT		0x0028
#define GF_ACTIVE_COUNT		0x002c
#define GF_TEMPERATURE		0x0030
#define GF_CUR_FGPU		0x0044	/* kHz */
#define GF_CUR_FSTACK		0x0048
#define GF_CUR_VGPU		0x005c	/* 10 uV */
#define GF_CUR_VSTACK		0x0060
#define GF_CUR_VSRAM_GPU	0x0064
#define GF_CUR_POWER_GPU	0x007c	/* mW */
#define GF_MAX_POWER_GPU	0x0084
#define GF_MIN_POWER_GPU	0x008c
#define GF_CUR_CEILING		0x00ac
#define GF_CUR_FLOOR		0x00b0
#define GF_CUR_C_LIMITER	0x00b4
#define GF_CUR_F_LIMITER	0x00b8
#define GF_CUR_C_PRIORITY	0x00bc
#define GF_CUR_F_PRIORITY	0x00c0
#define GF_POWER_CONTROL	0x00c4
#define GF_ACTIVE_SLEEP_CONTROL	0x00c8
#define GF_DVFS_STATE		0x00cc
#define GF_SHADER_PRESENT	0x00d0
#define GF_AVS_ENABLE		0x00e0
#define GF_SB_VERSION		0x00e8
#define GF_PTP_VERSION		0x00ec
#define GF_DBG_VERSION		0x00f0
#define GF_GPM1_MODE		0x00f4
#define GF_DUAL_BUCK		0x00fc
#define GF_SEGMENT_ID		0x0100
#define GF_POWER_TIME_H		0x0104
#define GF_POWER_TIME_L		0x0108
#define GF_MFG_PWR_STATUS	0x010c
#define GF_TEST_MODE		0x0114
#define GF_WORKING_TABLE_GPU	0x03ec
/*
 * struct gpuppm_limit_info limit_table[LIMIT_NUM] at 0x1e2c, 44 bytes each:
 * limiter, name[20], priority, ceiling, c_enable, floor, f_enable.
 */
#define GF_LIMIT_TABLE		0x1e2c
#define GF_LIMIT_ENTRY_SIZE	44
#define GF_LIMIT_PRIORITY	24
#define GF_LIMIT_CEILING	28
#define GF_LIMIT_C_ENABLE	32
#define GF_LIMIT_FLOOR		36
#define GF_LIMIT_F_ENABLE	40
#define GF_SHARED_STATUS_SIZE	0x24ec

/* struct gpufreq_opp_info: freq(kHz), volt, vsram (10 uV), posdiv, margin, power(mW) */
#define GF_OPP_ENTRY_SIZE	24
#define GF_MAX_OPP_NUM		70

#define GPUFREQ_MAGIC		0xBABADADA
/* gpueb-mem-table index 1 of the vendor DT: MEM_ID_GPUFREQ is 0x4000 bytes */
#define GPUFREQ_MEM_SIZE	0x4000

#define IPI_TIMEOUT_MS		1000
#define IPI_MSG_SIZE		32
#define MBOX_MAX_RX_SIZE	32

enum mt6897_gf_cmd {
	CMD_INIT_SHARED_MEM	= 0,
	CMD_SET_LIMIT		= 5,
	CMD_POWER_CONTROL	= 6,
	CMD_ACTIVE_SLEEP_CONTROL = 7,
	CMD_COMMIT		= 8,
	CMD_FIX_TARGET_OPPIDX	= 13,
};

/* enum gpufreq_dvfs_state (vendor gpufreq_v2.h) */
#define DVFS_DISABLE		BIT(0)
#define DVFS_POWEROFF		BIT(1)
#define DVFS_FIX_OPP		BIT(2)
#define DVFS_FIX_FREQ_VOLT	BIT(3)
#define DVFS_SLEEP		BIT(5)

#define TARGET_GPU		1

/* enum gpuppm_limiter and the reserved indices (vendor gpufreq_v2.h) */
#define LIMIT_SEGMENT		0
#define LIMIT_DEBUG		1	/* OPP index, priority 8: our static cap */
#define LIMIT_THERMAL_AP	4	/* kHz, priority 6: left for a future thermal driver */
#define LIMIT_NUM		13
#define GPUPPM_DEFAULT_IDX	-1	/* drop this limiter's limit */
#define GPUPPM_KEEP_IDX		-3	/* leave this side unchanged */

/*
 * Without a GPU thermal zone or cooling device the GPU must not go above this
 * (OPP 37, 800 MHz, about 1.5 W). max_freq_khz can only be raised past it
 * together with allow_above_cap=1.
 */
#define MT6897_HARD_CAP_KHZ	800000
#define GPU_PWR_OFF		0
#define GPU_PWR_ON		1

/*
 * struct gpufreq_ipi_data (vendor gpufreq_ipi.h). The union holds a u64, so
 * the compiler aligns it to 8 and inserts 4 bytes after @target. Total 32
 * bytes = the 8-slot gpufreq channel.
 */
struct mt6897_gf_ipi_msg {
	__le32 magic;
	__le32 cmd;
	__le32 target;
	__le32 pad;
	union {
		__le32 value;		/* oppidx, return_value, power_state */
		struct {
			__le32 base_lo;
			__le32 base_hi;
			__le32 size;
		} shared_mem;
		__le32 raw[4];
	} u;
} __packed;
static_assert(sizeof(struct mt6897_gf_ipi_msg) == IPI_MSG_SIZE);

static const char *const mt6897_mbox_names[] = {
	"fast-dvfs-event", "gpufreq", "sleep", "timer", "fhctl", "ccf",
	"gpumpu", "fast-dvfs", "ipir-c-met", "ipis-c-met", "brisket",
};
#define MBOX_GPUFREQ		1

static int fix_oppidx = -1;
module_param(fix_oppidx, int, 0644);
MODULE_PARM_DESC(fix_oppidx,
		 "Stage C: send CMD_FIX_TARGET_OPPIDX(GPU, n) after the first power-on (-1 = leave EB default)");

static bool clear_ipi_magic = true;
module_param(clear_ipi_magic, bool, 0644);
MODULE_PARM_DESC(clear_ipi_magic, "Clear GPR13 after reading the IPI magic, like the vendor driver");

static bool allow_power_off = true;
module_param(allow_power_off, bool, 0644);
MODULE_PARM_DESC(allow_power_off,
		 "Stage C: send CMD_POWER_CONTROL(off) when the genpd powers off (0 = keep the GPU on)");

/*
 * No GPU thermal zone or cooling device exists yet and the top OPPs draw up
 * to 3.8 W, so every commit (devfreq or manual) is clamped to OPPs at or below
 * this frequency, and only those OPPs are offered to the GPU device.
 */
static unsigned int max_freq_khz = MT6897_HARD_CAP_KHZ;
static bool allow_above_cap;
module_param(allow_above_cap, bool, 0644);
MODULE_PARM_DESC(allow_above_cap,
		 "Allow max_freq_khz and the debugfs limit above the built-in 800 MHz cap (no thermal protection!)");

static int max_freq_khz_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops max_freq_khz_ops = {
	.set = max_freq_khz_set,
	.get = param_get_uint,
};
module_param_cb(max_freq_khz, &max_freq_khz_ops, &max_freq_khz, 0644);
MODULE_PARM_DESC(max_freq_khz,
		 "Highest GPU frequency, kHz: enforced in the EB with CMD_SET_LIMIT and by every commit (devfreq OPP table is cut at attach time)");

static bool auto_active = true;
module_param(auto_active, bool, 0644);
MODULE_PARM_DESC(auto_active,
		 "Stage C: send CMD_ACTIVE_SLEEP_CONTROL(active) after power-on when the EB reports active_sleep_control");

static bool verbose = true;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose,
		 "Log every IPI and status readback at info level (0 = debug level; errors always logged)");

/* Routine per-IPI / per-readback logging, at info level only while bring-up wants it. */
#define mfg_vinfo(dev, fmt, ...)					\
	do {								\
		if (verbose)						\
			dev_info(dev, fmt, ##__VA_ARGS__);		\
		else							\
			dev_dbg(dev, fmt, ##__VA_ARGS__);		\
	} while (0)

/*
 * Breadcrumbs for silent deaths: one short line before each step that could
 * hang the bus or the EB (lock, IPI, MMIO read group). printk() writes the
 * pstore/ramoops console synchronously in the caller's context on this
 * (non-PREEMPT_RT) kernel, so the last "@" line in console-ramoops is the
 * last step that was entered.
 */
#define mfg_step(mfg, fmt, ...)						\
	do {								\
		if (verbose)						\
			dev_info((mfg)->dev, "@ " fmt "\n", ##__VA_ARGS__);	\
	} while (0)

static bool force;
module_param(force, bool, 0644);
MODULE_PARM_DESC(force,
		 "Let debugfs triggers idle the EB while a consumer (panthor) holds the power domain on (dangerous)");

static bool follow_devfreq = true;
module_param(follow_devfreq, bool, 0644);
MODULE_PARM_DESC(follow_devfreq,
		 "Stage C: turn genpd performance-state requests (devfreq) into CMD_COMMIT (0 = record only)");

struct mt6897_mfg;

struct mt6897_mbox {
	struct mbox_client cl;
	struct mbox_chan *ch;
	struct mt6897_mfg *mfg;
	unsigned int idx;
};

struct mt6897_mfg {
	struct device *dev;
	struct generic_pm_domain pd;
	struct clk_hw clk_hw;

	void __iomem *gpr;
	void __iomem *rpc;
	void __iomem *spm;
	void __iomem *pdca;
	void __iomem *shared;
	phys_addr_t shared_phys;
	struct resource gpu_res;

	struct mutex lock;		/* serialises every EB access below */
	struct mt6897_mbox mbox[ARRAY_SIZE(mt6897_mbox_names)];
	bool mbox_ready;
	struct completion rx_done;
	u8 rx_buf[MBOX_MAX_RX_SIZE];
	struct mt6897_gf_ipi_msg tx_buf;	/* the mailbox core keeps a pointer */
	bool ipi_broken;		/* an IPI went unanswered: stop talking */

	u32 ipi_magic;
	bool initialized;		/* CMD_INIT_SHARED_MEM acknowledged */
	bool init_failed;
	bool fix_applied;
	unsigned int dbg_power_refs;	/* stage B power-on requests */

	/*
	 * EB-side state. Power is ours to track (the EB has no other client);
	 * active/idle is NOT: the EB may come out of POWER_CONTROL(on) already
	 * active, so it is always read from shared active_count.
	 */
	bool eb_powered;		/* POWER_CONTROL(on) sent, no off since */
	bool going_down;		/* power-off started: no MFG register reads */
	/*
	 * ACTIVE references held by this driver (0 or 1). The EB's
	 * active_count also contains a reference the EB takes by itself when
	 * the GPU starts running and drops when it goes idle; that one is
	 * never ours to release (boot 75: releasing it killed the SoC).
	 */
	int our_active;
	int seen_active;		/* last logged EB active_count */

	/*
	 * After power-on the EB activates itself once the GPU runs (after
	 * genpd power_on has returned) and may then re-apply an OPP of its
	 * own; this re-checks opp_idx against the target a few times.
	 */
	struct delayed_work verify_work;
	unsigned int verify_left;
	int target_idx;			/* OPP index to commit (devfreq / manual) */
	int last_commit;		/* last index acknowledged by CMD_COMMIT */
	int eb_limit;			/* LIMIT_DEBUG ceiling we set, -1 = none */

	void __iomem *mfgpll_con1;	/* optional, read-only */
	void __iomem *mfgscpll_con1;
	void __iomem *topck_cfg30;

	struct dentry *dbg;
};

/* the single instance, for the module-parameter callback and genpd dev_ops */
static struct mt6897_mfg *mt6897_the_mfg;

static inline u32 gf_read(struct mt6897_mfg *mfg, unsigned int off)
{
	return readl(mfg->shared + off);
}

/* ------------------------------------------------------------------------ */
/* Read-only status helpers                                                 */

static void mt6897_log_gpr(struct mt6897_mfg *mfg, const char *when)
{
	u32 v[GPR_NUM];
	unsigned int i;

	for (i = 0; i < GPR_NUM; i++)
		v[i] = readl(mfg->gpr + i * 4);

	dev_info(mfg->dev, "[%s] GPR0-7:   %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 when, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
	dev_info(mfg->dev, "[%s] GPR8-15:  %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 when, v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
	dev_info(mfg->dev, "[%s] GPR16-24: %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 when, v[16], v[17], v[18], v[19], v[20], v[21], v[22], v[23], v[24]);
}

struct mt6897_pwr {
	u32 spm, spm2, rpc, rpc2, mfg1_con, pdca;
	bool mfg0_on, mfg1_on;
};

static void mt6897_read_pwr(struct mt6897_mfg *mfg, struct mt6897_pwr *p)
{
	mfg_step(mfg, "rd SPM/RPC/PDCA status");
	p->spm = readl(mfg->spm + SPM_XPU_PWR_STATUS);
	p->spm2 = readl(mfg->spm + SPM_XPU_PWR_STATUS_2ND);
	p->rpc = readl(mfg->rpc + RPC_PWR_CON_STATUS);
	p->rpc2 = readl(mfg->rpc + RPC_PWR_CON_2ND_STATUS);
	p->mfg1_con = readl(mfg->rpc + RPC_MFG1_PWR_CON);
	p->pdca = readl(mfg->pdca);
	p->mfg0_on = (p->spm & SPM_MFG0_PWR_BIT) && (p->spm2 & SPM_MFG0_PWR_BIT);
	p->mfg1_on = (p->rpc & BIT(1)) && (p->rpc2 & BIT(1));
}

static void mt6897_log_pwr(struct mt6897_mfg *mfg, const char *when, struct mt6897_pwr *p)
{
	struct mt6897_pwr tmp;

	if (!p)
		p = &tmp;
	mt6897_read_pwr(mfg, p);
	mfg_vinfo(mfg->dev,
		 "[%s] SPM XPU_PWR_STATUS=%08x/%08x (MFG0 %s), MFG_RPC PWR_CON_STATUS=%08x/%08x (MFG1 %s, MFG1-14 mask %08lx), MFG1_PWR_CON=%08x, PDCA_STA=%08x\n",
		 when, p->spm, p->spm2, str_on_off(p->mfg0_on), p->rpc, p->rpc2,
		 str_on_off(p->mfg1_on), p->rpc & RPC_MFG1_14_MASK, p->mfg1_con, p->pdca);
}

static void mt6897_log_shared(struct mt6897_mfg *mfg, const char *when)
{
	dev_info(mfg->dev,
		 "[%s] shared: magic=%08x opp_idx=%d/%d opp_num=%d/%d signed=%d power_count=%d buck=%d mtcmos=%d cg=%d active=%d\n",
		 when, gf_read(mfg, GF_MAGIC), (int)gf_read(mfg, GF_CUR_OPPIDX_GPU),
		 (int)gf_read(mfg, GF_CUR_OPPIDX_STACK), (int)gf_read(mfg, GF_OPP_NUM_GPU),
		 (int)gf_read(mfg, GF_OPP_NUM_STACK), (int)gf_read(mfg, GF_SIGNED_OPP_NUM_GPU),
		 (int)gf_read(mfg, GF_POWER_COUNT), (int)gf_read(mfg, GF_BUCK_COUNT),
		 (int)gf_read(mfg, GF_MTCMOS_COUNT), (int)gf_read(mfg, GF_CG_COUNT),
		 (int)gf_read(mfg, GF_ACTIVE_COUNT));
	dev_info(mfg->dev,
		 "[%s] shared: fgpu=%u kHz fstack=%u kHz vgpu=%u vstack=%u vsram=%u (10uV) power=%u mW (max %u min %u) temp=%d\n",
		 when, gf_read(mfg, GF_CUR_FGPU), gf_read(mfg, GF_CUR_FSTACK),
		 gf_read(mfg, GF_CUR_VGPU), gf_read(mfg, GF_CUR_VSTACK),
		 gf_read(mfg, GF_CUR_VSRAM_GPU), gf_read(mfg, GF_CUR_POWER_GPU),
		 gf_read(mfg, GF_MAX_POWER_GPU), gf_read(mfg, GF_MIN_POWER_GPU),
		 (int)gf_read(mfg, GF_TEMPERATURE));
	dev_info(mfg->dev,
		 "[%s] shared: ceil=%d floor=%d c_limiter=%u f_limiter=%u power_control=%u active_sleep_control=%u dvfs_state=%#x shader_present=%#x\n",
		 when, (int)gf_read(mfg, GF_CUR_CEILING), (int)gf_read(mfg, GF_CUR_FLOOR),
		 gf_read(mfg, GF_CUR_C_LIMITER), gf_read(mfg, GF_CUR_F_LIMITER),
		 gf_read(mfg, GF_POWER_CONTROL), gf_read(mfg, GF_ACTIVE_SLEEP_CONTROL),
		 gf_read(mfg, GF_DVFS_STATE), gf_read(mfg, GF_SHADER_PRESENT));
	dev_info(mfg->dev,
		 "[%s] shared: avs=%u sb_ver=%#x ptp_ver=%#x dbg_ver=%#x gpm1=%u dual_buck=%u segment=%u test_mode=%u mfg_pwr_status=%#x power_time=%08x%08x\n",
		 when, gf_read(mfg, GF_AVS_ENABLE), gf_read(mfg, GF_SB_VERSION),
		 gf_read(mfg, GF_PTP_VERSION), gf_read(mfg, GF_DBG_VERSION),
		 gf_read(mfg, GF_GPM1_MODE), gf_read(mfg, GF_DUAL_BUCK),
		 gf_read(mfg, GF_SEGMENT_ID), gf_read(mfg, GF_TEST_MODE),
		 gf_read(mfg, GF_MFG_PWR_STATUS), gf_read(mfg, GF_POWER_TIME_H),
		 gf_read(mfg, GF_POWER_TIME_L));
}

static int mt6897_opp_num(struct mt6897_mfg *mfg)
{
	int n = gf_read(mfg, GF_OPP_NUM_GPU);

	return (n > 0 && n <= GF_MAX_OPP_NUM) ? n : 0;
}

static void mt6897_read_opp(struct mt6897_mfg *mfg, unsigned int i, u32 e[6])
{
	unsigned int j;

	for (j = 0; j < 6; j++)
		e[j] = gf_read(mfg, GF_WORKING_TABLE_GPU + i * GF_OPP_ENTRY_SIZE + j * 4);
}

static void mt6897_log_opps(struct mt6897_mfg *mfg)
{
	int i, n = mt6897_opp_num(mfg);
	u32 e[6];

	if (!n) {
		dev_info(mfg->dev, "[A1] opp_num_gpu out of range, not dumping the working table\n");
		return;
	}
	for (i = 0; i < n; i++) {
		mt6897_read_opp(mfg, i, e);
		dev_info(mfg->dev,
			 "[A1] OPP[%02d] %7u kHz  vgpu %6u  vsram %6u (10uV)  posdiv %u  margin %u  %5u mW\n",
			 i, e[0], e[1], e[2], e[3], e[4], e[5]);
	}
}

/* Fastest OPP index at or below @khz (indices run fastest -> slowest). */
static int mt6897_idx_for_khz(struct mt6897_mfg *mfg, unsigned int khz)
{
	int i, n = mt6897_opp_num(mfg);
	u32 e[6];

	for (i = 0; i < n; i++) {
		mt6897_read_opp(mfg, i, e);
		if (e[0] <= khz)
			return i;
	}
	return n - 1;
}

/* Fastest OPP index allowed by max_freq_khz. */
static int mt6897_ceil_idx(struct mt6897_mfg *mfg)
{
	return mt6897_idx_for_khz(mfg, max_freq_khz);
}

/* The EB's own view; it can turn active on its own at POWER_CONTROL(on). */
static int mt6897_active_count(struct mt6897_mfg *mfg)
{
	return (int)gf_read(mfg, GF_ACTIVE_COUNT);
}

static void mt6897_log_limits(struct mt6897_mfg *mfg, const char *when)
{
	static const int show[] = { LIMIT_SEGMENT, LIMIT_DEBUG, LIMIT_THERMAL_AP };
	unsigned int i;

	mfg_vinfo(mfg->dev, "[%s] EB limits: ceiling=%d (limiter %u, prio %u) floor=%d (limiter %u, prio %u)\n",
		  when, (int)gf_read(mfg, GF_CUR_CEILING), gf_read(mfg, GF_CUR_C_LIMITER),
		  gf_read(mfg, GF_CUR_C_PRIORITY), (int)gf_read(mfg, GF_CUR_FLOOR),
		  gf_read(mfg, GF_CUR_F_LIMITER), gf_read(mfg, GF_CUR_F_PRIORITY));
	for (i = 0; i < ARRAY_SIZE(show); i++) {
		unsigned int off = GF_LIMIT_TABLE + show[i] * GF_LIMIT_ENTRY_SIZE;
		char name[21];

		memcpy_fromio(name, mfg->shared + off + 4, 20);
		name[20] = '\0';
		mfg_vinfo(mfg->dev, "[%s]   limit_table[%d] id=%u name='%s' prio=%u ceiling=%d (en %u) floor=%d (en %u)\n",
			  when, show[i], gf_read(mfg, off), name,
			  gf_read(mfg, off + GF_LIMIT_PRIORITY),
			  (int)gf_read(mfg, off + GF_LIMIT_CEILING),
			  gf_read(mfg, off + GF_LIMIT_C_ENABLE),
			  (int)gf_read(mfg, off + GF_LIMIT_FLOOR),
			  gf_read(mfg, off + GF_LIMIT_F_ENABLE));
	}
}

static unsigned int mt6897_pll_khz(u32 con1)
{
	u64 pcw = FIELD_GET(PLL_CON1_PCW, con1);

	return (unsigned int)(((pcw * 26000) >> 14) >> FIELD_GET(PLL_CON1_POSDIV, con1));
}

/*
 * One line with what the EB says (shared memory) and, when MFG1 is powered,
 * what the PLL and mux registers say. All reads, no side effects (the
 * vendor's frequency meter needs FQMTR_CON writes and is not used).
 */
/*
 * MFG PLL / mux registers are only read while this driver has the GPU powered,
 * no power-off is in progress, and MFG1 reports on.
 */
static bool mt6897_mfg_readable(struct mt6897_mfg *mfg, const struct mt6897_pwr *p)
{
	return mfg->eb_powered && !mfg->going_down && p->mfg1_on &&
	       mfg->mfgpll_con1 && mfg->mfgscpll_con1 && mfg->topck_cfg30;
}

static void mt6897_log_freq(struct mt6897_mfg *mfg, const char *when)
{
	struct mt6897_pwr p;
	u32 c1 = 0, c2 = 0, sel = 0;
	bool hw;

	mt6897_read_pwr(mfg, &p);
	hw = mt6897_mfg_readable(mfg, &p);
	if (hw) {
		mfg_step(mfg, "rd MFG PLL/TOPCK (%s)", when);
		c1 = readl(mfg->mfgpll_con1);
		c2 = readl(mfg->mfgscpll_con1);
		sel = readl(mfg->topck_cfg30);
	}
	mfg_vinfo(mfg->dev,
		 "[%s] EB: opp_idx=%d fgpu=%u kHz fstack=%u kHz vgpu=%u vsram=%u (10uV) power=%u mW active_count=%d dvfs_state=%#x%s%s%s ceil/floor=%d/%d\n",
		 when, (int)gf_read(mfg, GF_CUR_OPPIDX_GPU), gf_read(mfg, GF_CUR_FGPU),
		 gf_read(mfg, GF_CUR_FSTACK), gf_read(mfg, GF_CUR_VGPU),
		 gf_read(mfg, GF_CUR_VSRAM_GPU), gf_read(mfg, GF_CUR_POWER_GPU),
		 (int)gf_read(mfg, GF_ACTIVE_COUNT), gf_read(mfg, GF_DVFS_STATE),
		 (gf_read(mfg, GF_DVFS_STATE) & DVFS_SLEEP) ? " SLEEP" : "",
		 (gf_read(mfg, GF_DVFS_STATE) & DVFS_POWEROFF) ? " POWEROFF" : "",
		 (gf_read(mfg, GF_DVFS_STATE) & DVFS_FIX_OPP) ? " FIX_OPP" : "",
		 (int)gf_read(mfg, GF_CUR_CEILING), (int)gf_read(mfg, GF_CUR_FLOOR));
	if (hw)
		mfg_vinfo(mfg->dev,
			 "[%s] HW: MFG_PLL_CON1=%08x (%u kHz) MFGSC_PLL_CON1=%08x (%u kHz) TOPCK_CLK_CFG_30=%08x (MFG mux %s, MFGSC mux %s)\n",
			 when, c1, mt6897_pll_khz(c1), c2, mt6897_pll_khz(c2), sel,
			 (sel & TOPCK_MFG_SEL_PLL) ? "PLL" : "26M", (sel & TOPCK_MFGSC_SEL_PLL) ? "PLL" : "26M");
	else
		mfg_vinfo(mfg->dev, "[%s] HW: PLL/mux registers not read (MFG1 off or not described)\n", when);
}

/* ------------------------------------------------------------------------ */
/* Mailbox / IPI                                                            */

static void mt6897_mbox_rx(struct mbox_client *cl, void *msg)
{
	struct mt6897_mbox *mb = container_of(cl, struct mt6897_mbox, cl);
	struct mt6897_mfg *mfg = mb->mfg;
	u32 *w = msg;

	if (mb->idx == MBOX_GPUFREQ) {
		memcpy(mfg->rx_buf, msg, MBOX_MAX_RX_SIZE);
		complete(&mfg->rx_done);
		return;
	}

	/* Unsolicited traffic on a channel nobody here uses: log and drop. */
	dev_info_ratelimited(mfg->dev, "unexpected EB message on '%s': %08x %08x %08x %08x\n",
			     mt6897_mbox_names[mb->idx], w[0], w[1], w[2], w[3]);
}

/*
 * Every EB->AP channel is requested, not just "gpufreq". The receive IRQ is
 * level-triggered and only the handler of the channel whose status bit is set
 * acknowledges it; a message on an unrequested channel would leave the line
 * asserted and get the shared IRQ disabled as spurious.
 */
static int mt6897_mbox_request(struct mt6897_mfg *mfg)
{
	unsigned int i;

	if (mfg->mbox_ready)
		return 0;

	for (i = 0; i < ARRAY_SIZE(mt6897_mbox_names); i++) {
		struct mt6897_mbox *mb = &mfg->mbox[i];

		mb->mfg = mfg;
		mb->idx = i;
		mb->cl.dev = mfg->dev;
		mb->cl.rx_callback = mt6897_mbox_rx;
		mb->cl.tx_block = false;
		mb->cl.knows_txdone = false;
		/* startup writes BIT(i) to MBOX_CTL_IRQ_CLR (0x13c62074) */
		mb->ch = mbox_request_channel_byname(&mb->cl, mt6897_mbox_names[i]);
		if (IS_ERR(mb->ch)) {
			int ret = PTR_ERR(mb->ch);

			mb->ch = NULL;
			dev_err(mfg->dev, "cannot get mailbox channel '%s': %pe\n",
				mt6897_mbox_names[i], ERR_PTR(ret));
			while (i--) {
				mbox_free_channel(mfg->mbox[i].ch);
				mfg->mbox[i].ch = NULL;
			}
			return ret;
		}
	}

	dev_info(mfg->dev, "requested %zu mailbox channels (wrote IRQ_CLR bits 0-%zu)\n",
		 ARRAY_SIZE(mt6897_mbox_names), ARRAY_SIZE(mt6897_mbox_names) - 1);
	mfg->mbox_ready = true;
	return 0;
}

/*
 * Send one gpufreq IPI and wait for the EB's reply on the same channel.
 * Returns 0 when a reply arrived (its return_value is stored in @retval) or a
 * negative errno when the mailbox transport failed or the EB did not answer.
 */
static int mt6897_send_ipi(struct mt6897_mfg *mfg, struct mt6897_gf_ipi_msg *msg,
			   const char *what, int *retval)
{
	u32 *w = (u32 *)&mfg->tx_buf;
	u32 *r = (u32 *)mfg->rx_buf;
	unsigned long left;
	int ret;

	lockdep_assert_held(&mfg->lock);

	if (mfg->ipi_broken) {
		dev_err(mfg->dev, "IPI %s: not sent, an earlier IPI was never answered\n", what);
		return -EIO;
	}

	msg->magic = cpu_to_le32(mfg->ipi_magic);
	mfg->tx_buf = *msg;
	reinit_completion(&mfg->rx_done);

	mfg_vinfo(mfg->dev, "IPI %s -> EB: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 what, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);

	/* data words -> 0x13c2fd90..0x13c2fdac, then BIT(1) -> IRQ_SET 0x13c62004 */
	ret = mbox_send_message(mfg->mbox[MBOX_GPUFREQ].ch, &mfg->tx_buf);
	if (ret < 0) {
		dev_err(mfg->dev, "IPI %s: mailbox send failed: %pe\n", what, ERR_PTR(ret));
		mfg->ipi_broken = true;
		return ret;
	}

	left = wait_for_completion_timeout(&mfg->rx_done, msecs_to_jiffies(IPI_TIMEOUT_MS));
	if (!left) {
		dev_err(mfg->dev, "IPI %s: no reply from EB within %d ms; no further IPIs this boot\n",
			what, IPI_TIMEOUT_MS);
		mfg->ipi_broken = true;
		return -ETIMEDOUT;
	}

	mfg_vinfo(mfg->dev, "IPI %s <- EB: %08x %08x %08x %08x %08x %08x %08x %08x (return %d)\n",
		 what, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], (int)r[4]);
	*retval = (int)r[4];

	return 0;
}

/* ------------------------------------------------------------------------ */
/* Stage A1: shared memory init                                             */

static int mt6897_set_limit_locked(struct mt6897_mfg *mfg, int ceil, const char *who);

static int mt6897_init_locked(struct mt6897_mfg *mfg)
{
	struct mt6897_gf_ipi_msg msg = {};
	int ret, rv;

	lockdep_assert_held(&mfg->lock);

	if (mfg->initialized)
		return 0;
	if (mfg->init_failed)
		return -EIO;

	dev_info(mfg->dev, "[A1] start: CMD_INIT_SHARED_MEM sequence\n");
	mt6897_log_gpr(mfg, "A1 before");
	mt6897_log_pwr(mfg, "A1 before", NULL);

	ret = mt6897_mbox_request(mfg);
	if (ret)
		return ret;

	mfg->ipi_magic = readl(mfg->gpr + GPR_IPI_MAGIC);
	dev_info(mfg->dev, "[A1] read IPI magic GPR13 (0x13c2fd50) = %08x\n", mfg->ipi_magic);
	if (!mfg->ipi_magic) {
		dev_err(mfg->dev,
			"[A1] IPI magic is 0: already consumed this boot (by a previous client) or EB not up; not sending anything\n");
		mfg->init_failed = true;
		return -ENODEV;
	}
	if (clear_ipi_magic) {
		dev_info(mfg->dev, "[A1] write GPR13 (0x13c2fd50) <- 0 (vendor clears the magic after reading it)\n");
		writel(0, mfg->gpr + GPR_IPI_MAGIC);
		dev_info(mfg->dev, "[A1] GPR13 now %08x\n", readl(mfg->gpr + GPR_IPI_MAGIC));
	}

	dev_info(mfg->dev, "[A1] zeroing shared memory %pa + %#x\n",
		 &mfg->shared_phys, GPUFREQ_MEM_SIZE);
	memset_io(mfg->shared, 0, GPUFREQ_MEM_SIZE);

	msg.cmd = cpu_to_le32(CMD_INIT_SHARED_MEM);
	msg.u.shared_mem.base_lo = cpu_to_le32(lower_32_bits(mfg->shared_phys));
	msg.u.shared_mem.base_hi = cpu_to_le32(upper_32_bits(mfg->shared_phys));
	msg.u.shared_mem.size = cpu_to_le32(GPUFREQ_MEM_SIZE);

	ret = mt6897_send_ipi(mfg, &msg, "INIT_SHARED_MEM", &rv);
	if (ret) {
		mt6897_log_gpr(mfg, "A1 after failure");
		mt6897_log_pwr(mfg, "A1 after failure", NULL);
		mfg->init_failed = true;
		return ret;
	}
	/* The vendor ignores the reply value of this command; only log it. */
	dev_info(mfg->dev, "[A1] INIT_SHARED_MEM reply value %d\n", rv);

	mt6897_log_shared(mfg, "A1");
	mt6897_log_opps(mfg);
	mt6897_log_gpr(mfg, "A1 after");
	mt6897_log_pwr(mfg, "A1 after", NULL);

	if (gf_read(mfg, GF_MAGIC) != GPUFREQ_MAGIC) {
		dev_err(mfg->dev, "[A1] shared memory magic is %08x, expected %08x: layout or init wrong\n",
			gf_read(mfg, GF_MAGIC), GPUFREQ_MAGIC);
		mfg->init_failed = true;
		return -EIO;
	}
	if (!mt6897_opp_num(mfg))
		dev_warn(mfg->dev, "[A1] opp_num_gpu %d is out of range\n",
			 (int)gf_read(mfg, GF_OPP_NUM_GPU));

	mfg->initialized = true;
	mfg->target_idx = mt6897_opp_num(mfg) - 1;

	/* the cap goes into the EB before the GPU is ever powered */
	mt6897_log_limits(mfg, "A1");
	if (mt6897_set_limit_locked(mfg, mt6897_ceil_idx(mfg), "A1"))
		dev_err(mfg->dev, "[A1] !!! EB-side ceiling NOT set; only the driver clamp protects the GPU\n");
	dev_info(mfg->dev, "[A1] done: EB accepted the shared memory; max_freq_khz=%u -> fastest allowed OPP %d\n",
		 max_freq_khz, mt6897_ceil_idx(mfg));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Stage B: power control                                                   */

static void mt6897_read_mali_ids(struct mt6897_mfg *mfg)
{
	void __iomem *gpu;

	if (!mfg->gpu_res.start) {
		dev_info(mfg->dev, "[B] no 'gpu' reg given, skipping Mali ID read\n");
		return;
	}

	gpu = ioremap(mfg->gpu_res.start, SZ_4K);
	if (!gpu) {
		dev_err(mfg->dev, "[B] cannot map Mali registers\n");
		return;
	}

	dev_info(mfg->dev, "[B] reading Mali ID registers at %pa\n", &mfg->gpu_res.start);
	dev_info(mfg->dev,
		 "[B] GPU_ID=%08x L2_FEATURES=%08x CORE_FEATURES=%08x TILER_FEATURES=%08x MEM_FEATURES=%08x MMU_FEATURES=%08x\n",
		 readl(gpu + MALI_GPU_ID), readl(gpu + MALI_L2_FEATURES),
		 readl(gpu + MALI_CORE_FEATURES), readl(gpu + MALI_TILER_FEATURES),
		 readl(gpu + MALI_MEM_FEATURES), readl(gpu + MALI_MMU_FEATURES));
	dev_info(mfg->dev,
		 "[B] AS_PRESENT=%08x CSF_ID=%08x GPU_STATUS=%08x SHADER_PRESENT=%08x%08x TILER_PRESENT=%08x L2_PRESENT=%08x COHERENCY_FEATURES=%08x\n",
		 readl(gpu + MALI_AS_PRESENT), readl(gpu + MALI_CSF_ID),
		 readl(gpu + MALI_GPU_STATUS), readl(gpu + MALI_SHADER_PRESENT_HI),
		 readl(gpu + MALI_SHADER_PRESENT_LO), readl(gpu + MALI_TILER_PRESENT_LO),
		 readl(gpu + MALI_L2_PRESENT_LO), readl(gpu + MALI_COHERENCY_FEATURES));
	iounmap(gpu);
}

/*
 * Mirrors gpufreq_power_control() in EB mode: only sent when the EB reports
 * power control as enabled; the reply carries the EB's power_count.
 */
static int mt6897_power_control_locked(struct mt6897_mfg *mfg, bool on, const char *who)
{
	struct mt6897_gf_ipi_msg msg = {};
	struct mt6897_pwr p;
	int ret, rv;

	lockdep_assert_held(&mfg->lock);

	if (!gf_read(mfg, GF_POWER_CONTROL)) {
		dev_info(mfg->dev, "[%s] EB reports power_control=0: vendor sends no POWER_CONTROL (GPU kept on by EB)\n",
			 who);
		mfg->eb_powered = on;
		if (!on)
			return 0;
		mt6897_read_pwr(mfg, &p);
		return (p.mfg0_on && p.mfg1_on) ? 0 : -EIO;
	}

	msg.cmd = cpu_to_le32(CMD_POWER_CONTROL);
	msg.u.value = cpu_to_le32(on ? GPU_PWR_ON : GPU_PWR_OFF);
	ret = mt6897_send_ipi(mfg, &msg, on ? "POWER_CONTROL(on)" : "POWER_CONTROL(off)", &rv);
	if (ret)
		return ret;
	if (rv < 0) {
		dev_err(mfg->dev, "[%s] EB refused POWER_CONTROL(%s): %d\n", who, str_on_off(on), rv);
		return -EIO;
	}
	mfg->eb_powered = on;
	mfg_vinfo(mfg->dev, "[%s] POWER_CONTROL(%s) ok, EB power_count=%d (shared %d)\n",
		 who, str_on_off(on), rv, (int)gf_read(mfg, GF_POWER_COUNT));
	return 0;
}

static int mt6897_fix_opp_locked(struct mt6897_mfg *mfg, int idx)
{
	struct mt6897_gf_ipi_msg msg = {};
	int ret, rv = 0;

	lockdep_assert_held(&mfg->lock);

	msg.cmd = cpu_to_le32(CMD_FIX_TARGET_OPPIDX);
	msg.target = cpu_to_le32(TARGET_GPU);
	msg.u.value = cpu_to_le32((u32)idx);
	ret = mt6897_send_ipi(mfg, &msg, "FIX_TARGET_OPPIDX", &rv);
	dev_info(mfg->dev, "FIX_TARGET_OPPIDX(GPU, %d) -> transport %d, EB %d; now opp_idx=%d fgpu=%u kHz vgpu=%u\n",
		 idx, ret, rv, (int)gf_read(mfg, GF_CUR_OPPIDX_GPU),
		 gf_read(mfg, GF_CUR_FGPU), gf_read(mfg, GF_CUR_VGPU));
	if (ret)
		return ret;
	return rv < 0 ? -EIO : 0;
}

/*
 * CMD_ACTIVE_SLEEP_CONTROL, as gpufreq_active_sleep_control() in EB mode:
 * only sent when the EB reports active_sleep_control. On this firmware the EB
 * powers the GPU on with both muxes on the 26 MHz reference and DVFS_SLEEP
 * set; "active" moves them to the PLLs and unfreezes DVFS. The vendor rule is
 * active only while powered, idle before power-off.
 */
static int mt6897_active_locked(struct mt6897_mfg *mfg, bool active, const char *who)
{
	struct mt6897_gf_ipi_msg msg = {};
	int ret, rv, before;

	lockdep_assert_held(&mfg->lock);

	if (!gf_read(mfg, GF_ACTIVE_SLEEP_CONTROL)) {
		dev_info(mfg->dev, "[%s] EB reports active_sleep_control=0: not sending ACTIVE_SLEEP_CONTROL\n", who);
		return 0;
	}
	if (active && !mfg->eb_powered) {
		dev_err(mfg->dev, "[%s] refusing ACTIVE_SLEEP_CONTROL(active) while not powered\n", who);
		return -EPERM;
	}

	before = mt6897_active_count(mfg);
	if (active ? mfg->our_active > 0 : mfg->our_active <= 0) {
		mfg_vinfo(mfg->dev, "[%s] %s: we hold %d ACTIVE reference(s), EB active_count=%d; nothing sent\n",
			  who, active ? "active" : "idle", mfg->our_active, before);
		return 0;
	}

	msg.cmd = cpu_to_le32(CMD_ACTIVE_SLEEP_CONTROL);
	msg.u.value = cpu_to_le32(active ? GPU_PWR_ON : GPU_PWR_OFF);
	ret = mt6897_send_ipi(mfg, &msg, active ? "ACTIVE_SLEEP_CONTROL(active)" :
			      "ACTIVE_SLEEP_CONTROL(idle)", &rv);
	if (ret)
		return ret;
	if (rv < 0) {
		dev_err(mfg->dev, "[%s] EB refused ACTIVE_SLEEP_CONTROL(%s): %d\n",
			who, active ? "active" : "idle", rv);
		return -EIO;
	}
	mfg->our_active += active ? 1 : -1;
	mfg_vinfo(mfg->dev, "[%s] ACTIVE_SLEEP_CONTROL(%s) ok, EB active_count %d -> %d (shared %d), ours now %d\n",
		  who, active ? "active" : "idle", before, rv, mt6897_active_count(mfg),
		  mfg->our_active);
	mt6897_log_freq(mfg, who);
	return 0;
}

static bool mt6897_is_active(struct mt6897_mfg *mfg)
{
	return mt6897_active_count(mfg) > 0 || !gf_read(mfg, GF_ACTIVE_SLEEP_CONTROL);
}

/*
 * CMD_SET_LIMIT, as gpufreq_set_limit() sends it in EB mode
 * (gpufreq_v2.c:940-968): target, then u.set_limit = {limiter, ceiling_info,
 * floor_info}. For LIMIT_DEBUG both infos are OPP indices
 * (__gpuppm_convert_limit_to_idx); -1 (GPUPPM_DEFAULT_IDX) drops the limit,
 * -3 (GPUPPM_KEEP_IDX) leaves that side alone. The most restrictive ceiling
 * of all enabled limiters wins (__gpuppm_sort_limit). The EB moves a running
 * GPU down at once (instant DVFS); a powered-off or sleeping GPU is not moved.
 */
static int mt6897_set_limit_locked(struct mt6897_mfg *mfg, int ceil, const char *who)
{
	struct mt6897_gf_ipi_msg msg = {};
	unsigned int off = GF_LIMIT_TABLE + LIMIT_DEBUG * GF_LIMIT_ENTRY_SIZE;
	int ret, rv;

	lockdep_assert_held(&mfg->lock);

	msg.cmd = cpu_to_le32(CMD_SET_LIMIT);
	msg.target = cpu_to_le32(TARGET_GPU);
	msg.u.raw[0] = cpu_to_le32(LIMIT_DEBUG);
	msg.u.raw[1] = cpu_to_le32((u32)(ceil >= 0 ? ceil : GPUPPM_DEFAULT_IDX));
	msg.u.raw[2] = cpu_to_le32((u32)GPUPPM_KEEP_IDX);
	ret = mt6897_send_ipi(mfg, &msg, "SET_LIMIT", &rv);
	if (ret)
		return ret;
	if (rv < 0) {
		dev_err(mfg->dev, "[%s] EB refused SET_LIMIT(DEBUG, ceiling %d): %d\n", who, ceil, rv);
		return -EIO;
	}
	mfg->eb_limit = ceil;
	mt6897_log_limits(mfg, who);

	/* the effective ceiling decides; the per-limiter entry is informational */
	if (ceil >= 0 && (int)gf_read(mfg, GF_CUR_CEILING) < ceil) {
		dev_err(mfg->dev, "[%s] EB acked SET_LIMIT but shows ceiling %d (DEBUG entry %d), wanted %d\n",
			who, (int)gf_read(mfg, GF_CUR_CEILING),
			(int)gf_read(mfg, off + GF_LIMIT_CEILING), ceil);
		return -EIO;
	}
	if (ceil >= 0 && (int)gf_read(mfg, off + GF_LIMIT_CEILING) != ceil)
		dev_warn(mfg->dev, "[%s] effective ceiling ok but limit_table[DEBUG].ceiling reads %d\n",
			 who, (int)gf_read(mfg, off + GF_LIMIT_CEILING));
	dev_info(mfg->dev, "[%s] EB ceiling now OPP %d (%u kHz), limiter %u\n", who,
		 (int)gf_read(mfg, GF_CUR_CEILING),
		 gf_read(mfg, GF_WORKING_TABLE_GPU +
			 max((int)gf_read(mfg, GF_CUR_CEILING), 0) * GF_OPP_ENTRY_SIZE),
		 gf_read(mfg, GF_CUR_C_LIMITER));
	return 0;
}

/* Re-send our ceiling if the EB no longer shows it (e.g. lost across power). */
static void mt6897_ensure_limit_locked(struct mt6897_mfg *mfg, const char *who)
{
	int want = mfg->eb_limit;

	if (want < 0)
		return;
	if ((int)gf_read(mfg, GF_CUR_CEILING) >= want)
		return;
	dev_warn(mfg->dev, "[%s] EB ceiling is %d, ours is %d: re-sending SET_LIMIT\n",
		 who, (int)gf_read(mfg, GF_CUR_CEILING), want);
	mt6897_set_limit_locked(mfg, want, who);
}

/*
 * Read-only check that the GPU is not clocked above max_freq_khz. If it is,
 * shout and pull it down with a commit (only possible while active).
 */
static void mt6897_commit_locked_quiet(struct mt6897_mfg *mfg, int idx, const char *who);

static bool mt6897_check_cap_locked(struct mt6897_mfg *mfg, const char *who)
{
	struct mt6897_pwr p;
	u32 c1, sel;
	unsigned int khz;

	mt6897_read_pwr(mfg, &p);
	if (!mt6897_mfg_readable(mfg, &p))
		return true;
	mfg_step(mfg, "rd MFG PLL/TOPCK (cap check, %s)", who);
	c1 = readl(mfg->mfgpll_con1);
	sel = readl(mfg->topck_cfg30);
	khz = mt6897_pll_khz(c1);
	if (!(sel & TOPCK_MFG_SEL_PLL) || khz <= max_freq_khz)
		return true;

	dev_err(mfg->dev, "[%s] !!! GPU clocked at %u kHz (MFG_PLL_CON1 %08x, mux PLL) above max_freq_khz=%u; EB opp_idx=%d ceiling=%d active_count=%d: committing the ceiling now\n",
		who, khz, c1, max_freq_khz, (int)gf_read(mfg, GF_CUR_OPPIDX_GPU),
		(int)gf_read(mfg, GF_CUR_CEILING), mt6897_active_count(mfg));
	mt6897_commit_locked_quiet(mfg, mt6897_ceil_idx(mfg), who);
	mt6897_log_freq(mfg, who);
	return false;
}


/*
 * CMD_COMMIT, as gpufreq_commit(TARGET_DEFAULT, idx) in EB mode: the wrapper's
 * dual-buck is 0 (gpufreq-wrapper DT), so TARGET_DEFAULT becomes TARGET_GPU.
 * The EB scales both MFGPLL and MFGSCPLL and applies VGPU/VSRAM itself.
 */
static int mt6897_commit_locked(struct mt6897_mfg *mfg, int idx, const char *who)
{
	struct mt6897_gf_ipi_msg msg = {};
	int ret, rv, ceil, n = mt6897_opp_num(mfg);

	lockdep_assert_held(&mfg->lock);

	if (!n)
		return -EINVAL;
	ceil = mt6897_ceil_idx(mfg);
	if (idx < ceil) {
		dev_info(mfg->dev, "[%s] OPP %d above max_freq_khz=%u, clamped to %d\n",
			 who, idx, max_freq_khz, ceil);
		idx = ceil;
	}
	if (idx >= n)
		idx = n - 1;
	if (!mfg->eb_powered) {
		dev_err(mfg->dev, "[%s] refusing CMD_COMMIT while not powered\n", who);
		return -EPERM;
	}

	msg.cmd = cpu_to_le32(CMD_COMMIT);
	msg.target = cpu_to_le32(TARGET_GPU);
	msg.u.value = cpu_to_le32((u32)idx);
	ret = mt6897_send_ipi(mfg, &msg, "COMMIT", &rv);
	if (ret)
		return ret;
	if (rv < 0) {
		dev_err(mfg->dev, "[%s] EB refused COMMIT(GPU, %d): %d\n", who, idx, rv);
		return -EIO;
	}
	mfg->last_commit = idx;
	if ((int)gf_read(mfg, GF_CUR_OPPIDX_GPU) != idx)
		dev_warn(mfg->dev, "[%s] COMMIT(GPU, %d) acked but EB opp_idx is %d (dvfs_state %#x, active_count %d)\n",
			 who, idx, (int)gf_read(mfg, GF_CUR_OPPIDX_GPU), gf_read(mfg, GF_DVFS_STATE),
			 mt6897_active_count(mfg));
	return 0;
}

/* Effective index a commit of @idx ends at, after our clamp. */
static int mt6897_clamped(struct mt6897_mfg *mfg, int idx)
{
	return min(max(idx, mt6897_ceil_idx(mfg)), mt6897_opp_num(mfg) - 1);
}

/*
 * Commit and make sure the EB really sits at that index afterwards: re-read
 * opp_idx, re-send up to twice. Only meaningful while active (the EB ignores
 * commits while DVFS is frozen).
 */
static int mt6897_commit_verified_locked(struct mt6897_mfg *mfg, int idx, const char *who)
{
	int want = mt6897_clamped(mfg, idx), tries, ret = 0;

	for (tries = 0; tries < 3; tries++) {
		if ((int)gf_read(mfg, GF_CUR_OPPIDX_GPU) == want && tries)
			break;
		ret = mt6897_commit_locked(mfg, idx, who);
		if (ret)
			return ret;
		if ((int)gf_read(mfg, GF_CUR_OPPIDX_GPU) == want)
			break;
		usleep_range(500, 1000);
	}
	if ((int)gf_read(mfg, GF_CUR_OPPIDX_GPU) != want) {
		dev_err(mfg->dev, "[%s] EB opp_idx %d after %d COMMIT(%d) (active_count %d, dvfs_state %#x)\n",
			who, (int)gf_read(mfg, GF_CUR_OPPIDX_GPU), tries, want,
			mt6897_active_count(mfg), gf_read(mfg, GF_DVFS_STATE));
		return -EIO;
	}
	return 0;
}

static void mt6897_verify_work(struct work_struct *work)
{
	struct mt6897_mfg *mfg = container_of(to_delayed_work(work), struct mt6897_mfg,
					      verify_work);
	int cnt, want, cur;

	mfg_step(mfg, "verify: lock");
	mutex_lock(&mfg->lock);
	if (!mfg->eb_powered || mfg->going_down)
		goto out;

	cnt = mt6897_active_count(mfg);
	if (cnt != mfg->seen_active) {
		mfg_vinfo(mfg->dev, "[verify] EB active_count %d -> %d (ours %d)\n",
			  mfg->seen_active, cnt, mfg->our_active);
		mfg->seen_active = cnt;
	}
	mt6897_check_cap_locked(mfg, "verify");

	want = mt6897_clamped(mfg, mfg->target_idx);
	cur = (int)gf_read(mfg, GF_CUR_OPPIDX_GPU);
	if (follow_devfreq && fix_oppidx < 0 && mt6897_is_active(mfg) && cur != want) {
		dev_info(mfg->dev, "[verify] EB at OPP %d, target %d: committing\n", cur, want);
		mt6897_commit_verified_locked(mfg, mfg->target_idx, "verify");
		mt6897_log_freq(mfg, "verify");
	}
	if (mfg->verify_left) {
		mfg->verify_left--;
		schedule_delayed_work(&mfg->verify_work, msecs_to_jiffies(200));
	}
out:
	mutex_unlock(&mfg->lock);
}

static void mt6897_commit_locked_quiet(struct mt6897_mfg *mfg, int idx, const char *who)
{
	if (mt6897_commit_locked(mfg, idx, who))
		dev_warn(mfg->dev, "[%s] COMMIT(%d) failed\n", who, idx);
}

/*
 * Bring the GPU up so that it never runs above the ceiling for longer than
 * one IPI round trip, as far as the AP can influence it:
 *
 *  1. The LIMIT_DEBUG ceiling has been in the EB since INIT (and is re-sent
 *     here if the EB lost it), so every EB-side DVFS decision is clamped.
 *  2. POWER_CONTROL(on). The EB may come back already active, at whatever OPP
 *     it last had (seen: OPP 0 after an idle); commit at once in that case.
 *  3. Otherwise commit before ACTIVE (vendor reference skips commits while
 *     DVFS_SLEEP is set; if this firmware records it instead, ACTIVE comes up
 *     at the right OPP), then ACTIVE, then commit again.
 *  4. Verify on the PLL and mux registers; if above max_freq_khz, log loudly
 *     and commit the ceiling.
 *
 * @want_active false skips our ACTIVE reference. The EB may still turn
 * active by itself once the GPU runs; that reference is the EB's and is
 * never released from here.
 */
static int mt6897_eb_up_locked(struct mt6897_mfg *mfg, bool want_active, int idx,
			       const char *who)
{
	struct mt6897_pwr p;
	int ret;

	lockdep_assert_held(&mfg->lock);

	mt6897_ensure_limit_locked(mfg, who);

	if (!mfg->eb_powered) {
		ret = mt6897_power_control_locked(mfg, true, who);
		if (ret)
			return ret;
	}
	mfg->going_down = false;

	mt6897_log_pwr(mfg, who, &p);
	if (!p.mfg0_on || !p.mfg1_on) {
		dev_err(mfg->dev, "[%s] EB acked power-on but MFG0/MFG1 are off\n", who);
		return -EIO;
	}
	mt6897_log_freq(mfg, who);
	mt6897_ensure_limit_locked(mfg, who);

	if (mt6897_active_count(mfg) > 0) {
		mfg_vinfo(mfg->dev, "[%s] EB came up active (active_count=%d) at OPP %d: committing %d now\n",
			  who, mt6897_active_count(mfg), (int)gf_read(mfg, GF_CUR_OPPIDX_GPU), idx);
		mt6897_commit_locked_quiet(mfg, idx, who);
	} else if (want_active) {
		mt6897_commit_locked_quiet(mfg, idx, who);	/* pre-active, see 3. */
	}

	if (want_active) {
		ret = mt6897_active_locked(mfg, true, who);
		if (ret)
			return ret;
	} else {
		mfg_vinfo(mfg->dev, "[%s] not sending ACTIVE (auto_active=0), EB active_count=%d\n",
			  who, mt6897_active_count(mfg));
	}

	if (want_active && mt6897_commit_verified_locked(mfg, idx, who))
		dev_warn(mfg->dev, "[%s] OPP not confirmed yet, the verify work retries\n", who);
	mt6897_check_cap_locked(mfg, who);
	mt6897_log_freq(mfg, who);

	/* re-check at ~20 ms, then every 200 ms, 5 times: covers self-activation */
	mfg->seen_active = mt6897_active_count(mfg);
	mfg->verify_left = 5;
	mod_delayed_work(system_wq, &mfg->verify_work, msecs_to_jiffies(20));
	return 0;
}

/* Idle (whatever the EB's count says), PDCA ack, POWER_CONTROL(off). */
static int mt6897_eb_down_locked(struct mt6897_mfg *mfg, const char *who)
{
	u32 val;
	int ret;

	lockdep_assert_held(&mfg->lock);

	/* from here on, no MFG PLL/mux reads (status, cap check, log_freq) */
	mfg->going_down = true;
	mfg_step(mfg, "%s: power-off sequence starts, EB active_count=%d", who,
		 mt6897_active_count(mfg));

	ret = mt6897_active_locked(mfg, false, who);
	if (ret)
		goto fail;

	/*
	 * The EB drops its own reference when it sees the GPU idle (the
	 * consumer has suspended by now). Never power off under it.
	 */
	if (gf_read(mfg, GF_ACTIVE_SLEEP_CONTROL)) {
		int cnt, waited;

		mfg_step(mfg, "%s: wait for EB active_count 0 (now %d)", who, mt6897_active_count(mfg));
		for (waited = 0; waited < 50 && (cnt = mt6897_active_count(mfg)) > 0; waited++)
			usleep_range(1000, 1500);
		cnt = mt6897_active_count(mfg);
		if (cnt > 0) {
			dev_err(mfg->dev, "[%s] EB still counts %d ACTIVE reference(s) (not ours) after ~50 ms: staying powered\n",
				who, cnt);
			ret = -EBUSY;
			goto fail;
		}
		if (waited)
			mfg_vinfo(mfg->dev, "[%s] EB dropped its own ACTIVE reference after ~%d ms\n",
				  who, waited);
	}

	/* Vendor kbase: gpufreq_pdca_polling_ack(GPU_PWR_OFF) before power off */
	mfg_step(mfg, "%s: poll PDCA_STA 0x13c620a8 bit16", who);
	if (readl_poll_timeout(mfg->pdca, val, !(val & PDCA_STA_BUSY), 10, 10000))
		dev_warn(mfg->dev, "[%s] PDCA_STA still busy (%08x) before power off\n", who, val);

	ret = mt6897_power_control_locked(mfg, false, who);
	if (ret)
		goto fail;
	mfg->our_active = 0;
	mt6897_log_pwr(mfg, who, NULL);
	return 0;
fail:
	/* still powered: readback allowed again */
	mfg->going_down = !mfg->eb_powered;
	return ret;
}

/* ------------------------------------------------------------------------ */
/* Stage C: genpd + clock                                                   */

static inline struct mt6897_mfg *pd_to_mfg(struct generic_pm_domain *pd)
{
	return container_of(pd, struct mt6897_mfg, pd);
}

/*
 * genpd performance levels grow with performance (genpd aggregates with max
 * and drops a suspended device to 0), EB OPP indices shrink with it. Level 0
 * is therefore the slowest OPP: level = opp_num - 1 - index.
 */
static int mt6897_level_to_idx(struct mt6897_mfg *mfg, unsigned int level)
{
	int n = mt6897_opp_num(mfg);

	return level >= n ? 0 : n - 1 - (int)level;
}

static int mt6897_pd_power_on(struct generic_pm_domain *pd)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	int ret;

	mfg_step(mfg, "genpd power_on: lock");
	mutex_lock(&mfg->lock);
	mfg_step(mfg, "genpd power_on: locked");
	ret = mt6897_init_locked(mfg);
	if (ret)
		goto out;

	if (fix_oppidx >= 0) {
		/* legacy fixed mode: power only, the EB holds the fixed OPP */
		ret = mt6897_eb_up_locked(mfg, true, fix_oppidx, "C on");
		if (!ret && !mfg->fix_applied) {
			mfg->fix_applied = true;
			mt6897_fix_opp_locked(mfg, fix_oppidx);
		}
		goto out;
	}

	/* the level genpd restored just before powering on, or the last one */
	ret = mt6897_eb_up_locked(mfg, auto_active, mfg->target_idx, "C on");
out:
	mutex_unlock(&mfg->lock);
	return ret;
}

static int mt6897_pd_power_off(struct generic_pm_domain *pd)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	int ret = 0;

	if (!allow_power_off) {
		dev_info_ratelimited(mfg->dev, "[C] power off skipped (allow_power_off=0)\n");
		return 0;
	}

	mfg_step(mfg, "genpd power_off: cancel verify work");
	cancel_delayed_work_sync(&mfg->verify_work);
	mfg_step(mfg, "genpd power_off: lock");
	mutex_lock(&mfg->lock);
	mfg_step(mfg, "genpd power_off: locked (powered=%d)", mfg->eb_powered);
	if (mfg->initialized && mfg->eb_powered)
		ret = mt6897_eb_down_locked(mfg, "C off");
	mutex_unlock(&mfg->lock);
	mfg_step(mfg, "genpd power_off: done %d", ret);
	return ret;
}

static int mt6897_pd_set_perf_state(struct generic_pm_domain *pd, unsigned int state)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	int idx, ret = 0;

	mfg_step(mfg, "genpd set_performance(%u): lock", state);
	mutex_lock(&mfg->lock);
	if (!mfg->initialized || fix_oppidx >= 0)
		goto out;

	idx = max(mt6897_level_to_idx(mfg, state), mt6897_ceil_idx(mfg));
	mfg->target_idx = idx;

	/*
	 * genpd sets the level before power-on and after power-off; that is
	 * only recorded and committed by mt6897_pd_power_on().
	 */
	if (!follow_devfreq || !mfg->eb_powered || pd->status != GENPD_STATE_ON ||
	    !mt6897_is_active(mfg))
		goto out;
	if ((int)gf_read(mfg, GF_CUR_OPPIDX_GPU) == mt6897_clamped(mfg, idx))
		goto out;

	ret = mt6897_commit_verified_locked(mfg, idx, "C dvfs");
	dev_info_ratelimited(mfg->dev, "[C dvfs] level %u -> OPP %d: %d, EB now %u kHz\n",
			     state, idx, ret, gf_read(mfg, GF_CUR_FGPU));
out:
	mutex_unlock(&mfg->lock);
	return ret;
}

static int mt6897_pd_attach_dev(struct generic_pm_domain *pd, struct device *dev)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	struct dev_pm_opp_data opp = {};
	int i, idx, n, ceil, ret, added = 0;
	u32 e[6];

	mutex_lock(&mfg->lock);
	ret = mt6897_init_locked(mfg);
	if (ret)
		goto out;

	n = mt6897_opp_num(mfg);
	if (!n) {
		dev_err(mfg->dev, "[C] EB OPP table is empty\n");
		ret = -EINVAL;
		goto out;
	}

	if (fix_oppidx >= 0) {
		/* fixed mode: one OPP, the EB holds it (CMD_FIX_TARGET_OPPIDX) */
		idx = min(fix_oppidx, n - 1);
		mt6897_read_opp(mfg, idx, e);
		opp.freq = (unsigned long)e[0] * HZ_PER_KHZ;
		opp.u_volt = e[1] * 10;
		opp.level = n - 1 - idx;
		ret = dev_pm_opp_add_dynamic(dev, &opp);
		dev_info(mfg->dev, "[C] %s: fixed OPP %lu Hz (EB index %d): %d\n",
			 dev_name(dev), opp.freq, idx, ret);
		goto out;
	}

	/* OPPs from the max_freq_khz ceiling down to the slowest one */
	ceil = mt6897_ceil_idx(mfg);
	for (i = n - 1; i >= ceil; i--) {
		mt6897_read_opp(mfg, i, e);
		opp.freq = (unsigned long)e[0] * HZ_PER_KHZ;
		opp.u_volt = e[1] * 10;	/* informational: the EB applies it */
		opp.level = n - 1 - i;
		ret = dev_pm_opp_add_dynamic(dev, &opp);
		if (ret) {
			dev_err(mfg->dev, "[C] cannot add OPP %d (%lu Hz) to %s: %pe\n",
				i, opp.freq, dev_name(dev), ERR_PTR(ret));
			dev_pm_opp_remove_all_dynamic(dev);
			goto out;
		}
		added++;
	}
	mfg->target_idx = n - 1;
	mt6897_read_opp(mfg, ceil, e);
	dev_info(mfg->dev, "[C] %s: %d OPPs, EB index %d..%d, max %u kHz (max_freq_khz=%u)\n",
		 dev_name(dev), added, n - 1, ceil, e[0], max_freq_khz);
out:
	mutex_unlock(&mfg->lock);
	return ret;
}

/*
 * genpd calls start() right before the consumer's runtime_resume and stop()
 * right after its runtime_suspend. Logging only: tells a death inside
 * panthor's suspend/resume apart from one in this driver.
 */
static int mt6897_pd_dev_start(struct device *dev)
{
	struct mt6897_mfg *mfg = READ_ONCE(mt6897_the_mfg);

	if (mfg)
		mfg_step(mfg, "consumer %s: runtime resume starts", dev_name(dev));
	return 0;
}

static int mt6897_pd_dev_stop(struct device *dev)
{
	struct mt6897_mfg *mfg = READ_ONCE(mt6897_the_mfg);

	if (mfg)
		mfg_step(mfg, "consumer %s: runtime suspend finished", dev_name(dev));
	return 0;
}

static void mt6897_pd_detach_dev(struct generic_pm_domain *pd, struct device *dev)
{
	dev_pm_opp_remove_all_dynamic(dev);
}

static unsigned long mt6897_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct mt6897_mfg *mfg = container_of(hw, struct mt6897_mfg, clk_hw);

	/* No lock: a racy read of one word the EB updates is fine here. */
	if (!READ_ONCE(mfg->initialized))
		return 0;

	return (unsigned long)gf_read(mfg, GF_CUR_FGPU) * HZ_PER_KHZ;
}

static int mt6897_clk_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	/* Read-only view of the EB's frequency: accept any request, change nothing. */
	return 0;
}

static const struct clk_ops mt6897_clk_ops = {
	.recalc_rate = mt6897_clk_recalc_rate,
	.determine_rate = mt6897_clk_determine_rate,
};

/* ------------------------------------------------------------------------ */
/* debugfs triggers                                                         */

static bool mt6897_genpd_in_use(struct mt6897_mfg *mfg)
{
	return mfg->pd.device_count > 0;
}

static ssize_t a1_init_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mt6897_mfg *mfg = f->private_data;
	int ret;

	mutex_lock(&mfg->lock);
	if (mfg->initialized) {
		dev_info(mfg->dev, "[A1] already done this boot, not sending INIT_SHARED_MEM again\n");
		mt6897_log_shared(mfg, "A1 again");
		ret = -EALREADY;
	} else {
		ret = mt6897_init_locked(mfg);
	}
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

static ssize_t b_power_write(struct mt6897_mfg *mfg, size_t len, bool on)
{
	const char *who = on ? "B on" : "B off";
	struct mt6897_pwr p;
	int ret;

	if (!on && !mt6897_genpd_in_use(mfg))
		cancel_delayed_work_sync(&mfg->verify_work);
	mfg_step(mfg, "%s: lock", who);
	mutex_lock(&mfg->lock);
	if (!mfg->initialized) {
		dev_err(mfg->dev, "[%s] run a1_init first\n", who);
		ret = -EPERM;
		goto out;
	}
	if (mt6897_genpd_in_use(mfg)) {
		dev_err(mfg->dev, "[%s] a device is attached to the power domain (stage C); refusing\n", who);
		ret = -EBUSY;
		goto out;
	}
	if (!on && !mfg->dbg_power_refs) {
		dev_err(mfg->dev, "[%s] no outstanding b_power_on, refusing to unbalance the EB count\n", who);
		ret = -EPERM;
		goto out;
	}
	if (on && mfg->eb_powered) {
		dev_err(mfg->dev, "[%s] already powered\n", who);
		ret = -EALREADY;
		goto out;
	}

	dev_info(mfg->dev, "[%s] start\n", who);
	mt6897_log_shared(mfg, who);
	mt6897_log_pwr(mfg, who, NULL);

	/*
	 * On: POWER only, no ACTIVE of ours. With nothing running on the GPU
	 * the EB stays idle (26 MHz, DVFS frozen): the state for testing
	 * commits without ACTIVE. Off: release our reference, wait for the
	 * EB's own one to go, POWER off.
	 */
	if (on)
		ret = mt6897_eb_up_locked(mfg, false, mt6897_opp_num(mfg) - 1, who);
	else
		ret = mt6897_eb_down_locked(mfg, who);
	if (ret)
		goto out;

	if (on)
		mfg->dbg_power_refs++;
	else
		mfg->dbg_power_refs--;

	mt6897_log_shared(mfg, who);
	mt6897_log_pwr(mfg, who, &p);
	if (on) {
		if (p.mfg0_on && p.mfg1_on)
			mt6897_read_mali_ids(mfg);
		else
			dev_err(mfg->dev, "[B on] EB acked but MFG0/MFG1 not on: NOT touching Mali registers\n");
	}
	mt6897_log_freq(mfg, who);
	dev_info(mfg->dev, "[%s] done\n", who);
out:
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

static ssize_t b_power_on_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	return b_power_write(f->private_data, len, true);
}

static ssize_t b_power_off_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	return b_power_write(f->private_data, len, false);
}

static ssize_t fix_opp_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mt6897_mfg *mfg = f->private_data;
	int idx, ret;

	ret = kstrtoint_from_user(ubuf, len, 0, &idx);
	if (ret)
		return ret;

	mutex_lock(&mfg->lock);
	if (!mfg->initialized) {
		ret = -EPERM;
	} else if (idx < -1 || idx >= mt6897_opp_num(mfg)) {
		ret = -EINVAL;
	} else {
		dev_info(mfg->dev, "[fix_opp] request %d\n", idx);
		ret = mt6897_fix_opp_locked(mfg, idx);
	}
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

/* Common checks for the manual DVFS triggers: initialised and powered by us. */
static int mt6897_manual_ok(struct mt6897_mfg *mfg, const char *who)
{
	if (!mfg->initialized) {
		dev_err(mfg->dev, "[%s] run a1_init first\n", who);
		return -EPERM;
	}
	if (!mfg->eb_powered) {
		dev_err(mfg->dev, "[%s] GPU not powered (b_power_on, or keep panthor resumed)\n", who);
		return -EPERM;
	}
	if (mt6897_genpd_in_use(mfg) && mfg->pd.status != GENPD_STATE_ON) {
		dev_err(mfg->dev, "[%s] power domain is off\n", who);
		return -EPERM;
	}
	return 0;
}

static ssize_t active_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mt6897_mfg *mfg = f->private_data;
	bool active;
	int ret;

	ret = kstrtobool_from_user(ubuf, len, &active);
	if (ret)
		return ret;

	mutex_lock(&mfg->lock);
	ret = mt6897_manual_ok(mfg, "active");
	if (!ret && !active && mt6897_genpd_in_use(mfg) && !force) {
		dev_err(mfg->dev, "[active] refusing to idle the EB under a running consumer (power domain on); set force=1 to override\n");
		ret = -EBUSY;
	}
	if (!ret) {
		dev_info(mfg->dev, "[active] request %s (currently %s)\n",
			 active ? "active" : "idle", mt6897_active_count(mfg) > 0 ? "active" : "idle");
		mt6897_log_freq(mfg, "active before");
		ret = mt6897_active_locked(mfg, active, "active");
		mt6897_log_freq(mfg, "active after");
	}
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

static ssize_t commit_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mt6897_mfg *mfg = f->private_data;
	int idx, ret;

	ret = kstrtoint_from_user(ubuf, len, 0, &idx);
	if (ret)
		return ret;

	mutex_lock(&mfg->lock);
	ret = mt6897_manual_ok(mfg, "commit");
	if (!ret && (idx < 0 || idx >= mt6897_opp_num(mfg)))
		ret = -EINVAL;
	if (!ret) {
		if (mt6897_genpd_in_use(mfg) && follow_devfreq)
			dev_warn(mfg->dev, "[commit] devfreq is also committing (follow_devfreq=1)\n");
		dev_info(mfg->dev, "[commit] request OPP %d\n", idx);
		mt6897_log_freq(mfg, "commit before");
		ret = mt6897_commit_locked(mfg, idx, "commit");
		mt6897_log_freq(mfg, "commit after");
	}
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

static ssize_t limit_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mt6897_mfg *mfg = f->private_data;
	int idx, ret;

	ret = kstrtoint_from_user(ubuf, len, 0, &idx);
	if (ret)
		return ret;

	mutex_lock(&mfg->lock);
	if (!mfg->initialized) {
		ret = -EPERM;
	} else if (idx < -1 || idx >= mt6897_opp_num(mfg)) {
		ret = -EINVAL;
	} else if (!allow_above_cap &&
		   (idx == -1 || idx < mt6897_idx_for_khz(mfg, MT6897_HARD_CAP_KHZ))) {
		dev_err(mfg->dev, "[limit] %d is above the %u kHz cap; set allow_above_cap=1 first\n",
			idx, MT6897_HARD_CAP_KHZ);
		ret = -EPERM;
	} else {
		dev_info(mfg->dev, "[limit] request LIMIT_DEBUG ceiling %d\n", idx);
		mt6897_log_freq(mfg, "limit before");
		ret = mt6897_set_limit_locked(mfg, idx, "limit");
		mt6897_log_freq(mfg, "limit after");
	}
	mutex_unlock(&mfg->lock);
	return ret ? ret : len;
}

static int status_show(struct seq_file *s, void *unused)
{
	struct mt6897_mfg *mfg = s->private;
	struct mt6897_pwr p;
	unsigned int i;

	mutex_lock(&mfg->lock);
	seq_printf(s, "initialized=%d init_failed=%d ipi_broken=%d magic=%08x dbg_power_refs=%u genpd_devices=%u\n",
		   mfg->initialized, mfg->init_failed, mfg->ipi_broken, mfg->ipi_magic,
		   mfg->dbg_power_refs, mfg->pd.device_count);
	mt6897_read_pwr(mfg, &p);
	seq_printf(s, "spm=%08x/%08x rpc=%08x/%08x mfg1_con=%08x pdca=%08x mfg0=%d mfg1=%d\n",
		   p.spm, p.spm2, p.rpc, p.rpc2, p.mfg1_con, p.pdca, p.mfg0_on, p.mfg1_on);
	seq_printf(s, "our_active=%d ", mfg->our_active);
	seq_printf(s, "eb_powered=%d eb_active_count=%d target_idx=%d last_commit=%d max_freq_khz=%u ceil_idx=%d allow_above_cap=%d\n",
		   mfg->eb_powered, mt6897_active_count(mfg), mfg->target_idx, mfg->last_commit,
		   max_freq_khz, mfg->initialized ? mt6897_ceil_idx(mfg) : -1, allow_above_cap);
	seq_printf(s, "limit: ours(LIMIT_DEBUG)=%d eb_ceiling=%d c_limiter=%u c_prio=%u eb_floor=%d f_limiter=%u f_prio=%u",
		   mfg->eb_limit, (int)gf_read(mfg, GF_CUR_CEILING), gf_read(mfg, GF_CUR_C_LIMITER),
		   gf_read(mfg, GF_CUR_C_PRIORITY), (int)gf_read(mfg, GF_CUR_FLOOR),
		   gf_read(mfg, GF_CUR_F_LIMITER), gf_read(mfg, GF_CUR_F_PRIORITY));
	for (i = 0; i < LIMIT_NUM; i++) {
		unsigned int off = GF_LIMIT_TABLE + i * GF_LIMIT_ENTRY_SIZE;
		int c = gf_read(mfg, off + GF_LIMIT_CEILING), f = gf_read(mfg, off + GF_LIMIT_FLOOR);

		if (c != GPUPPM_DEFAULT_IDX || f != GPUPPM_DEFAULT_IDX)
			seq_printf(s, " [%u:c%d,f%d]", i, c, f);
	}
	seq_puts(s, "\n");
	seq_printf(s, "opp_idx=%d fgpu=%u fstack=%u vgpu=%u vsram=%u power_mw=%u active_count=%d power_count=%d dvfs_state=%#x active_sleep_control=%u\n",
		   (int)gf_read(mfg, GF_CUR_OPPIDX_GPU), gf_read(mfg, GF_CUR_FGPU),
		   gf_read(mfg, GF_CUR_FSTACK), gf_read(mfg, GF_CUR_VGPU),
		   gf_read(mfg, GF_CUR_VSRAM_GPU), gf_read(mfg, GF_CUR_POWER_GPU),
		   (int)gf_read(mfg, GF_ACTIVE_COUNT), (int)gf_read(mfg, GF_POWER_COUNT),
		   gf_read(mfg, GF_DVFS_STATE), gf_read(mfg, GF_ACTIVE_SLEEP_CONTROL));
	if (mt6897_mfg_readable(mfg, &p)) {
		u32 c1 = readl(mfg->mfgpll_con1), c2 = readl(mfg->mfgscpll_con1);
		u32 sel = readl(mfg->topck_cfg30);

		seq_printf(s, "hw: mfgpll_con1=%08x (%u kHz) mfgscpll_con1=%08x (%u kHz) topck_cfg30=%08x mfg_mux=%s mfgsc_mux=%s\n",
			   c1, mt6897_pll_khz(c1), c2, mt6897_pll_khz(c2), sel,
			   (sel & TOPCK_MFG_SEL_PLL) ? "pll" : "26m",
			   (sel & TOPCK_MFGSC_SEL_PLL) ? "pll" : "26m");
	} else {
		seq_puts(s, "hw: not read (not powered by us, powering down, MFG1 off, or not described)\n");
	}
	seq_puts(s, "gpr:");
	for (i = 0; i < GPR_NUM; i++)
		seq_printf(s, " %08x", readl(mfg->gpr + i * 4));
	seq_puts(s, "\nshared[0x000-0x13f]:\n");
	for (i = 0; i < 0x140; i += 4)
		seq_printf(s, "%08x%c", gf_read(mfg, i), (i % 32 == 28) ? '\n' : ' ');
	if (mfg->initialized) {
		int n = mt6897_opp_num(mfg);
		u32 e[6];

		for (i = 0; i < n; i++) {
			mt6897_read_opp(mfg, i, e);
			seq_printf(s, "opp[%02u] %u kHz %u %u %u %u %u mW\n",
				   i, e[0], e[1], e[2], e[3], e[4], e[5]);
		}
	}
	mutex_unlock(&mfg->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(status);

#define WO_FOPS(name) \
	static const struct file_operations name##_fops = { \
		.owner = THIS_MODULE, .open = simple_open, .write = name##_write, \
		.llseek = noop_llseek, }
WO_FOPS(a1_init);
WO_FOPS(b_power_on);
WO_FOPS(b_power_off);
WO_FOPS(fix_opp);
WO_FOPS(active);
WO_FOPS(commit);
WO_FOPS(limit);

static void mt6897_debugfs_init(struct mt6897_mfg *mfg)
{
	mfg->dbg = debugfs_create_dir("mt6897_gpu", NULL);
	debugfs_create_file("a1_init", 0200, mfg->dbg, mfg, &a1_init_fops);
	debugfs_create_file("b_power_on", 0200, mfg->dbg, mfg, &b_power_on_fops);
	debugfs_create_file("b_power_off", 0200, mfg->dbg, mfg, &b_power_off_fops);
	debugfs_create_file("fix_opp", 0200, mfg->dbg, mfg, &fix_opp_fops);
	debugfs_create_file("active", 0200, mfg->dbg, mfg, &active_fops);
	debugfs_create_file("commit", 0200, mfg->dbg, mfg, &commit_fops);
	debugfs_create_file("limit", 0200, mfg->dbg, mfg, &limit_fops);
	debugfs_create_file("status", 0400, mfg->dbg, mfg, &status_fops);
}

/* ------------------------------------------------------------------------ */
/* max_freq_khz runtime changes                                             */

static int max_freq_khz_set(const char *val, const struct kernel_param *kp)
{
	struct mt6897_mfg *mfg = READ_ONCE(mt6897_the_mfg);
	unsigned int khz;
	int ret, ceil;

	ret = kstrtouint(val, 0, &khz);
	if (ret)
		return ret;
	if (khz > MT6897_HARD_CAP_KHZ && !allow_above_cap) {
		pr_warn("mtk_mfg_mt6897: max_freq_khz %u above the %u kHz cap needs allow_above_cap=1\n",
			khz, MT6897_HARD_CAP_KHZ);
		return -EINVAL;
	}
	max_freq_khz = khz;

	if (!mfg)
		return 0;	/* boot parameter: applied at A1 */

	mfg_step(mfg, "max_freq_khz=%u: lock", khz);
	mutex_lock(&mfg->lock);
	if (mfg->initialized) {
		ceil = mt6897_ceil_idx(mfg);
		dev_info(mfg->dev, "[cap] max_freq_khz=%u -> ceiling OPP %d\n", khz, ceil);
		ret = mt6897_set_limit_locked(mfg, ceil, "cap");
		if (mfg->eb_powered && mt6897_is_active(mfg) &&
		    (int)gf_read(mfg, GF_CUR_OPPIDX_GPU) < ceil)
			mt6897_commit_locked_quiet(mfg, ceil, "cap");
		if (mfg->eb_powered)
			mt6897_check_cap_locked(mfg, "cap");
	}
	mutex_unlock(&mfg->lock);
	return ret;
}

/* ------------------------------------------------------------------------ */
/* Probe: stage A, passive                                                  */

static void __iomem *mt6897_map(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* Shared with other blocks (SPM, MFG_RPC): map without claiming. */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

static int mt6897_mfg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {};
	struct mt6897_mfg *mfg;
	struct resource *res, shm;
	int ret;

	mfg = devm_kzalloc(dev, sizeof(*mfg), GFP_KERNEL);
	if (!mfg)
		return -ENOMEM;
	mfg->dev = dev;
	mutex_init(&mfg->lock);
	init_completion(&mfg->rx_done);
	INIT_DELAYED_WORK(&mfg->verify_work, mt6897_verify_work);
	platform_set_drvdata(pdev, mfg);

	mfg->gpr = mt6897_map(pdev, "gpr");
	mfg->rpc = mt6897_map(pdev, "rpc");
	mfg->spm = mt6897_map(pdev, "spm-status");
	mfg->pdca = mt6897_map(pdev, "pdca-sta");
	if (IS_ERR(mfg->gpr) || IS_ERR(mfg->rpc) || IS_ERR(mfg->spm) || IS_ERR(mfg->pdca))
		return dev_err_probe(dev, -EINVAL, "missing gpr/rpc/spm-status/pdca-sta registers\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpu");
	if (res)
		mfg->gpu_res = *res;

	/* optional read-only clock readback; only read while MFG1 is on */
	mfg->mfgpll_con1 = mt6897_map(pdev, "mfgpll-con1");
	mfg->mfgscpll_con1 = mt6897_map(pdev, "mfgscpll-con1");
	mfg->topck_cfg30 = mt6897_map(pdev, "topck-mfg-sel");
	if (IS_ERR(mfg->mfgpll_con1) || IS_ERR(mfg->mfgscpll_con1) || IS_ERR(mfg->topck_cfg30)) {
		dev_info(dev, "[A] no PLL/mux readback registers described\n");
		mfg->mfgpll_con1 = NULL;
		mfg->mfgscpll_con1 = NULL;
		mfg->topck_cfg30 = NULL;
	}
	mfg->target_idx = 0;
	mfg->last_commit = -1;
	mfg->eb_limit = -1;

	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &shm);
	if (ret)
		return dev_err_probe(dev, ret, "no GPUEB shared memory region\n");
	if (resource_size(&shm) < GPUFREQ_MEM_SIZE)
		return dev_err_probe(dev, -EINVAL, "shared memory %pR smaller than %#x\n",
				     &shm, GPUFREQ_MEM_SIZE);
	mfg->shared_phys = shm.start;
	mfg->shared = devm_ioremap(dev, shm.start, GPUFREQ_MEM_SIZE);
	if (!mfg->shared)
		return dev_err_probe(dev, -ENOMEM, "cannot map shared memory\n");

	dev_info(dev, "[A] passive probe: shared memory %pa + %#x, no EB access yet\n",
		 &mfg->shared_phys, GPUFREQ_MEM_SIZE);
	mt6897_log_gpr(mfg, "A");
	dev_info(dev, "[A] IPI magic (GPR13) = %08x%s\n", readl(mfg->gpr + GPR_IPI_MAGIC),
		 readl(mfg->gpr + GPR_IPI_MAGIC) ? "" : "  <- zero: EB down or magic consumed");
	dev_info(dev, "[A] reading SPM 0x1c001fc0/c4, MFG_RPC 0x13f91070/0x13f91200/04, PDCA_STA 0x13c620a8\n");
	mt6897_log_pwr(mfg, "A", NULL);
	dev_info(dev, "[A] shared memory before init: magic=%08x power_count=%d fgpu=%u power_control=%u\n",
		 gf_read(mfg, GF_MAGIC), (int)gf_read(mfg, GF_POWER_COUNT),
		 gf_read(mfg, GF_CUR_FGPU), gf_read(mfg, GF_POWER_CONTROL));

	init.name = "mt6897-gpu-core";
	init.ops = &mt6897_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	mfg->clk_hw.init = &init;
	ret = devm_clk_hw_register(dev, &mfg->clk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register GPU clock\n");
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &mfg->clk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "cannot add clock provider\n");

	mfg->pd.name = "mt6897-mfg";
	mfg->pd.power_on = mt6897_pd_power_on;
	mfg->pd.power_off = mt6897_pd_power_off;
	mfg->pd.set_performance_state = mt6897_pd_set_perf_state;
	mfg->pd.attach_dev = mt6897_pd_attach_dev;
	mfg->pd.detach_dev = mt6897_pd_detach_dev;
	mfg->pd.dev_ops.start = mt6897_pd_dev_start;
	mfg->pd.dev_ops.stop = mt6897_pd_dev_stop;
	mfg->pd.flags = GENPD_FLAG_OPP_TABLE_FW;
	ret = pm_genpd_init(&mfg->pd, NULL, true);
	if (ret)
		return dev_err_probe(dev, ret, "cannot init power domain\n");
	ret = of_genpd_add_provider_simple(dev->of_node, &mfg->pd);
	if (ret) {
		pm_genpd_remove(&mfg->pd);
		return dev_err_probe(dev, ret, "cannot add power domain provider\n");
	}

	mt6897_debugfs_init(mfg);
	WRITE_ONCE(mt6897_the_mfg, mfg);
	dev_info(dev, "[A] ready: triggers in /sys/kernel/debug/mt6897_gpu/\n");
	return 0;
}

static const struct of_device_id mt6897_mfg_of_match[] = {
	{ .compatible = "mediatek,mt6897-gpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6897_mfg_of_match);

static struct platform_driver mt6897_mfg_driver = {
	.driver = {
		.name = "mtk-mfg-mt6897",
		.of_match_table = mt6897_mfg_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = mt6897_mfg_probe,
};
builtin_platform_driver(mt6897_mfg_driver);

MODULE_DESCRIPTION("MediaTek MT6897 MFlexGraphics power domain via GPUEB");
MODULE_LICENSE("GPL");
