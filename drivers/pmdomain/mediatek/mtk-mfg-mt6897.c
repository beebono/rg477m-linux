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
	CMD_POWER_CONTROL	= 6,
	CMD_ACTIVE_SLEEP_CONTROL = 7,
	CMD_FIX_TARGET_OPPIDX	= 13,
};

#define TARGET_GPU		1
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

	struct dentry *dbg;
};

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
	dev_info(mfg->dev,
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

	dev_info(mfg->dev, "IPI %s -> EB: %08x %08x %08x %08x %08x %08x %08x %08x\n",
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

	dev_info(mfg->dev, "IPI %s <- EB: %08x %08x %08x %08x %08x %08x %08x %08x (return %d)\n",
		 what, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], (int)r[4]);
	*retval = (int)r[4];

	return 0;
}

/* ------------------------------------------------------------------------ */
/* Stage A1: shared memory init                                             */

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
	dev_info(mfg->dev, "[A1] done: EB accepted the shared memory\n");
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
	dev_info(mfg->dev, "[%s] POWER_CONTROL(%s) ok, EB power_count=%d (shared %d)\n",
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

/* ------------------------------------------------------------------------ */
/* Stage C: genpd + clock                                                   */

static inline struct mt6897_mfg *pd_to_mfg(struct generic_pm_domain *pd)
{
	return container_of(pd, struct mt6897_mfg, pd);
}

static int mt6897_pd_power_on(struct generic_pm_domain *pd)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	struct mt6897_pwr p;
	int ret;

	mutex_lock(&mfg->lock);
	ret = mt6897_init_locked(mfg);
	if (ret)
		goto out;

	ret = mt6897_power_control_locked(mfg, true, "C on");
	if (ret)
		goto out;

	mt6897_log_pwr(mfg, "C on", &p);
	if (!p.mfg0_on || !p.mfg1_on) {
		dev_err(mfg->dev, "[C] EB acked power-on but MFG0/MFG1 are off: refusing to hand the GPU over\n");
		ret = -EIO;
		goto out;
	}

	if (fix_oppidx >= 0 && !mfg->fix_applied) {
		mfg->fix_applied = true;
		mt6897_fix_opp_locked(mfg, fix_oppidx);
	}
out:
	mutex_unlock(&mfg->lock);
	return ret;
}

static int mt6897_pd_power_off(struct generic_pm_domain *pd)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	u32 val;
	int ret;

	if (!allow_power_off) {
		dev_info_ratelimited(mfg->dev, "[C] power off skipped (allow_power_off=0)\n");
		return 0;
	}

	mutex_lock(&mfg->lock);
	if (!mfg->initialized) {
		ret = 0;
		goto out;
	}

	/* Vendor kbase: gpufreq_pdca_polling_ack(GPU_PWR_OFF) before power off */
	if (readl_poll_timeout(mfg->pdca, val, !(val & PDCA_STA_BUSY), 10, 10000))
		dev_warn(mfg->dev, "[C] PDCA_STA still busy (%08x) before power off\n", val);

	ret = mt6897_power_control_locked(mfg, false, "C off");
	if (!ret)
		mt6897_log_pwr(mfg, "C off", NULL);
out:
	mutex_unlock(&mfg->lock);
	return ret;
}

static int mt6897_pd_set_perf_state(struct generic_pm_domain *pd, unsigned int state)
{
	/* Single fixed OPP: the EB keeps its own (or the fix_oppidx) setting. */
	return 0;
}

static int mt6897_pd_attach_dev(struct generic_pm_domain *pd, struct device *dev)
{
	struct mt6897_mfg *mfg = pd_to_mfg(pd);
	struct dev_pm_opp_data opp = {};
	int idx, n, ret;
	u32 e[6];

	mutex_lock(&mfg->lock);
	ret = mt6897_init_locked(mfg);
	if (ret)
		goto out;

	n = mt6897_opp_num(mfg);
	idx = fix_oppidx >= 0 ? fix_oppidx : (int)gf_read(mfg, GF_CUR_OPPIDX_GPU);
	if (!n || idx < 0 || idx >= n) {
		dev_err(mfg->dev, "[C] no usable OPP (num %d, idx %d)\n", n, idx);
		ret = -EINVAL;
		goto out;
	}

	mt6897_read_opp(mfg, idx, e);
	opp.freq = (unsigned long)e[0] * HZ_PER_KHZ;
	opp.u_volt = e[1] * 10;
	opp.level = idx;
	ret = dev_pm_opp_add_dynamic(dev, &opp);
	if (ret)
		dev_err(mfg->dev, "[C] cannot add OPP to %s: %pe\n", dev_name(dev), ERR_PTR(ret));
	else
		dev_info(mfg->dev, "[C] %s: single OPP %lu Hz (EB index %d, %lu uV, informational)\n",
			 dev_name(dev), opp.freq, idx, opp.u_volt);
out:
	mutex_unlock(&mfg->lock);
	return ret;
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

	dev_info(mfg->dev, "[%s] start\n", who);
	mt6897_log_shared(mfg, who);
	mt6897_log_pwr(mfg, who, NULL);

	ret = mt6897_power_control_locked(mfg, on, who);
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

static void mt6897_debugfs_init(struct mt6897_mfg *mfg)
{
	mfg->dbg = debugfs_create_dir("mt6897_gpu", NULL);
	debugfs_create_file("a1_init", 0200, mfg->dbg, mfg, &a1_init_fops);
	debugfs_create_file("b_power_on", 0200, mfg->dbg, mfg, &b_power_on_fops);
	debugfs_create_file("b_power_off", 0200, mfg->dbg, mfg, &b_power_off_fops);
	debugfs_create_file("fix_opp", 0200, mfg->dbg, mfg, &fix_opp_fops);
	debugfs_create_file("status", 0400, mfg->dbg, mfg, &status_fops);
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
