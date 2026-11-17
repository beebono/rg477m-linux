// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6897 MCUPM bring-up client: the AP side of what the vendor mcupm.ko does
 * at probe, one switchable step at a time, to find out what the MCUPM's CPU
 * power-management task waits for before it reports itself ready to the
 * trusted firmware (which then accepts PSCI CPU_SUSPEND).
 *
 * Vendor sequence (mcupm.ko, mcupm_device_probe -> mcupm_plt_module_init ->
 * mcupm_timesync_init), on a device without MCUPM reserved memory:
 *   1. IPI 0 (mailbox 0, "platform"): send {0xDEAD, 0, 0} and wait up to
 *      2000 ms for the one-word answer ("MCUPM: Alive" when it is non-zero);
 *   2. timesync: write the arch counter and the matching sched_clock time
 *      into mailbox 4 slots 12, 13, 15, 14 (no interrupt).
 *
 * Read-only throughout: the ATF/MCUPM state words in mailbox 3's memory
 * (0x0C0BF7E0-0x0C0BF81F, TASK_STA at +0x20) and the mailbox status
 * registers. The only writes are the ping (through the mailbox driver, into
 * mailbox 0) and, if enabled, the four timesync words in mailbox 4.
 */

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched/clock.h>
#include <linux/workqueue.h>
#include <clocksource/arm_arch_timer.h>

/* "pm" region: 0x0C0BF7E0, 16 words (mailbox 3 slots 0-15) */
#define PM_AP_READY		0x00
#define PM_PWR_CTRL_EN		0x10
#define PM_BUCK			0x18
#define PM_ARMPLL		0x1c
#define PM_TASK_STA		0x20
#define PM_WORDS		16

/* "ctl" region: the mailbox control block, read only here */
#define CTL_SEND_STS		0x00
#define CTL_RECV_STS		0x10

/* "timesync" region: mailbox 4 slots 12-15 (0x0C0BF8B0) */
#define TS_TICK_H		0x0	/* slot 12 */
#define TS_TICK_L		0x4	/* slot 13 */
#define TS_TS_H			0x8	/* slot 14 */
#define TS_TS_L			0xc	/* slot 15 */
#define TS_MBOX			4

#define PING_MAGIC		0xDEAD
#define PING_TIMEOUT_MS		2000

static bool ping = true;
module_param(ping, bool, 0444);
MODULE_PARM_DESC(ping, "Send the vendor's 0xDEAD platform IPI on mailbox 0 at probe");

static bool timesync = true;
module_param(timesync, bool, 0444);
MODULE_PARM_DESC(timesync, "Write the vendor's timesync words into mailbox 4 at probe (after the ping)");

struct mcupm {
	struct device *dev;
	void __iomem *pm;
	void __iomem *ctl;
	void __iomem *ts;
	void __iomem *mbox0;	/* mailbox 0 memory, for logging */

	struct mutex lock;
	struct mbox_client cl;
	struct mbox_chan *ch;
	struct completion done;
	u32 reply;
	u32 tx[3];
	bool pinged;
	u8 ts_ver;

	struct delayed_work log_work;
	unsigned int log_step;
	unsigned long t0;
	struct dentry *dbg;
};

static void mcupm_log_state(struct mcupm *m, const char *when)
{
	u32 w[PM_WORDS];
	int i;

	for (i = 0; i < PM_WORDS; i++)
		w[i] = readl(m->pm + i * 4);

	dev_info(m->dev,
		 "[%s] TASK_STA=%#x AP_READY=%#x PWR_CTRL_EN=%#x BUCK=%#x ARMPLL=%#x | send_sts=%#x recv_sts=%#x\n",
		 when, w[PM_TASK_STA / 4], w[PM_AP_READY / 4], w[PM_PWR_CTRL_EN / 4],
		 w[PM_BUCK / 4], w[PM_ARMPLL / 4],
		 readl(m->ctl + CTL_SEND_STS), readl(m->ctl + CTL_RECV_STS));
	dev_info(m->dev, "[%s] pm words: %08x %08x %08x %08x %08x %08x %08x %08x  %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 when, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
		 w[8], w[9], w[10], w[11], w[12], w[13], w[14], w[15]);
}

static void mcupm_rx(struct mbox_client *cl, void *msg)
{
	struct mcupm *m = container_of(cl, struct mcupm, cl);

	m->reply = *(u32 *)msg;
	complete(&m->done);
}

/* The vendor's platform IPI. One attempt, never retried. */
static int mcupm_ping_locked(struct mcupm *m)
{
	unsigned long left;
	int ret;

	if (m->pinged) {
		dev_info(m->dev, "[ping] already sent this boot, not sending again\n");
		return -EALREADY;
	}
	m->pinged = true;

	/*
	 * The vendor's send path refuses while the pin's send bit is set; the
	 * mainline mailbox driver does not check it before writing. A pending
	 * receive would mean MCUPM is in a state this was not written for.
	 */
	if ((readl(m->ctl + CTL_SEND_STS) | readl(m->ctl + CTL_RECV_STS)) & BIT(0)) {
		dev_err(m->dev, "[ping] mailbox 0 busy (send_sts=%#x recv_sts=%#x): not sending\n",
			readl(m->ctl + CTL_SEND_STS), readl(m->ctl + CTL_RECV_STS));
		return -EBUSY;
	}

	if (!m->ch) {
		m->cl.dev = m->dev;
		m->cl.rx_callback = mcupm_rx;
		m->cl.tx_block = false;
		m->cl.knows_txdone = false;
		dev_info(m->dev, "[ping] requesting mailbox 0 (writes IRQ_CLR bit 0 at 0x0C072018)\n");
		m->ch = mbox_request_channel_byname(&m->cl, "platform");
		if (IS_ERR(m->ch)) {
			ret = PTR_ERR(m->ch);
			m->ch = NULL;
			dev_err(m->dev, "[ping] cannot get mailbox 0: %pe\n", ERR_PTR(ret));
			return ret;
		}
	}

	m->tx[0] = PING_MAGIC;
	m->tx[1] = 0;
	m->tx[2] = 0;
	reinit_completion(&m->done);
	m->reply = 0;

	mcupm_log_state(m, "ping before");
	dev_info(m->dev, "[ping] sending {0xdead, 0, 0} to 0x0C0BF600, then BIT(0) to 0x0C072004\n");
	ret = mbox_send_message(m->ch, m->tx);
	if (ret < 0) {
		dev_err(m->dev, "[ping] send failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	left = wait_for_completion_timeout(&m->done, msecs_to_jiffies(PING_TIMEOUT_MS));
	if (!left) {
		dev_err(m->dev, "[ping] no answer within %d ms (vendor would print \"Dead\")\n",
			PING_TIMEOUT_MS);
		mcupm_log_state(m, "ping timeout");
		return -ETIMEDOUT;
	}

	dev_info(m->dev, "[ping] answer %#x after %u ms: MCUPM %s\n", m->reply,
		 jiffies_to_msecs(msecs_to_jiffies(PING_TIMEOUT_MS) - left),
		 m->reply ? "Alive" : "Dead (answer 0)");
	mcupm_log_state(m, "ping after");
	return 0;
}

/*
 * mcupm.ko timesync_sync_base(TIMESYNC_FLAG_SYNC): version (1-7) in bits
 * 30:28 and the freeze flag (0) in bit 31 of the two high words; written
 * tick-high, tick-low, ts-low, ts-high with a barrier after each, after
 * checking that mailbox 4's send bit is clear. No interrupt.
 */
static int mcupm_timesync_locked(struct mcupm *m)
{
	u64 tick, ts;
	u32 hi;

	if (readl(m->ctl + CTL_SEND_STS) & BIT(TS_MBOX)) {
		dev_err(m->dev, "[timesync] mailbox 4 send bit set, not writing\n");
		return -EBUSY;
	}

	tick = arch_timer_read_counter();
	ts = sched_clock();
	m->ts_ver = (m->ts_ver + 1) & 7;
	hi = (u32)m->ts_ver << 28;

	dev_info(m->dev, "[timesync] writing tick=%llu ts=%llu ns ver=%u to 0x0C0BF8B0..BF\n",
		 tick, ts, m->ts_ver);
	writel(hi | upper_32_bits(tick), m->ts + TS_TICK_H);
	dsb(sy);
	writel(lower_32_bits(tick), m->ts + TS_TICK_L);
	dsb(sy);
	writel(lower_32_bits(ts), m->ts + TS_TS_L);
	dsb(sy);
	writel(hi | upper_32_bits(ts), m->ts + TS_TS_H);
	dsb(sy);
	mcupm_log_state(m, "timesync after");
	return 0;
}

/* TASK_STA right after an action, then ~1 s and ~45 s later */
static void mcupm_log_work(struct work_struct *work)
{
	struct mcupm *m = container_of(to_delayed_work(work), struct mcupm, log_work);
	char when[32];

	snprintf(when, sizeof(when), "+%u ms", jiffies_to_msecs(jiffies - m->t0));
	mcupm_log_state(m, when);
	if (++m->log_step == 1)
		schedule_delayed_work(&m->log_work, msecs_to_jiffies(44000));
}

static void mcupm_arm_log(struct mcupm *m)
{
	m->t0 = jiffies;
	m->log_step = 0;
	mod_delayed_work(system_wq, &m->log_work, msecs_to_jiffies(1000));
}

static ssize_t ping_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mcupm *m = f->private_data;
	int ret;

	mutex_lock(&m->lock);
	ret = mcupm_ping_locked(m);
	mutex_unlock(&m->lock);
	mcupm_arm_log(m);
	return ret ? ret : len;
}

static ssize_t timesync_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mcupm *m = f->private_data;
	int ret;

	mutex_lock(&m->lock);
	ret = mcupm_timesync_locked(m);
	mutex_unlock(&m->lock);
	mcupm_arm_log(m);
	return ret ? ret : len;
}

static ssize_t state_write(struct file *f, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct mcupm *m = f->private_data;

	mcupm_log_state(m, "debugfs");
	return len;
}

#define MCUPM_FOPS(name) \
	static const struct file_operations name##_fops = { \
		.owner = THIS_MODULE, .open = simple_open, .write = name##_write, \
		.llseek = noop_llseek, }
MCUPM_FOPS(ping);
MCUPM_FOPS(timesync);
MCUPM_FOPS(state);

static void __iomem *mcupm_map(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* inside the mailbox driver's regions: map without claiming */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?: IOMEM_ERR_PTR(-ENOMEM);
}

static void mcupm_cancel_work(void *data)
{
	struct mcupm *m = data;

	debugfs_remove_recursive(m->dbg);
	cancel_delayed_work_sync(&m->log_work);
	if (m->ch)
		mbox_free_channel(m->ch);
}

static int mcupm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcupm *m;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->dev = dev;
	mutex_init(&m->lock);
	init_completion(&m->done);
	INIT_DELAYED_WORK(&m->log_work, mcupm_log_work);

	m->pm = mcupm_map(pdev, "pm");
	m->ctl = mcupm_map(pdev, "ctl");
	m->ts = mcupm_map(pdev, "timesync");
	m->mbox0 = mcupm_map(pdev, "mbox0");
	if (IS_ERR(m->pm) || IS_ERR(m->ctl) || IS_ERR(m->ts) || IS_ERR(m->mbox0))
		return dev_err_probe(dev, -EINVAL, "missing pm/ctl/timesync/mbox0 regions\n");

	ret = devm_add_action_or_reset(dev, mcupm_cancel_work, m);
	if (ret)
		return ret;

	dev_info(dev, "probe: ping=%d timesync=%d (parameters mtk_mcupm_mt6897.ping/.timesync)\n",
		 ping, timesync);
	mcupm_log_state(m, "probe");
	dev_info(dev, "[probe] mailbox 0 words: %08x %08x %08x %08x  rx %08x\n",
		 readl(m->mbox0), readl(m->mbox0 + 4), readl(m->mbox0 + 8),
		 readl(m->mbox0 + 12), readl(m->mbox0 + 0x50));

	mutex_lock(&m->lock);
	if (ping)
		mcupm_ping_locked(m);
	if (timesync)
		mcupm_timesync_locked(m);
	mutex_unlock(&m->lock);
	if (ping || timesync)
		mcupm_arm_log(m);

	m->dbg = debugfs_create_dir("mcupm", NULL);
	debugfs_create_file("ping", 0200, m->dbg, m, &ping_fops);
	debugfs_create_file("timesync", 0200, m->dbg, m, &timesync_fops);
	debugfs_create_file("state", 0200, m->dbg, m, &state_fops);

	return 0;
}

static const struct of_device_id mcupm_of_match[] = {
	{ .compatible = "mediatek,mt6897-mcupm" },
	{ }
};
MODULE_DEVICE_TABLE(of, mcupm_of_match);

static struct platform_driver mcupm_driver = {
	.driver = {
		.name = "mtk-mcupm-mt6897",
		.of_match_table = mcupm_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = mcupm_probe,
};
module_platform_driver(mcupm_driver);

MODULE_DESCRIPTION("MediaTek MT6897 MCUPM bring-up client");
MODULE_LICENSE("GPL");
