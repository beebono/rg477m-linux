// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6897 OVLSYS adaptor.
 *
 * On MT6897 (MT6985 generation) the overlay engines are not part of the
 * display subsystem that drives DSI0. They live in OVLSYS0, which has its own
 * config/crossbar block (0x14400000) and its own mutex (0x14401000), and they
 * reach DISPSYS0 through the OVLSYS DLO_ASYNC3 -> DISPSYS DLI_ASYNC0 relay
 * pair. mediatek-drm assumes one MMSYS and one mutex per CRTC, so, in the
 * spirit of mtk_disp_ovl_adaptor.c, this pseudo component (no DT node) stands
 * for the whole OVLSYS sub-pipeline at the head of the DISPSYS0 CRTC:
 *
 *   CRTC path: [ OVLSYS adaptor ] -> DSI0
 *     OVLSYS:   OVL0_2L -> BLEND_CROSSBAR0 -> DLO_ASYNC3     (OVLSYS mutex)
 *     DISPSYS:  DLI_ASYNC0 -> PQ bypass crossbars -> DSI0    (DISPSYS mutex)
 *
 * It owns OVL0_2L (mtk_disp_ovl.c with MT6897 data), the OVLSYS routing (via
 * the OVLSYS0 mtk-mmsys device) and the OVLSYS mutex; on the DISPSYS side it
 * represents DLI_ASYNC0 for routing and for the CRTC's mutex. Both mutexes
 * take their SOF/EOF from DSI0.
 *
 * It also keeps the display clocks the bootloader left running enabled from
 * bind on (the "clocks" of the OVL node beyond the first one), so that the
 * pipeline survives simpledrm releasing its clocks, and it provides a register
 * dump in the format of the bootloader reference dump (debugfs and, once,
 * after the first modeset).
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk-mutex.h>
#include <linux/workqueue.h>

#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_drm_drv.h"

/* BRINGUP(mt6897): dump the register set once, after the first modeset */
static bool regdump = true;
module_param(regdump, bool, 0644);
MODULE_PARM_DESC(regdump, "MT6897: dump display registers 2 s after the first modeset");

/*
 * BRINGUP(mt6897) fallback: keep the bootloader's DISPSYS routing through the
 * full PQ chain (RSZ0 ... POSTALIGN0, engines left configured by the
 * bootloader and never reset) instead of the PQ bypass, and put the same
 * modules in the DISPSYS mutex as the bootloader does. For A/B testing only.
 */
static bool lk_pq;
module_param(lk_pq, bool, 0444);
MODULE_PARM_DESC(lk_pq, "MT6897: keep the bootloader's PQ routing instead of the PQ bypass");

/*
 * Bootloader DISPSYS mutex 0, decoded with downstream MT6897_MUTEX_MOD*_DISP_*:
 * MOD0 = AAL0 C3D0 CCORR0 CCORR1 CHIST0 CHIST1 COLOR0 DITHER0 DITHER1
 *        DLI_ASYNC0 (bits 0-9), DSI0 (20), GAMMA0 (21), MDP_AAL0 (22),
 *        ODDMR0 (26), POSTALIGN0 (27), POSTMASK0 (28), PWM0 (29), RSZ0 (31)
 * MOD1 = SPR0, TDSHP0
 */
#define MT6897_LK_DISP_MUTEX_MOD0	0xbc7003ff
#define MT6897_LK_DISP_MUTEX_MOD1	0x00000003

struct mtk_disp_ovlsys {
	struct device *dev;
	struct device *mmsys_dev;	/* DISPSYS0 mtk-mmsys */
	struct device *ovl_dev;		/* OVL0_2L */
	struct device *ovlsys_dev;	/* OVLSYS0 mtk-mmsys */
	struct device *mutex_dev;	/* OVLSYS0 mutex */
	struct mtk_mutex *mutex;
	unsigned int next;		/* component after DLI_ASYNC0 */

	struct clk **clks;
	unsigned int num_clks;
	bool clks_on;

	bool started_once;
	struct delayed_work dump_work;
	struct dentry *debugfs;
};

static struct device *ovlsys_find_dev(struct device *dev, const char *compat)
{
	struct device_node *np;
	struct platform_device *pdev;

	for_each_compatible_node(np, NULL, compat) {
		if (!of_device_is_available(np))
			continue;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!pdev)
			return ERR_PTR(-EPROBE_DEFER);
		if (!device_is_bound(&pdev->dev)) {
			put_device(&pdev->dev);
			return ERR_PTR(-EPROBE_DEFER);
		}
		return &pdev->dev;
	}

	dev_err(dev, "no available %s node\n", compat);
	return ERR_PTR(-ENODEV);
}

static void ovlsys_put_device(void *data)
{
	put_device(data);
}

/* --- register dump in the format of survey/display/lk-regdump.txt --- */

struct ovlsys_dump_range {
	u32 start;
	u32 end;
};

static const struct ovlsys_dump_range ovlsys_range_full[] = { { 0x000, 0x1000 }, { } };
static const struct ovlsys_dump_range ovlsys_range_mutex[] = { { 0x000, 0x400 }, { } };
static const struct ovlsys_dump_range ovlsys_range_dsi[] = {
	{ 0x000, 0x500 }, { 0xc00, 0xc10 }, { }
};
static const struct ovlsys_dump_range ovlsys_range_mipi[] = { { 0x000, 0x400 }, { } };

static const struct {
	const char *name;
	const char *compat;
	const struct ovlsys_dump_range *ranges;
} ovlsys_dump_blocks[] = {
	{ "dispsys0_cfg", "mediatek,mt6897-dispsys0_config", ovlsys_range_full },
	{ "disp_mutex", "mediatek,mt6897-disp-mutex", ovlsys_range_mutex },
	{ "ovlsys0_cfg", "mediatek,mt6897-ovlsys0_config", ovlsys_range_full },
	{ "ovl_mutex", "mediatek,mt6897-ovlsys-mutex", ovlsys_range_mutex },
	{ "ovl0_2l", "mediatek,mt6897-disp-ovl", ovlsys_range_full },
	{ "dsi0", "mediatek,mt6897-dsi", ovlsys_range_dsi },
	{ "mipi_tx0", "mediatek,mt6897-mipi-tx", ovlsys_range_mipi },
};

#define ovlsys_dump_line(m, fmt, ...)				\
	do {							\
		if (m)						\
			seq_printf(m, fmt, ##__VA_ARGS__);	\
		else						\
			pr_info(fmt, ##__VA_ARGS__);		\
	} while (0)

/*
 * Only the blocks this driver programs are dumped, and only while the
 * adaptor holds their clocks (from bind on), since reading an unclocked
 * display block hangs the bus on this SoC.
 */
static void ovlsys_regdump(struct mtk_disp_ovlsys *priv, struct seq_file *m)
{
	unsigned int b;

	if (!priv->clks_on) {
		ovlsys_dump_line(m, "mt6897-regdump: clocks not held, refusing to dump\n");
		return;
	}

	ovlsys_dump_line(m, "mt6897-regdump: begin\n");
	for (b = 0; b < ARRAY_SIZE(ovlsys_dump_blocks); b++) {
		const struct ovlsys_dump_range *r;
		struct device_node *np;
		struct resource res;
		void __iomem *base;
		u32 off, val;

		np = of_find_compatible_node(NULL, NULL, ovlsys_dump_blocks[b].compat);
		if (!np)
			continue;
		if (!of_device_is_available(np) || of_address_to_resource(np, 0, &res)) {
			of_node_put(np);
			continue;
		}
		of_node_put(np);

		base = ioremap(res.start, 0x1000);
		if (!base)
			continue;

		ovlsys_dump_line(m, "BLOCK %s 0x%08llx\n", ovlsys_dump_blocks[b].name,
				 (unsigned long long)res.start);
		for (r = ovlsys_dump_blocks[b].ranges; r->end; r++) {
			for (off = r->start; off < r->end; off += 4) {
				val = readl(base + off);
				if (val)
					ovlsys_dump_line(m, "%03x 0x%08X\n", off, val);
			}
		}
		ovlsys_dump_line(m, "END %s\n", ovlsys_dump_blocks[b].name);
		iounmap(base);
	}
	ovlsys_dump_line(m, "mt6897-regdump: end\n");
}

static int ovlsys_regdump_show(struct seq_file *m, void *unused)
{
	ovlsys_regdump(m->private, m);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ovlsys_regdump);

static void ovlsys_dump_work(struct work_struct *work)
{
	struct mtk_disp_ovlsys *priv = container_of(to_delayed_work(work),
						    struct mtk_disp_ovlsys,
						    dump_work);

	ovlsys_regdump(priv, NULL);
}

/* --- clocks kept on for the bootloader pipeline and for us --- */

static void ovlsys_clks_off(struct mtk_disp_ovlsys *priv)
{
	unsigned int i;

	if (!priv->clks_on)
		return;
	for (i = 0; i < priv->num_clks; i++)
		if (!IS_ERR_OR_NULL(priv->clks[i]))
			clk_disable_unprepare(priv->clks[i]);
	priv->clks_on = false;
}

static void ovlsys_clks_on(struct mtk_disp_ovlsys *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_clks; i++) {
		if (IS_ERR_OR_NULL(priv->clks[i]))
			continue;
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret) {
			/* best effort: the clock is on from the bootloader anyway */
			dev_warn(priv->dev, "bringup: keep-on clock %u (%pC) failed: %d\n",
				 i, priv->clks[i], ret);
			clk_put(priv->clks[i]);
			priv->clks[i] = NULL;
			continue;
		}
		dev_info(priv->dev, "bringup: holding clock %pC\n", priv->clks[i]);
	}
	priv->clks_on = true;
}

static void ovlsys_clks_put(void *data)
{
	struct mtk_disp_ovlsys *priv = data;
	unsigned int i;

	for (i = 0; i < priv->num_clks; i++)
		if (!IS_ERR_OR_NULL(priv->clks[i]))
			clk_put(priv->clks[i]);
}

static int ovlsys_clks_get(struct mtk_disp_ovlsys *priv)
{
	struct device_node *np = priv->ovl_dev->of_node;
	unsigned int i;

	priv->num_clks = of_clk_get_parent_count(np);
	if (!priv->num_clks)
		return 0;

	priv->clks = devm_kcalloc(priv->dev, priv->num_clks, sizeof(*priv->clks),
				  GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < priv->num_clks; i++) {
		priv->clks[i] = of_clk_get(np, i);
		if (IS_ERR(priv->clks[i])) {
			int ret = PTR_ERR(priv->clks[i]);

			priv->clks[i] = NULL;
			ovlsys_clks_put(priv);
			return dev_err_probe(priv->dev, ret, "failed to get clock %u\n", i);
		}
	}

	return devm_add_action_or_reset(priv->dev, ovlsys_clks_put, priv);
}

/* --- mtk_ddp_comp_funcs --- */

int mtk_ovlsys_adaptor_power_on(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_resume_and_get(priv->ovl_dev);
	if (ret < 0)
		return ret;

	ret = mtk_mutex_prepare(priv->mutex);
	if (ret < 0) {
		pm_runtime_put(priv->ovl_dev);
		return ret;
	}

	dev_info(dev, "bringup: power on (ovl runtime pm, ovlsys mutex prepared)\n");
	return 0;
}

void mtk_ovlsys_adaptor_power_off(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_mutex_unprepare(priv->mutex);
	pm_runtime_put(priv->ovl_dev);
	dev_info(dev, "bringup: power off\n");
}

int mtk_ovlsys_adaptor_clk_enable(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_clk_enable(priv->ovl_dev);
}

void mtk_ovlsys_adaptor_clk_disable(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_clk_disable(priv->ovl_dev);
}

void mtk_ovlsys_adaptor_config(struct device *dev, unsigned int w,
			       unsigned int h, unsigned int vrefresh,
			       unsigned int bpc, struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_config(priv->ovl_dev, w, h, vrefresh, bpc, cmdq_pkt);
}

void mtk_ovlsys_adaptor_start(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_start(priv->ovl_dev);
	mtk_mutex_enable(priv->mutex);
	dev_info(dev, "bringup: start (ovl enabled, ovlsys mutex enabled)\n");

	if (!priv->started_once) {
		priv->started_once = true;
		if (regdump)
			schedule_delayed_work(&priv->dump_work, msecs_to_jiffies(2000));
	}
}

void mtk_ovlsys_adaptor_stop(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_mutex_disable(priv->mutex);
	mtk_ovl_stop(priv->ovl_dev);
	dev_info(dev, "bringup: stop\n");
}

void mtk_ovlsys_adaptor_register_vblank_cb(struct device *dev,
					   void (*vblank_cb)(void *),
					   void *vblank_cb_data)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_register_vblank_cb(priv->ovl_dev, vblank_cb, vblank_cb_data);
}

void mtk_ovlsys_adaptor_unregister_vblank_cb(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_unregister_vblank_cb(priv->ovl_dev);
}

void mtk_ovlsys_adaptor_enable_vblank(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_enable_vblank(priv->ovl_dev);
}

void mtk_ovlsys_adaptor_disable_vblank(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_disable_vblank(priv->ovl_dev);
}

unsigned int mtk_ovlsys_adaptor_supported_rotations(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_supported_rotations(priv->ovl_dev);
}

unsigned int mtk_ovlsys_adaptor_layer_nr(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_layer_nr(priv->ovl_dev);
}

int mtk_ovlsys_adaptor_layer_check(struct device *dev, unsigned int idx,
				   struct mtk_plane_state *state)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_layer_check(priv->ovl_dev, idx, state);
}

void mtk_ovlsys_adaptor_layer_config(struct device *dev, unsigned int idx,
				     struct mtk_plane_state *state,
				     struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_ovl_layer_config(priv->ovl_dev, idx, state, cmdq_pkt);
}

struct device *mtk_ovlsys_adaptor_dma_dev_get(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return priv->ovl_dev;
}

u32 mtk_ovlsys_adaptor_get_blend_modes(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_get_blend_modes(priv->ovl_dev);
}

const u32 *mtk_ovlsys_adaptor_get_formats(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_get_formats(priv->ovl_dev);
}

size_t mtk_ovlsys_adaptor_get_num_formats(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_get_num_formats(priv->ovl_dev);
}

bool mtk_ovlsys_adaptor_is_afbc_supported(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	return mtk_ovl_is_afbc_supported(priv->ovl_dev);
}

void mtk_ovlsys_adaptor_connect(struct device *dev, struct device *mmsys_dev,
				unsigned int next)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	priv->next = next;
	dev_info(dev, "bringup: connect OVL0_2L -> DLO_ASYNC3 (ovlsys), DLI_ASYNC0 -> comp %u (dispsys)\n",
		 next);
	mtk_mmsys_ddp_connect(priv->ovlsys_dev, DDP_COMPONENT_OVL_2L0,
			      DDP_COMPONENT_OVLSYS_DLO_ASYNC3);
	if (lk_pq)
		dev_info(dev, "bringup: lk_pq: keeping the bootloader's DISPSYS PQ routing\n");
	else
		mtk_mmsys_ddp_connect(mmsys_dev, DDP_COMPONENT_DLI_ASYNC0, next);
}

void mtk_ovlsys_adaptor_disconnect(struct device *dev, struct device *mmsys_dev,
				   unsigned int next)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	if (!lk_pq)
		mtk_mmsys_ddp_disconnect(mmsys_dev, DDP_COMPONENT_DLI_ASYNC0, next);
	mtk_mmsys_ddp_disconnect(priv->ovlsys_dev, DDP_COMPONENT_OVL_2L0,
				 DDP_COMPONENT_OVLSYS_DLO_ASYNC3);
}

void mtk_ovlsys_adaptor_add_comp(struct device *dev, struct mtk_mutex *mutex)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	/* DISPSYS mutex (the CRTC's): the relay input of DISPSYS0 */
	mtk_mutex_add_comp(mutex, DDP_COMPONENT_DLI_ASYNC0);
	if (lk_pq) {
		mtk_mutex_update_mod_raw(mutex, 0, MT6897_LK_DISP_MUTEX_MOD0, true);
		mtk_mutex_update_mod_raw(mutex, 1, MT6897_LK_DISP_MUTEX_MOD1, true);
	}

	/* OVLSYS mutex: the engines, and the same SOF/EOF source as the CRTC */
	mtk_mutex_add_comp(priv->mutex, DDP_COMPONENT_OVL_2L0);
	mtk_mutex_add_comp(priv->mutex, DDP_COMPONENT_OVLSYS_DLO_ASYNC3);
	mtk_mutex_add_comp(priv->mutex, priv->next);
}

void mtk_ovlsys_adaptor_remove_comp(struct device *dev, struct mtk_mutex *mutex)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	mtk_mutex_remove_comp(priv->mutex, priv->next);
	mtk_mutex_remove_comp(priv->mutex, DDP_COMPONENT_OVLSYS_DLO_ASYNC3);
	mtk_mutex_remove_comp(priv->mutex, DDP_COMPONENT_OVL_2L0);
	if (lk_pq) {
		mtk_mutex_update_mod_raw(mutex, 1, MT6897_LK_DISP_MUTEX_MOD1, false);
		mtk_mutex_update_mod_raw(mutex, 0, MT6897_LK_DISP_MUTEX_MOD0, false);
	}
	mtk_mutex_remove_comp(mutex, DDP_COMPONENT_DLI_ASYNC0);
}

/*
 * First modeset only, after the DSI has left video mode: stop the bootloader's
 * OVLSYS side. The bootloader uses mutex 0 of the OVLSYS mutex block, which is
 * the one mtk_mutex_get() handed us. OVL1_2L/OVL2_2L, which the bootloader
 * also uses, are not mapped by this driver; once they are out of the mutex and
 * the crossbars they no longer receive SOF and stay idle.
 */
void mtk_ovlsys_adaptor_handoff(struct device *dev)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	dev_info(dev, "bringup: handoff: stopping bootloader OVLSYS mutex and OVL0_2L\n");
	mtk_mutex_disable(priv->mutex);
	mtk_ovl_stop(priv->ovl_dev);
}

/* --- component glue --- */

static int mtk_disp_ovlsys_bind(struct device *dev, struct device *master, void *data)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "bringup: bind: ovl %s, ovlsys %s, mutex %s, %u keep-on clocks\n",
		 dev_name(priv->ovl_dev), dev_name(priv->ovlsys_dev),
		 dev_name(priv->mutex_dev), priv->num_clks);

	/*
	 * Hold the clocks of the running pipeline before mediatek-drm removes
	 * simpledrm (which drops its references). No register is touched.
	 */
	ovlsys_clks_on(priv);

	/*
	 * No IOMMU: OVL fetches physical addresses. The OVL could address
	 * 34 bits (ADDR_MSB), but scan-out buffers are kept in the low 4 GiB
	 * like the bootloader's, where the default CMA area lives.
	 */
	ret = dma_set_mask_and_coherent(priv->ovl_dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(dev, "failed to set OVL DMA mask: %d\n", ret);

	priv->debugfs = debugfs_create_dir("mtk_mt6897_disp", NULL);
	debugfs_create_file("regdump", 0400, priv->debugfs, priv, &ovlsys_regdump_fops);

	return 0;
}

static void mtk_disp_ovlsys_unbind(struct device *dev, struct device *master, void *data)
{
	struct mtk_disp_ovlsys *priv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&priv->dump_work);
	debugfs_remove_recursive(priv->debugfs);
	priv->debugfs = NULL;
	ovlsys_clks_off(priv);
}

static const struct component_ops mtk_disp_ovlsys_comp_ops = {
	.bind = mtk_disp_ovlsys_bind,
	.unbind = mtk_disp_ovlsys_unbind,
};

static int mtk_disp_ovlsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_ovlsys *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->mmsys_dev = dev->platform_data;
	INIT_DELAYED_WORK(&priv->dump_work, ovlsys_dump_work);

	priv->ovl_dev = ovlsys_find_dev(dev, "mediatek,mt6897-disp-ovl");
	if (IS_ERR(priv->ovl_dev))
		return PTR_ERR(priv->ovl_dev);
	ret = devm_add_action_or_reset(dev, ovlsys_put_device, priv->ovl_dev);
	if (ret)
		return ret;

	priv->ovlsys_dev = ovlsys_find_dev(dev, "mediatek,mt6897-ovlsys0_config");
	if (IS_ERR(priv->ovlsys_dev))
		return PTR_ERR(priv->ovlsys_dev);
	ret = devm_add_action_or_reset(dev, ovlsys_put_device, priv->ovlsys_dev);
	if (ret)
		return ret;

	priv->mutex_dev = ovlsys_find_dev(dev, "mediatek,mt6897-ovlsys-mutex");
	if (IS_ERR(priv->mutex_dev))
		return PTR_ERR(priv->mutex_dev);
	ret = devm_add_action_or_reset(dev, ovlsys_put_device, priv->mutex_dev);
	if (ret)
		return ret;

	ret = ovlsys_clks_get(priv);
	if (ret)
		return ret;

	priv->mutex = mtk_mutex_get(priv->mutex_dev);
	if (IS_ERR(priv->mutex))
		return dev_err_probe(dev, PTR_ERR(priv->mutex), "failed to get ovlsys mutex\n");

	platform_set_drvdata(pdev, priv);

	ret = component_add(dev, &mtk_disp_ovlsys_comp_ops);
	if (ret) {
		mtk_mutex_put(priv->mutex);
		return dev_err_probe(dev, ret, "failed to add component\n");
	}

	dev_info(dev, "bringup: probed (no register access)\n");
	return 0;
}

static void mtk_disp_ovlsys_remove(struct platform_device *pdev)
{
	struct mtk_disp_ovlsys *priv = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &mtk_disp_ovlsys_comp_ops);
	mtk_mutex_put(priv->mutex);
}

struct platform_driver mtk_disp_ovlsys_driver = {
	.probe = mtk_disp_ovlsys_probe,
	.remove = mtk_disp_ovlsys_remove,
	.driver = {
		.name = "mediatek-disp-ovlsys",
	},
};
