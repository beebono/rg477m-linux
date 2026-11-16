// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6897 MIPI TX (D-PHY) PLL.
 *
 * MT6897 uses the MT6983-generation MIPI TX layout (see
 * phy-mtk-mipi-dsi-mt6895.c). The programming sequence follows downstream
 * mediatek_v2 mtk_mipi_tx_pll_prepare_mt6897() / _dsi_get_pcw_mt6897():
 * the feedback divider is fixed to 1 and DIV3 is never used, so
 *
 *   pcw = (rate / 2) * txdiv / 26 MHz   (8.24 fixed point in PLL_CON0)
 *
 * e.g. 1170 Mbps -> txdiv 2 (posdiv 1) -> pcw 45.0 -> PLL_CON0 0x2d000000,
 * which is what the bootloader leaves behind.
 *
 * Like downstream, an already running PLL with the requested settings is
 * left alone, so that taking over the bootloader's display does not glitch
 * the lane clock.
 */

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON			0x0004
#define MIPITX_VOLTAGE_SEL		0x0008
#define FLD_RG_DSI_PRD_REF_SEL			GENMASK(5, 0)
#define MIPITX_PRESERVED		0x000c
#define MIPITX_PLL_PWR			0x0028
#define AD_DSI_PLL_SDM_PWR_ON			BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN			BIT(1)
#define MIPITX_PLL_CON0			0x002c
#define MIPITX_PLL_CON1			0x0030
#define RG_DSI_PLL_EN				BIT(0)
#define FLD_RG_DSI_PLL_POSDIV			GENMASK(10, 8)
#define FLD_RG_DSI_PLL_FBSEL			BIT(13)
#define FLD_RG_DSI_PLL_DIV3_EN			BIT(28)
#define MIPITX_PLL_CON4			0x003c
#define MIPITX_SW_CTRL_CON4		0x0050
#define MIPITX_CK_CKMODE_EN		0x0320

static void mtk_mipi_tx_mt6897_div(unsigned int rate_mhz, unsigned int *txdiv,
				   unsigned int *txdiv0)
{
	if (rate_mhz >= 2000) {
		*txdiv = 1;
		*txdiv0 = 0;
	} else if (rate_mhz >= 1000) {
		*txdiv = 2;
		*txdiv0 = 1;
	} else if (rate_mhz >= 500) {
		*txdiv = 4;
		*txdiv0 = 2;
	} else if (rate_mhz >= 250) {
		*txdiv = 8;
		*txdiv0 = 3;
	} else {
		*txdiv = 16;
		*txdiv0 = 4;
	}
}

static u32 mtk_mipi_tx_mt6897_pcw(unsigned int rate_mhz, unsigned int txdiv)
{
	unsigned int r = (rate_mhz >> 1) * txdiv;
	unsigned int pcw = r / 26;
	unsigned int pf = r % 26;

	return ((pcw & 0xff) << 24) |
	       (((256 * pf / 26) & 0xff) << 16) |
	       (((256 * (256 * pf % 26) / 26) & 0xff) << 8) |
	       ((256 * (256 * (256 * pf % 26) % 26) / 26) & 0xff);
}

static int mtk_mipi_tx_pll_prepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int rate_mhz = mipi_tx->data_rate / 1000000;
	unsigned int txdiv, txdiv0;
	u32 pcw, con1;

	if (rate_mhz < 125) {
		dev_err(mipi_tx->dev, "data rate too low: %u MHz\n", rate_mhz);
		return -EINVAL;
	}

	mtk_mipi_tx_mt6897_div(rate_mhz, &txdiv, &txdiv0);
	pcw = mtk_mipi_tx_mt6897_pcw(rate_mhz, txdiv);

	/* BRINGUP(mt6897): PLL state before we touch it */
	con1 = readl(base + MIPITX_PLL_CON1);
	dev_info(mipi_tx->dev,
		 "bringup: pll prepare %u Mbps: want pcw 0x%08x posdiv %u; have PWR 0x%x CON0 0x%08x CON1 0x%08x\n",
		 rate_mhz, pcw, txdiv0, readl(base + MIPITX_PLL_PWR),
		 readl(base + MIPITX_PLL_CON0), con1);

	if ((con1 & RG_DSI_PLL_EN) &&
	    FIELD_GET(FLD_RG_DSI_PLL_POSDIV, con1) == txdiv0 &&
	    !(con1 & (FLD_RG_DSI_PLL_FBSEL | FLD_RG_DSI_PLL_DIV3_EN)) &&
	    readl(base + MIPITX_PLL_CON0) == pcw &&
	    (readl(base + MIPITX_PLL_PWR) & (AD_DSI_PLL_SDM_PWR_ON | AD_DSI_PLL_SDM_ISO_EN)) ==
	    AD_DSI_PLL_SDM_PWR_ON) {
		dev_info(mipi_tx->dev, "bringup: pll already running at %u Mbps, kept\n",
			 rate_mhz);
		return 0;
	}

	mtk_phy_update_field(base + MIPITX_VOLTAGE_SEL, FLD_RG_DSI_PRD_REF_SEL,
			     rate_mhz < 2500 ? 0x0 : 0x4);
	writel(rate_mhz < 2500 ? 0xffff00f0 : 0xffff0030, base + MIPITX_PRESERVED);
	writel(0x00ff12e0, base + MIPITX_PLL_CON4);

	/* BG_LPF_EN / BG_CORE_EN */
	writel(0x3fff0180, base + MIPITX_LANE_CON);
	usleep_range(500, 600);
	writel(0x3fff0080, base + MIPITX_LANE_CON);

	/* SDM power on, then release isolation */
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	usleep_range(30, 100);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_FBSEL);
	writel(pcw, base + MIPITX_PLL_CON0);
	mtk_phy_update_field(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_POSDIV, txdiv0);
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_DIV3_EN);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	usleep_range(50, 100);

	dev_info(mipi_tx->dev, "bringup: pll programmed: PWR 0x%x CON0 0x%08x CON1 0x%08x\n",
		 readl(base + MIPITX_PLL_PWR), readl(base + MIPITX_PLL_CON0),
		 readl(base + MIPITX_PLL_CON1));

	return 0;
}

static void mtk_mipi_tx_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	dev_info(mipi_tx->dev, "bringup: pll unprepare\n");

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	mtk_phy_clear_bits(base + MIPITX_SW_CTRL_CON4, BIT(0));
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	writel(0x3fff0180, base + MIPITX_LANE_CON);
	writel(0x3fff0100, base + MIPITX_LANE_CON);
}

static int mtk_mipi_tx_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	req->rate = clamp_val(req->rate, 125000000, 2500000000UL);

	return 0;
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.prepare = mtk_mipi_tx_pll_prepare,
	.unprepare = mtk_mipi_tx_pll_unprepare,
	.determine_rate = mtk_mipi_tx_pll_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);

	/*
	 * Downstream dphy lane config without lane swap: the clock lane runs
	 * in clock mode. The DISPSYS-side lane mapping (DSI_SEL_CONFIG_0,
	 * 0x00a43210) is the reset/bootloader value and is not touched.
	 */
	mtk_phy_set_bits(mipi_tx->regs + MIPITX_CK_CKMODE_EN, BIT(0));
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
}

const struct mtk_mipitx_data mt6897_mipitx_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};
