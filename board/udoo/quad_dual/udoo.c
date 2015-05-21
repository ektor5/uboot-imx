/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/sata.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <command.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                   \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define WDT_EN		IMX_GPIO_NR(5, 4)
#define WDT_TRG		IMX_GPIO_NR(3, 19)

#define MX6_PAD_GPIO_2__GPIO_1_2    IOMUX_PAD(0x0604, 0x0234, 5, 0x0000, 0, 0)
#define MX6_PAD_GPIO_4__GPIO_1_4    IOMUX_PAD(0x0608, 0x0238, 5, 0x0000, 0, 0)


int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	MX6_PAD_EIM_A24__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_D19__GPIO3_IO19,
};

static iomux_v3_cfg_t const lvds_pads[] = {
	MX6_PAD_GPIO_2__GPIO_1_2 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_4__GPIO_1_4 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int mx6_rgmii_rework(struct phy_device *phydev)
{
	/*
	 * Bug: Apparently uDoo does not works with Gigabit switches...
	 * Limiting speed to 10/100Mbps, and setting master mode, seems to
	 * be the only way to have a successfull PHY auto negotiation.
	 * How to fix: Understand why Linux kernel do not have this issue.
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, MII_CTRL1000, 0x1c00);

	/* control data pad skew - devaddr = 0x02, register = 0x04 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* rx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* tx data pad skew - devaddr = 0x02, register = 0x05 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
	/* gtx and rx clock pad skew - devaddr = 0x02, register = 0x08 */
	ksz9031_phy_extended_write(phydev, 0x02,
				   MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
				   MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03FF);
	return 0;
}

static iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* RGMII reset */
	MX6_PAD_EIM_D23__GPIO3_IO23		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* Ethernet power supply */
	MX6_PAD_EIM_EB3__GPIO2_IO31		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads2[] = {
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	udelay(20);
	gpio_direction_output(IMX_GPIO_NR(2, 31), 1); /* Power supply on */

	gpio_direction_output(IMX_GPIO_NR(3, 23), 0); /* assert PHY rst */

	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	udelay(1000);

	gpio_set_value(IMX_GPIO_NR(3, 23), 1); /* deassert PHY rst */

	/* Need 100ms delay to exit from reset. */
	udelay(1000 * 100);

	gpio_free(IMX_GPIO_NR(6, 24));
	gpio_free(IMX_GPIO_NR(6, 25));
	gpio_free(IMX_GPIO_NR(6, 27));
	gpio_free(IMX_GPIO_NR(6, 28));
	gpio_free(IMX_GPIO_NR(6, 29));

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

static void setup_iomux_wdog(void)
{
	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	gpio_direction_output(WDT_TRG, 0);
	gpio_direction_output(WDT_EN, 1);
	gpio_direction_input(WDT_TRG);
}

static struct fsl_esdhc_cfg usdhc_cfg = { USDHC3_BASE_ADDR };

int board_mmc_getcd(struct mmc *mmc)
{
	return 1; /* Always present */
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);

	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}
#endif
	return 0;
}

int board_mmc_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg.max_bus_width = 4;

	return fsl_esdhc_initialize(bis, &usdhc_cfg);
}

#if defined(CONFIG_VIDEO_IPUV3)

struct display_info_t {
        int     bus;
        int     addr;
        int     pixfmt;
        int     (*detect)(struct display_info_t const *dev);
        void    (*enable)(struct display_info_t const *dev);
        struct  fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
        struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
        return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
        enable_ipu_clock();
        imx_enable_hdmi_phy();
        imx_setup_hdmi();
}

static int enable_pll_video(u32 pll_div, u32 pll_num, u32 pll_denom)
{
        struct mxc_ccm_reg *imx_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	u32 reg = 0;
	ulong start;

	debug("pll5 div = %d, num = %d, denom = %d\n",
		pll_div, pll_num, pll_denom);

	/* Power up PLL5 video */
	writel(BM_ANADIG_PLL_VIDEO_POWERDOWN | BM_ANADIG_PLL_VIDEO_BYPASS |
		BM_ANADIG_PLL_VIDEO_DIV_SELECT | BM_ANADIG_PLL_VIDEO_TEST_DIV_SELECT,
		&imx_ccm->analog_pll_video_clr);

	/* Set div, num and denom */
	writel(BF_ANADIG_PLL_VIDEO_DIV_SELECT(pll_div) |
		BF_ANADIG_PLL_VIDEO_TEST_DIV_SELECT(0x2),
		&imx_ccm->analog_pll_video_set);

	writel(BF_ANADIG_PLL_VIDEO_NUM_A(pll_num),
		&imx_ccm->analog_pll_video_num);

	writel(BF_ANADIG_PLL_VIDEO_DENOM_B(pll_denom),
		&imx_ccm->analog_pll_video_denon);

	/* Wait PLL5 lock */
	start = get_timer(0);	/* Get current timestamp */

	do {
		reg = readl(&imx_ccm->analog_pll_video);
		if (reg & BM_ANADIG_PLL_VIDEO_LOCK) {
			/* Enable PLL out */
			writel(BM_ANADIG_PLL_VIDEO_ENABLE,
					&imx_ccm->analog_pll_video_set);
			return 0;
		}
	} while (get_timer(0) < (start + 10)); /* Wait 10ms */

	printf("Lock PLL5 timeout\n");
	return 1;

}

static int detect_lvds(struct display_info_t const *dev)
{
        return 0;
}

static void do_enable_lvds(struct display_info_t const *dev)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
    
        int reg;
        
        enable_ipu_clock();
        
	imx_iomux_v3_setup_multiple_pads(lvds_pads, ARRAY_SIZE(lvds_pads));
	gpio_direction_output(IMX_GPIO_NR(1, 2), 1); /* LVDS power On */
	gpio_direction_output(IMX_GPIO_NR(1, 4), 1); /* LVDS backlight On */
        
        /* Turn on LDB0,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);


        /* set LDB0 clk select to 001 (pll2) */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);
	reg |= (1 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);
        /* this will set the clock @352MHz -> 50.2857MHz */
        /* too much for 7" and too few for 15" */
        /* but enough to see something in both */
        
        /* LDB clock div by 7 */
	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

        /* derive ipu1_di0_clk_root clock from ldb_di0_clk */
	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

//  not using this anymore
//         
//         /* set registers for PLL5 clk */
//         u32 freq = 1000000000000 / dev->mode.pixclock;
//         u32 freq = 266000000;
// 	u32 hck = MXC_HCLK/1000;           // 24000000
// 	u32 min = hck * 27;
// 	u32 max = hck * 54;
// 	u32 temp, best = 0;
// 	u32 i, j, pred = 1, postd = 1;
// 	u32 pll_div, pll_num, pll_denom;
// 
//         for (i = 1; i <= 8; i++) {
// 		for (j = 1; j <= 8; j++) {
// 			temp = freq * i * j;
// 			if (temp > max || temp < min)
// 				continue;
// 
// 			if (best == 0 || temp < best) {
// 				best = temp;
// 				pred = i;
// 				postd = j;
// 			}
// 		}
// 	}
// 
//         pll_div = best / hck;
// 	pll_denom = 1000000;
// 	pll_num = (best - hck * pll_div) * pll_denom / hck;
// 
//         enable_pll_video(pll_div, pll_num, pll_denom);
//         
        return;
}

static struct display_info_t const displays[] = {{
        .bus    = -1,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = detect_hdmi,
        .enable = do_enable_hdmi,
        .mode   = {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1920,
		.yres           = 1080,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} },{
        .bus    = -1,
        .addr   = -1,
        .pixfmt = IPU_PIX_FMT_RGB666,
        .detect = detect_lvds,
        .enable = do_enable_lvds,
        .mode   = {
		// Rif. 800x480 Panel UMSH-8596MD-20T @33.36MHz
		// To activate write "setenv panel LDB-WVGA" or leave empty.
		.name           = "LDB-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 19886,      //adjusting for 55MHz
		.left_margin    = 56,
		.right_margin   = 50,
		.upper_margin   = 23,
		.lower_margin   = 20,
		.hsync_len      = 150,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} },  {
        .bus    = -1,
        .addr   = -1,
        .pixfmt = IPU_PIX_FMT_RGB666,
        .detect = detect_lvds,
        .enable = do_enable_lvds,
        .mode   = {
		// Rif. 1366x768 Panel CHIMEI M156B3-LA1 @76Mhz
		// To activate write "setenv panel LDB-WXGA".
		.name           = "LDB-WXGA",
		.refresh        = 59,
		.xres           = 1366,
		.yres           = 768,
		.pixclock       = 19886,      //adjusting for 55MHz
		.left_margin    = 93,
		.right_margin   = 33,
		.upper_margin   = 22,
		.lower_margin   = 7,
		.hsync_len      = 40,
		.vsync_len      = 4,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, 
};

int board_video_skip(void)
{
        int i;
        int ret;
        char const *panel = getenv("panel");

        if (!panel) {
            // AUTODETECT
                for (i = 0; i < ARRAY_SIZE(displays); i++) {
                        struct display_info_t const *dev = displays+i;
                        if (dev->detect(dev)) {
                                panel = dev->mode.name;
                                printf("auto-detected panel %s\n", panel);
                                break;
                        }
                }
                if (!panel) {
                        panel = displays[0].mode.name;
                        printf("No panel detected: default to %s\n", panel);
                        i = 0;
                }
        } else {
            // ENV PRESET
                for (i = 0; i < ARRAY_SIZE(displays); i++) {
                    // is a supported panel?
                        if (!strcmp(panel, displays[i].mode.name))
                                break;
                }
        }
        if (i < ARRAY_SIZE(displays)) {
            //found one, setting that
                ret = ipuv3_fb_init(&displays[i].mode, 0,
                                    displays[i].pixfmt);
                if (!ret) {
                        // enable
                        displays[i].enable(displays+i);
                        printf("Display: %s (%ux%u)\n", 
                               displays[i].mode.name,
                               displays[i].mode.xres,
                               displays[i].mode.yres);
                } else {
                        printf("LCD %s cannot be configured: %d\n",
                               displays[i].mode.name, ret);
                }
        } else {
            //wrong env
                printf("unsupported panel %s\n", panel);
                ret = -EINVAL;
        }
        return (0 != ret);
}


static void setup_display(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;
        
	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
		|IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
		|IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
		|IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
		|IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
		|IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
		|IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
		|IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
		|IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
		| (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
		<<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}

/*
 * Show device feature strings on current display
 * around uDOO Logo.
 */
void show_boot_messages(void) 
{
	int i;
	ulong cycles = 0;
	int repeatable;
	char *plotmsg_cmd[2];
#if defined(CONFIG_MX6DL)
	char *boot_messages[7] = {
"UDOO Board 2013",
"CPU Freescale i.MX6 DualLite 1GHz",
"dual ARMv7 Cortex-A9 core",
"1GB RAM DDR3",
"Vivante GC880 GPU",
"Atmel SAM3X8E ARM Cortex-M3 CPU",
"Arduino-compatible R3 1.0 pinout",
};
#else
	char *boot_messages[7] = {
"UDOO Board 2013",
"CPU Freescale i.MX6 Quad/Dual 1GHz",
"quad/dual ARMv7 Cortex-A9 core",
"1GB RAM DDR3",
"Vivante GC2000 / GC880",
"Atmel SAM3X8E ARM Cortex-M3 CPU",
"Arduino-compatible R3 1.0 pinout",
};
#endif

	for (i=0; i<7; i++) {
		plotmsg_cmd[0] = "plotmsg";
		plotmsg_cmd[1] = boot_messages[i];
		cmd_process(0, 2, plotmsg_cmd, &repeatable, &cycles);
	}
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
        setup_display();
#endif
	setup_iomux_wdog();
	setup_iomux_uart();

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: Udoo\n");

	return 0;
}
