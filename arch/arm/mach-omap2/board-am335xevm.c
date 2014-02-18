/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>
#include <plat/dma-33xx.h>

#include <media/soc_camera.h>
#include <media/mt9t112.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

#include "control.h"
/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* BBB PHY IDs */
#define BBB_PHY_ID		0x7c0f1
#define BBB_PHY_MASK		0xfffffffe

/* AM335X EVM Phy ID and Debug Registers */
#define AM335X_EVM_PHY_ID		0x4dd074
#define AM335X_EVM_PHY_MASK		0xfffffffe
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		BIT(8)

static int beaglebone_lcd_avdd_en;

#define AM33XX_CTRL_REGADDR(reg)				\
		AM33XX_L4_WK_IO_ADDRESS(AM33XX_SCM_BASE + (reg))

/* bit 3: 0 - enable, 1 - disable for pull enable */
#define AM33XX_PULL_DISA		(1 << 3)
#define AM33XX_PULL_ENBL		(0 << 3)

int selected_pad;
int pad_mux_value;

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static const struct display_panel bone_lcd_cape_disp_panel = {
	WVGA,
	16,
	16,
	COLOR_ACTIVE,
};

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    100
#define AM335X_PWM_PERIOD_NANO_SECONDS        (5000 * 10)

static struct platform_pwm_backlight_data am335x_backlight_data0 = {
	.pwm_id         = "ecap.0",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

static struct platform_pwm_backlight_data am335x_backlight_data2 = {
	.pwm_id         = "ecap.2",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

/* Beaglebone LCD7 Backlight platform data */
#define BB_LCD7_PWM_DEVICE_ID   "ehrpwm.1"

static struct platform_pwm_backlight_data lcd7_bl_pdata = {
	.pwm_id         = BB_LCD7_PWM_DEVICE_ID,
	.ch             = 0,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

static struct lcd_ctrl_config bone_lcd_cape_cfg = {
	&bone_lcd_cape_disp_panel,
	.ac_bias                = 255,
	.ac_bias_intrpt         = 0,
	.dma_burst_sz           = 16,
	.bpp                    = 16,
	.fdd                    = 0x80,
	.tft_alt_mode           = 0,
	.stn_565_mode           = 0,
	.mono_8bit_mode         = 0,
	.invert_line_clock      = 1,
	.invert_frm_clock       = 1,
	.sync_edge              = 0,
	.sync_ctrl              = 1,
	.raster_order           = 0,
};

struct da8xx_lcdc_platform_data TFC_S9700RTWV35TR_01B_pdata = {
	.manu_name		= "ThreeFive",
	.controller_data	= &lcd_cfg,
	.type			= "TFC_S9700RTWV35TR_01B",
};

struct da8xx_lcdc_platform_data TFC_S9700RTWV35TR_01B_bone_lcd_cape_pdata = {
	.manu_name		= "ThreeFive",
	.controller_data	= &bone_lcd_cape_cfg,
	.type			= "TFC_S9700RTWV35TR_01B",
};

struct da8xx_lcdc_platform_data Sharp_LCD035Q3DG01_pdata = {
	.manu_name		= "Sharp",
	.controller_data	= &lcd_cfg,
	.type			= "Sharp_LCD035Q3DG01",
};

struct da8xx_lcdc_platform_data  NHD_480272MF_ATXI_pdata = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

struct da8xx_lcdc_platform_data CDTech_S035Q01_pdata = {
	.manu_name		= "BBToys",
	.controller_data	= &bone_lcd_cape_cfg,
	.type			= "CDTech_S035Q01",
};

struct da8xx_lcdc_platform_data  NHD_480272MF_ATXI_bone_lcd_cape_pdata = {
	.manu_name              = "BBToys",
	.controller_data        = &bone_lcd_cape_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

#include "common.h"

#include <linux/lis3lv02d.h>

static const struct display_panel dvi_panel = {
	WVGA,
	16,
	16,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config dvi_cfg = {
	&dvi_panel,
	.ac_bias    = 255,
	.ac_bias_intrpt    = 0,
	.dma_burst_sz    = 16,
	.bpp      = 16,
	.fdd      = 0x80,
	.tft_alt_mode    = 0,
	.stn_565_mode    = 0,
	.mono_8bit_mode    = 0,
	.invert_line_clock  = 1,
	.invert_frm_clock  = 1,
	.sync_edge    = 0,
	.sync_ctrl    = 1,
	.raster_order    = 0,
};

struct da8xx_lcdc_platform_data dvi_pdata = {
	.manu_name    = "BBToys",
	.controller_data  = &dvi_cfg,
	.type      = "1024x768@60",
};

struct da8xx_lcdc_platform_data hdmi_pdata = {
	.manu_name    = "NXP HDMI",
	.controller_data  = &dvi_cfg,
	.type      = "nxp-1280x720@60",
};

/* Touchscreen Controller Data for AM335xEVM */
/* Calibrated on AM335xEVM Rev. 1.1A and 1.2A */
/* The values have to be fine tuned for other revisions, if requred */
static struct tsc_data am335xevm_touchscreen_data = {
	.wires = 4,
	.x = {
		.min = 0xCB,
		.max = 0xF9B,
		.inverted = 1,
	},
	.y = {
		.min = 0xC8,
		.max = 0xE93,
		.inverted = 1,
	},
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct adc_data am335x_adc_data = {
	.adc_channels = 4,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init = &am335xevm_touchscreen_data,
	.adc_init = &am335x_adc_data,
};

/* Touchscreen Controller Data for Beaglebone Cape */
/* Calibrated on Beaglebone LCD7 Cape Rev. A1 and A2 */
/* The values have to be fine tuned for other revisions, if requred */
static struct tsc_data beaglebone_touchscreen_data = {
	.wires = 4,
	.x = {
		.min = 0x64,
		.max = 0xF86,
		.inverted = 0,
	},
	.y = {
		.min = 0x10C,
		.max = 0xEBD,
		.inverted = 0,
	},
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct mfd_tscadc_board beaglebone_tscadc = {
	.tsc_init = &beaglebone_touchscreen_data,
};

static u8 am335x_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,	RX_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static u8 am335x_iis_serializer_direction0[] = {
    TX_MODE,  RX_MODE,  INACTIVE_MODE,    INACTIVE_MODE,
    INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
    INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
    INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
};
static struct snd_platform_data am335x_evm_snd_data1 = {
	.tx_dma_offset	= 0x46400000,	/* McASP1 */
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.rxnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};

static struct snd_platform_data am335x_evm_snd_data0 = {
    .tx_dma_offset  = 0x46000000,   /* McASP0 */
    .rx_dma_offset  = 0x46000000,
    .op_mode    = DAVINCI_MCASP_IIS_MODE,
    .num_serializer = ARRAY_SIZE(am335x_iis_serializer_direction0),
    .tdm_slots  = 2,
    .serial_dir = am335x_iis_serializer_direction0,
    .asp_chan_q = EVENTQ_2,
    .version    = MCASP_VERSION_3,
    .txnumevt   = 1,
    .rxnumevt   = 1,
    .sync_mode  = 1,
};

static u8 am335x_evm_sk_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data am335x_evm_sk_snd_data1 = {
	.tx_dma_offset	= 0x46400000,	/* McASP1 */
	/*.rx_dma_offset	= 0x46400000,*/
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_evm_sk_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_evm_sk_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};

static struct snd_platform_data am335x_evm_sk_snd_data0 = {
    .tx_dma_offset  = 0x46000000,   /* McASP1 */
    /*.rx_dma_offset    = 0x46400000,*/
    .op_mode    = DAVINCI_MCASP_IIS_MODE,
    .num_serializer = ARRAY_SIZE(am335x_evm_sk_iis_serializer_direction1),
    .tdm_slots  = 2,
    .serial_dir = am335x_evm_sk_iis_serializer_direction1,
    .asp_chan_q = EVENTQ_2,
    .version    = MCASP_VERSION_3,
    .txnumevt   = 1,
};

static struct snd_platform_data bone_black_snd_data0 = {
	.tx_dma_offset	= 0x46000000,	/* McASP0*/
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 6),
		.gpio_wp        = GPIO_TO_PIN(3, 18),
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

/*
* If the device is required on both baseboard & daughter board (ex i2c),
* specify DEV_ON_BASEBOARD
*/
#define DEV_ON_BASEBOARD	0
#define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define	CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define	CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define	CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define	CPLD_CFG_REG	0x10 /* Configuration Register */

static struct i2c_client *cpld_client;
static u32 am335x_evm_id;
static struct omap_board_config_kernel am335x_evm_config[] __initdata = {
};

/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				Example "A33515BB" = "AM335x 15x15 Base Board"
*
*  Version		4	Hardware version code for board	in ASCII.
*				"1.0A" = rev.01.0A
*
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is WWYY4P16nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*
*  Configuration option	32	Codes(TBD) to show the configuration
*				setup on this board.
*
*  Available		32720	Available space for other non-volatile data.
*/
struct am335x_evm_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};

/*
* EVM Config held in daughter board eeprom device.
*
* Header Format
*
*  Name			Size		Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A335GPBD" = "AM335x
*				General Purpose Daughterboard"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is: WWYY4P13nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*  Configuration Option	32	Codes to show the configuration
*				setup on this board.
*  CPLD Version	8		CPLD code version for board in ASCII
*				"CPLD1.0A" = rev. 01.0A of the CPLD
*  Available	32700		Available space for other non-volatile
*				codes/data
*/

struct am335x_eeprom_config1 {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
	u8	cpld_ver[8];
};

/*
*Beaglebone cape EEPROM config
*
*-------------------------------------------------------------------
*Name		offset	size 	contents
*--------------------------------------------------------------------

*Header 	0 	4 	0xAA, 0x55, 0x33, 0xEE
*EEPROM Format
*
*Revision	4 	2 	Revision number of the overall format
*				of this EEPROM in ASCII =A0
*
*Board Name 	6 	32 	Name of board in ASCII
*
*Version 	38 	4 	Hardware version code for board in ASCII
*
*Manufacturer 	42 	16 	ASCII name of the manufacturer
*
*Part Number 	60 	16 	ASCII Characters for the part number
*
*Number of Pins 74 	2 	Number of pins used by the daughter board
*
*Serial Number	76	12	Serial number of the board. This is a 12
*				character string which is:
*				WWYY4P13nnnn where:
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*/

struct bone_cape_eeprom_config {
	u8	header[4];
	u8 	revision[2];
	u8	board_name[32];
	u8	version[4];
	u8	manufacturer[16];
	u8	part_no[16];
	u8 	no_of_pins[2];
	u8 	serial_no[12];
};

static struct am335x_evm_eeprom_config config;
static struct am335x_eeprom_config1 config1;
static struct bone_cape_eeprom_config cape_eeprom_config;

static bool daughter_brd_detected;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

static int am33xx_evmid = -EINVAL;

/*
* am335x_evm_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);

/* current profile if exists else PROFILE_0 on error */
static u32 am335x_get_profile_selection(void)
{
	int val = 0;

	if (!cpld_client)
		/* error checking is not done in func's calling this routine.
		so return profile 0 on error */
		return 0;

	val = i2c_smbus_read_word_data(cpld_client, CPLD_CFG_REG);
	if (val < 0)
		return 0;	/* default to Profile 0 on Error */
	else
		return val & 0x7;
}

static struct pinmux_config haptics_pin_mux[] = {
	{"gpmc_ad9.ehrpwm2B",		OMAP_MUX_MODE4 |
		AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for LCD7 Cape. LCDC pinmux already being set */
static struct pinmux_config lcd_cape_pin_mux[] = {
	{"gpmc_a2.ehrpwm1A", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};


/* Module pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"gpmc_ad8.lcd_data16",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for HDMI board */
static struct pinmux_config hdmi_pin_mux[] = {
	{"lcd_data0.lcd_data0",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",      OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0
		| AM33XX_PIN_OUTPUT}, /*DVIEN*/
	{NULL, 0},
};

/* Module pin mux for DVI board */
static struct pinmux_config dvi_pin_mux[] = {
	{"lcd_data0.lcd_data0",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",  OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
		| AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",      OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0
		| AM33XX_PIN_OUTPUT}, /*DVIEN*/
	{"gpmc_a2.rgmii2_td3", OMAP_MUX_MODE7
		| AM33XX_PIN_OUTPUT}, /* USR0 LED*/
	{"gpmc_a3.rgmii2_td2", OMAP_MUX_MODE7
		| AM33XX_PIN_OUTPUT}, /* USR1 LED*/
	{"gpmc_ad7.gpmc_ad7", OMAP_MUX_MODE7
		| AM33XX_PIN_OUTPUT}, /* DVI PDn */
	{NULL, 0},
};

/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for SPI fash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for SPI flash */
static struct pinmux_config spi1_pin_mux[] = {
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
		| AM33XX_INPUT_EN},
	{"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
		| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
		| AM33XX_INPUT_EN},
	{"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
		| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for rgmii1 */
static struct pinmux_config rgmii1_pin_mux[] = {
	{"mii1_txen.rgmii1_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.rgmii1_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.rgmii1_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.rgmii1_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.rgmii1_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.rgmii1_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.rgmii1_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxclk.rgmii1_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.rgmii1_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.rgmii1_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.rgmii1_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.rgmii1_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rgmii2 */
static struct pinmux_config rgmii2_pin_mux[] = {
	{"gpmc_a0.rgmii2_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a1.rgmii2_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.rgmii2_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a3.rgmii2_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.rgmii2_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.rgmii2_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.rgmii2_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a7.rgmii2_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.rgmii2_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.rgmii2_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.rgmii2_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.rgmii2_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mii1 */
static struct pinmux_config mii1_pin_mux[] = {
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.mii1_rxdv", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.mii1_txd3", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.mii1_txd2", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.mii1_txclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxclk.mii1_rxclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.mii1_rxd3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.mii1_rxd2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config i2c1_pin_mux[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Pin mux for GPMC bus */
static struct pinmux_config gpmc_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad8.gpmc_ad8",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad9.gpmc_ad9",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad10.gpmc_ad10",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad11.gpmc_ad11",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad12.gpmc_ad12",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad13.gpmc_ad13",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad14.gpmc_ad14",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad15.gpmc_ad15",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.gpmc_csn1",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_clk.gpmc_clk",	 OMAP_MUX_MODE0 | AM33XX_PIN_INPUT},
	{"ecap0_in_pwm0_out.xdma_event_intr2", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT}, // DMAREQ
	{NULL, 0},
};

static struct pinmux_config camera_cape_pin_mux[] = {
	{"spi0_d1.gpio0_4",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },		// QL CSSP and Camera Sensor Reset
	{"spi0_cs0.gpio0_5",   OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP },	// 1V8 and 2V8 Power Enable
	{"gpmc_wait0.gpio0_30", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},		// Sensor orientation detect: low -> frontfacing, high -> backfacing
	{NULL, 0},
};

static struct pinmux_config i2c2_pin_mux[] = {
	{"uart1_ctsn.i2c2_sda",    OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"uart1_rtsn.i2c2_scl",   OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mcasp1 */
static struct pinmux_config mcasp1_pin_mux[] = {
	{"mii1_crs.mcasp1_aclkx", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mcasp1_fsx", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_col.mcasp1_axr2", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.mcasp1_axr3", OMAP_MUX_MODE4 |
						AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for mcasp1 */
static struct pinmux_config mcasp0_pin_mux[] = {
    {"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 |AM33XX_PIN_INPUT_PULLDOWN },
    {"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN },
    {"mcasp0_axr0.mcasp0_axr0", OMAP_MUX_MODE0 |AM33XX_PIN_OUTPUT_PULLUP },
    {"mcasp0_axr1.mcasp0_axr1", OMAP_MUX_MODE0 |AM33XX_PIN_INPUT_PULLUP
                        },
    {NULL, 0},
};

/* Module pin mux for BeagleBone Black mcasp0 and hdmi */
static struct pinmux_config bone_black_mcasp0_pin_mux[] = {
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 |AM33XX_PIN_OUTPUT},
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	/* Same pin is used for rx, but for hdmi we don't need rx */
	{"mcasp0_ahclkx.mcasp0_ahclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_common_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc0_wp_only_pin_mux[] = {
	{"mcasp0_aclkr.gpio3_18", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc0_cd_only_pin_mux[] = {
	{"spi0_cs1.gpio0_6",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc1 */
static struct pinmux_config mmc1_common_pin_mux[] = {
	{"gpmc_ad3.mmc1_dat3",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.mmc1_dat2",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.mmc1_dat1",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad0.mmc1_dat0",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc1_dat4_7_pin_mux[] = {
	{"gpmc_ad7.mmc1_dat7",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.mmc1_dat6",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.mmc1_dat5",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.mmc1_dat4",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc1_wp_only_pin_mux[] = {
	{"gpmc_csn0.gpio1_29",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc1_cd_only_pin_mux[] = {
	{"gpmc_advn_ale.gpio2_2", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for uart3 */
static struct pinmux_config uart3_pin_mux[] = {
	{"spi0_cs1.uart3_rxd", AM33XX_PIN_INPUT_PULLUP},
	{"ecap0_in_pwm0_out.uart3_txd", AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config d_can_gp_pin_mux[] = {
	{"uart0_ctsn.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart0_rtsn.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config d_can_ia_pin_mux[] = {
	{"uart0_rxd.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart0_txd.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for uart2 */
static struct pinmux_config uart2_pin_mux[] = {
	{"spi0_sclk.uart2_rxd", OMAP_MUX_MODE1 | AM33XX_SLEWCTRL_SLOW |
						AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d0.uart2_txd", OMAP_MUX_MODE1 | AM33XX_PULL_UP |
						AM33XX_PULL_DISA |
						AM33XX_SLEWCTRL_SLOW},
	{NULL, 0},
};

/* pinmux for gpio based key */
static struct pinmux_config gpio_keys_pin_mux[] = {
	{"gpmc_wait0.gpio0_30", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_oen_ren.gpio2_3", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_advn_ale.gpio2_2", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ben0_cle.gpio2_5", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* pinmux for led device */
static struct pinmux_config gpio_led_mux[] = {
	{"gpmc_ad4.gpio1_4", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad5.gpio1_5", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad6.gpio1_6", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad7.gpio1_7", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

static struct pinmux_config gpio_ddr_vtt_enb_pin_mux[] = {
	{"ecap0_in_pwm0_out.gpio0_7", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

/* Matrix GPIO Keypad Support for profile-0 only: TODO */

/* pinmux for keypad device */
static struct pinmux_config matrix_keypad_pin_mux[] = {
	{"gpmc_a5.gpio1_21",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.gpio1_22",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"gpmc_a9.gpio1_25",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a10.gpio1_26", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a11.gpio1_27", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Keys mapping */
static const uint32_t am335x_evm_matrix_keys[] = {
	KEY(0, 0, KEY_MENU),
	KEY(1, 0, KEY_BACK),
	KEY(2, 0, KEY_LEFT),

	KEY(0, 1, KEY_RIGHT),
	KEY(1, 1, KEY_POWER),
	KEY(2, 1, KEY_DOWN),
};

const struct matrix_keymap_data am335x_evm_keymap_data = {
	.keymap      = am335x_evm_matrix_keys,
	.keymap_size = ARRAY_SIZE(am335x_evm_matrix_keys),
};

static const unsigned int am335x_evm_keypad_row_gpios[] = {
	GPIO_TO_PIN(1, 25), GPIO_TO_PIN(1, 26), GPIO_TO_PIN(1, 27)
};

static const unsigned int am335x_evm_keypad_col_gpios[] = {
	GPIO_TO_PIN(1, 21), GPIO_TO_PIN(1, 22)
};

static struct matrix_keypad_platform_data am335x_evm_keypad_platform_data = {
	.keymap_data       = &am335x_evm_keymap_data,
	.row_gpios         = am335x_evm_keypad_row_gpios,
	.num_row_gpios     = ARRAY_SIZE(am335x_evm_keypad_row_gpios),
	.col_gpios         = am335x_evm_keypad_col_gpios,
	.num_col_gpios     = ARRAY_SIZE(am335x_evm_keypad_col_gpios),
	.active_low        = false,
	.debounce_ms       = 5,
	.col_scan_delay_us = 2,
};

static struct platform_device am335x_evm_keyboard = {
	.name  = "matrix-keypad",
	.id    = -1,
	.dev   = {
		.platform_data = &am335x_evm_keypad_platform_data,
	},
};

static void matrix_keypad_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(matrix_keypad_pin_mux);
	err = platform_device_register(&am335x_evm_keyboard);
	if (err) {
		pr_err("failed to register matrix keypad (2x3) device\n");
	}
}


/* pinmux for keypad device */
static struct pinmux_config volume_keys_pin_mux[] = {
	{"spi0_sclk.gpio0_2",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"spi0_d0.gpio0_3",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Configure GPIOs for Volume Keys */
static struct gpio_keys_button am335x_evm_volume_gpio_buttons[] = {
	{
		.code                   = KEY_VOLUMEUP,
		.gpio                   = GPIO_TO_PIN(0, 2),
		.active_low             = true,
		.desc                   = "volume-up",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_VOLUMEDOWN,
		.gpio                   = GPIO_TO_PIN(0, 3),
		.active_low             = true,
		.desc                   = "volume-down",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data am335x_evm_volume_gpio_key_info = {
	.buttons        = am335x_evm_volume_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(am335x_evm_volume_gpio_buttons),
};

static struct platform_device am335x_evm_volume_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_volume_gpio_key_info,
	},
};

static void volume_keys_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(volume_keys_pin_mux);
	err = platform_device_register(&am335x_evm_volume_keys);
	if (err)
		pr_err("failed to register matrix keypad (2x3) device\n");
}

/* Pinmux for Beaglebone LCD cape keypad device */
static struct pinmux_config lcd_cape_keys_pin_mux[] = {
	{"gpmc_a0.gpio1_16",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a1.gpio1_17",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a3.gpio1_19",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_axr0.gpio3_16",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_fsr.gpio3_19",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Configure GPIOs for Beaglebone LCD cape keys */
static struct gpio_keys_button lcd_cape_gpio_keys[] = {
	{
		.code                   = KEY_LEFT,
		.gpio                   = GPIO_TO_PIN(1, 16),
		.active_low             = true,
		.desc                   = "left",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_RIGHT,
		.gpio                   = GPIO_TO_PIN(1, 17),
		.active_low             = true,
		.desc                   = "right",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_UP,
		.gpio                   = GPIO_TO_PIN(1, 19),
		.active_low             = true,
		.desc                   = "up",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_DOWN,
		.gpio                   = GPIO_TO_PIN(3, 16),
		.active_low             = true,
		.desc                   = "down",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_ENTER,
		.gpio                   = GPIO_TO_PIN(3, 19),
		.active_low             = true,
		.desc                   = "enter",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data lcd_cape_gpio_key_info = {
	.buttons        = lcd_cape_gpio_keys,
	.nbuttons       = ARRAY_SIZE(lcd_cape_gpio_keys),
};

static struct platform_device lcd_cape_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &lcd_cape_gpio_key_info,
	},
};

static void lcd_cape_keys_init(int evm_id, int profile)
{
	int err;
	setup_pin_mux(lcd_cape_keys_pin_mux);
	err = platform_device_register(&lcd_cape_keys);
	if (err)
		pr_err("Failed to register Keypad for Beaglebone LCD cape\n");
}

/* Pinmux for Beaglebone LCD7A3 cape keypad device */
static struct pinmux_config lcd7a3_cape_keys_pin_mux[] = {
	{"gpmc_a0.gpio1_16",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a1.gpio1_17",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a3.gpio1_19",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_axr0.gpio3_16",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"sdpi0_d0.gpio0_3",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Configure GPIOs for Beaglebone LCD7A3 cape keys */
static struct gpio_keys_button lcd7a3_cape_gpio_keys[] = {
	{
		.code                   = KEY_LEFT,
		.gpio                   = GPIO_TO_PIN(1, 16),
		.active_low             = true,
		.desc                   = "left",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_RIGHT,
		.gpio                   = GPIO_TO_PIN(1, 17),
		.active_low             = true,
		.desc                   = "right",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_UP,
		.gpio                   = GPIO_TO_PIN(1, 19),
		.active_low             = true,
		.desc                   = "up",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_DOWN,
		.gpio                   = GPIO_TO_PIN(3, 16),
		.active_low             = true,
		.desc                   = "down",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_ENTER,
		.gpio                   = GPIO_TO_PIN(0, 3),
		.active_low             = true,
		.desc                   = "enter",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data lcd7a3_cape_gpio_key_info = {
	.buttons        = lcd7a3_cape_gpio_keys,
	.nbuttons       = ARRAY_SIZE(lcd7a3_cape_gpio_keys),
};

static struct platform_device lcd7a3_cape_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &lcd7a3_cape_gpio_key_info,
	},
};

static void lcd7a3_cape_keys_init(int evm_id, int profile)
{
	int err;
	setup_pin_mux(lcd7a3_cape_keys_pin_mux);
	err = platform_device_register(&lcd7a3_cape_keys);
	if (err)
		pr_err("Failed to register Keypad for Beaglebone LCD cape\n");
}

/* Pinmux for Beaglebone LCD4 cape keypad device */
static struct pinmux_config lcd4_cape_keys_pin_mux[] = {
	{"gpmc_a0.gpio1_16",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a1.gpio1_17",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a3.gpio1_19",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_axr0.gpio3_16",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"uart1_txd.gpio0_15",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Configure GPIOs for Beaglebone LCD4 cape keys */
static struct gpio_keys_button lcd4_cape_gpio_keys[] = {
	{
		.code                   = KEY_LEFT,
		.gpio                   = GPIO_TO_PIN(1, 16),
		.active_low             = true,
		.desc                   = "left",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_RIGHT,
		.gpio                   = GPIO_TO_PIN(1, 17),
		.active_low             = true,
		.desc                   = "right",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_UP,
		.gpio                   = GPIO_TO_PIN(1, 19),
		.active_low             = true,
		.desc                   = "up",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_DOWN,
		.gpio                   = GPIO_TO_PIN(3, 16),
		.active_low             = true,
		.desc                   = "down",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_ENTER,
		.gpio                   = GPIO_TO_PIN(0, 15),
		.active_low             = true,
		.desc                   = "enter",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data lcd4_cape_gpio_key_info = {
	.buttons        = lcd4_cape_gpio_keys,
	.nbuttons       = ARRAY_SIZE(lcd4_cape_gpio_keys),
};

static struct platform_device lcd4_cape_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &lcd4_cape_gpio_key_info,
	},
};

static void lcd4_cape_keys_init(int evm_id, int profile)
{
	int err;
	setup_pin_mux(lcd4_cape_keys_pin_mux);
	err = platform_device_register(&lcd4_cape_keys);
	if (err)
		pr_err("Failed to register Keypad for Beaglebone LCD cape\n");
}

/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}


/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for profibus */
static struct pinmux_config profibus_pin_mux[] = {
	{"uart1_rxd.pr1_uart0_rxd_mux1", OMAP_MUX_MODE5 | AM33XX_PIN_INPUT},
	{"uart1_txd.pr1_uart0_txd_mux1", OMAP_MUX_MODE5 | AM33XX_PIN_OUTPUT},
	{"mcasp0_fsr.pr1_pru0_pru_r30_5", OMAP_MUX_MODE5 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for eCAP0 */
static struct pinmux_config ecap0_pin_mux[] = {
	{"ecap0_in_pwm0_out.ecap0_in_pwm0_out",
		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Module pin mux for eCAP */
static struct pinmux_config ecap2_pin_mux[] = {
	{"mcasp0_ahclkr.ecap2_in_pwm2_out", AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#define AM335XEVM_WLAN_PMENA_GPIO	GPIO_TO_PIN(1, 30)
#define AM335XEVM_WLAN_IRQ_GPIO		GPIO_TO_PIN(3, 17)
#define AM335XEVM_SK_WLAN_IRQ_GPIO      GPIO_TO_PIN(0, 31)

struct wl12xx_platform_data am335xevm_wlan_data = {
	.irq = OMAP_GPIO_IRQ(AM335XEVM_WLAN_IRQ_GPIO),
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
        .board_ref_clock = WL12XX_REFCLOCK_38,
        .board_tcxo_clock = WL12XX_TCXOCLOCK_26,
#else
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
#endif
	.bt_enable_gpio = GPIO_TO_PIN(3, 21),
	.wlan_enable_gpio = GPIO_TO_PIN(1, 16),
};

/* Module pin mux for wlan and bluetooth */
static struct pinmux_config mmc2_wl12xx_pin_mux[] = {
	{"gpmc_a1.mmc2_dat0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a2.mmc2_dat1", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a3.mmc2_dat2", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ben1.mmc2_dat3", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn3.mmc2_cmd", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_clk.mmc2_clk", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config uart1_wl12xx_pin_mux[] = {
	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT},
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config wl12xx_pin_mux[] = {
	{"gpmc_a0.gpio1_16", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkr.gpio3_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
 };

static struct pinmux_config wl12xx_pin_mux_sk[] = {
	{"gpmc_wpn.gpio0_31", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_csn0.gpio1_29", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static int ehrpwm_backlight_enable;
static bool backlight_enable;

static void enable_ecap0(int evm_id, int profile)
{
	backlight_enable = true;
	setup_pin_mux(ecap0_pin_mux);
}

static void enable_ecap2(int evm_id, int profile)
{
	backlight_enable = true;
	setup_pin_mux(ecap2_pin_mux);
}

/* Setup pwm-backlight */
static struct platform_device am335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev		= {
		.platform_data = &am335x_backlight_data0,
	},
};

static struct pwmss_platform_data  pwm_pdata[3] = {
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
};

static int __init backlight_init(void)
{
	int status = 0;

	if (backlight_enable) {
		int ecap_index = 0;

		switch (am335x_evm_get_id()) {
		case GEN_PURP_EVM:
		case GEN_PURP_DDR3_EVM:
			ecap_index = 0;
			break;
		case EVM_SK:
			/*
			 * Invert polarity of PWM wave from ECAP to handle
			 * backlight intensity to pwm brightness
			 */
			ecap_index = 2;
			pwm_pdata[ecap_index].chan_attrib[0].inverse_pol = true;
			am335x_backlight.dev.platform_data =
				&am335x_backlight_data2;
			break;
		default:
			pr_err("%s: Error on attempting to enable backlight,"
				" not supported\n", __func__);
			return -EINVAL;
		}

		am33xx_register_ecap(ecap_index, &pwm_pdata[ecap_index]);
		platform_device_register(&am335x_backlight);
	}
	return status;
}
late_initcall(backlight_init);

static void enable_ehrpwm1(int evm_id, int profile)
{
	am33xx_register_ehrpwm(1, &pwm_pdata[1]);
	ehrpwm_backlight_enable = true;
}

/* Setup pwm-backlight for LCD7 Cape */
static struct platform_device bone_lcd7_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev            = {
		.platform_data  = &lcd7_bl_pdata,
	}
};

static int __init ehrpwm1_init(void)
{
	int status = 0;
	if (ehrpwm_backlight_enable) {
		platform_device_register(&bone_lcd7_backlight);
	}
	return status;
}
late_initcall(ehrpwm1_init);

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}


static void lcdc_init(int evm_id, int profile)
{
	struct da8xx_lcdc_platform_data *lcdc_pdata;
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}
	switch (evm_id) {
	case GEN_PURP_EVM:
	case GEN_PURP_DDR3_EVM:
		lcdc_pdata = &TFC_S9700RTWV35TR_01B_pdata;
		break;
	case EVM_SK:
		lcdc_pdata = &NHD_480272MF_ATXI_pdata;
		break;
	default:
		pr_err("LCDC not supported on this evm (%d)\n",evm_id);
		return;
	}

	lcdc_pdata->get_context_loss_count = omap_pm_get_dev_context_loss_count;

	if (am33xx_register_lcdc(lcdc_pdata))
		pr_info("Failed to register LCDC device\n");

	return;
}

static void bone_lcd7_lcdc_init(int evm_id, int profile)
{

	pr_info("IN : %s \n", __FUNCTION__);
	setup_pin_mux(lcd_cape_pin_mux);
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	gpio_request(beaglebone_lcd_avdd_en, "BONE_LCD_AVDD_EN");
	gpio_direction_output(beaglebone_lcd_avdd_en, 1);

	if (am33xx_register_lcdc(&TFC_S9700RTWV35TR_01B_bone_lcd_cape_pdata))
		pr_info("Failed to register LCDC device\n");


	pr_info("Setup LCD display\n");
	return;
}

static void vnc_lcdc_init(int evm_id, int profile)
{
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}
	if (am33xx_register_lcdc(&Sharp_LCD035Q3DG01_pdata))
		pr_info("Failed to register LCDC device\n");
	return;
}

static void bone_lcd3_lcdc_init(int evm_id, int profile)
{
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(16000000)) {
		pr_info("Failed to set pixclock to 16000000, not attempting to"
				"register LCD cape\n");
		return;
	}

	if (am33xx_register_lcdc(&CDTech_S035Q01_pdata))
		pr_info("Failed to register Beagleboardtoys 3.5\" LCD cape device\n");

	return;
}

static void bone_lcd4_lcdc_init(int evm_id, int profile)
{
	setup_pin_mux(lcdc_pin_mux);
	setup_pin_mux(lcd_cape_pin_mux);

	if (conf_disp_pll(18000000)) {
		pr_info("Failed to set pixclock to 18000000, not attempting to"
				"register LCD cape\n");
		return;
	}

	gpio_request(beaglebone_lcd_avdd_en, "BONE_LCD_AVDD_EN");
	gpio_direction_output(beaglebone_lcd_avdd_en, 1);

	if (am33xx_register_lcdc(&NHD_480272MF_ATXI_bone_lcd_cape_pdata))
		pr_info("Failed to register Beagleboardtoys 4.3\" LCD cape device\n");

	return;
}

static struct i2c_board_info tda99X_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tda998x",  0x70),
	},
};

static void hdmi_init(int evm_id, int profile)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	unsigned int i2c_instance;

	setup_pin_mux(hdmi_pin_mux);

	if (conf_disp_pll(371000000)) {
		pr_info("Failed to set pixclock to 371000000, not attempting to"
				"register DVI adapter\n");
		return;
	}

	if (am33xx_register_lcdc(&hdmi_pdata))
		pr_info("Failed to register BeagleBoardToys HDMI adapter\n");


	if(evm_id == BEAGLE_BONE_BLACK)
		i2c_instance = 1;

	/* I2C adapter request */
	adapter = i2c_get_adapter(i2c_instance);
	if (!adapter) {
		pr_err("Failed to get adapter i2c%u\n", i2c_instance);
		return;
	}

	client = i2c_new_device(adapter, tda99X_i2c_boardinfo);
	if (!client)
		pr_err("Failed to register HDMI tda998x to i2c%u\n", i2c_instance);

	i2c_put_adapter(adapter);

	pr_info("Setup HDMI display complete\n");

	return;
}

#define BEAGLEBONEDVI_PDn  GPIO_TO_PIN(1, 7)

static void dvi_init(int evm_id, int profile)
{
	pr_info("IN : %s \n", __FUNCTION__);
	setup_pin_mux(dvi_pin_mux);
	gpio_request(BEAGLEBONEDVI_PDn, "DVI_PDn");
	gpio_direction_output(BEAGLEBONEDVI_PDn, 1);

	/* we are being stupid and setting pixclock
	   from here instead of da8xx-fb.c */
	if (conf_disp_pll(560000000)) {
		pr_info("Failed to set pixclock to 56000000, not attempting to"
				"register DVI adapter\n");
		return;
	}

	if (am33xx_register_lcdc(&dvi_pdata))
		pr_info("Failed to register BeagleBoardToys DVI adapter\n");

	pr_info("Setup DVI display\n");
	return;
}

static void mfd_tscadc_init(int evm_id, int profile)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void lcd_cape_tsc_init(int evm_id, int profile)
{
	int err;

	pr_info("IN : %s \n", __FUNCTION__);
	err = am33xx_register_mfd_tscadc(&beaglebone_tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");

	pr_info("Setup LCD cape touchscreen\n");

}

static struct resource cssp_camera_resources[] = {
	{
		.name = "gpmc_phys_mem_slot",
		.flags = IORESOURCE_MEM,
	},
};

static struct mt9t112_camera_info mt9t111_cam_info = {
	/* divider calculated for 32Mhz CAM_MCLK */
	.divider = {
		.m = 24, .n = 1,
		.p1 = 0, .p2 = 7, .p3 = 0, .p4 = 10, .p5 = 14, .p6 = 7, .p7 = 0,
	},
};

static struct soc_camera_link mt9t111_camera_link =  {
	.priv = &mt9t111_cam_info,
	.i2c_adapter_id = 3,
};

static struct i2c_board_info i2c_camera = {
	I2C_BOARD_INFO("mt9t112", 0x3c),
	.platform_data = &mt9t111_camera_link,
};

struct cssp_cam_platform_data {
	struct i2c_board_info *cam_i2c_board_info;
	const char *cam_clk_name;
	int dma_ch;
	int cssp_reset_pin;
};

static struct cssp_cam_platform_data cssp_cam_platform_data = {
	.cam_i2c_board_info = &i2c_camera,
	.cam_clk_name = "clkout2_ck",
	.dma_ch = AM33XX_DMA_XDMA_EVENT_INTR2,
	.cssp_reset_pin = GPIO_TO_PIN(0, 4),
};

static struct platform_device cssp_camera = {
	.name  = "cssp-camera",
	.id    = -1,
	.dev   = {
		.platform_data = &cssp_cam_platform_data,
	},
	.num_resources = sizeof(cssp_camera_resources) / sizeof(cssp_camera_resources[0]),
	.resource = cssp_camera_resources,
};

static struct gpmc_timings cssp_timings = {
	/* Minimum clock period for synchronous mode (in picoseconds) */
	.sync_clk = 20000,

	/* CS signal timings corresponding to GPMC_CONFIG2 */
	.cs_on = 1,
	.cs_rd_off = 16 * 10,		/* Read deassertion time */
	.cs_wr_off = 31 * 10,		/* Write deassertion time */

	/* ADV signal timings corresponding to GPMC_CONFIG3 */
	.adv_on = 0,			/* Assertion time */
	.adv_rd_off = 4 * 10,		/* Read deassertion time */
	.adv_wr_off = 4 * 10,		/* Write deassertion time */

	/* WE signals timings corresponding to GPMC_CONFIG4 */
	.we_on = 6 * 10,		/* WE assertion time */
	.we_off = 31 * 10,		/* WE deassertion time */

	/* OE signals timings corresponding to GPMC_CONFIG4 */
	.oe_on = 6 * 10,		/* OE assertion time */
	.oe_off = 16 * 10,		/* OE deassertion time */

	/* Access time and cycle time timings corresponding to GPMC_CONFIG5 */
	.page_burst_access = 2 * 10,	/* Multiple access word delay */
	.access = 14 * 10,		/* Start-cycle to first data valid delay */
	.rd_cycle = 16 * 10,		/* Total read cycle time */
	.wr_cycle = 31 * 10,		/* Total write cycle time */

	/* The following are only on OMAP3430 */
	.wr_access = 10 * 10,		/* WRACCESSTIME */
	.wr_data_mux_bus = 6 * 10,	/* WRDATAONADMUXBUS */
};

static int gpmc_cssp_init(void)
{
	int cs = 1; /* Chip Select on GPMC bus */
	int val;
	long unsigned int cssp_gpmc_mem_base_phys;

	if (gpmc_cs_request(cs, SZ_16M, &cssp_gpmc_mem_base_phys) < 0) {
			printk(KERN_ERR "[cssp_cam platform init]: gpmc_cs_request failed\n");
			return -1;
	}

	cssp_camera_resources[0].start = cssp_gpmc_mem_base_phys;
	cssp_camera_resources[0].end = cssp_gpmc_mem_base_phys + 0x1ffff;

	if (gpmc_cs_configure(cs, GPMC_CONFIG_DEV_TYPE, GPMC_DEVICETYPE_NOR) < 0) {
			printk(KERN_ERR "[cssp_cam platform init]: gpmc_cs_configure failed\n");
			return -1;
	}

	val = GPMC_CONFIG1_READMULTIPLE_SUPP;
	val |= GPMC_CONFIG1_READTYPE_SYNC;
	val |= GPMC_CONFIG1_WRITETYPE_SYNC;
	val |= GPMC_CONFIG1_CLKACTIVATIONTIME(2);
	val |= GPMC_CONFIG1_PAGE_LEN(2);
	val |= GPMC_CONFIG1_DEVICESIZE_16;
	val |= GPMC_CONFIG1_DEVICETYPE_NOR;
	val |= GPMC_CONFIG1_MUXADDDATA;
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, val);

	if (gpmc_cs_set_timings(cs, &cssp_timings) < 0) {
		printk(KERN_ERR "Failed gpmc_cs_set_timings for QuickLogic CAMIF device\n");
		goto free;
	}

	val = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG6);
	val &= 0xe0f0f030;
	val |= 0x0a060484;
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, val);

	printk(KERN_INFO "gpmc_cssp_init for QuickLogic CAMIF device succeeded\n");

	return 0;

free:
	gpmc_cs_free(cs);

	printk(KERN_ERR "Could not initialize QuickLogic CAMIF device\n");

	return -1;
}

static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void bone_camera_cape_clkout2_init(void)
{
	void __iomem *base;
	unsigned int val;

	/* XXX: HACK */
	base = ioremap(0x44E00700, SZ_4K);
	val = (5 << 3) | (3 << 0); //32 MHz
	writel(val, base);
	iounmap(base);

	setup_pin_mux(clkout2_pin_mux);
}

#define BEAGLEBONE_CAMERA_ORIENTATION GPIO_TO_PIN(0, 30)

static void cssp_gpmc_init(void)
{
	struct gpmc_devices_info gpmc_device[2] = {
			{ NULL, GPMC_DEVICE_NOR },
		};
	int status;

	setup_pin_mux(gpmc_pin_mux);
	setup_pin_mux(camera_cape_pin_mux);

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	gpmc_cssp_init();

	/* Get the sensor orientation and setup sensor flags as appropriate */
	status = gpio_request(BEAGLEBONE_CAMERA_ORIENTATION, "camera orientation");
	if (status < 0) {
		pr_err("Failed to request gpio for camera sensor orientation");
	} else {
		int orientation;

		orientation = gpio_get_value(BEAGLEBONE_CAMERA_ORIENTATION);
		if (orientation == 0) {
			mt9t111_cam_info.flags = MT9T112_FLAG_VFLIP;
			pr_info("Camera cape sensor is facing forward\n");
		} else {
			pr_info("Camera cape sensor is facing backward\n");
		}
	}

	bone_camera_cape_clkout2_init();

	platform_device_register(&cssp_camera);

	printk(KERN_INFO "[cssp_cam platform init]: cssp_gpmc_init: DONE\n");
}

static void rgmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rgmii1_pin_mux);
	return;
}

static void rgmii2_init(int evm_id, int profile)
{
	setup_pin_mux(rgmii2_pin_mux);
	return;
}

static void mii1_init(int evm_id, int profile)
{
	setup_pin_mux(mii1_pin_mux);
	return;
}

static void rmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rmii1_pin_mux);
	return;
}

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}

/* setup uart3 */
static void uart3_init(int evm_id, int profile)
{
	setup_pin_mux(uart3_pin_mux);
	return;
}

/* setup uart2 */
static void uart2_init(int evm_id, int profile)
{
	setup_pin_mux(uart2_pin_mux);
	return;
}

/*
 * gpio0_7 was driven HIGH in u-boot before DDR configuration
 *
 * setup gpio0_7 for EVM-SK 1.2
 */
static void gpio_ddr_vtt_enb_init(int evm_id, int profile)
{
	setup_pin_mux(gpio_ddr_vtt_enb_pin_mux);
	return;
}

/* setup haptics */
#define HAPTICS_MAX_FREQ 250
static void haptics_init(int evm_id, int profile)
{
	setup_pin_mux(haptics_pin_mux);
	pwm_pdata[2].chan_attrib[1].max_freq = HAPTICS_MAX_FREQ;
	am33xx_register_ehrpwm(2, &pwm_pdata[2]);
}

/* NAND partition information */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 40 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* SPI 0/1 Platform Data */
/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = 2 * SZ_128K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = 2 * SZ_4K,
	},
	{
		.name       = "Kernel",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x62000 */
		.size       = 28 * SZ_128K,
	},
	{
		.name       = "File System",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x3E2000 */
		.size       = MTDPART_SIZ_FULL,		/* size ~= 4.1 MiB */
	}
};

static const struct flash_platform_data am335x_spi_flash = {
	.type      = "w25q64",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi0_slave_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &am335x_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 24000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
};

static struct spi_board_info am335x_spi1_slave_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &am335x_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 12000000,
		.bus_num       = 2,
		.chip_select   = 0,
	},
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void evm_nand_init(int evm_id, int profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* TPS65217 voltage regulator support */

/* 1.8V */
static struct regulator_consumer_supply tps65217_dcdc1_consumers[] = {
	{
		.supply = "vdds_osc",
	},
	{
		.supply = "vdds_pll_ddr",
	},
	{
		.supply = "vdds_pll_mpu",
	},
	{
		.supply = "vdds_pll_core_lcd",
	},
	{
		.supply = "vdds_sram_mpu_bb",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdda_usb0_1p8v",
	},
	{
		.supply = "vdds_ddr",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_hvx_1p8v",
	},
	{
		.supply = "vdda_adc",
	},
	{
		.supply = "ddr2",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc2_consumers[] = {
	{
		.supply = "vdd_mpu",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc3_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo1_consumers[] = {
	{
		.supply = "vdds_rtc",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo2_consumers[] = {
	{
		.supply = "vdds_any_pn",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo3_consumers[] = {
	{
		.supply = "vdds_hvx_ldo3_3p3v",
	},
	{
		.supply = "vdda_usb0_3p3v",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo4_consumers[] = {
	{
		.supply = "vdds_hvx_ldo4_3p3v",
	},
};

/*
 * FIXME: Some BeagleBones reuire a ramp_delay to settle down the set
 * voltage from 0.95v to 1.25v. By default a minimum of 70msec is set
 * based on experimentation. This will be removed/modified to exact
 * value, once the root cause is known.
 *
 * The reason for extended ramp time requirement on BeagleBone is not
 * known and the delay varies from board - board, if the board hangs
 * with this 70msec delay then try to increase the value.
 */
static struct tps65217_rdelay dcdc2_ramp_delay = {
	.ramp_delay = 70000,
};

static struct regulator_init_data tps65217_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1800000,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc1_consumers),
		.consumer_supplies = tps65217_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc2_consumers),
		.consumer_supplies = tps65217_dcdc2_consumers,
		.driver_data = &dcdc2_ramp_delay,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1500000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc3_consumers),
		.consumer_supplies = tps65217_dcdc3_consumers,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo1_consumers),
		.consumer_supplies = tps65217_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo2_consumers),
		.consumer_supplies = tps65217_ldo2_consumers,
	},

	/* ldo3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo3_consumers),
		.consumer_supplies = tps65217_ldo3_consumers,
	},

	/* ldo4 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo4_consumers),
		.consumer_supplies = tps65217_ldo4_consumers,
	},
};

struct tps65217_bl_pdata bone_lcd3_bl_pdata[] = {
	{
		.isel = TPS65217_BL_ISET1,
		.fdim = TPS65217_BL_FDIM_200HZ,
	},
};

static struct tps65217_board beaglebone_tps65217_info = {
	.tps65217_init_data = &tps65217_regulator_data[0],
	.bl_pdata = bone_lcd3_bl_pdata,
	.status_off = true,
};

static struct lis3lv02d_platform_data lis331dlh_pdata = {
	.click_flags = LIS3_CLICK_SINGLE_X |
			LIS3_CLICK_SINGLE_Y |
			LIS3_CLICK_SINGLE_Z,
	.wakeup_flags = LIS3_WAKEUP_X_LO | LIS3_WAKEUP_X_HI |
			LIS3_WAKEUP_Y_LO | LIS3_WAKEUP_Y_HI |
			LIS3_WAKEUP_Z_LO | LIS3_WAKEUP_Z_HI,
	.irq_cfg = LIS3_IRQ1_CLICK | LIS3_IRQ2_CLICK,
	.wakeup_thresh	= 10,
	.click_thresh_x = 10,
	.click_thresh_y = 10,
	.click_thresh_z = 10,
	.g_range	= 2,
	.st_min_limits[0] = 120,
	.st_min_limits[1] = 120,
	.st_min_limits[2] = 140,
	.st_max_limits[0] = 550,
	.st_max_limits[1] = 550,
	.st_max_limits[2] = 750,
};

static struct i2c_board_info lis331dlh_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("lis331dlh", 0x18),
		.platform_data = &lis331dlh_pdata,
	},
};

static void lis331dlh_init(int evm_id, int profile)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	unsigned int i2c_instance;

	switch (evm_id) {
	case GEN_PURP_EVM:
	case GEN_PURP_DDR3_EVM:
		i2c_instance = 2;
		break;
	case EVM_SK:
		i2c_instance = 1;
		break;
	default:
		pr_err("lis331dlh is not supported on this evm (%d)\n", evm_id);
		return;
	}

	/* I2C adapter request */
	adapter = i2c_get_adapter(i2c_instance);
	if (!adapter) {
		pr_err("failed to get adapter i2c%u\n", i2c_instance);
		return;
	}

	client = i2c_new_device(adapter, lis331dlh_i2c_boardinfo);
	if (!client)
		pr_err("failed to register lis331dlh to i2c%u\n", i2c_instance);

	i2c_put_adapter(adapter);
}

static struct i2c_board_info am335x_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
	{
		I2C_BOARD_INFO("tsl2550", 0x39),
	},
	{
		I2C_BOARD_INFO("tmp275", 0x48),
	},
	{
		/* bq27421 Gas Gauge */
		I2C_BOARD_INFO("bq27421", 0x55),
	},
};

static void i2c1_init(int evm_id, int profile)
{
	setup_pin_mux(i2c1_pin_mux);
	omap_register_i2c_bus(2, 100, am335x_i2c1_boardinfo,
			ARRAY_SIZE(am335x_i2c1_boardinfo));
	return;
}

static struct at24_platform_data bone_display_daughter_board_eeprom_info;
static struct at24_platform_data bone_daughter_board_eeprom_info;

static struct i2c_board_info am335x_i2c2_boardinfo[] = {
	{
		/* Cape EEPROM - Only for Display Capes */
		I2C_BOARD_INFO("24c256", DISPLAY_CAPE_I2C_ADDR),
		.platform_data  = &bone_display_daughter_board_eeprom_info,
	},
	{
		/* Cape EEPROM */
		I2C_BOARD_INFO("24c256", 0x55),
		.platform_data  = &bone_daughter_board_eeprom_info,
	},
	{
		/* Cape EEPROM */
		I2C_BOARD_INFO("24c256", 0x56),
		.platform_data  = &bone_daughter_board_eeprom_info,
	},
	{
		/* Cape EEPROM */
		I2C_BOARD_INFO("24c256", 0x57),
		.platform_data  = &bone_daughter_board_eeprom_info,
	},
};

static void i2c2_init(int evm_id, int profile)
{
	setup_pin_mux(i2c2_pin_mux);
	omap_register_i2c_bus(3, 100, am335x_i2c2_boardinfo,
			ARRAY_SIZE(am335x_i2c2_boardinfo));
	return;
}

/* BeagleBone Black HDMI Audio */
static struct platform_device am335x_hdmi_codec_device = {
	.name		= "hdmi-audio-codec",
	.id		= -1,
};

/* Setup McASP 0*/
static void mcasp0_init(int evm_id, int profile)
{
	/* Configure McASP */
	switch (evm_id) {
	case EVM_SK:
		setup_pin_mux(mcasp0_pin_mux);
		am335x_register_mcasp(&am335x_evm_sk_snd_data0, 0);
		break;
	case BEAGLE_BONE_BLACK:
		gpio_request(GPIO_TO_PIN(1, 27), "BONE_AUDIO_CLOCK");
		gpio_direction_output(GPIO_TO_PIN(1, 27), 1);
		setup_pin_mux(bone_black_mcasp0_pin_mux);
		am335x_register_mcasp(&bone_black_snd_data0, 0);
		platform_device_register(&am335x_hdmi_codec_device);
		break;
	default:
		setup_pin_mux(mcasp0_pin_mux);
		am335x_register_mcasp(&am335x_evm_snd_data0, 0);
	}

	return;
}

/* Setup McASP 1 */
static void mcasp1_init(int evm_id, int profile)
{
	/* Configure McASP */
	setup_pin_mux(mcasp1_pin_mux);
	switch (evm_id) {
	case EVM_SK:
		am335x_register_mcasp(&am335x_evm_sk_snd_data1, 1);
		break;
	default:
		am335x_register_mcasp(&am335x_evm_snd_data1, 1);
	}

	return;
}

static void mmc1_init(int evm_id, int profile)
{
	switch (evm_id) {
	case BEAGLE_BONE_BLACK:
		setup_pin_mux(mmc1_common_pin_mux);
		setup_pin_mux(mmc1_dat4_7_pin_mux);

		am335x_mmc[1].mmc		= 2;
		am335x_mmc[1].caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA;
		am335x_mmc[1].nonremovable	= true;
		am335x_mmc[1].gpio_cd		= -EINVAL;
		am335x_mmc[1].gpio_wp		= -EINVAL;
		am335x_mmc[1].ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */
		break;
	default:
		setup_pin_mux(mmc1_common_pin_mux);
		setup_pin_mux(mmc1_dat4_7_pin_mux);
		setup_pin_mux(mmc1_wp_only_pin_mux);
		setup_pin_mux(mmc1_cd_only_pin_mux);

		am335x_mmc[1].mmc		= 2;
		am335x_mmc[1].caps		= MMC_CAP_4_BIT_DATA;
		am335x_mmc[1].gpio_cd		= GPIO_TO_PIN(2, 2);
		am335x_mmc[1].gpio_wp		= GPIO_TO_PIN(1, 29);
		am335x_mmc[1].ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */
		break;
	}
	/* mmc will be initialized when mmc0_init is called */
	return;
}

static void mmc1_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(mmc1_common_pin_mux);
	am335x_mmc[1].mmc = 2;
	am335x_mmc[1].name = "wl1271";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */
}

static void mmc2_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(mmc2_wl12xx_pin_mux);

	am335x_mmc[1].mmc = 3;
	am335x_mmc[1].name = "wl1271";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */

	/* mmc will be initialized when mmc0_init is called */
	return;
}

static void uart1_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(uart1_wl12xx_pin_mux);
}

#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

/* Bluetooth Enable PAD for EVM Rev 1.1 and up */
#define AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET		0x09AC

/* Bluetooth Enable PAD for EVM Rev 1.0 */
#define AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET			0x0884

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.1 */
	selected_pad = AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET;
	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.0 */
	/*selected_pad = AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET;*/

	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	/* Enable pullup on the enable pin for keeping BT active during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value &= (~AM33XX_PULL_DISA);
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_direction_output(kim_data->nshutdown, 0);

	/* Disable pullup on the enable pin to allow BT shut down during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value |= AM33XX_PULL_DISA;
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = GPIO_TO_PIN(3, 21),
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

#ifdef CONFIG_MACH_AM335XEVM_WILINK8
static struct platform_device nfcwilink_device = {
        .name = "nfcwilink",
        .id = -1,
};
#endif

static inline void __init am335xevm_init_btwilink(void)
{
	pr_info("am335xevm: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
	platform_device_register(&nfcwilink_device);
#endif
}
#endif

/* WL1271 Audio */
static struct platform_device wl1271bt_codec_device = {
	.name		= "wl1271bt-dummy-codec",
	.id		= -1,
};

static void wl12xx_bluetooth_enable(void)
{
#ifndef CONFIG_TI_ST
	int status = gpio_request(am335xevm_wlan_data.bt_enable_gpio,
		"bt_en\n");
	if (status < 0)
		pr_err("Failed to request gpio for bt_enable");

	pr_info("Configure Bluetooth Enable pin...\n");
	gpio_direction_output(am335xevm_wlan_data.bt_enable_gpio, 0);
#else
	am335xevm_init_btwilink();
#endif
	 platform_device_register(&wl1271bt_codec_device);
}

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_direction_output(am335xevm_wlan_data.wlan_enable_gpio, 1);
		mdelay(70);
	} else {
		gpio_direction_output(am335xevm_wlan_data.wlan_enable_gpio, 0);
	}

	return 0;
}

static void wl12xx_init(int evm_id, int profile)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	if (evm_id == EVM_SK) {
		am335xevm_wlan_data.wlan_enable_gpio = GPIO_TO_PIN(1, 29);
		am335xevm_wlan_data.bt_enable_gpio = GPIO_TO_PIN(3, 21);
		am335xevm_wlan_data.irq =
				OMAP_GPIO_IRQ(AM335XEVM_SK_WLAN_IRQ_GPIO);
		setup_pin_mux(wl12xx_pin_mux_sk);
	} else {
		setup_pin_mux(wl12xx_pin_mux);
	}
	am335xevm_wlan_data.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ;
	wl12xx_bluetooth_enable();

	if (wl12xx_set_platform_data(&am335xevm_wlan_data))
		pr_err("error setting wl12xx data\n");

	dev = am335x_mmc[1].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	ret = gpio_request_one(am335xevm_wlan_data.wlan_enable_gpio,
		GPIOF_OUT_INIT_LOW, "wlan_en");
	if (ret) {
		pr_err("Error requesting wlan enable gpio: %d\n", ret);
		goto out;
	}


	pdata->slots[0].set_power = wl12xx_set_power;
out:
	return;
}

static void d_can_init(int evm_id, int profile)
{
	switch (evm_id) {
	case IND_AUT_MTR_EVM:
		if ((profile == PROFILE_0) || (profile == PROFILE_1)) {
			setup_pin_mux(d_can_ia_pin_mux);
			/* Instance Zero */
			am33xx_d_can_init(0);
		}
		break;
	case GEN_PURP_EVM:
	case GEN_PURP_DDR3_EVM:
		if (profile == PROFILE_1) {
			setup_pin_mux(d_can_gp_pin_mux);
			/* Instance One */
			am33xx_d_can_init(1);
		}
		break;
	default:
		break;
	}
}

static void mmc0_init(int evm_id, int profile)
{
	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
	case EVM_SK:
	case BEAGLE_BONE_BLACK:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		break;
	default:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		setup_pin_mux(mmc0_wp_only_pin_mux);
		break;
	}

	omap2_hsmmc_init(am335x_mmc);
	return;
}

static struct i2c_board_info tps65217_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65217", TPS65217_I2C_ID),
		.platform_data  = &beaglebone_tps65217_info,
	},
};

static void tps65217_init(int evm_id, int profile)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct device *mpu_dev;
	struct tps65217 *tps;
	unsigned int val;
	int ret;

	mpu_dev = omap_device_get_by_hwmod_name("mpu");
	if (!mpu_dev)
		pr_warning("%s: unable to get the mpu device\n", __func__);

	/* I2C1 adapter request */
	adapter = i2c_get_adapter(1);
	if (!adapter) {
		pr_err("failed to get adapter i2c1\n");
		return;
	}

	client = i2c_new_device(adapter, tps65217_i2c_boardinfo);
	if (!client)
		pr_err("failed to register tps65217 to i2c1\n");

	i2c_put_adapter(adapter);

	tps = (struct tps65217 *)i2c_get_clientdata(client);

	ret = tps65217_reg_read(tps, TPS65217_REG_STATUS, &val);
	if (ret) {
		pr_err("failed to read tps65217 status reg\n");
		return;
	}

	if (!(val & TPS65217_STATUS_ACPWR)) {
		/* If powered by USB then disable OPP120, OPPTURBO and OPPNITRO*/
		pr_info("Maximum current provided by the USB port is 500mA"
			" which is not sufficient\nwhen operating @OPP120, OPPTURBO and"
			" OPPNITRO. The current requirement for some\nuse-cases"
			" using OPP100 might also exceed the maximum current"
			" that the\nUSB port can provide. Unless you are fully"
			" confident that the current\nrequirements for OPP100"
			" use-case don't exceed the USB limits, switching\nto"
			" AC power is recommended.\n");
		opp_disable(mpu_dev, 600000000);
		opp_disable(mpu_dev, 720000000);
		opp_disable(mpu_dev, 800000000);
		opp_disable(mpu_dev, 1000000000);
	}
}

static void mmc0_no_cd_init(int evm_id, int profile)
{
	setup_pin_mux(mmc0_common_pin_mux);
	setup_pin_mux(mmc0_wp_only_pin_mux);

	omap2_hsmmc_init(am335x_mmc);
	return;
}

/* Configure GPIOs for GPIO Keys */
static struct gpio_keys_button am335x_evm_gpio_buttons[] = {
	{
		.code                   = BTN_0,
		.gpio                   = GPIO_TO_PIN(2, 3),
		.desc                   = "SW1",
	},
	{
		.code                   = BTN_1,
		.gpio                   = GPIO_TO_PIN(2, 2),
		.desc                   = "SW2",
	},
	{
		.code                   = BTN_2,
		.gpio                   = GPIO_TO_PIN(0, 30),
		.desc                   = "SW3",
		.wakeup                 = 1,
	},
	{
		.code                   = BTN_3,
		.gpio                   = GPIO_TO_PIN(2, 5),
		.desc                   = "SW4",
	},
};

static struct gpio_keys_platform_data am335x_evm_gpio_key_info = {
	.buttons        = am335x_evm_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(am335x_evm_gpio_buttons),
};

static struct platform_device am335x_evm_gpio_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_gpio_key_info,
	},
};

static void gpio_keys_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_keys_pin_mux);
	err = platform_device_register(&am335x_evm_gpio_keys);
	if (err)
		pr_err("failed to register gpio key device\n");
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "am335x:EVM_SK:usr0",
		.gpio			= GPIO_TO_PIN(1, 4),	/* D1 */
	},
	{
		.name			= "am335x:EVM_SK:usr1",
		.gpio			= GPIO_TO_PIN(1, 5),	/* D2 */
	},
	{
		.name			= "am335x:EVM_SK:mmc0",
		.gpio			= GPIO_TO_PIN(1, 7),	/* D3 */
		.default_trigger	= "mmc0",
	},
	{
		.name			= "am335x:EVM_SK:heartbeat",
		.gpio			= GPIO_TO_PIN(1, 6),	/* D4 */
		.default_trigger	= "heartbeat",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void gpio_led_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

/* setup spi0 */
static void spi0_init(int evm_id, int profile)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(am335x_spi0_slave_info,
			ARRAY_SIZE(am335x_spi0_slave_info));
	return;
}

/* setup spi1 */
static void spi1_init(int evm_id, int profile)
{
	setup_pin_mux(spi1_pin_mux);
	spi_register_board_info(am335x_spi1_slave_info,
			ARRAY_SIZE(am335x_spi1_slave_info));
	return;
}


static int beaglebone_phy_fixup(struct phy_device *phydev)
{
	phydev->supported &= ~(SUPPORTED_100baseT_Half |
				SUPPORTED_100baseT_Full);

	return 0;
}

static void profibus_init(int evm_id, int profile)
{
	setup_pin_mux(profibus_pin_mux);
	return;
}

static struct omap_rtc_pdata am335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static void am335x_rtc_init(int evm_id, int profile)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
	case BEAGLE_BONE_BLACK:
		am335x_rtc_info.pm_off = true;
		break;
	default:
		break;
	}

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() >= AM335X_REV_ES2_0)
		am335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return;
	}

	pdev = omap_device_build(dev_name, -1, oh, &am335x_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
}

/* Enable clkout2 */
static void clkout2_enable(int evm_id, int profile)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}

static void sgx_init(int evm_id, int profile)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}
/* General Purpose EVM */
static struct evm_dev_cfg gen_purp_evm_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{clkout2_enable, DEV_ON_BASEBOARD, PROFILE_ALL},
	{enable_ecap0,	DEV_ON_DGHTR_BRD, (PROFILE_0 | PROFILE_1 |
						PROFILE_2 | PROFILE_7) },
	{lcdc_init,	DEV_ON_DGHTR_BRD, (PROFILE_0 | PROFILE_1 |
						PROFILE_2 | PROFILE_7) },
	{mfd_tscadc_init,	DEV_ON_DGHTR_BRD, (PROFILE_0 | PROFILE_1 |
						PROFILE_2 | PROFILE_7) },
	{rgmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rgmii2_init,	DEV_ON_DGHTR_BRD, (PROFILE_1 | PROFILE_2 |
						PROFILE_4 | PROFILE_6) },
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{evm_nand_init, DEV_ON_DGHTR_BRD,
		(PROFILE_ALL & ~PROFILE_2 & ~PROFILE_3)},
	{i2c1_init,     DEV_ON_DGHTR_BRD, (PROFILE_ALL & ~PROFILE_2)},
	{lis331dlh_init, DEV_ON_DGHTR_BRD, (PROFILE_ALL & ~PROFILE_2)},
	{mcasp1_init,	DEV_ON_DGHTR_BRD, (PROFILE_0 | PROFILE_3 | PROFILE_7)},
	{mcasp0_init,	DEV_ON_BASEBOARD, (PROFILE_ALL)},
	{mmc1_init,	DEV_ON_DGHTR_BRD, PROFILE_2},
	{mmc2_wl12xx_init,	DEV_ON_BASEBOARD, (PROFILE_0 | PROFILE_3 |
								PROFILE_5)},
	{mmc0_init,	DEV_ON_BASEBOARD, (PROFILE_ALL & ~PROFILE_5)},
	{mmc0_no_cd_init,	DEV_ON_BASEBOARD, PROFILE_5},
	{spi0_init,	DEV_ON_DGHTR_BRD, PROFILE_2},
	{uart1_wl12xx_init,	DEV_ON_BASEBOARD, (PROFILE_0 | PROFILE_3 |
								PROFILE_5)},
	{wl12xx_init,	DEV_ON_BASEBOARD, (PROFILE_0 | PROFILE_3 | PROFILE_5)},
	{d_can_init,	DEV_ON_DGHTR_BRD, PROFILE_1},
	{matrix_keypad_init, DEV_ON_DGHTR_BRD, PROFILE_0},
	{volume_keys_init,  DEV_ON_DGHTR_BRD, PROFILE_0},
	{uart2_init,	DEV_ON_DGHTR_BRD, PROFILE_3},
	{haptics_init,	DEV_ON_DGHTR_BRD, (PROFILE_4)},
	{sgx_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* Industrial Auto Motor Control EVM */
static struct evm_dev_cfg ind_auto_mtrl_evm_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{clkout2_enable, DEV_ON_BASEBOARD, PROFILE_ALL},
	{mii1_init,	DEV_ON_DGHTR_BRD, PROFILE_ALL},
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{profibus_init, DEV_ON_DGHTR_BRD, PROFILE_ALL},
	{evm_nand_init, DEV_ON_DGHTR_BRD, PROFILE_ALL},
	{spi1_init,	DEV_ON_DGHTR_BRD, PROFILE_ALL},
	{uart3_init,	DEV_ON_DGHTR_BRD, PROFILE_ALL},
	{i2c1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{mmc0_no_cd_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* Beaglebone < Rev A3 */
static struct evm_dev_cfg beaglebone_old_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_NONE},
	{clkout2_enable, DEV_ON_BASEBOARD, PROFILE_NONE},
	{rmii1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{i2c2_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{sgx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

/* Beaglebone Rev A3 and after */
static struct evm_dev_cfg beaglebone_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_NONE},
	{tps65217_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mii1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{i2c2_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{sgx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

/* Beaglebone Black */
static struct evm_dev_cfg beaglebone_black_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_NONE},
	{tps65217_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mii1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{i2c1_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{i2c2_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{mcasp0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{sgx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};
/* EVM - Starter Kit */
static struct evm_dev_cfg evm_sk_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{mmc1_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rgmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rgmii2_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{lcdc_init,     DEV_ON_BASEBOARD, PROFILE_ALL},
	{enable_ecap2,     DEV_ON_BASEBOARD, PROFILE_ALL},
	{mfd_tscadc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{gpio_keys_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
	{gpio_led_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
	{lis331dlh_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{mcasp1_init,   DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart1_wl12xx_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{wl12xx_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
	{gpio_ddr_vtt_enb_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{sgx_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

static int am33xx_evm_tx_clk_dly_phy_fixup(struct phy_device *phydev)
{
	phy_write(phydev, AR8051_PHY_DEBUG_ADDR_REG,
		  AR8051_DEBUG_RGMII_CLK_DLY_REG);
	phy_write(phydev, AR8051_PHY_DEBUG_DATA_REG, AR8051_RGMII_TX_CLK_DLY);

	return 0;
}

#define AM33XX_VDD_CORE_OPP50_UV		1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000

static void am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return;
	}

	pr_debug("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}
}

#ifdef CONFIG_MACH_AM335XEVM_VIBRATOR
int am335xevm_vibrator_init(void);
#endif

static void setup_general_purpose_evm(void)
{
	u32 prof_sel = am335x_get_profile_selection();
	u32 boardid = GEN_PURP_EVM;

	if (!strncmp("1.5A", config.version, 4))
		boardid = GEN_PURP_DDR3_EVM;

	pr_info("The board is general purpose EVM %sin profile %d\n",
			((boardid == GEN_PURP_DDR3_EVM) ? "with DDR3 " : ""),
			prof_sel);

	_configure_device(boardid, gen_purp_evm_dev_cfg, (1L << prof_sel));

	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
	/* Atheros Tx Clk delay Phy fixup */
	phy_register_fixup_for_uid(AM335X_EVM_PHY_ID, AM335X_EVM_PHY_MASK,
				   am33xx_evm_tx_clk_dly_phy_fixup);

#ifdef CONFIG_MACH_AM335XEVM_VIBRATOR
	/* Initialize Vibrator on AM335xEVM */
	am335xevm_vibrator_init();
#endif
}

static void setup_ind_auto_motor_ctrl_evm(void)
{
	u32 prof_sel = am335x_get_profile_selection();

	pr_info("The board is an industrial automation EVM in profile %d\n",
		prof_sel);

	/* Only Profile 0 is supported */
	if ((1L << prof_sel) != PROFILE_0) {
		pr_err("AM335X: Only Profile 0 is supported\n");
		pr_err("Assuming profile 0 & continuing\n");
		prof_sel = PROFILE_0;
	}

	_configure_device(IND_AUT_MTR_EVM, ind_auto_mtrl_evm_dev_cfg,
		PROFILE_0);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:1e", "0:00");
}

/* BeagleBone < Rev A3 */
static void setup_beaglebone_old(void)
{
	pr_info("The board is a AM335x Beaglebone < Rev A3.\n");

	/* Beagle Bone has Micro-SD slot which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(BEAGLE_BONE_OLD, beaglebone_old_dev_cfg,
								PROFILE_NONE);

	phy_register_fixup_for_uid(BBB_PHY_ID, BBB_PHY_MASK,
					beaglebone_phy_fixup);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII, NULL, NULL);
}

/* BeagleBone after Rev A3 */
static void setup_beaglebone(void)
{
	pr_info("The board is a AM335x Beaglebone.\n");

	/* Beagle Bone has Micro-SD slot which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(BEAGLE_BONE_A3, beaglebone_dev_cfg, PROFILE_NONE);

	/* TPS65217 regulator has full constraints */
	regulator_has_full_constraints();

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);
}

/* BeagleBone Black */
static void setup_beaglebone_black(void)
{
	pr_info("The board is a AM335x Beaglebone Black.\n");

	/* Beagle Bone has Micro-SD slot which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(BEAGLE_BONE_BLACK, beaglebone_black_dev_cfg, PROFILE_NONE);

	/* TPS65217 regulator has full constraints */
	regulator_has_full_constraints();

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);
}

/* EVM - Starter Kit */
static void setup_starterkit(void)
{
	pr_info("The board is a AM335x Starter Kit.\n");

	/* Starter Kit has Micro-SD slot which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(EVM_SK, evm_sk_dev_cfg, PROFILE_NONE);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
	/* Atheros Tx Clk delay Phy fixup */
	phy_register_fixup_for_uid(AM335X_EVM_PHY_ID, AM335X_EVM_PHY_MASK,
				   am33xx_evm_tx_clk_dly_phy_fixup);
}

static void am335x_setup_daughter_board(struct memory_accessor *m, void *c)
{
	int ret;

	/*
	 * Read from the EEPROM to see the presence of daughter board.
	 * If present, print the cpld version.
	 */

	ret = m->read(m, (char *)&config1, 0, sizeof(config1));
	if (ret == sizeof(config1)) {
		pr_info("Detected a daughter card on AM335x EVM..");
		daughter_brd_detected = true;
	}
	 else {
		pr_info("No daughter card found on AM335x EVM\n");
		daughter_brd_detected = false;
		return;
	}

	if (!strncmp("CPLD", config1.cpld_ver, 4))
		pr_info("CPLD version: %s\n", config1.cpld_ver);
	else
		pr_err("Unknown CPLD version found\n");
}

static void bone_setup_display_daughter_board(struct memory_accessor *m, void *c)
{
	int ret;

	/*
	 * Read from the EEPROM to see the presence
	 * of daughter board. If present, get daughter board
	 * specific data.
	 */
	pr_info("IN : %s \n", __FUNCTION__);
	ret = m->read(m, (char *) &cape_eeprom_config, 0,
			sizeof( struct bone_cape_eeprom_config));

	if (ret == sizeof (struct bone_cape_eeprom_config))
	{
		pr_info("Detected a daughter card on BeagleBone..");

		if (!strncmp("BB-BONE-LCD7-01", cape_eeprom_config.part_no, 15)) {
			pr_info("BeagleBone LCD7 cape board detected\n");
			pr_info("Board Name: %s\n", cape_eeprom_config.board_name);
			pr_info("Hardware revision: %s\n", cape_eeprom_config.version);

			if (!strncmp("00A1", cape_eeprom_config.version, 4))
				/* rev A1 */
				beaglebone_lcd_avdd_en = GPIO_TO_PIN(0, 7);
			else if (!strncmp("00A2", cape_eeprom_config.version, 4))
				/* rev A2 */
				beaglebone_lcd_avdd_en = GPIO_TO_PIN(1, 31);
			else
				/* Rev A3 or later */
				beaglebone_lcd_avdd_en = GPIO_TO_PIN(0, 2);

			bone_lcd7_lcdc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			pr_info("BeagleBone cape: Registering PWM backlight for LCD cape\n");
			enable_ehrpwm1(0,0);
			if ((!strncmp("00A1", cape_eeprom_config.version, 4)) ||
				(!strncmp("00A2", cape_eeprom_config.version, 4)))
				lcd_cape_keys_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			else
				lcd7a3_cape_keys_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);

			lcd_cape_tsc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);

			return;
		}
		else if (!strncmp("BB-BONE-LCD3-01", cape_eeprom_config.part_no, 15)) {
			pr_info("BeagleBone LCD3 cape board detected\n");
			pr_info("Board Name: %s\n", cape_eeprom_config.board_name);
			pr_info("Hardware revision: %s\n", cape_eeprom_config.version);

			bone_lcd3_lcdc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			lcd_cape_keys_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			lcd_cape_tsc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);

			return;
		}
		else if (!strncmp("BB-BONE-LCD4-01", cape_eeprom_config.part_no, 15)) {
			pr_info("BeagleBone LCD4 cape board detected\n");
			pr_info("Board Name: %s\n", cape_eeprom_config.board_name);
			pr_info("Hardware revision: %s\n", cape_eeprom_config.version);

			if (!strncmp("00A1", cape_eeprom_config.version, 4)) {
				beaglebone_lcd_avdd_en = GPIO_TO_PIN(3, 19);
			}

			bone_lcd4_lcdc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			enable_ehrpwm1(0,0);
			lcd4_cape_keys_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);
			lcd_cape_tsc_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);

			return;
		}
		else if (!strncmp("BB-BONE-DVID-01", cape_eeprom_config.part_no, 15) ||
			!strncmp("BB-BONE-DVID-02", cape_eeprom_config.part_no, 15)) {
			pr_info("BeagleBone DVI-D cape board detected\n");
			dvi_init(DEV_ON_DGHTR_BRD, PROFILE_NONE);

			return;
		}
	}

	/* Display needs to be initialized even if display daughter card is not found so as
	 * to enable framebuffer driver which is needed for successful Android bootup
	 */
	pr_info("No display cape found on BeagleBone\n");

	/* HDMI display is setup for BeagleBone Black */
	if (am335x_evm_get_id() == BEAGLE_BONE_BLACK)
		hdmi_init(DEV_ON_BASEBOARD, PROFILE_NONE);
	else
		vnc_lcdc_init(DEV_ON_BASEBOARD, PROFILE_NONE);
}

static void bone_setup_daughter_board(struct memory_accessor *m, void *c)
{
	int ret;

	/*
	 * Read from the EEPROM to see the presence
	 * of daughter board. If present, get daughter board
	 * specific data.
	 */
	pr_info("IN : %s \n", __FUNCTION__);
	ret = m->read(m, (char *) &cape_eeprom_config, 0,
			sizeof( struct bone_cape_eeprom_config));

	if (ret == sizeof (struct bone_cape_eeprom_config))
	{
		pr_info("Detected a daughter card on BeagleBone..");

		if (!strncmp("BB-BONE-CAM-01", cape_eeprom_config.part_no, 14) ||
			!strncmp("BB-BONE-CAM3-01", cape_eeprom_config.part_no, 15)) {
			pr_info("BeagleBone Camera cape board detected\n");
			cssp_gpmc_init();

			return;
		}
	}
}

static void am335x_evm_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
	char tmp[10];
	struct device *mpu_dev;

	/* 1st get the MAC address from EEPROM */
	ret = mem_acc->read(mem_acc, (char *)&am335x_mac_addr,
		EEPROM_MAC_ADDRESS_OFFSET, sizeof(am335x_mac_addr));

	if (ret != sizeof(am335x_mac_addr)) {
		pr_warning("AM335X: EVM Config read fail: %d\n", ret);
		return;
	}

	/* Fillup global mac id */
	am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);

	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)&config, 0, sizeof(config));
	if (ret != sizeof(config)) {
		pr_err("AM335X EVM config read fail, read %d bytes\n", ret);
		pr_err("This likely means that there either is no/or a failed EEPROM\n");
		goto out;
	}

	if (config.header != AM335X_EEPROM_HEADER) {
		pr_err("AM335X: wrong header 0x%x, expected 0x%x\n",
			config.header, AM335X_EEPROM_HEADER);
		goto out;
	}

	if (strncmp("A335", config.name, 4)) {
		pr_err("Board %s\ndoesn't look like an AM335x board\n",
			config.name);
		goto out;
	}

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	if (!strncmp("A335BONE", config.name, 8)) {
		daughter_brd_detected = false;
		if(!strncmp("00A1", config.version, 4) ||
		   !strncmp("00A2", config.version, 4))
			setup_beaglebone_old();
		else
			setup_beaglebone();
	} else if (!strncmp("A335BNLT", config.name, 8)) {
		daughter_brd_detected = false;
		if(!strncmp("0A5A", config.version, 4) ||
		   !strncmp("0A5B", config.version, 4) ||
		   !strncmp("0A5C", config.version, 4)) {
			mpu_dev = omap_device_get_by_hwmod_name("mpu");
			opp_enable(mpu_dev, AM33XX_ES2_0_OPPNITRO_FREQ);
		}
		setup_beaglebone_black();
	} else if (!strncmp("A335X_SK", config.name, 8)) {
		daughter_brd_detected = false;
		setup_starterkit();
	} else {
		/* only 6 characters of options string used for now */
		snprintf(tmp, 7, "%s", config.opt);
		pr_info("SKU: %s\n", tmp);

		if (!strncmp("SKU#01", config.opt, 6))
			setup_general_purpose_evm();
		else if (!strncmp("SKU#02", config.opt, 6))
			setup_ind_auto_motor_ctrl_evm();
		else
			goto out;
	}

	am335x_opp_update();

	return;

out:
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("The error message above indicates that there is an issue with\n"
		   "the EEPROM or the EEPROM contents.  After verifying the EEPROM\n"
		   "contents, if any, refer to the %s function in the\n"
		   "%s file to modify the board\n"
		   "initialization code to match the hardware configuration\n",
		   __func__ , __FILE__);
	machine_halt();
}

static struct at24_platform_data am335x_daughter_board_eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = am335x_setup_daughter_board,
	.context        = (void *)NULL,
};

static struct at24_platform_data am335x_baseboard_eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = am335x_evm_setup,
	.context        = (void *)NULL,
};

static struct at24_platform_data bone_display_daughter_board_eeprom_info = {
        .byte_len       = (256*1024) / 8,
        .page_size      = 64,
        .flags          = AT24_FLAG_ADDR16,
        .setup          = bone_setup_display_daughter_board,
        .context        = (void *)NULL,
};

static struct at24_platform_data bone_daughter_board_eeprom_info = {
        .byte_len       = (256*1024) / 8,
        .page_size      = 64,
        .flags          = AT24_FLAG_ADDR16,
        .setup          = bone_setup_daughter_board,
        .context        = (void *)NULL,
};


static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

/*
* Daughter board Detection.
* Every board has a ID memory (EEPROM) on board. We probe these devices at
* machine init, starting from daughter board and ending with baseboard.
* Assumptions :
*	1. probe for i2c devices are called in the order they are included in
*	   the below struct. Daughter boards eeprom are probed 1st. Baseboard
*	   eeprom probe is called last.
*/
static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
	{
		/* Daughter Board EEPROM */
		I2C_BOARD_INFO("24c256", DAUG_BOARD_I2C_ADDR),
		.platform_data  = &am335x_daughter_board_eeprom_info,
	},
	{
		/* Baseboard board EEPROM */
		I2C_BOARD_INFO("24c256", BASEBOARD_I2C_ADDR),
		.platform_data  = &am335x_baseboard_eeprom_info,
	},
	{
		I2C_BOARD_INFO("cpld_reg", 0x35),
	},
	{
		I2C_BOARD_INFO("tlc59108", 0x40),
	},
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &am335x_tps65910_info,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

static int cpld_reg_probe(struct i2c_client *client,
	    const struct i2c_device_id *id)
{
	cpld_client = client;
	return 0;
}

static int __devexit cpld_reg_remove(struct i2c_client *client)
{
	cpld_client = NULL;
	return 0;
}

static const struct i2c_device_id cpld_reg_id[] = {
	{ "cpld_reg", 0 },
	{ }
};

static struct i2c_driver cpld_reg_driver = {
	.driver = {
		.name	= "cpld_reg",
	},
	.probe		= cpld_reg_probe,
	.remove		= cpld_reg_remove,
	.id_table	= cpld_reg_id,
};

static void evm_init_cpld(void)
{
	i2c_add_driver(&cpld_reg_driver);
}

static void __init am335x_evm_i2c_init(void)
{
	/* Initially assume General Purpose EVM Config */
	am335x_evm_id = GEN_PURP_EVM;

	evm_init_cpld();

	omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo,
				ARRAY_SIZE(am335x_i2c0_boardinfo));
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_evm_i2c_init();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&musb_board_data);
	omap_board_config = am335x_evm_config;
	omap_board_config_size = ARRAY_SIZE(am335x_evm_config);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END

MACHINE_START(AM335XIAEVM, "am335xiaevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
