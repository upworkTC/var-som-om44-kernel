/*
 * Board support file for Variscite VAR-SOM-OM4460 board.
 *
 * Copyright (C) 2011 Variscite Ltd.
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/omapfb.h>
#include <linux/reboot.h>
#include <linux/twl6040-vib.h>
#include <linux/i2c/tsc2004.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/netdevice.h>
#include <linux/ti_wilink_st.h>
#include <linux/i2c/at24.h>
#include <plat/omap-serial.h>
#include <plat/android-display.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/dmm.h>
#include <mach/omap4_ion.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <video/omap-panel-generic-dpi.h>
#include <linux/wakelock.h>
#include "board-blaze.h"
#include "omap_ram_console.h"
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "variscite_blob.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#define KEY_BACK_GPIO 		184

#define ETH_KS8851_IRQ			171
#define BACKLIGHT_EN_GPIO		122


#define OMAP_UART_GPIO_MUX_MODE_143	143
#define VAR_SOM_OM44_PHY_RESET		177
#define VAR_SOM_OM44_GPIO_AUD_PWRON	182

#define VAR_DVK_OM44_USB_HUB_RESET	113

/* VAR-SOM-OM44 uses only 1.5kB (0x600) of EEPROM for U-Boot environment. */
#define VAR_SOM_OM44_PHY_REV           0x600

#define HDMI2_GPIO			1

#define GPIO_TSC2004_IRQ	101
#define GPIO_TSC_CTW_IRQ	174
#define GPIO_TSC2004_RESET	102
#define HDMI_GPIO_HPD 63 /* Hot plug pin for HDMI */
#define TWL6030_TOGGLE3		0x92

#define TPS62361_GPIO   7

#define GPIO_WIFI_PMENA		43
#define GPIO_WIFI_IRQ		41
#define OMAP_HDMI_HPD_ADDR	0x4A100098
#define OMAP_HDMI_PULLTYPE_MASK	0x00000010


#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)

#if defined(CONFIG_BT_WILINK) || defined(CONFIG_BT_WILINK_MODULE)
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

/* wl127x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 42,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
#else
static int wl1271_gpios[] = {-1, -1, -1};
#endif /* CONFIG_BT_WILINK */

static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
#if defined(CONFIG_BT_WILINK) || defined(CONFIG_BT_WILINK_MODULE)
		.platform_data	= &wilink_pdata
#else
		.platform_data	= &wl1271_gpios
#endif
	},
};
#endif

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct spi_board_info var_som_om44_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
};

static struct gpio var_som_om44_eth_gpios[] __initdata = {
	{ ETH_KS8851_IRQ,	GPIOF_IN,		"eth_irq"	},
};

static int __init omap_ethernet_init(void)
{
	int status;

	/* Request of GPIO lines */
	status = gpio_request_array(var_som_om44_eth_gpios,
				    ARRAY_SIZE(var_som_om44_eth_gpios));
	if (status)
		pr_err("Cannot request ETH GPIOs\n");

	return status;
}
#endif

static struct wake_lock st_wk_lock;

static struct omap_board_config_kernel var_som_om44_config[] __initdata = {
};

static void __init var_som_om44_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};


static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,

		.power_saving	= false,

	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA /*| MMC_CAP_POWER_OFF_CARD*/,
		.gpio_cd	= 110,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_32_33,
		.power_saving	= false,
	},	
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	{
		.name		= "wl1271",
		.mmc		= 4,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp        = -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
static struct regulator_consumer_supply var_som_om44_vmmc4_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.3",
};

static struct regulator_init_data var_som_om44_vmmc4 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &var_som_om44_vmmc4_supply,
};

static struct fixed_voltage_config var_som_om44_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &var_som_om44_vmmc4,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &var_som_om44_vwlan,
	},
};

static struct wl12xx_platform_data var_som_om44_wlan_data  __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = 2,
};
#endif

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

/* 
 * GPIO Buttons
 */
static struct gpio_keys_button gpio_keys_button_pins[] = {
	{
		.code		= KEY_BACK,
		.gpio		= KEY_BACK_GPIO,
		.desc		= "Back",
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data var_som_button_data = {
	.buttons	= gpio_keys_button_pins,
	.nbuttons	= ARRAY_SIZE(gpio_keys_button_pins),
};

static struct platform_device var_som_buttons = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &var_som_button_data,
	},
};

static struct platform_device *var_som_om44_devices[] __initdata = {
#if defined(CONFIG_BT_WILINK) || defined(CONFIG_BT_WILINK_MODULE)
	&btwilink_device,
	&wl1271_device,
	&omap_vwlan_device,
#endif
	&var_som_buttons, 
};

static int tsc2004_init_irq(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_TSC2004_IRQ, "tsc2004-irq");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO#%d: %d\n",
			__func__, GPIO_TSC2004_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC2004_IRQ)) {
		pr_err("%s: GPIO#%d cannot be configured as input\n",
			__func__, GPIO_TSC2004_IRQ);
		return -ENXIO;
	}

	ret = gpio_request(GPIO_TSC2004_RESET, "tsc2004-reset");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO#%d: %d\n",
			__func__, GPIO_TSC2004_RESET, ret);
		return ret;
	}

	if (gpio_direction_output(GPIO_TSC2004_RESET, 1)) {
		pr_err("%s: GPIO#%d cannot be configured as output 1\n",
			__func__, GPIO_TSC2004_RESET);
		return -ENXIO;
	}

	/* FIXME: meantime do not need debounce. */
	//omap_set_gpio_debounce(GPIO_TSC2004_IRQ, 1);
	//omap_set_gpio_debounce_time(GPIO_TSC2004_IRQ, 0xa);
	return ret;
}
#ifdef CONFIG_TOUCHSCREEN_CTW6120
static int tsc_ctw6120_init_irq(void)
{
	int ret = 0;
	ret = gpio_request(GPIO_TSC_CTW_IRQ, "GPIO_TSC_CTW_IRQ-irq");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO#%d: %d\n",
			__func__, GPIO_TSC_CTW_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC_CTW_IRQ)) {
		pr_err("%s: GPIO#%d cannot be configured as input\n",
			__func__, GPIO_TSC_CTW_IRQ);
		return -ENXIO;
	}

	return ret;
}
#endif

static void tsc2004_exit_irq(void)
{
	gpio_free(GPIO_TSC2004_IRQ);
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}
struct tsc2004_platform_data VAR_SOM_OM4460_tsc2004data = {
	.model			= 2004,
	.x_plate_ohms		= 180,
	.get_pendown_state	= tsc2004_get_irq_level,
	.init_platform_hw	= tsc2004_init_irq,
	.exit_platform_hw	= tsc2004_exit_irq,
};

static struct i2c_board_info __initdata var_som_om44_i2c_1_boardinfo[] = {

};

#define VAR_SOM_OM44_EEPROM_BLOB_OFFSET 0x600
#define VAR_SOM_OM44_EEPROM_UBOOT_ENV_OFFSET 0x600

static void emulate_variscite_blob(struct memory_accessor *macc, void *context)
{
	vatiscite_blob var_blob;
	static char uboot_env[VAR_SOM_OM44_EEPROM_BLOB_OFFSET];
	char *ptr;
	int size_left, found_eth_mac = 0;

	/* init BLOB structure */
	memset(&var_blob, 0xff, sizeof(vatiscite_blob));
	memcpy(var_blob.magic, VAR_BLOB_MAGIC, VAR_BLOB_MAGIC_SIZE);
	memcpy(var_blob.som_rev_str, "1.??", sizeof(var_blob.som_rev_str));

	/* read U-Boot environmant */
	macc->read(macc, uboot_env, 0, VAR_SOM_OM44_EEPROM_BLOB_OFFSET); 

	/* search Variscite ethernet MAC address */
	size_left = VAR_SOM_OM44_EEPROM_BLOB_OFFSET - strlen("ethaddr=00:11:22:33:44:5");
	ptr = uboot_env;

	while (size_left--) 
	{
		// search 'ethaddr='
		if (*(ptr+0) == 'e' && 
				*(ptr+1) == 't' &&
				*(ptr+2) == 'h' &&
				*(ptr+3) == 'a' &&
				*(ptr+4) == 'd' &&
				*(ptr+5) == 'd' &&
				*(ptr+6) == 'r' &&
				*(ptr+7) == '=' &&
			  	is_valid_ether_addr(ptr+8)) {
			found_eth_mac=1;
			ptr+=strlen("ethaddr=");
			break;
		}
		ptr++;
	}

	if (found_eth_mac) {
		int i;
		pr_info("Found Variscite MAC address at U-Boot environmant\n");

		/* Parse the colon separated Ethernet station address */
		for (i = 0; i <  ETH_ALEN; i++) {
			unsigned int tmp;
			sscanf(ptr + 3*i, "%2x", &tmp);
			var_blob.eth_mac_base[i] = (u8)tmp;
		}
		ptr++;
	}

	vatiscite_blob_setup(&var_blob);
}

static void read_variscite_blob(struct memory_accessor *macc, void *context)
{
	int	rc;
	vatiscite_blob var_blob;

	rc = macc->read(macc, (char*)&var_blob, VAR_SOM_OM44_EEPROM_BLOB_OFFSET, sizeof(var_blob));
	if (rc != sizeof(var_blob)) {
		pr_err("Failed to read Variscite BLOB from EEPROM!!!\n");
		return;
	}

	if (is_vatiscite_blob_valid(&var_blob)) {
		pr_info("Found Variscite BLOB at EEPROM\n");
		vatiscite_blob_setup(&var_blob);
	}
	else {
		pr_info("Could not find Variscite BLOB at EEPROM, try U-Boot environmant\n");
		emulate_variscite_blob(macc, context);
	}
}

static struct at24_platform_data eeprom_data = {
	.byte_len		= 4096,
	.page_size		= 32,
	.flags			= AT24_FLAG_READONLY | AT24_FLAG_ADDR16,
	.setup			= read_variscite_blob,
	.context		= NULL
};


static struct i2c_board_info __initdata var_som_om44_i2c_3_boardinfo[] = {
		{
			I2C_BOARD_INFO("tsc2004", 0x48),
			.type		= "tsc2004",
			.platform_data	= &VAR_SOM_OM4460_tsc2004data,
		},
		{
			I2C_BOARD_INFO("24c32", 0x50),
			.platform_data	= &eeprom_data
		},
#ifdef CONFIG_TOUCHSCREEN_CTW6120
	{
		I2C_BOARD_INFO("ctw6120", 0x38),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP_GPIO_IRQ(GPIO_TSC_CTW_IRQ),
	},
#endif
};


static void __init blaze_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata var_som_om44_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata var_som_om44_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata var_som_om44_i2c_4_bus_pdata;
static int __init omap4_i2c_init(void)
{

	omap_i2c_hwspinlock_init(1, 0, &var_som_om44_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &var_som_om44_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &var_som_om44_i2c_4_bus_pdata);

	var_som_om44_i2c_3_boardinfo[0].irq = gpio_to_irq(GPIO_TSC2004_IRQ);
	omap_register_i2c_bus_board_data(1, &var_som_om44_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(3, &var_som_om44_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &var_som_om44_i2c_4_bus_pdata);

	omap4_power_init();
	i2c_register_board_info(1, var_som_om44_i2c_1_boardinfo,
				ARRAY_SIZE(var_som_om44_i2c_1_boardinfo));
	omap_register_i2c_bus(3, 400, var_som_om44_i2c_3_boardinfo,	ARRAY_SIZE(var_som_om44_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);


static int var_som_om44_enable_lcd(struct omap_dss_device *dssdev)
{

	gpio_set_value(dssdev->reset_gpio, 1);
	return 0;
}

static void var_som_om44_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 0);

}



static void var_som_om44_hdmi_mux_init(void)
{
	u32 r;

	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);
	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA | OMAP_PULL_UP);
	gpio_direction_input(HDMI_GPIO_HPD);
}

/* Using generic display panel */
static struct panel_generic_dpi_data var_som_om44_panel = {
#ifdef CONFIG_TOUCHSCREEN_CTW6120
	.name 			="ETM_070001ADH6",
#else
	.name			= "URT_UMSH_8423MD_T",
#endif
	.platform_enable	= var_som_om44_enable_lcd,
	.platform_disable	= var_som_om44_disable_lcd,
};

static struct omap_dss_device var_som_om44_panel_device = {
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.reset_gpio	= BACKLIGHT_EN_GPIO,
	.data			= &var_som_om44_panel,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};



static struct omap_dss_device var_som_om44_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *var_som_om44_dss_devices[] = {
	&var_som_om44_panel_device,
	&var_som_om44_hdmi_device,
};

static struct omap_dss_board_info var_som_om44_dss_data = {
	.num_devices	= ARRAY_SIZE(var_som_om44_dss_devices),
	.devices	= var_som_om44_dss_devices,
	.default_device	= &var_som_om44_panel_device,
};

#define BLAZE_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
static struct omapfb_platform_data var_som_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = BLAZE_FB_RAM_SIZE,
			},
		},
	},
};

static void var_som_om44_display_init(void)
{
	gpio_request(BACKLIGHT_EN_GPIO, "backlight_en");
	gpio_direction_output(BACKLIGHT_EN_GPIO , 0); 

	var_som_om44_hdmi_mux_init();
	omap_vram_set_sdram_vram(BLAZE_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&var_som_fb_pdata);
	omap_display_init(&var_som_om44_dss_data);
}

//#ifdef CONFIG_OMAP_MUX
#if 1
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* TSC IRQ; input; GPIO 101 */
	OMAP4_MUX(GPMC_NCS4, OMAP_PIN_INPUT | OMAP_MUX_MODE3),
	/* TSC reset; output = 1; GPIO 102 */
	OMAP4_MUX(GPMC_NCS5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
#ifdef CONFIG_TOUCHSCREEN_CTW6120
	/* TSC CTW IRQ; input; GPIO 174 */
	OMAP4_MUX(KPD_COL3, OMAP_PIN_INPUT | OMAP_MUX_MODE3),	
#endif
        /* Audio ON GPIO 182 */
	OMAP4_MUX(FREF_CLK2_OUT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),

	/*  SD-MMC5, VAR-DVK-44 SD Card slot */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(ABE_DMIC_DIN3, OMAP_MUX_MODE3| OMAP_PIN_OUTPUT),

	/* GPIO 110, SD-MMC5 CD */
	OMAP4_MUX(ABE_MCBSP2_CLKX, OMAP_MUX_MODE3| OMAP_PIN_INPUT),

	/* GPIO 133, VAR-DVK-44 USB HUB Reset */
	OMAP4_MUX(ABE_MCBSP2_FSX, OMAP_MUX_MODE3| OMAP_PIN_OUTPUT),

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	OMAP4_MUX(MCSPI4_CLK, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(MCSPI4_SIMO, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(MCSPI4_SOMI, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(UART4_TX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(UART4_RX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(MCSPI4_CS0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),

	/* BT_EN - GPIO 42 */
	OMAP4_MUX(GPMC_A18, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN IRQ - GPIO 41 */
	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
#endif

#if defined(CONFIG_BT_WILINK) || defined(CONFIG_BT_WILINK_MODULE)
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
#endif

	/* GPIO 171, KS8851 ETH IRQ */
	OMAP4_MUX(KPD_COL3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	
	/* GPIO 1, VAR-DVK-44 HDMI PD */
	OMAP4_MUX(KPD_COL2, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
  	OMAP4_MUX(HDMI_HPD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
    
	OMAP4_MUX(I2C2_SDA, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),	/* UART1 TX */
	OMAP4_MUX(I2C2_SCL, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),	/* UART1 RX */
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),/* UART1 CTS */
	OMAP4_MUX(MCSPI1_CS3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),	/* UART1 RTS */
	
	/* GPIO 184, BACK key */
	OMAP4_MUX(SYS_BOOT0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),	/* GPIO 184 */

	OMAP4_MUX(ABE_MCBSP1_CLKX, OMAP_MUX_MODE0| OMAP_PIN_INPUT),
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0| OMAP_PIN_INPUT),
	OMAP4_MUX(ABE_MCBSP1_FSX, OMAP_MUX_MODE0| OMAP_PIN_INPUT),
	OMAP4_MUX(ABE_MCBSP1_DX, OMAP_MUX_MODE0| OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};



#else
#define board_mux	NULL
#define board_wkup_mux NULL
#endif

static void omap4_sdp4430_wifi_init(void)
{
	if (wl12xx_set_platform_data(&var_som_om44_wlan_data))
		pr_err("Error setting wl12xx data\n");
}
#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
struct usbhs_omap_board_data var_som_om44_usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_ohci_init(void)
{
	struct clk *phy_ref_clk;
	int error;
	unsigned long * addr;
	/* USB PHY Reset */
	omap_mux_init_gpio(VAR_SOM_OM44_PHY_RESET, OMAP_PIN_OUTPUT);

	error = gpio_request(VAR_SOM_OM44_PHY_RESET, "VAR_SOM_OM44_PHY_RESET");
	if (error < 0) {
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, VAR_SOM_OM44_PHY_RESET, error);
		return;
	}

	error = gpio_direction_output(VAR_SOM_OM44_PHY_RESET , 0);
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
			__func__, VAR_SOM_OM44_PHY_RESET, error);
		gpio_free(VAR_SOM_OM44_PHY_RESET);
    }
    

	/*  DVK USB HUB Reset */
	omap_mux_init_gpio(VAR_DVK_OM44_USB_HUB_RESET, OMAP_PIN_OUTPUT);

	error = gpio_request(VAR_DVK_OM44_USB_HUB_RESET, "VAR_DVK_OM44_USB_HUB_RESET");
	if (error < 0) {
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, VAR_DVK_OM44_USB_HUB_RESET, error);
		return;
	}

	error = gpio_direction_output(VAR_DVK_OM44_USB_HUB_RESET , 0);
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
			__func__, VAR_DVK_OM44_USB_HUB_RESET, error);
		gpio_free(VAR_DVK_OM44_USB_HUB_RESET);
	}


	addr=(unsigned long *)ioremap(0x4a100600,4); // PBIAS Control Register
	*(unsigned long* )addr = 0x1d700000; // Set VDDS voltage

    addr=(unsigned long *)ioremap(0x4A31E600,4);
    *(unsigned long* )addr = 0x10000000;//Notify VDDS stable, disable strong pull down
   
	 mdelay(100); // Wait for VDDS power on WK pins

	addr=(unsigned long *)ioremap(0x4A31E040,4);
	*(unsigned long* )addr = 0xf000f;//GPIOWK pads I/O control. wk0 wk1 as pull down


	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		goto error1;
	}
	clk_set_rate(phy_ref_clk, 19200000);
	clk_enable(phy_ref_clk);
	mdelay(20);

	// VAR-SOM-OM44 Phy reset de-assert
	gpio_set_value(VAR_SOM_OM44_PHY_RESET, 1);

	usbhs_init(&var_som_om44_usbhs_bdata);
	mdelay(250);
   *(unsigned long* )addr = 0x1f000f;//wk1 pull up, VAR-SOM-OM44 hub on
   *(unsigned long* )addr = 0x11b011b;//Set wk1 pull up, Enabled ethernet controller

	// DVK USB HUB de-assert
	gpio_set_value(VAR_DVK_OM44_USB_HUB_RESET, 1);
	
	return;
error1:
	pr_err("Unable to initialize EHCI power/reset\n");
	return;
}
#else
static void __init omap4_ehci_ohci_init(void){}
#endif

#define GPIO_CAM_GLOBAL_POWER 83
#define GPIO_CAM1_RESET       3
#define GPIO_CAM2_RESET       175

static void var_som_om44_camera_init(void)
{
	omap_mux_init_gpio(GPIO_CAM_GLOBAL_POWER, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_CAM1_RESET, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_CAM2_RESET, OMAP_PIN_OUTPUT);
}


static void __init var_som_om44_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	omap4_mux_init(board_mux, NULL, package);

	omap_mux_init_gpio(VAR_SOM_OM44_GPIO_AUD_PWRON, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	omap_board_config = var_som_om44_config;
	omap_board_config_size = ARRAY_SIZE(var_som_om44_config);

	omap_init_board_version(0);

	omap4_audio_conf();
	omap4_create_board_props();
	blaze_pmic_mux_init();
	omap4_i2c_init();


	omap4_register_ion();
	platform_add_devices(var_som_om44_devices, ARRAY_SIZE(var_som_om44_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	board_serial_init();
	omap4_sdp4430_wifi_init();
	omap4_twl6030_hsmmc_init(mmc);

	omap4_ehci_ohci_init();

	usb_musb_init(&musb_board_data);

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		var_som_om44_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(var_som_om44_spi_board_info,
				ARRAY_SIZE(var_som_om44_spi_board_info));
	}
#endif

	omap_dmm_init();
	var_som_om44_display_init();

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

#ifdef CONFIG_TOUCHSCREEN_CTW6120
	tsc_ctw6120_init_irq();
#endif
	omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();

	var_som_om44_camera_init();
}

static void __init var_som_om44_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init var_som_om44_reserve(void)
{
	omap_init_ram_size();

#ifdef CONFIG_ION_OMAP
	omap_android_display_setup(&var_som_om44_dss_data,
				   NULL,
				   NULL,
				   &var_som_fb_pdata,
				   get_omap_ion_platform_data());
 	omap_ion_init();
#else
	omap_android_display_setup(&var_som_om44_dss_data,
				   NULL,
				   NULL,
				   &var_som_fb_pdata,
				   NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM,
					PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(VAR_SOM_OM4460, "VAR-SOM-OM4460")
	/* Maintainer: Uri Yosef - Variscite Ltd */
	.boot_params	= 0x80000100,
	.reserve	= var_som_om44_reserve,
	.map_io		= var_som_om44_map_io,
	.init_early	= var_som_om44_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= var_som_om44_init,
	.timer		= &omap_timer,
MACHINE_END
