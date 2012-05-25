/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/spi/spi.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/mcspi.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

#ifdef CONFIG_PM
static struct omap_opp * _omap35x_mpu_rate_table        = omap35x_mpu_rate_table;
static struct omap_opp * _omap37x_mpu_rate_table        = omap37x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table        = omap35x_dsp_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table        = omap37x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table         = omap35x_l3_rate_table;
static struct omap_opp * _omap37x_l3_rate_table         = omap37x_l3_rate_table;
#else   /* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table        = NULL;
static struct omap_opp * _omap37x_mpu_rate_table        = NULL;
static struct omap_opp * _omap35x_dsp_rate_table        = NULL;
static struct omap_opp * _omap37x_dsp_rate_table        = NULL;
static struct omap_opp * _omap35x_l3_rate_table         = NULL;
static struct omap_opp * _omap37x_l3_rate_table         = NULL;
#endif  /* CONFIG_PM */

#define LCD_I2C_ADDR         ( 0x78 >> 1 )

#if defined(CONFIG_SOC_CAMERA_MT9P031)
#include <media/v4l2-int-device.h>
#include <media/mt9p031.h>
extern struct mt9p031_platform_data mt9p031_pdata;
#endif

#if defined(CONFIG_VIDEO_MT9V113) || defined(CONFIG_VIDEO_MT9V113_MODULE)
#include <media/v4l2-int-device.h>
#include <media/mt9v113.h>
extern struct mt9v113_platform_data mt9v113_pdata;
#endif

#if defined(CONFIG_VIDEO_MT9T112) || defined(CONFIG_VIDEO_MT9T112_MODULE)
#include <media/v4l2-int-device.h>
#include <media/mt9t112.h>
extern struct mt9t112_platform_data mt9t112_pdata;
#endif

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

char expansionboard_name[16];
char cameraboard_name[16];

#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_ENC28J60_IRQ 157

static struct omap2_mcspi_device_config enc28j60_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy_spi_board_info[] __initdata = {
	{
		.modalias		= "enc28j60",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 20000000,
		.controller_data	= &enc28j60_spi_chip_info,
	},
};

static void __init omap3beagle_enc28j60_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, "ENC28J60_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_ENC28J60_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, 0);
		omap3beagle_zippy_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_ENC28J60_IRQ);
		set_irq_type(omap3beagle_zippy_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for ENC28J60_IRQ\n");
		return;
	}

	spi_register_board_info(omap3beagle_zippy_spi_board_info,
			ARRAY_SIZE(omap3beagle_zippy_spi_board_info));
}

#else
static inline void __init omap3beagle_enc28j60_init(void) { return; }
#endif

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_KS8851_IRQ 157

static struct omap2_mcspi_device_config ks8851_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy2_spi_board_info[] __initdata = {
	{
		.modalias		= "ks8851",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 36000000,
		.controller_data	= &ks8851_spi_chip_info,
	},
};

static void __init omap3beagle_ks8851_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_KS8851_IRQ, "KS8851_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_KS8851_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_KS8851_IRQ, 0);
		omap3beagle_zippy2_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_KS8851_IRQ);
		set_irq_type(omap3beagle_zippy2_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for KS8851_IRQ\n");
		return;
	}

	spi_register_board_info(omap3beagle_zippy2_spi_board_info,
							ARRAY_SIZE(omap3beagle_zippy2_spi_board_info));
}

#else
static inline void __init omap3beagle_ks8851_init(void) { return; }
#endif

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3beagle_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= omap3beagle_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3beagle_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource omap3beagle_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3beagle_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3beagle_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3beagle_nand_resource,
};


#include "sdram-micron-mt46h32m32lf-6.h"

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_wp	= 29,
	},
	{
		.mmc		= 2,
		.wires		= 4,
		.transceiver	= true,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vmmc2_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;

    mmc[2].gpio_cd = 21; 

	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
	beagle_vsim_supply.dev = mmc[0].dev;
    beagle_vmmc2_supply.dev = mmc[2].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */

	if (cpu_is_omap3630()) {
		/* Power on camera interface */
		gpio_request(gpio + 2, "CAM_EN");
		gpio_direction_output(gpio + 2, 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	}
	else {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	}


	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};


static struct platform_device beagle_cam_device = {
	.name		= "beagle_cam",
	.id		= -1,
};

static struct regulator_consumer_supply beagle_vaux3_supply = {
	.supply		= "cam_1v8",
	.dev		= &beagle_cam_device.dev,
};

static struct regulator_consumer_supply beagle_vaux4_supply = {
	.supply		= "cam_2v8",
	.dev		= &beagle_cam_device.dev,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data beagle_vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vaux3_supply,
};

/* VAUX4 for CAM_2V8 */
static struct regulator_init_data beagle_vaux4 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vaux4_supply,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vsim_supply,
};

/* VMMC2 for MMC3 */
static struct regulator_init_data beagle_vmmc2 = {
    .constraints = {
        .min_uV         = 3150000,
        .max_uV         = 3150000,
        .apply_uV       = true,
        .valid_modes_mask   = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies  = 1,
    .consumer_supplies  = &beagle_vmmc2_supply,
};

static struct twl4030_usb_data beagle_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data beagle_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data beagle_codec_data = {
	.audio_mclk = 26000000,
	.audio = &beagle_audio_data,
};

static struct twl4030_madc_platform_data beagle_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data beagle_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &beagle_usb_data,
	.gpio		= &beagle_gpio_data,
	.codec		= &beagle_codec_data,
	.madc		= &beagle_madc_data,
	.vmmc1		= &beagle_vmmc1,
	.vmmc2		= &beagle_vmmc2,
	.vsim		= &beagle_vsim,
	.vaux3		= &beagle_vaux3,
	.vaux4		= &beagle_vaux4,
};

static struct i2c_board_info __initdata beagle_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &beagle_twldata,
	},
};


#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
#include <linux/i2c/at24.h>

static struct at24_platform_data m24c01 = {
	        .byte_len       = SZ_1K / 8,
	        .page_size      = 16,
};

#if defined(CONFIG_RTC_DRV_DS1307) || \
	defined(CONFIG_RTC_DRV_DS1307_MODULE)

static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data	= &m24c01,
	},
};
#else
static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data  = &m24c01,
	},
};
#endif
#else
static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {};
#endif

static struct i2c_board_info __initdata beagle_lbcmvga_i2c2_boardinfo[] = {
#if defined(CONFIG_VIDEO_MT9V113) || defined(CONFIG_VIDEO_MT9V113_MODULE)
	{
		I2C_BOARD_INFO("mt9v113", MT9V113_I2C_ADDR),
		.platform_data	= &mt9v113_pdata,
	},
#endif
};

static struct i2c_board_info __initdata beagle_lbcm3m1_i2c2_boardinfo[] = {
#if defined(CONFIG_VIDEO_MT9T112) || defined(CONFIG_VIDEO_MT9T112_MODULE)
	{
		I2C_BOARD_INFO("mt9t112", MT9T112_I2C_ADDR),
		.platform_data	= &mt9t112_pdata,
	},
#endif
};

static struct i2c_board_info __initdata beagle_li5m03_lcd_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("lcdpug", LCD_I2C_ADDR),
	},
#if defined(CONFIG_SOC_CAMERA_MT9P031)
	{
		I2C_BOARD_INFO("mt9p031", MT9P031_I2C_ADDR),
		.platform_data	= &mt9p031_pdata,
	},
#endif
};

static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c1_boardinfo,
			ARRAY_SIZE(beagle_i2c1_boardinfo));

	if (!strcmp(expansionboard_name, "zippy") ||
	   !strcmp(expansionboard_name, "zippy2")) {
		printk(KERN_INFO "Beagle expansionboard:"
				 " registering i2c2 bus for zippy/zippy2\n");
		omap_register_i2c_bus(2, 400,  beagle_zippy_i2c2_boardinfo,
				ARRAY_SIZE(beagle_zippy_i2c2_boardinfo));
	} else {
		if (!strcmp(cameraboard_name, "lbcmvga")) {
			printk(KERN_INFO "Beagle cameraboard:"
					 " registering i2c2 bus for lbcmvga\n");
			omap_register_i2c_bus(2, 400,  beagle_lbcmvga_i2c2_boardinfo,
					ARRAY_SIZE(beagle_lbcmvga_i2c2_boardinfo));
		} else if (!strcmp(cameraboard_name, "lbcm3m1")) {
			printk(KERN_INFO "Beagle cameraboard:"
					 " registering i2c2 bus for lbcm3m1\n");
			omap_register_i2c_bus(2, 400,  beagle_lbcm3m1_i2c2_boardinfo,
					ARRAY_SIZE(beagle_lbcm3m1_i2c2_boardinfo));
		} else if (!strcmp(cameraboard_name, "li5m03")) {
			printk(KERN_INFO "Beagle Leopard 5MP cameraboard:"
					 " registering i2c2 bus for li5m03 and LCD %x\n",  ARRAY_SIZE(beagle_li5m03_lcd_i2c2_boardinfo));
			omap_register_i2c_bus(2, 100,  beagle_li5m03_lcd_i2c2_boardinfo,
					ARRAY_SIZE(beagle_li5m03_lcd_i2c2_boardinfo));
		} else {
			printk(KERN_INFO "Unknown cameraboard: %s\n", cameraboard_name);
			omap_register_i2c_bus(2, 400, NULL, 0);
		}
	}
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
	{
		.name			= "pugboard::exam_mode",
        .gpio           = 15,
        .active_low     = true,
	},
    {
        .name           = "pugboard::camera0",
        .gpio           = 144,
        .active_low     = true,
    },
    {
         .name          = "pugboard::camera1",
         .gpio          = 146,
        .active_low     = true,
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

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
    {
            .code                   = KEY_UP,
            .gpio                   = 157,
            .desc                   = "up",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_DOWN,
            .gpio                   = 139,
            .desc                   = "down",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_LEFT,
            .gpio                   = 138,
            .desc                   = "left",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_RIGHT,
            .gpio                   = 137,
            .desc                   = "right",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_MENU,
            .gpio                   = 136,
            .desc                   = "menu",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_VOLUMEUP,
            .gpio                   = 135,
            .desc                   = "volume",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_FASTFORWARD,
            .gpio                   = 134,
            .desc                   = "velocidade",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_SPACE,
            .gpio                   = 133,
            .desc                   = "space",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_POWER2,
            .gpio                   = 132,
            .desc                   = "charging",
            .wakeup                 = 1,
            .active_low     = 0,
    },
    {
            .code                   = KEY_AUDIO,
            .gpio                   = 131,
            .desc                   = "audio",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_SUBTITLE,
            .gpio                   = 130,
            .desc                   = "braille",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_WLAN,
            .gpio                   = 170,
            .desc                   = "wifi",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_SCREENLOCK,
            .gpio                   = 22,
            .desc                   = "camera lock",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_SLEEP,
            .gpio                   = 23,
            .desc                   = "on",
            .wakeup                 = 1,
            .active_low     = 0,
    },
    {
            .code                   = KEY_POWER,
            .gpio                   = 14,
            .desc                   = "ext power",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_MODE,
            .gpio                   = 143,
            .desc                   = "mode",
            .wakeup                 = 1,
            .active_low     = 1,
    },
    {
            .code                   = KEY_CAMERA,
            .gpio                   = 142,
            .desc                   = "photo",
            .wakeup                 = 1,
            .active_low     = 1,
    },
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

#ifdef CONFIG_PUGBOARD
static struct platform_device metec_device = {
    .name = "metec",
    .id = -1.
};
#endif

static struct spi_board_info beaglefpga_mcspi_board_info[] = {
	// spi 4.0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, //48 Mbps
		.bus_num	= 4,
		.chip_select	= 0,
		.mode = SPI_MODE_1,
	},
};

static void __init beaglefpga_init_spi(void)
{
	/* hook the spi ports to the spidev driver */
	spi_register_board_info(beaglefpga_mcspi_board_info,
		ARRAY_SIZE(beaglefpga_mcspi_board_info));
}

static void __init omap3_beagle_init_irq(void)
{
        if (cpu_is_omap3630())
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap37x_mpu_rate_table,
                                        _omap37x_dsp_rate_table,
                                        _omap37x_l3_rate_table);
        }
        else
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap35x_mpu_rate_table,
                                        _omap35x_dsp_rate_table,
                                        _omap35x_l3_rate_table);
        }
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
}

static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
#ifdef CONFIG_PUGBOARD
    &metec_device,
#endif
	&beagle_cam_device,
};

static void __init omap3beagle_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3beagle_nand_data.cs = nandcs;
		omap3beagle_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3beagle_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3beagle_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* Camera - Parallel Data */
	OMAP3_MUX(CAM_D0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D8, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D9, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D10, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D11, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* Camera - HS/VS signals */
	OMAP3_MUX(CAM_HS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_VS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

    /* I2C 2 : Mux enabling */
    OMAP3_MUX(I2C2_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
    OMAP3_MUX(I2C2_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* Camera - Reset GPIO 98 */
	OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* Camera - XCLK */
	OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(expansionboard_name, str, 16);
	printk(KERN_INFO "Beagle expansionboard: %s\n", expansionboard_name);
	return 0;
}

static int __init cameraboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(cameraboard_name, str, 16);
	printk(KERN_INFO "Beagle cameraboard: %s\n", cameraboard_name);
	return 0;
}

static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_i2c_init();

	if (cpu_is_omap3630()) {
		gpio_buttons[0].gpio = 4;
	}

	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));

	omap_serial_init();

	if(!strcmp(expansionboard_name, "zippy"))
	{
		printk(KERN_INFO "Beagle expansionboard: initializing enc28j60\n");
		omap3beagle_enc28j60_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}

	if(!strcmp(expansionboard_name, "zippy2"))
	{
		printk(KERN_INFO "Beagle expansionboard: initializing ks_8851\n");
		omap3beagle_ks8851_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}

	if(!strcmp(expansionboard_name, "trainer"))
	{
		printk(KERN_INFO "Beagle expansionboard: exporting GPIOs 130-141,162 to userspace\n");
		gpio_request(130, "sysfs");
		gpio_export(130, 1);
		gpio_request(131, "sysfs");
		gpio_export(131, 1);
		gpio_request(132, "sysfs");
		gpio_export(132, 1);
		gpio_request(133, "sysfs");
		gpio_export(133, 1);
		gpio_request(134, "sysfs");
		gpio_export(134, 1);
		gpio_request(135, "sysfs");
		gpio_export(135, 1);
		gpio_request(136, "sysfs");
		gpio_export(136, 1);
		gpio_request(137, "sysfs");
		gpio_export(137, 1);
		gpio_request(138, "sysfs");
		gpio_export(138, 1);
		gpio_request(139, "sysfs");
		gpio_export(139, 1);
		gpio_request(140, "sysfs");
		gpio_export(140, 1);
		gpio_request(141, "sysfs");
		gpio_export(141, 1);
		gpio_request(162, "sysfs");
		gpio_export(162, 1);
	}

	if(!strcmp(expansionboard_name, "beaglefpga"))
	{
		printk(KERN_INFO "Beagle expansionboard: Using McSPI for SPI\n");
		beaglefpga_init_spi();
	}

	usb_musb_init();
	usb_ehci_init(&ehci_pdata);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
}
static void __init omap3_beagle_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

early_param("buddy", expansionboard_setup);
early_param("camera", cameraboard_setup);

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_beagle_map_io,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
