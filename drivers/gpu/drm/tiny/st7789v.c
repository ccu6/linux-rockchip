// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for ST7789V panels
 *
 * Copyright 2018 David Lechner <david@lechnology.com>
 *
 * Based on mi0283qt.c:
 * Copyright 2016 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

// ST7789V specific commands used in init
#define ST7789V_NOP			0x00
#define ST7789V_SWRESET		0x01
#define ST7789V_RDDID		0x04
#define ST7789V_RDDST		0x09

#define ST7789V_RDDPM		0x0A      // Read display power mode
#define ST7789V_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7789V_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7789V_RDDIM		0x0D      // Read display image mode
#define ST7789V_RDDSM		0x0E      // Read display signal mode
#define ST7789V_RDDSR		0x0F      // Read display self-diagnostic result (ST7789VV)

#define ST7789V_SLPIN		0x10
#define ST7789V_SLPOUT		0x11
#define ST7789V_PTLON		0x12
#define ST7789V_NORON		0x13

#define ST7789V_INVOFF		0x20
#define ST7789V_INVON		0x21
#define ST7789V_GAMSET		0x26      // Gamma set
#define ST7789V_DISPOFF		0x28
#define ST7789V_DISPON		0x29
#define ST7789V_CASET		0x2A
#define ST7789V_RASET		0x2B
#define ST7789V_RAMWR		0x2C
#define ST7789V_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7789V_RAMRD		0x2E

#define ST7789V_PTLAR		0x30
#define ST7789V_VSCRDEF		0x33      // Vertical scrolling definition (ST7789VV)
#define ST7789V_TEOFF		0x34      // Tearing effect line off
#define ST7789V_TEON		0x35      // Tearing effect line on
#define ST7789V_MADCTL		0x36      // Memory data access control
#define ST7789V_IDMOFF		0x38      // Idle mode off
#define ST7789V_IDMON		0x39      // Idle mode on
#define ST7789V_RAMWRC		0x3C      // Memory write continue (ST7789VV)
#define ST7789V_RAMRDC		0x3E      // Memory read continue (ST7789VV)
#define ST7789V_COLMOD		0x3A

#define ST7789V_RAMCTRL		0xB0      // RAM control
#define ST7789V_RGBCTRL		0xB1      // RGB control
#define ST7789V_PORCTRL		0xB2      // Porch control
#define ST7789V_FRCTRL1		0xB3      // Frame rate control
#define ST7789V_PARCTRL		0xB5      // Partial mode control
#define ST7789V_GCTRL		0xB7      // Gate control
#define ST7789V_GTADJ		0xB8      // Gate on timing adjustment
#define ST7789V_DGMEN		0xBA      // Digital gamma enable
#define ST7789V_VCOMS		0xBB      // VCOMS setting
#define ST7789V_LCMCTRL		0xC0      // LCM control
#define ST7789V_IDSET		0xC1      // ID setting
#define ST7789V_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7789V_VRHS			0xC3      // VRH set
#define ST7789V_VDVSET		0xC4      // VDV setting
#define ST7789V_VCMOFSET		0xC5      // VCOMS offset set
#define ST7789V_FRCTR2		0xC6      // FR Control 2
#define ST7789V_CABCCTRL		0xC7      // CABC control
#define ST7789V_REGSEL1		0xC8      // Register value section 1
#define ST7789V_REGSEL2		0xCA      // Register value section 2
#define ST7789V_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7789V_PWCTRL1		0xD0      // Power control 1
#define ST7789V_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7789V_CMD2EN		0xDF      // Command 2 enable
#define ST7789V_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7789V_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7789V_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7789V_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7789V_GATECTRL		0xE4      // Gate control
#define ST7789V_SPI2EN		0xE7      // SPI2 enable
#define ST7789V_PWCTRL2		0xE8      // Power control 2
#define ST7789V_EQCTRL		0xE9      // Equalize time control
#define ST7789V_PROMCTRL		0xEC      // Program control
#define ST7789V_PROMEN		0xFA      // Program mode enable
#define ST7789V_NVMSET		0xFC      // NVM setting
#define ST7789V_PROMACT		0xFE      // Program action

#define ST7789V_MADCTL_BGR	BIT(3)
#define ST7789V_MADCTL_MV	BIT(5)
#define ST7789V_MADCTL_MX	BIT(6)
#define ST7789V_MADCTL_MY	BIT(7)

static void st7789v_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	mipi_dbi_command(dbi, ST7789V_SLPOUT);   // Sleep out
	msleep(120);

	mipi_dbi_command(dbi, ST7789V_NORON);    // Normal display mode on

  //------------------------------display and color format setting--------------------------------//
	mipi_dbi_command(dbi, ST7789V_MADCTL, ST7789V_MADCTL_BGR);

  // JLX240 display datasheet
	mipi_dbi_command(dbi, 0xB6, 0x0A, 0x82);

	mipi_dbi_command(dbi, ST7789V_COLMOD, 0x55);
	msleep(10);

  //--------------------------------ST7789V Frame rate setting----------------------------------//
	mipi_dbi_command(dbi, ST7789V_PORCTRL, 0x0c, 0x0c, 0x00, 0x33, 0x33);

	mipi_dbi_command(dbi, ST7789V_GCTRL, 0x35);      // Voltages: VGH / VGL

  //---------------------------------ST7789V Power setting--------------------------------------//
	mipi_dbi_command(dbi, ST7789V_VCOMS, 0x28);		// JLX240 display datasheet
	mipi_dbi_command(dbi, ST7789V_LCMCTRL, 0x0C);
	mipi_dbi_command(dbi, ST7789V_VDVVRHEN, 0x01, 0xFF);
	mipi_dbi_command(dbi, ST7789V_VRHS, 0x10);       // voltage VRHS
	mipi_dbi_command(dbi, ST7789V_VDVSET, 0x20);
	mipi_dbi_command(dbi, ST7789V_FRCTR2, 0x0f);
	mipi_dbi_command(dbi, ST7789V_PWCTRL1, 0xa4, 0xa1);

  //--------------------------------ST7789V gamma setting---------------------------------------//
	mipi_dbi_command(dbi, ST7789V_PVGAMCTRL, 0xd0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0e, 0x12, 0x14, 0x17);
	mipi_dbi_command(dbi, ST7789V_NVGAMCTRL, 0xd0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x31, 0x54, 0x47, 0x0e, 0x1c, 0x17, 0x1b, 0x1e);

	mipi_dbi_command(dbi, ST7789V_INVON);

	mipi_dbi_command(dbi, ST7789V_CASET, 0x00, 0x00, 0x00, 0xE5);    // Column address set
	mipi_dbi_command(dbi, ST7789V_RASET, 0x00, 0x00, 0x00, 0xE5);    // Row address set
	msleep(120);

	mipi_dbi_command(dbi, ST7789V_DISPON);    //Display on
	msleep(120);

out_enable:
	switch (dbidev->rotation) {
	default:
		//addr_mode = ST7789_MADCTL_MX;
		addr_mode = 0x00;
		break;
	case 90:
		//addr_mode = ST7789_MADCTL_MV;
		addr_mode = ST7789V_MADCTL_MX | ST7789V_MADCTL_MV;
		break;
	case 180:
		//addr_mode = ST7789_MADCTL_MY;
		addr_mode = ST7789V_MADCTL_MX | ST7789V_MADCTL_MY;
                // rowstart 80
		break;
	case 270:
		//addr_mode = ST7789_MADCTL_MV | ST7789_MADCTL_MY | ST7789_MADCTL_MX;
		addr_mode = ST7789V_MADCTL_MV | ST7789V_MADCTL_MY;
                // colstart 80;
		break;
	}
	addr_mode |= ST7789V_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = st7789v_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
};

static const struct drm_display_mode st7789v_mode = {
	DRM_SIMPLE_MODE(240, 320, 31, 41),
};

DEFINE_DRM_GEM_DMA_FOPS(st7789v_fops);

static const struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v",
	.desc			= "itronix ST7789V",
	.date			= "20241022",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id st7789v_of_match[] = {
	{ .compatible = "CCU6,ST7789V-tinydrm" },
	{ }
};
MODULE_DEVICE_TABLE(of, st7789v_of_match);

static const struct spi_device_id st7789v_id[] = {
	{ "st7789v-240x320LCD", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7789v_id);

static int st7789v_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &st7789v_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_dev_init(dbidev, &st7789v_pipe_funcs, &st7789v_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static void st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7789v_spi_driver = {
	.driver = {
		.name = "st7789v",
		.of_match_table = st7789v_of_match,
	},
	.id_table = st7789v_id,
	.probe = st7789v_probe,
	.remove = st7789v_remove,
	.shutdown = st7789v_shutdown,
};
module_spi_driver(st7789v_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7789V DRM driver");
MODULE_AUTHOR("CCU6 <holydeal1202@gmail.com>");
MODULE_LICENSE("GPL");
