/*
 * A V4L2 driver for OmniVision OV5647 cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

#define SENSOR_NAME "ov5647"

#define MIPI_CTRL00_CLOCK_LANE_GATE		BIT(5)
#define MIPI_CTRL00_LINE_SYNC_ENABLE		BIT(4)
#define MIPI_CTRL00_BUS_IDLE			BIT(2)
#define MIPI_CTRL00_CLOCK_LANE_DISABLE		BIT(0)

#define OV5647_SW_STANDBY		0x0100
#define OV5647_SW_RESET			0x0103
#define OV5647_REG_CHIPID_H		0x300A
#define OV5647_REG_CHIPID_L		0x300B
#define OV5640_REG_PAD_OUT		0x300D
#define OV5647_REG_FRAME_OFF_NUMBER	0x4202
#define OV5647_REG_MIPI_CTRL00		0x4800
#define OV5647_REG_MIPI_CTRL14		0x4814

/* AEC/AGC control register */
#define OV5647_REG_AEC_AGC		0x3503
#define OV5647_AEC_ENABLE		BIT(0)  /* 0=auto, 1=manual */
#define OV5647_AGC_ENABLE		BIT(1)  /* 0=auto, 1=manual */

/* Exposure registers (20-bit, but we use 16-bit) */
#define OV5647_REG_EXPOSURE_HI		0x3500
#define OV5647_REG_EXPOSURE_MID		0x3501
#define OV5647_REG_EXPOSURE_LO		0x3502

/* Gain registers (10-bit) */
#define OV5647_REG_GAIN_HI		0x350a
#define OV5647_REG_GAIN_LO		0x350b

/* AWB control register */
#define OV5647_REG_AWB			0x5001

#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xffff

#define OV5647_ROW_START		0x01
#define OV5647_ROW_START_MIN		0
#define OV5647_ROW_START_MAX		2004
#define OV5647_ROW_START_DEF		54

#define OV5647_COLUMN_START		0x02
#define OV5647_COLUMN_START_MIN		0
#define OV5647_COLUMN_START_MAX		2750
#define OV5647_COLUMN_START_DEF		16

#define OV5647_WINDOW_HEIGHT		0x03
#define OV5647_WINDOW_HEIGHT_MIN	2
#define OV5647_WINDOW_HEIGHT_MAX	2006
#define OV5647_WINDOW_HEIGHT_DEF	1944

#define OV5647_WINDOW_WIDTH		0x04
#define OV5647_WINDOW_WIDTH_MIN		2
#define OV5647_WINDOW_WIDTH_MAX		2752
#define OV5647_WINDOW_WIDTH_DEF		2592

/* Pixel array dimensions for get_selection */
#define OV5647_NATIVE_WIDTH		2624U
#define OV5647_NATIVE_HEIGHT		1956U
#define OV5647_PIXEL_ARRAY_LEFT		16U
#define OV5647_PIXEL_ARRAY_TOP		16U
#define OV5647_PIXEL_ARRAY_WIDTH	2592U
#define OV5647_PIXEL_ARRAY_HEIGHT	1944U

struct regval_list {
	u16 addr;
	u8 data;
};

/*
 * OV5647 pixel rate for 640x480@30fps mode
 * Based on: (HTS * VTS * fps) = (1896 * 984 * 30) ≈ 56MHz
 * Reference: mainline Linux ov5647.c
 */
#define OV5647_PIXEL_RATE		55969920

/*
 * Link frequency for MIPI CSI-2
 * For 2-lane, 8-bit Bayer, DDR:
 * link_freq = pixel_rate * bits_per_pixel / (2 * lanes)
 * link_freq = 55969920 * 8 / 4 = 111939840 Hz
 */
#define OV5647_LINK_FREQ		111939840

static const s64 ov5647_link_freq_menu[] = {
	OV5647_LINK_FREQ,
};

struct ov5647 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct v4l2_mbus_framefmt	format;
	unsigned int			width;
	unsigned int			height;
	int				power_count;
	struct clk			*xclk;
	struct gpio_desc		*reset_gpio;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_ctrl		*pixel_rate;
	struct v4l2_ctrl		*link_freq;
	struct v4l2_ctrl		*auto_exp;
	struct v4l2_ctrl		*auto_gain;
	struct v4l2_ctrl		*auto_wb;
	struct v4l2_ctrl		*exposure;
	struct v4l2_ctrl		*gain;
};

static inline struct ov5647 *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5647, sd);
}

static struct regval_list sensor_oe_disable_regs[] = {
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
};

static struct regval_list sensor_oe_enable_regs[] = {
	{0x3000, 0x0f},
	{0x3001, 0xff},
	{0x3002, 0xe4},
};

static struct regval_list ov5647_640x480[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3034, 0x08},
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x303c, 0x11},
	{0x3106, 0xf5},
	{0x3821, 0x07},
	{0x3820, 0x41},
	{0x3827, 0xec},
	{0x370c, 0x0f},
	{0x3612, 0x59},
	{0x3618, 0x00},
	{0x5000, 0x06},
	{0x5001, 0x01},
	{0x5002, 0x41},
	{0x5003, 0x00},  /* Disable embedded data output */
	{0x503d, 0x00},  /* Disable test pattern - output real image data */
	{0x5a00, 0x08},
	/* Auto exposure and auto gain enabled by default */
	{0x3503, 0x00},  /* AEC/AGC auto: bit0=AEC, bit1=AGC, 0=auto */
	{0x3500, 0x00},  /* Exposure [19:16] - initial value */
	{0x3501, 0x40},  /* Exposure [15:8] - initial value */
	{0x3502, 0x00},  /* Exposure [7:0] */
	{0x350a, 0x00},  /* Gain [9:8] - initial value */
	{0x350b, 0x40},  /* Gain [7:0] - initial value */
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3814, 0x31},  /* X subsample (working) */
	{0x3815, 0x31},  /* Y subsample (working) */
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x3808, 0x02},  /* X output: 640 */
	{0x3809, 0x80},
	{0x380a, 0x01},  /* Y output: 480 */
	{0x380b, 0xe0},
	{0x3800, 0x00},  /* X addr start high */
	{0x3801, 0x00},  /* X addr start low */
	{0x3802, 0x00},  /* Y addr start high */
	{0x3803, 0x00},  /* Y addr start low */
	{0x3804, 0x0a},  /* X addr end high */
	{0x3805, 0x3f},  /* X addr end low = 2623 */
	{0x3806, 0x07},  /* Y addr end high */
	{0x3807, 0xa1},  /* Y addr end low = 1953 */
	{0x3811, 0x08},  /* ISP X offset */
	{0x3813, 0x02},  /* ISP Y offset */
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x04},
	{0x3a0e, 0x03},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4000, 0x09},
	{0x4837, 0x24},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	/*
	 * MIPI_CTRL00: critical for frame sync
	 * 0x34 = clock_lane_gate(0x20) + lp11_when_idle(0x10) + bus_idle(0x04)
	 * Reference: mainline Linux ov5647 driver
	 */
	{0x4800, 0x34},
	{0x0100, 0x01},
};

static int ov5647_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data, 3);
	if (ret < 0)
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);

	return ret;
}

static int ov5647_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data_w, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);

	return ret;
}

static int ov5647_write_array(struct v4l2_subdev *sd,
				struct regval_list *regs, int array_size)
{
	int i, ret;

	for (i = 0; i < array_size; i++) {
		ret = ov5647_write(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ov5647_set_virtual_channel(struct v4l2_subdev *sd, int channel)
{
	u8 channel_id;
	int ret;

	ret = ov5647_read(sd, OV5647_REG_MIPI_CTRL14, &channel_id);
	if (ret < 0)
		return ret;

	channel_id &= ~(3 << 6);
	return ov5647_write(sd, OV5647_REG_MIPI_CTRL14, channel_id | (channel << 6));
}

static int ov5647_stream_on(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 mipi_ctrl_before;
	int ret;

	/* Debug: read MIPI_CTRL00 before modification */
	ov5647_read(sd, OV5647_REG_MIPI_CTRL00, &mipi_ctrl_before);
	dev_info(&client->dev, "OV5647: stream_on, MIPI_CTRL00 before=0x%02x\n",
		 mipi_ctrl_before);

	/*
	 * Enable MIPI output with line sync (LP-11 when idle).
	 * LINE_SYNC_ENABLE (0x10) is critical for proper MIPI frame sync.
	 */
	ret = ov5647_write(sd, OV5647_REG_MIPI_CTRL00,
			   MIPI_CTRL00_LINE_SYNC_ENABLE | MIPI_CTRL00_BUS_IDLE);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, OV5647_REG_FRAME_OFF_NUMBER, 0x00);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, OV5640_REG_PAD_OUT, 0x00);
	dev_info(&client->dev, "OV5647: stream_on complete, ret=%d\n", ret);
	return ret;
}

static int ov5647_stream_off(struct v4l2_subdev *sd)
{
	int ret;

	ret = ov5647_write(sd, OV5647_REG_MIPI_CTRL00, MIPI_CTRL00_CLOCK_LANE_GATE
			   | MIPI_CTRL00_BUS_IDLE | MIPI_CTRL00_CLOCK_LANE_DISABLE);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, OV5647_REG_FRAME_OFF_NUMBER, 0x0f);
	if (ret < 0)
		return ret;

	return ov5647_write(sd, OV5640_REG_PAD_OUT, 0x01);
}

static int set_sw_standby(struct v4l2_subdev *sd, bool standby)
{
	int ret;
	u8 rdval;

	ret = ov5647_read(sd, OV5647_SW_STANDBY, &rdval);
	if (ret < 0)
		return ret;

	if (standby)
		rdval &= ~0x01;
	else
		rdval |= 0x01;

	return ov5647_write(sd, OV5647_SW_STANDBY, rdval);
}

static int __sensor_init(struct v4l2_subdev *sd)
{
	int ret;
	u8 resetval, rdval, mipi_ctrl;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "OV5647: __sensor_init starting\n");

	ret = ov5647_read(sd, OV5647_SW_STANDBY, &rdval);
	if (ret < 0)
		return ret;

	ret = ov5647_write_array(sd, ov5647_640x480,
					ARRAY_SIZE(ov5647_640x480));
	if (ret < 0) {
		dev_err(&client->dev, "write sensor default regs error\n");
		return ret;
	}

	/* Debug: verify MIPI_CTRL00 was set correctly */
	ov5647_read(sd, OV5647_REG_MIPI_CTRL00, &mipi_ctrl);
	dev_info(&client->dev, "OV5647: __sensor_init wrote %zu regs, MIPI_CTRL00=0x%02x\n",
		 ARRAY_SIZE(ov5647_640x480), mipi_ctrl);

	ret = ov5647_set_virtual_channel(sd, 0);
	if (ret < 0)
		return ret;

	ret = ov5647_read(sd, OV5647_SW_STANDBY, &resetval);
	if (ret < 0)
		return ret;

	if (!(resetval & 0x01)) {
		dev_err(&client->dev, "Device was in SW standby");
		ret = ov5647_write(sd, OV5647_SW_STANDBY, 0x01);
		if (ret < 0)
			return ret;
	}

	/*
	 * stream off to make the clock lane into LP-11 state.
	 */
	return ov5647_stream_off(sd);
}

static int ov5647_sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct ov5647 *ov5647 = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "OV5647: s_power called, on=%d, sd=%px, sd->ctrl_handler=%px\n",
		 on, sd, sd->ctrl_handler);

	mutex_lock(&ov5647->lock);

	if (on && !ov5647->power_count)	{
		dev_info(&client->dev, "OV5647 power on, power_count=%d\n", ov5647->power_count);

		ret = clk_prepare_enable(ov5647->xclk);
		if (ret < 0) {
			dev_err(&client->dev, "clk prepare enable failed\n");
			goto out;
		}

		/* Deassert reset GPIO */
		if (!IS_ERR_OR_NULL(ov5647->reset_gpio)) {
			gpiod_set_value_cansleep(ov5647->reset_gpio, 0);
			usleep_range(5000, 10000);
		}

		ret = ov5647_write_array(sd, sensor_oe_enable_regs,
				ARRAY_SIZE(sensor_oe_enable_regs));
		if (ret < 0) {
			clk_disable_unprepare(ov5647->xclk);
			dev_err(&client->dev,
				"write sensor_oe_enable_regs error\n");
			goto out;
		}

		ret = __sensor_init(sd);
		if (ret < 0) {
			clk_disable_unprepare(ov5647->xclk);
			dev_err(&client->dev,
				"Camera not available, check Power\n");
			goto out;
		}
	} else if (!on && ov5647->power_count == 1) {
		dev_dbg(&client->dev, "OV5647 power off\n");

		ret = ov5647_write_array(sd, sensor_oe_disable_regs,
				ARRAY_SIZE(sensor_oe_disable_regs));

		if (ret < 0)
			dev_dbg(&client->dev, "disable oe failed\n");

		ret = set_sw_standby(sd, true);

		if (ret < 0)
			dev_dbg(&client->dev, "soft stby failed\n");

		clk_disable_unprepare(ov5647->xclk);

		/* Assert reset GPIO */
		if (!IS_ERR_OR_NULL(ov5647->reset_gpio))
			gpiod_set_value_cansleep(ov5647->reset_gpio, 1);
	}

	/* Update the power count. */
	ov5647->power_count += on ? 1 : -1;
	WARN_ON(ov5647->power_count < 0);

out:
	mutex_unlock(&ov5647->lock);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5647_sensor_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	u8 val;
	int ret;

	ret = ov5647_read(sd, reg->reg & 0xff, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int ov5647_sensor_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	return ov5647_write(sd, reg->reg & 0xff, reg->val & 0xff);
}
#endif

/*
 * Subdev core operations registration
 */
static const struct v4l2_subdev_core_ops ov5647_subdev_core_ops = {
	.s_power		= ov5647_sensor_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= ov5647_sensor_get_register,
	.s_register		= ov5647_sensor_set_register,
#endif
};

static int ov5647_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		return ov5647_stream_on(sd);
	else
		return ov5647_stream_off(sd);
}

static int ov5647_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	/* OV5647 runs at 30fps in 640x480 mode */
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static const struct v4l2_subdev_video_ops ov5647_subdev_video_ops = {
	.s_stream =		ov5647_s_stream,
	.g_frame_interval =	ov5647_g_frame_interval,
};

static int ov5647_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR8_1X8;

	return 0;
}

static int ov5647_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR8_1X8)
		return -EINVAL;

	fse->min_width = 640;
	fse->max_width = 640;
	fse->min_height = 480;
	fse->max_height = 480;

	return 0;
}

static int ov5647_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index > 0)
		return -EINVAL;

	fie->code = MEDIA_BUS_FMT_SBGGR8_1X8;
	fie->width = 640;
	fie->height = 480;
	fie->interval.numerator = 1;
	fie->interval.denominator = 30;

	return 0;
}

static int ov5647_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5647 *ov5647 = to_state(sd);

	mutex_lock(&ov5647->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov5647->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = 640;
		fmt->format.height = 480;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR8_1X8;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	}

	mutex_unlock(&ov5647->lock);

	return 0;
}

static int ov5647_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5647 *ov5647 = to_state(sd);

	mutex_lock(&ov5647->lock);

	/* Only support 640x480 SBGGR8 for now */
	fmt->format.width = 640;
	fmt->format.height = 480;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR8_1X8;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#endif
	}

	mutex_unlock(&ov5647->lock);

	return 0;
}

static int ov5647_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	/* OV5647 uses 2-lane MIPI CSI-2 */
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = V4L2_MBUS_CSI2_2_LANE |
			V4L2_MBUS_CSI2_CHANNEL_0 |
			V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int ov5647_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		/*
		 * Current crop - for 640x480 mode, we use the full active area
		 * with 4x4 subsampling
		 */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 640;
		sel->r.height = 480;
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV5647_NATIVE_WIDTH;
		sel->r.height = OV5647_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/*
		 * Report 640x480 as crop bounds to tell CIF the expected
		 * output size, not the full pixel array
		 */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 640;
		sel->r.height = 480;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops ov5647_subdev_pad_ops = {
	.enum_mbus_code =	ov5647_enum_mbus_code,
	.enum_frame_size =	ov5647_enum_frame_size,
	.enum_frame_interval =	ov5647_enum_frame_interval,
	.get_fmt =		ov5647_get_fmt,
	.set_fmt =		ov5647_set_fmt,
	.get_selection =	ov5647_get_selection,
	.get_mbus_config =	ov5647_get_mbus_config,
};

static const struct v4l2_subdev_ops ov5647_subdev_ops = {
	.core		= &ov5647_subdev_core_ops,
	.video		= &ov5647_subdev_video_ops,
	.pad		= &ov5647_subdev_pad_ops,
};

static int ov5647_detect(struct v4l2_subdev *sd)
{
	u8 read;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = ov5647_write(sd, OV5647_SW_RESET, 0x01);
	if (ret < 0)
		return ret;

	ret = ov5647_read(sd, OV5647_REG_CHIPID_H, &read);
	if (ret < 0)
		return ret;

	if (read != 0x56) {
		dev_err(&client->dev, "ID High expected 0x56 got %x", read);
		return -ENODEV;
	}

	ret = ov5647_read(sd, OV5647_REG_CHIPID_L, &read);
	if (ret < 0)
		return ret;

	if (read != 0x47) {
		dev_err(&client->dev, "ID Low expected 0x47 got %x", read);
		return -ENODEV;
	}

	return ov5647_write(sd, OV5647_SW_RESET, 0x00);
}

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	struct v4l2_rect *crop =
				v4l2_subdev_get_try_crop(sd, fh->pad, 0);

	crop->left = OV5647_COLUMN_START_DEF;
	crop->top = OV5647_ROW_START_DEF;
	crop->width = OV5647_WINDOW_WIDTH_DEF;
	crop->height = OV5647_WINDOW_HEIGHT_DEF;

	format->code = MEDIA_BUS_FMT_SBGGR8_1X8;

	format->width = OV5647_WINDOW_WIDTH_DEF;
	format->height = OV5647_WINDOW_HEIGHT_DEF;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
	.open = ov5647_open,
};

static int ov5647_parse_dt(struct device_node *np)
{
	struct v4l2_fwnode_endpoint bus_cfg = { .bus_type = 0 };
	struct device_node *ep;

	int ret;

	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);

	of_node_put(ep);
	return ret;
}

static int ov5647_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647 *sensor = container_of(ctrl->handler,
					     struct ov5647, ctrl_handler);
	struct v4l2_subdev *sd = &sensor->sd;
	u8 reg;
	int ret;

	/* Only apply controls when powered on */
	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov5647_write(sd, OV5647_REG_AWB, ctrl->val ? 1 : 0);
		break;

	case V4L2_CID_AUTOGAIN:
		ret = ov5647_read(sd, OV5647_REG_AEC_AGC, &reg);
		if (ret < 0)
			break;
		if (ctrl->val)
			reg &= ~OV5647_AGC_ENABLE;  /* 0 = auto */
		else
			reg |= OV5647_AGC_ENABLE;   /* 1 = manual */
		ret = ov5647_write(sd, OV5647_REG_AEC_AGC, reg);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov5647_read(sd, OV5647_REG_AEC_AGC, &reg);
		if (ret < 0)
			break;
		if (ctrl->val == V4L2_EXPOSURE_AUTO)
			reg &= ~OV5647_AEC_ENABLE;  /* 0 = auto */
		else
			reg |= OV5647_AEC_ENABLE;   /* 1 = manual */
		ret = ov5647_write(sd, OV5647_REG_AEC_AGC, reg);
		break;

	case V4L2_CID_EXPOSURE:
		/* 16-bit exposure value spread across 3 registers */
		ret = ov5647_write(sd, OV5647_REG_EXPOSURE_HI,
				   (ctrl->val >> 12) & 0x0f);
		if (ret < 0)
			break;
		ret = ov5647_write(sd, OV5647_REG_EXPOSURE_MID,
				   (ctrl->val >> 4) & 0xff);
		if (ret < 0)
			break;
		ret = ov5647_write(sd, OV5647_REG_EXPOSURE_LO,
				   (ctrl->val << 4) & 0xf0);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/* 10-bit gain value */
		ret = ov5647_write(sd, OV5647_REG_GAIN_HI,
				   (ctrl->val >> 8) & 0x03);
		if (ret < 0)
			break;
		ret = ov5647_write(sd, OV5647_REG_GAIN_LO,
				   ctrl->val & 0xff);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret < 0 ? ret : 0;
}

static const struct v4l2_ctrl_ops ov5647_ctrl_ops = {
	.s_ctrl = ov5647_s_ctrl,
};

static int ov5647_init_controls(struct ov5647 *sensor)
{
	struct v4l2_ctrl_handler *handler = &sensor->ctrl_handler;
	int ret;

	pr_info("OV5647: init_controls START, handler=%px, sensor=%px\n",
		handler, sensor);

	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret) {
		pr_err("OV5647: v4l2_ctrl_handler_init FAILED ret=%d\n", ret);
		return ret;
	}
	pr_info("OV5647: v4l2_ctrl_handler_init OK, handler=%px\n", handler);

	handler->lock = &sensor->lock;

	/* Link frequency - required by Rockchip CSI2 DPHY */
	pr_info("OV5647: Adding LINK_FREQ control, menu_size=%zu, freq=%lld\n",
		ARRAY_SIZE(ov5647_link_freq_menu), ov5647_link_freq_menu[0]);
	sensor->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(ov5647_link_freq_menu) - 1,
						   0, ov5647_link_freq_menu);
	pr_info("OV5647: LINK_FREQ ctrl=%px, handler->error=%d\n",
		sensor->link_freq, handler->error);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Pixel rate is read-only */
	pr_info("OV5647: Adding PIXEL_RATE control, rate=%d\n", OV5647_PIXEL_RATE);
	sensor->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE,
					       OV5647_PIXEL_RATE,
					       OV5647_PIXEL_RATE, 1,
					       OV5647_PIXEL_RATE);
	pr_info("OV5647: PIXEL_RATE ctrl=%px, handler->error=%d\n",
		sensor->pixel_rate, handler->error);
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Auto exposure control */
	sensor->auto_exp = v4l2_ctrl_new_std_menu(handler, &ov5647_ctrl_ops,
						  V4L2_CID_EXPOSURE_AUTO,
						  V4L2_EXPOSURE_MANUAL, 0,
						  V4L2_EXPOSURE_AUTO);

	/* Auto gain control */
	sensor->auto_gain = v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops,
					      V4L2_CID_AUTOGAIN,
					      0, 1, 1, 1);

	/* Auto white balance control */
	sensor->auto_wb = v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops,
					    V4L2_CID_AUTO_WHITE_BALANCE,
					    0, 1, 1, 1);

	/* Manual exposure (when auto is off) */
	sensor->exposure = v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     1, 65535, 1, 1000);

	/* Manual gain (when auto is off) */
	sensor->gain = v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops,
					 V4L2_CID_ANALOGUE_GAIN,
					 16, 1023, 1, 64);

	if (handler->error) {
		ret = handler->error;
		pr_err("OV5647: handler->error=%d, freeing handler\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	sensor->sd.ctrl_handler = handler;
	pr_info("OV5647: init_controls DONE, sd.ctrl_handler=%px, handler->nr_of_buckets=%d\n",
		sensor->sd.ctrl_handler, handler->nr_of_buckets);
	return 0;
}

static int __ov5647_power_on(struct ov5647 *ov5647)
{
	int ret;
	struct device *dev = ov5647->sd.dev;

	dev_dbg(dev, "OV5647 power on\n");

	/* Enable clock first */
	ret = clk_prepare_enable(ov5647->xclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xclk\n");
		return ret;
	}

	/* Deassert reset GPIO (active low) */
	if (!IS_ERR_OR_NULL(ov5647->reset_gpio)) {
		gpiod_set_value_cansleep(ov5647->reset_gpio, 0);
		usleep_range(5000, 10000);
	}

	return 0;
}

static void __ov5647_power_off(struct ov5647 *ov5647)
{
	/* Assert reset GPIO */
	if (!IS_ERR_OR_NULL(ov5647->reset_gpio))
		gpiod_set_value_cansleep(ov5647->reset_gpio, 1);

	/* Disable clock */
	clk_disable_unprepare(ov5647->xclk);
}

static int ov5647_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov5647 *sensor;
	int ret;
	struct v4l2_subdev *sd;
	struct device_node *np = client->dev.of_node;
	u32 xclk_freq;

	dev_info(dev, "OV5647 probe start\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = ov5647_parse_dt(np);
		if (ret) {
			dev_err(dev, "DT parsing error: %d\n", ret);
			return ret;
		}
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(sensor->xclk);
	}

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != 25000000) {
		dev_err(dev, "Unsupported clock frequency: %u\n", xclk_freq);
		return -EINVAL;
	}

	/* Get reset GPIO (active low) */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio)) {
		dev_warn(dev, "Failed to get reset-gpios\n");
		sensor->reset_gpio = NULL;
	}

	mutex_init(&sensor->lock);
	dev_info(dev, "OV5647: mutex_init done\n");

	sd = &sensor->sd;
	dev_info(dev, "OV5647: BEFORE v4l2_i2c_subdev_init, sd=%px, sd->ctrl_handler=%px\n",
		 sd, sd->ctrl_handler);
	v4l2_i2c_subdev_init(sd, client, &ov5647_subdev_ops);
	dev_info(dev, "OV5647: AFTER v4l2_i2c_subdev_init, sd=%px, sd->ctrl_handler=%px\n",
		 sd, sd->ctrl_handler);
	sensor->sd.internal_ops = &ov5647_subdev_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/* Initialize V4L2 controls - must be after v4l2_i2c_subdev_init */
	dev_info(dev, "OV5647: Calling ov5647_init_controls...\n");
	ret = ov5647_init_controls(sensor);
	if (ret) {
		dev_err(dev, "Failed to initialize controls: %d\n", ret);
		goto mutex_remove;
	}
	dev_info(dev, "OV5647: AFTER init_controls, sd->ctrl_handler=%px, &sensor->ctrl_handler=%px\n",
		 sd->ctrl_handler, &sensor->ctrl_handler);

	/* Verify ctrl_handler has controls */
	if (sd->ctrl_handler) {
		struct v4l2_ctrl *ctrl;
		ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_LINK_FREQ);
		dev_info(dev, "OV5647: v4l2_ctrl_find(LINK_FREQ) = %px\n", ctrl);
		ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
		dev_info(dev, "OV5647: v4l2_ctrl_find(PIXEL_RATE) = %px\n", ctrl);
	} else {
		dev_err(dev, "OV5647: sd->ctrl_handler is NULL!\n");
	}

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto ctrl_free;

	/* Power on sensor for detection */
	ret = __ov5647_power_on(sensor);
	if (ret) {
		dev_err(dev, "Failed to power on sensor\n");
		goto error;
	}

	ret = ov5647_detect(sd);
	if (ret < 0) {
		dev_err(dev, "OV5647 not detected, ret=%d\n", ret);
		goto power_off;
	}

	dev_info(dev, "OV5647 detected!\n");

	/* Check ctrl_handler again before registration */
	dev_info(dev, "OV5647: BEFORE v4l2_async_register_subdev, sd->ctrl_handler=%px\n",
		 sd->ctrl_handler);

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(dev, "Failed to register subdev\n");
		goto power_off;
	}

	dev_info(dev, "OV5647: AFTER v4l2_async_register_subdev, sd->ctrl_handler=%px\n",
		 sd->ctrl_handler);

	/* Enable runtime PM and idle */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "OmniVision OV5647 camera driver probed successfully\n");
	return 0;

power_off:
	__ov5647_power_off(sensor);
error:
	media_entity_cleanup(&sd->entity);
ctrl_free:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
mutex_remove:
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5647 *ov5647 = to_state(sd);

	pm_runtime_disable(&client->dev);
	v4l2_async_unregister_subdev(&ov5647->sd);
	media_entity_cleanup(&ov5647->sd.entity);
	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&ov5647->ctrl_handler);
	mutex_destroy(&ov5647->lock);
	__ov5647_power_off(ov5647);

	return 0;
}

static int __maybe_unused ov5647_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5647 *ov5647 = to_state(sd);

	return __ov5647_power_on(ov5647);
}

static int __maybe_unused ov5647_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5647 *ov5647 = to_state(sd);

	__ov5647_power_off(ov5647);
	return 0;
}

static const struct dev_pm_ops ov5647_pm_ops = {
	SET_RUNTIME_PM_OPS(ov5647_runtime_suspend, ov5647_runtime_resume, NULL)
};

static const struct i2c_device_id ov5647_id[] = {
	{ "ov5647", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov5647_of_match[] = {
	{ .compatible = "ovti,ov5647" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov5647_of_match);
#endif

static struct i2c_driver ov5647_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov5647_of_match),
		.pm	= &ov5647_pm_ops,
		.name	= SENSOR_NAME,
	},
	.probe_new	= ov5647_probe,
	.remove		= ov5647_remove,
	.id_table	= ov5647_id,
};

module_i2c_driver(ov5647_driver);

MODULE_AUTHOR("Ramiro Oliveira <roliveir@synopsys.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov5647 sensors");
MODULE_LICENSE("GPL v2");
