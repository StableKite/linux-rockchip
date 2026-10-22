// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mini2 640 / RS300 non-radiometric LWIR V4L2 sub-device for StableKite.
 *
 * Command framing, registers and stream sequence are based on the
 * GPL-2.0 RS300 driver by Kodrea (mini2-thermal-driver) and on the
 * RS300/Jetson integration package supplied by Purple River.
 *
 * This driver does not receive frames itself: the SoC CSI-2 receiver
 * and rkcif provide the capture queues, DMA, timestamps and /dev/videoX.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define RS300_WIDTH                640U
#define RS300_HEIGHT               512U
#define RS300_FPS                  60U
#define RS300_LANES                2U
/* Confirmed by Cheng'en: 400 Mbps/lane. DDR clock = 200 MHz. */
#define RS300_LINK_FREQ            200000000LL
#define RS300_PIXEL_RATE           (RS300_LINK_FREQ * 2 * RS300_LANES / 16)
/* Kodrea's Rockchip CSI interface advertises YUYV8_2X8 for YUV422. */
#define RS300_BUS_CODE             MEDIA_BUS_FMT_YUYV8_2X8

#define RS300_CMD_REG              0x1d00
#define RS300_STATUS_REG           0x0200
#define RS300_CMD_LEN              18
#define RS300_STREAM_CMD_LEN       28
#define RS300_CTRL_BASE            (V4L2_CID_USER_BASE + 0x1100)
#define RS300_CID_FFC              (RS300_CTRL_BASE + 0)
#define RS300_CID_AUTO_FFC         (RS300_CTRL_BASE + 1)
#define RS300_CID_SPATIAL_NR       (RS300_CTRL_BASE + 2)
#define RS300_CID_TEMPORAL_NR      (RS300_CTRL_BASE + 3)
#define RS300_CID_DDE              (RS300_CTRL_BASE + 4)

struct rs300 {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock; /* serializes commands, controls, stream and suspend */
	struct regulator *vdd;
	struct gpio_desc *reset;
	bool powered;
	bool streaming;
};

static inline struct rs300 *to_rs300(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rs300, sd);
}

static const s64 rs300_link_freq[] = { RS300_LINK_FREQ };

static u16 rs300_crc(const u8 *data, size_t size)
{
	u16 crc = 0;
	int bit;

	while (size--) {
		crc ^= (u16)*data++ << 8;
		for (bit = 0; bit < 8; bit++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}

	return crc;
}

static void rs300_put_crc(u8 *buf, unsigned int at, u16 crc)
{
	buf[at] = crc & 0xff;
	buf[at + 1] = crc >> 8;
}

static int rs300_read(struct rs300 *rs, u16 reg, void *buffer, size_t len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[] = {
		{ .addr = rs->client->addr, .flags = 0,
		  .len = sizeof(addr), .buf = addr },
		{ .addr = rs->client->addr, .flags = I2C_M_RD,
		  .len = len, .buf = buffer },
	};
	int ret;

	ret = i2c_transfer(rs->client->adapter, msg, ARRAY_SIZE(msg));
	return ret == ARRAY_SIZE(msg) ? 0 : ret < 0 ? ret : -EIO;
}

static int rs300_write(struct rs300 *rs, u16 reg, const u8 *buf, size_t len)
{
	u8 cmd[2 + RS300_STREAM_CMD_LEN];
	int ret;

	if (len > RS300_STREAM_CMD_LEN)
		return -EINVAL;

	cmd[0] = reg >> 8;
	cmd[1] = reg & 0xff;
	memcpy(cmd + 2, buf, len);
	ret = i2c_master_send(rs->client, cmd, len + 2);
	return ret == len + 2 ? 0 : ret < 0 ? ret : -EIO;
}

/* The status register reports bit0=busy, bit1=failed, bits[7:2]=error. */
static int rs300_wait_ready(struct rs300 *rs, unsigned int timeout_ms)
{
	unsigned int i;
	u8 status;
	int ret;

	for (i = 0; i < DIV_ROUND_UP(timeout_ms, 20); i++) {
		ret = rs300_read(rs, RS300_STATUS_REG, &status, 1);
		if (ret)
			return ret;
		if (status & BIT(1)) {
			dev_err(&rs->client->dev, "command failed, status %#02x\n",
				status);
			return -EIO;
		}
		if (!(status & BIT(0)))
			return 0;
		usleep_range(20000, 22000);
	}

	return -ETIMEDOUT;
}

/* The 18-byte protocol and CRC16/XMODEM are common to the vendor SDK and Kodrea. */
static int rs300_command(struct rs300 *rs, u8 group, u8 module, u8 subcmd,
			 const u8 *params, size_t nparams, unsigned int timeout_ms)
{
	u8 cmd[RS300_CMD_LEN] = { 0 };
	int ret;

	if (nparams > 12)
		return -EINVAL;

	cmd[0] = group;
	cmd[1] = module;
	cmd[2] = subcmd;
	if (params && nparams)
		memcpy(cmd + 4, params, nparams);
	rs300_put_crc(cmd, 16, rs300_crc(cmd, 16));

	ret = rs300_write(rs, RS300_CMD_REG, cmd, sizeof(cmd));
	if (ret)
		return ret;
	usleep_range(20000, 22000);
	return rs300_wait_ready(rs, timeout_ms);
}

/* Stream start/stop packets use the RS300 transport header plus their own CRC. */
static int rs300_stream_packet(struct rs300 *rs, bool start)
{
	u8 cmd[RS300_STREAM_CMD_LEN] = { 0 };
	int ret;

	cmd[0] = 0x01;
	cmd[1] = 0x30;
	cmd[2] = start ? 0xc1 : 0xc2;
	cmd[12] = 0x0a;
	cmd[18] = start ? 0x00 : 0x01;
	cmd[19] = start ? 0x10 : 0x16; /* Kodrea 640x512 path */
	cmd[20] = start ? 0x03 : 0x00;
	cmd[21] = RS300_FPS;
	cmd[22] = RS300_WIDTH & 0xff;
	cmd[23] = RS300_WIDTH >> 8;
	cmd[24] = RS300_HEIGHT & 0xff;
	cmd[25] = RS300_HEIGHT >> 8;
	rs300_put_crc(cmd, 14, rs300_crc(cmd + 18, 10));
	rs300_put_crc(cmd, 16, rs300_crc(cmd, 16));

	ret = rs300_write(rs, RS300_CMD_REG, cmd, sizeof(cmd));
	if (ret)
		return ret;
	usleep_range(20000, 22000);
	return rs300_wait_ready(rs, start ? 2500 : 500);
}

static int rs300_start(struct rs300 *rs)
{
	const u8 rate[] = { 0x01, 0x03, 0x3c }; /* 60 Hz, progressive MIPI */
	const u8 yuv[] = { 0x00 }; /* physical byte order UYVY */
	int ret;

	/* The camera must be alive before we advertise a streaming device. */
	ret = rs300_command(rs, 0x10, 0x03, 0x4d, yuv, sizeof(yuv), 500);
	if (ret)
		return ret;
	ret = rs300_command(rs, 0x10, 0x10, 0x46, rate, sizeof(rate), 4500);
	if (ret)
		return ret;

	ret = rs300_stream_packet(rs, true);
	if (ret)
		rs300_stream_packet(rs, false); /* best effort unwind */
	return ret;
}

static int rs300_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rs300 *rs = to_rs300(sd);
	int ret = 0;

	mutex_lock(&rs->lock);
	if (!rs->powered) {
		ret = -EHOSTDOWN;
	} else if (!!enable != rs->streaming) {
		if (enable)
			ret = rs300_start(rs);
		else
			ret = rs300_stream_packet(rs, false);
		if (!ret)
			rs->streaming = !!enable;
	}
	mutex_unlock(&rs->lock);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
static int rs300_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 1;
	fi->interval.denominator = RS300_FPS;
	return 0;
}
#endif

static int rs300_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rs300 *rs = container_of(ctrl->handler, struct rs300, ctrls);
	u8 p = ctrl->val;

	/* ctrls.lock is rs->lock; V4L2 owns the lock during s_ctrl. */
	if (!rs->powered)
		return -EHOSTDOWN;

	switch (ctrl->id) {
	case RS300_CID_FFC:
		return rs300_command(rs, 0x10, 0x02, 0x43, NULL, 0, 5000);
	case RS300_CID_AUTO_FFC:
		return rs300_command(rs, 0x10, 0x02, 0x41, &p, 1, 500);
	case RS300_CID_SPATIAL_NR:
		return rs300_command(rs, 0x10, 0x04, 0x4b, &p, 1, 500);
	case RS300_CID_TEMPORAL_NR:
		return rs300_command(rs, 0x10, 0x04, 0x4c, &p, 1, 500);
	case RS300_CID_DDE:
		return rs300_command(rs, 0x10, 0x04, 0x45, &p, 1, 500);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops rs300_ctrl_ops = {
	.s_ctrl = rs300_set_ctrl,
};

static const struct v4l2_ctrl_config rs300_control_cfg[] = {
	{
		.ops = &rs300_ctrl_ops, .id = RS300_CID_FFC,
		.name = "RS300 FFC trigger", .type = V4L2_CTRL_TYPE_BUTTON,
		.min = 0, .max = 0, .step = 0, .def = 0,
	}, {
		.ops = &rs300_ctrl_ops, .id = RS300_CID_AUTO_FFC,
		.name = "RS300 automatic FFC", .type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0, .max = 1, .step = 1, .def = 1,
	}, {
		.ops = &rs300_ctrl_ops, .id = RS300_CID_SPATIAL_NR,
		.name = "RS300 spatial NR", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, .max = 100, .step = 1, .def = 0,
	}, {
		.ops = &rs300_ctrl_ops, .id = RS300_CID_TEMPORAL_NR,
		.name = "RS300 temporal NR", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, .max = 100, .step = 1, .def = 0,
	}, {
		.ops = &rs300_ctrl_ops, .id = RS300_CID_DDE,
		.name = "RS300 DDE", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, .max = 100, .step = 1, .def = 0,
	},
};

static void rs300_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->width = RS300_WIDTH;
	fmt->height = RS300_HEIGHT;
	fmt->code = RS300_BUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_709;
}

/* Linux 6.1 media subdevice TRY format initialization. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
static int rs300_init_cfg(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_get_try_format(sd, state, 0);
	rs300_fill_format(fmt);
	return 0;
}
#endif

static int rs300_enum_mbus_code(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index)
		return -EINVAL;
	code->code = RS300_BUS_CODE;
	return 0;
}

static int rs300_enum_frame_size(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad || fse->index || fse->code != RS300_BUS_CODE)
		return -EINVAL;
	fse->min_width = fse->max_width = RS300_WIDTH;
	fse->min_height = fse->max_height = RS300_HEIGHT;
	return 0;
}

static int rs300_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	if (fmt->pad)
		return -EINVAL;
	rs300_fill_format(&fmt->format);
	return 0;
}

static int rs300_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	/* Mini2 resolution is fixed by the physically installed detector. */
	return rs300_get_fmt(sd, state, fmt);
}

static int rs300_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_config *cfg)
{
	if (pad)
		return -EINVAL;
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = RS300_LANES;
	return 0;
}

static const struct v4l2_subdev_video_ops rs300_video_ops = {
	.s_stream = rs300_s_stream,
/* Kernel 6.1 has g_frame_interval; recent V4L2 cores moved the API. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.g_frame_interval = rs300_g_frame_interval,
#endif
};

static const struct v4l2_subdev_pad_ops rs300_pad_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.init_cfg = rs300_init_cfg,
#endif
	.enum_mbus_code = rs300_enum_mbus_code,
	.enum_frame_size = rs300_enum_frame_size,
	.get_fmt = rs300_get_fmt,
	.set_fmt = rs300_set_fmt,
	.get_mbus_config = rs300_get_mbus_config,
};

static const struct v4l2_subdev_ops rs300_subdev_ops = {
	.video = &rs300_video_ops,
	.pad = &rs300_pad_ops,
};

static int rs300_power_on(struct rs300 *rs)
{
	int ret;

	ret = regulator_enable(rs->vdd);
	if (ret)
		return ret;

	if (rs->reset) {
		gpiod_set_value_cansleep(rs->reset, 1);
		usleep_range(10000, 11000);
		gpiod_set_value_cansleep(rs->reset, 0);
	}

	msleep(100);
	rs->powered = true;
	return 0;
}

static void rs300_power_off(struct rs300 *rs)
{
	if (!rs->powered)
		return;
	if (rs->reset)
		gpiod_set_value_cansleep(rs->reset, 1);
	regulator_disable(rs->vdd);
	rs->powered = false;
	rs->streaming = false;
}

static int rs300_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "missing CSI-2 endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	if (ret)
		goto out;
	if (ep.bus.mipi_csi2.num_data_lanes != RS300_LANES ||
	    ep.nr_of_link_frequencies != 1 ||
	    ep.link_frequencies[0] != RS300_LINK_FREQ)
		ret = -EINVAL;
	else
		ret = 0;

out:
	v4l2_fwnode_endpoint_free(&ep);
	fwnode_handle_put(endpoint);
	if (ret)
		dev_err(dev, "invalid CSI-2 endpoint; need 2 lanes, link-frequencies=200MHz\n");
	return ret;
}

static int rs300_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rs300 *rs;
	int ret, i;
	u8 status;
	struct v4l2_ctrl *freq_ctrl, *rate_ctrl;

	if (client->addr != 0x3c)
		return dev_err_probe(dev, -EINVAL, "RS300 expects 7-bit address 0x3c\n");

	ret = rs300_check_endpoint(dev);
	if (ret)
		return ret;

	rs = devm_kzalloc(dev, sizeof(*rs), GFP_KERNEL);
	if (!rs)
		return -ENOMEM;

	rs->client = client;
	mutex_init(&rs->lock);
	rs->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(rs->vdd)) {
		ret = PTR_ERR(rs->vdd);
		goto err_mutex;
	}
	rs->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rs->reset)) {
		ret = PTR_ERR(rs->reset);
		goto err_mutex;
	}

	ret = rs300_power_on(rs);
	if (ret)
		goto err_mutex;
	ret = rs300_read(rs, RS300_STATUS_REG, &status, 1);
	if (ret) {
		dev_err(dev, "no RS300 command/status response (%d)\n", ret);
		goto err_power;
	}

	v4l2_i2c_subdev_init(&rs->sd, client, &rs300_subdev_ops);
	rs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	rs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = v4l2_ctrl_handler_init(&rs->ctrls,
				     ARRAY_SIZE(rs300_control_cfg) + 2);
	if (ret)
		goto err_power;
	rs->ctrls.lock = &rs->lock;
	for (i = 0; i < ARRAY_SIZE(rs300_control_cfg); i++)
		v4l2_ctrl_new_custom(&rs->ctrls, &rs300_control_cfg[i], NULL);
	freq_ctrl = v4l2_ctrl_new_int_menu(&rs->ctrls, NULL,
				V4L2_CID_LINK_FREQ, 0, 0, rs300_link_freq);
	rate_ctrl = v4l2_ctrl_new_std(&rs->ctrls, NULL,
			V4L2_CID_PIXEL_RATE, RS300_PIXEL_RATE,
			RS300_PIXEL_RATE, 1, RS300_PIXEL_RATE);
	if (freq_ctrl)
		freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (rate_ctrl)
		rate_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (rs->ctrls.error) {
		ret = rs->ctrls.error;
		goto err_ctrls;
	}
	rs->sd.ctrl_handler = &rs->ctrls;

	rs->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&rs->sd.entity, 1, &rs->pad);
	if (ret)
		goto err_ctrls;

	ret = v4l2_async_register_subdev_sensor(&rs->sd);
	if (ret)
		goto err_media;

	dev_info(dev, "Mini2 640x512 60Hz non-radiometric CSI-2 sensor ready\n");
	return 0;

err_media:
	media_entity_cleanup(&rs->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&rs->ctrls);
err_power:
	rs300_power_off(rs);
err_mutex:
	mutex_destroy(&rs->lock);
	return ret;
}

static void rs300_remove(struct i2c_client *client)
{
	struct rs300 *rs = to_rs300(i2c_get_clientdata(client));

	v4l2_async_unregister_subdev(&rs->sd);
	mutex_lock(&rs->lock);
	if (rs->streaming)
		rs300_stream_packet(rs, false);
	rs300_power_off(rs);
	mutex_unlock(&rs->lock);
	media_entity_cleanup(&rs->sd.entity);
	v4l2_ctrl_handler_free(&rs->ctrls);
	mutex_destroy(&rs->lock);
}

static int rs300_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rs300 *rs = to_rs300(i2c_get_clientdata(client));

	mutex_lock(&rs->lock);
	if (rs->streaming)
		rs300_stream_packet(rs, false);
	rs300_power_off(rs);
	mutex_unlock(&rs->lock);
	return 0;
}

static int rs300_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rs300 *rs = to_rs300(i2c_get_clientdata(client));
	int ret;

	mutex_lock(&rs->lock);
	ret = rs300_power_on(rs);
	mutex_unlock(&rs->lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(rs300_pm_ops, rs300_suspend, rs300_resume);

static const struct of_device_id rs300_of_match[] = {
	{ .compatible = "infisense,mini2-640" },
	{ .compatible = "infisense,rs300" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rs300_of_match);

static const struct i2c_device_id rs300_i2c_ids[] = {
	{ "rs300", 0 }, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, rs300_i2c_ids);

static struct i2c_driver rs300_i2c_driver = {
	.driver = {
		.name = "rs300",
		.of_match_table = rs300_of_match,
		.pm = &rs300_pm_ops,
	},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	.probe = rs300_probe,
#else
	.probe_new = rs300_probe,
#endif
	.remove = rs300_remove,
	.id_table = rs300_i2c_ids,
};
module_i2c_driver(rs300_i2c_driver);

MODULE_DESCRIPTION("StableKite Mini2 640 RS300 thermal camera V4L2 subdevice");
MODULE_AUTHOR("StableKite / adapted from Kodrea RS300 protocol");
MODULE_LICENSE("GPL");
