/*
 * STMicroelectronics hts221 sensor driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>
#include <asm/unaligned.h>

#include "hts221.h"

#define HTS221_REG_WHOAMI_ADDR		0x0f
#define HTS221_REG_WHOAMI_VAL		0xbc

#define HTS221_REG_CNTRL1_ADDR		0x20
#define HTS221_REG_CNTRL2_ADDR		0x21
#define HTS221_REG_CNTRL3_ADDR		0x22

#define HTS221_REG_AVG_ADDR		0x10
#define HTS221_REG_H_OUT_L		0x28
#define HTS221_REG_T_OUT_L		0x2a

#define HTS221_HUMIDITY_AVG_MASK	0x07
#define HTS221_TEMP_AVG_MASK		0x38

#define HTS221_ODR_MASK			0x87
#define HTS221_BDU_MASK			BIT(2)

#define HTS221_DRDY_MASK		BIT(2)

#define HTS221_ENABLE_SENSOR		BIT(7)

#define HTS221_HUMIDITY_AVG_4		0x00 /* 0.4 %RH */
#define HTS221_HUMIDITY_AVG_8		0x01 /* 0.3 %RH */
#define HTS221_HUMIDITY_AVG_16		0x02 /* 0.2 %RH */
#define HTS221_HUMIDITY_AVG_32		0x03 /* 0.15 %RH */
#define HTS221_HUMIDITY_AVG_64		0x04 /* 0.1 %RH */
#define HTS221_HUMIDITY_AVG_128		0x05 /* 0.07 %RH */
#define HTS221_HUMIDITY_AVG_256		0x06 /* 0.05 %RH */
#define HTS221_HUMIDITY_AVG_512		0x07 /* 0.03 %RH */

#define HTS221_TEMP_AVG_2		0x00 /* 0.08 degC */
#define HTS221_TEMP_AVG_4		0x08 /* 0.05 degC */
#define HTS221_TEMP_AVG_8		0x10 /* 0.04 degC */
#define HTS221_TEMP_AVG_16		0x18 /* 0.03 degC */
#define HTS221_TEMP_AVG_32		0x20 /* 0.02 degC */
#define HTS221_TEMP_AVG_64		0x28 /* 0.015 degC */
#define HTS221_TEMP_AVG_128		0x30 /* 0.01 degC */
#define HTS221_TEMP_AVG_256		0x38 /* 0.007 degC */

/* calibration registers */
#define HTS221_REG_0RH_CAL_X_H		0x36
#define HTS221_REG_1RH_CAL_X_H		0x3a
#define HTS221_REG_0RH_CAL_Y_H		0x30
#define HTS221_REG_1RH_CAL_Y_H		0x31
#define HTS221_REG_0T_CAL_X_L		0x3c
#define HTS221_REG_1T_CAL_X_L		0x3e
#define HTS221_REG_0T_CAL_Y_H		0x32
#define HTS221_REG_1T_CAL_Y_H		0x33
#define HTS221_REG_T1_T0_CAL_Y_H	0x35

struct hts221_odr {
	u8 hz;
	u8 val;
};

struct hts221_avg {
	u8 addr;
	u8 mask;
	struct hts221_avg_avl avg_avl[HTS221_AVG_DEPTH];
};

static const struct hts221_odr hts221_odr_table[] = {
	{ 1, 0x01 },	/* 1Hz */
	{ 7, 0x02 },	/* 7Hz */
	{ 13, 0x03 },	/* 12.5Hz */
};

static const struct hts221_avg hts221_avg_list[] = {
	{
		.addr = HTS221_REG_AVG_ADDR,
		.mask = HTS221_HUMIDITY_AVG_MASK,
		.avg_avl = {
			{ 4, HTS221_HUMIDITY_AVG_4 },
			{ 8, HTS221_HUMIDITY_AVG_8 },
			{ 16, HTS221_HUMIDITY_AVG_16 },
			{ 32, HTS221_HUMIDITY_AVG_32 },
			{ 64, HTS221_HUMIDITY_AVG_64 },
			{ 128, HTS221_HUMIDITY_AVG_128 },
			{ 256, HTS221_HUMIDITY_AVG_256 },
			{ 512, HTS221_HUMIDITY_AVG_512 },
		},
	},
	{
		.addr = HTS221_REG_AVG_ADDR,
		.mask = HTS221_TEMP_AVG_MASK,
		.avg_avl = {
			{ 2, HTS221_TEMP_AVG_2 },
			{ 4, HTS221_TEMP_AVG_4 },
			{ 8, HTS221_TEMP_AVG_8 },
			{ 16, HTS221_TEMP_AVG_16 },
			{ 32, HTS221_TEMP_AVG_32 },
			{ 64, HTS221_TEMP_AVG_64 },
			{ 128, HTS221_TEMP_AVG_128 },
			{ 256, HTS221_TEMP_AVG_256 },
		},
	},
};

static const struct iio_chan_spec hts221_channels[] = {
	{
		.type = IIO_HUMIDITYRELATIVE,
		.address = HTS221_REG_H_OUT_L,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_TEMP,
		.address = HTS221_REG_T_OUT_L,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static int hts221_write_with_mask(struct hts221_dev *dev, u8 addr, u8 mask,
				  u8 val)
{
	u8 data;
	int err;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, addr, sizeof(data), &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read %02x register\n", addr);
		mutex_unlock(&dev->lock);

		return err;
	}

	data = (data & ~mask) | (val & mask);

	err = dev->tf->write(dev->dev, addr, sizeof(data), &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to write %02x register\n", addr);
		mutex_unlock(&dev->lock);

		return err;
	}

	mutex_unlock(&dev->lock);

	return 0;
}

static int hts221_check_whoami(struct hts221_dev *dev)
{
	u8 data;
	int err;

	err = dev->tf->read(dev->dev, HTS221_REG_WHOAMI_ADDR, sizeof(data),
			    &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != HTS221_REG_WHOAMI_VAL) {
		dev_err(dev->dev, "wrong whoami {%02x vs %02x}\n",
			data, HTS221_REG_WHOAMI_VAL);
		return -ENODEV;
	}

	return 0;
}

int hts221_config_drdy(struct hts221_dev *dev, bool enable)
{
	u8 val = enable ? BIT(2) : 0;

	return hts221_write_with_mask(dev, HTS221_REG_CNTRL3_ADDR,
				      HTS221_DRDY_MASK, val);
}

static int hts221_update_odr(struct hts221_dev *dev, u8 odr)
{
	int i, err;
	u8 val;

	for (i = 0; i < ARRAY_SIZE(hts221_odr_table); i++)
		if (hts221_odr_table[i].hz == odr)
			break;

	if (i == ARRAY_SIZE(hts221_odr_table))
		return -EINVAL;

	val = HTS221_ENABLE_SENSOR | HTS221_BDU_MASK | hts221_odr_table[i].val;
	err = hts221_write_with_mask(dev, HTS221_REG_CNTRL1_ADDR,
				     HTS221_ODR_MASK, val);
	if (err < 0)
		return err;

	dev->odr = odr;

	return 0;
}

static int hts221_update_avg(struct hts221_sensor *sensor, u16 val)
{
	int i, err;
	const struct hts221_avg *avg = &hts221_avg_list[sensor->type];

	for (i = 0; i < HTS221_AVG_DEPTH; i++)
		if (avg->avg_avl[i].avg == val)
			break;

	if (i == HTS221_AVG_DEPTH)
		return -EINVAL;

	err = hts221_write_with_mask(sensor->dev, avg->addr, avg->mask,
				     avg->avg_avl[i].val);
	if (err < 0)
		return err;

	sensor->cur_avg_idx = i;

	return 0;
}

static ssize_t hts221_sysfs_sampling_freq(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int i;
	ssize_t len = 0;

	for (i = 0; i < ARRAY_SIZE(hts221_odr_table); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 hts221_odr_table[i].hz);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t
hts221_sysfs_rh_oversampling_avail(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	const struct hts221_avg *avg = &hts221_avg_list[HTS221_SENSOR_H];
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(avg->avg_avl); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 avg->avg_avl[i].avg);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t
hts221_sysfs_temp_oversampling_avail(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	const struct hts221_avg *avg = &hts221_avg_list[HTS221_SENSOR_T];
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(avg->avg_avl); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 avg->avg_avl[i].avg);
	buf[len - 1] = '\n';

	return len;
}

int hts221_dev_power_on(struct hts221_dev *dev)
{
	struct hts221_sensor *sensor;
	int i, err, val;

	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		sensor = &dev->sensors[i];
		val = hts221_avg_list[i].avg_avl[sensor->cur_avg_idx].avg;

		err = hts221_update_avg(sensor, val);
		if (err < 0)
			return err;
	}

	err = hts221_update_odr(dev, dev->odr);
	if (err < 0)
		return err;

	return 0;
}

int hts221_dev_power_off(struct hts221_dev *dev)
{
	u8 data[] = {0x00, 0x00};

	return dev->tf->write(dev->dev, HTS221_REG_CNTRL1_ADDR, sizeof(data),
			      data);
}

static int hts221_parse_caldata(struct hts221_sensor *sensor)
{
	int err, *slope, *b_gen;
	u8 addr_x0, addr_x1;
	s16 cal_x0, cal_x1, cal_y0, cal_y1;
	struct hts221_dev *dev = sensor->dev;

	switch (sensor->type) {
	case HTS221_SENSOR_H: {
		u8 data;

		addr_x0 = HTS221_REG_0RH_CAL_X_H;
		addr_x1 = HTS221_REG_1RH_CAL_X_H;
		cal_y1 = 0;
		cal_y0 = 0;

		err = dev->tf->read(dev->dev, HTS221_REG_0RH_CAL_Y_H,
				    sizeof(data), &data);
		if (err < 0)
			return err;
		cal_y0 = data;

		err = dev->tf->read(dev->dev, HTS221_REG_1RH_CAL_Y_H,
				    sizeof(data), &data);
		if (err < 0)
			return err;
		cal_y1 = data;

		break;
	}
	case HTS221_SENSOR_T: {
		u8 cal0, cal1;

		addr_x0 = HTS221_REG_0T_CAL_X_L;
		addr_x1 = HTS221_REG_1T_CAL_X_L;

		err = dev->tf->read(dev->dev, HTS221_REG_0T_CAL_Y_H,
				    sizeof(cal0), &cal0);
		if (err < 0)
			return err;

		err = dev->tf->read(dev->dev, HTS221_REG_T1_T0_CAL_Y_H,
				    sizeof(cal1), &cal1);
		if (err < 0)
			return err;
		cal_y0 = (le16_to_cpu(cal1 & 0x3) << 8) | cal0;

		err = dev->tf->read(dev->dev, HTS221_REG_1T_CAL_Y_H,
				    sizeof(cal0), &cal0);
		if (err < 0)
			return err;

		err = dev->tf->read(dev->dev, HTS221_REG_T1_T0_CAL_Y_H,
				    sizeof(cal1), &cal1);
		if (err < 0)
			return err;
		cal_y1 = (((cal1 & 0xc) >> 2) << 8) | cal0;
		break;
	}
	default:
		return -ENODEV;
	}

	err = dev->tf->read(dev->dev, addr_x0, sizeof(cal_x0), (u8 *)&cal_x0);
	if (err < 0)
		return err;
	cal_x0 = le16_to_cpu(cal_x0);

	err = dev->tf->read(dev->dev, addr_x1, sizeof(cal_x1), (u8 *)&cal_x1);
	if (err < 0)
		return err;
	cal_x1 = le16_to_cpu(cal_x1);

	slope = &sensor->slope;
	b_gen = &sensor->b_gen;

	*slope = ((cal_y1 - cal_y0) * 8000) / (cal_x1 - cal_x0);
	*b_gen = (((s32)cal_x1 * cal_y0 - (s32)cal_x0 * cal_y1) * 1000) /
		 (cal_x1 - cal_x0);
	*b_gen *= 8;

	return 0;
}

static int hts221_read_raw(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *ch,
			   int *val, int *val2, long mask)
{
	struct hts221_dev *dev = iio_priv(iio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		u8 data[HTS221_DATA_SIZE];

		ret = hts221_dev_power_on(dev);
		if (ret < 0)
			goto out;

		msleep(50);

		ret = dev->tf->read(dev->dev, ch->address, sizeof(data), data);
		if (ret < 0)
			goto out;

		ret = hts221_dev_power_off(dev);
		if (ret < 0)
			goto out;

		*val = (s16)get_unaligned_le16(data);
		ret = IIO_VAL_INT;

		break;
	}
	case IIO_CHAN_INFO_SCALE: {
		s64 tmp;
		s32 rem, div, data;

		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE:
			data = dev->sensors[HTS221_SENSOR_H].slope;
			div = (1 << 4) * 1000;
			break;
		case IIO_TEMP:
			data = dev->sensors[HTS221_SENSOR_T].slope;
			div = (1 << 6) * 1000;
			break;
		default:
			goto out;
		}

		tmp = div_s64(data * 1000000000LL, div);
		tmp = div_s64_rem(tmp, 1000000000LL, &rem);

		*val = tmp;
		*val2 = rem;
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	}
	case IIO_CHAN_INFO_OFFSET: {
		s64 tmp;
		s32 rem, div, data;

		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE:
			data = dev->sensors[HTS221_SENSOR_H].b_gen;
			div = dev->sensors[HTS221_SENSOR_H].slope;
			break;
		case IIO_TEMP:
			data = dev->sensors[HTS221_SENSOR_T].b_gen;
			div = dev->sensors[HTS221_SENSOR_T].slope;
			break;
		default:
			goto out;
		}

		tmp = div_s64(data * 1000000000LL, div);
		tmp = div_s64_rem(tmp, 1000000000LL, &rem);

		*val = tmp;
		*val2 = abs(rem);
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	}
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = dev->odr;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO: {
		u8 idx;
		const struct hts221_avg *avg;

		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE:
			avg = &hts221_avg_list[HTS221_SENSOR_H];
			idx = dev->sensors[HTS221_SENSOR_H].cur_avg_idx;
			break;
		case IIO_TEMP:
			avg = &hts221_avg_list[HTS221_SENSOR_T];
			idx = dev->sensors[HTS221_SENSOR_T].cur_avg_idx;
			break;
		default:
			goto out;
		}

		*val = avg->avg_avl[idx].avg;
		ret = IIO_VAL_INT;
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

out:
	iio_device_release_direct_mode(iio_dev);

	return ret;
}

static int hts221_write_raw(struct iio_dev *iio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct hts221_dev *dev = iio_priv(iio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = hts221_update_odr(dev, val);
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO: {
		enum hts221_sensor_type type;

		switch (chan->type) {
		case IIO_HUMIDITYRELATIVE:
			type = HTS221_SENSOR_H;
			break;
		case IIO_TEMP:
			type = HTS221_SENSOR_T;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}

		ret = hts221_update_avg(&dev->sensors[type], val);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

out:
	iio_device_release_direct_mode(iio_dev);

	return ret;
}

static int hts221_validate_trigger(struct iio_dev *iio_dev,
				   struct iio_trigger *trig)
{
	struct hts221_dev *dev = iio_priv(iio_dev);

	return dev->trig == trig ? 0 : -EINVAL;
}

static IIO_DEVICE_ATTR(in_humidity_oversampling_ratio_available, S_IRUGO,
		       hts221_sysfs_rh_oversampling_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_oversampling_ratio_available, S_IRUGO,
		       hts221_sysfs_temp_oversampling_avail, NULL, 0);
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(hts221_sysfs_sampling_freq);

static struct attribute *hts221_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_humidity_oversampling_ratio_available.dev_attr.attr,
	&iio_dev_attr_in_temp_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group hts221_attribute_group = {
	.attrs = hts221_attributes,
};

static const struct iio_info hts221_info = {
	.driver_module = THIS_MODULE,
	.attrs = &hts221_attribute_group,
	.read_raw = hts221_read_raw,
	.write_raw = hts221_write_raw,
	.validate_trigger = hts221_validate_trigger,
};

static const unsigned long hts221_scan_masks[] = {0x3, 0x0};

int hts221_probe(struct hts221_dev *dev)
{
	struct iio_dev *iio_dev = iio_priv_to_dev(dev);
	int i, err;

	mutex_init(&dev->lock);

	err = hts221_check_whoami(dev);
	if (err < 0)
		return err;

	err = hts221_update_odr(dev, 1);
	if (err < 0)
		return err;

	dev->odr = hts221_odr_table[0].hz;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = dev->dev;
	iio_dev->available_scan_masks = hts221_scan_masks;
	iio_dev->channels = hts221_channels;
	iio_dev->num_channels = ARRAY_SIZE(hts221_channels);
	iio_dev->name = HTS221_DEV_NAME;
	iio_dev->info = &hts221_info;

	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		dev->sensors[i].type = i;
		dev->sensors[i].dev = dev;

		err = hts221_update_avg(&dev->sensors[i],
					hts221_avg_list[i].avg_avl[3].avg);
		if (err < 0)
			goto power_off;

		err = hts221_parse_caldata(&dev->sensors[i]);
		if (err < 0)
			goto power_off;
	}

	err = hts221_dev_power_off(dev);
	if (err < 0)
		return err;

	if (dev->irq > 0) {
		err = hts221_allocate_buffers(dev);
		if (err < 0)
			return err;

		err = hts221_allocate_triggers(dev);
		if (err)
			return err;
	}

	return devm_iio_device_register(dev->dev, iio_dev);

power_off:
	hts221_dev_power_off(dev);

	return err;
}
EXPORT_SYMBOL(hts221_probe);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics hts221 sensor driver");
MODULE_LICENSE("GPL v2");
