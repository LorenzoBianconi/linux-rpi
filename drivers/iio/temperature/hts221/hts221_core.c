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

#define REG_WHOAMI_ADDR		0x0f
#define REG_WHOAMI_VAL		0xbc

#define REG_CNTRL1_ADDR		0x20
#define REG_CNTRL2_ADDR		0x21
#define REG_CNTRL3_ADDR		0x22
#define REG_STATUS_ADDR		0x27

#define REG_H_AVG_ADDR		0x10
#define REG_T_AVG_ADDR		0x10
#define REG_H_OUT_L		0x28
#define REG_T_OUT_L		0x2a

#define H_AVG_MASK		0x07
#define T_AVG_MASK		0x38

#define ODR_MASK		0x87
#define BDU_MASK		0x04

#define DRDY_MASK		0x04

#define ENABLE_SENSOR		0x80

#define H_AVG_4			0x00 /* 0.4 %RH */
#define H_AVG_8			0x01 /* 0.3 %RH */
#define H_AVG_16		0x02 /* 0.2 %RH */
#define H_AVG_32		0x03 /* 0.15 %RH */
#define H_AVG_64		0x04 /* 0.1 %RH */
#define H_AVG_128		0x05 /* 0.07 %RH */
#define H_AVG_256		0x06 /* 0.05 %RH */
#define H_AVG_512		0x07 /* 0.03 %RH */

#define T_AVG_2			0x00 /* 0.08 degC */
#define T_AVG_4			0x08 /* 0.05 degC */
#define T_AVG_8			0x10 /* 0.04 degC */
#define T_AVG_16		0x18 /* 0.03 degC */
#define T_AVG_32		0x20 /* 0.02 degC */
#define T_AVG_64		0x28 /* 0.015 degC */
#define T_AVG_128		0x30 /* 0.01 degC */
#define T_AVG_256		0x38 /* 0.007 degC */

/* caldata registers */
#define REG_0RH_CAL_X_H		0x36
#define REG_1RH_CAL_X_H		0x3a
#define REG_0RH_CAL_Y_H		0x30
#define REG_1RH_CAL_Y_H		0x31
#define REG_0T_CAL_X_L		0x3c
#define REG_1T_CAL_X_L		0x3e
#define REG_0T_CAL_Y_H		0x32
#define REG_1T_CAL_Y_H		0x33
#define REG_T1_T0_CAL_Y_H	0x35

struct hts221_odr {
	u32 hz;
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
	{ 13, 0x03 },	/* 12.5 HZ */
};

static const struct hts221_avg hts221_avg_list[] = {
	{
		.addr = REG_H_AVG_ADDR,
		.mask = H_AVG_MASK,
		.avg_avl = {
			{ 4, H_AVG_4 },
			{ 8, H_AVG_8 },
			{ 16, H_AVG_16 },
			{ 32, H_AVG_32 },
			{ 64, H_AVG_64 },
			{ 128, H_AVG_128 },
			{ 256, H_AVG_256 },
			{ 512, H_AVG_512 },
		},
	},
	{
		.addr = REG_T_AVG_ADDR,
		.mask = T_AVG_MASK,
		.avg_avl = {
			{ 2, T_AVG_2 },
			{ 4, T_AVG_4 },
			{ 8, T_AVG_8 },
			{ 16, T_AVG_16 },
			{ 32, T_AVG_32 },
			{ 64, T_AVG_64 },
			{ 128, T_AVG_128 },
			{ 256, T_AVG_256 },
		},
	},
};

static const struct iio_chan_spec hts221_channels[] = {
	{
		.type = IIO_HUMIDITYRELATIVE,
		.address = REG_H_OUT_L,
		.modified = 0,
		.channel2 = IIO_NO_MOD,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_TEMP,
		.address = REG_T_OUT_L,
		.modified = 0,
		.channel2 = IIO_NO_MOD,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
};

static int hts221_write_with_mask(struct hts221_dev *dev, u8 addr, u8 mask,
				  u8 val)
{
	u8 data;
	int err;

	err = dev->tf->read(dev->dev, addr, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read %02x register\n", addr);
		return err;
	}

	data = (data & ~mask) | (val & mask);

	err = dev->tf->write(dev->dev, addr, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to write %02x register\n", addr);
		return err;
	}

	return 0;
}

static int hts221_check_whoami(struct hts221_dev *dev)
{
	u8 data;
	int err;

	err = dev->tf->read(dev->dev, REG_WHOAMI_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != REG_WHOAMI_VAL) {
		dev_err(dev->dev, "wrong whoami {%02x-%02x}\n",
			data, REG_WHOAMI_VAL);
		return -ENODEV;
	}

	return 0;
}

int hts221_config_drdy(struct hts221_dev *dev, bool enable)
{
	int err;
	u8 val = (enable) ? 0x04 : 0;
	
	mutex_lock(&dev->lock);
	err = hts221_write_with_mask(dev, REG_CNTRL3_ADDR, DRDY_MASK, val);
	mutex_unlock(&dev->lock);

	return err;
}

int hts221_push_data(struct iio_dev *indio_dev)
{
	int err;
	s16 data;
	u8 status;
	struct hts221_dev *dev = iio_priv(indio_dev);

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, REG_STATUS_ADDR, 1, &status);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}

	if (status & 0x01) {
		err = dev->tf->read(dev->dev, REG_T_OUT_L, 2, (u8 *)&data);
		if (err < 0) {
			dev_err(dev->dev, "failed to read reg %02x\n",
				REG_T_OUT_L);
			mutex_unlock(&dev->lock);
			return err;
		}
		if (test_bit(0, indio_dev->active_scan_mask))
			dev->buffer[HTS221_SENSOR_T] = data;
	}

	if (status & 0x02) {
		err = dev->tf->read(dev->dev, REG_H_OUT_L, 2, (u8 *)&data);
		if (err < 0) {
			dev_err(dev->dev, "failed to read reg %02x\n",
				REG_H_OUT_L);
			mutex_unlock(&dev->lock);
			return err;
		}
		if (test_bit(1, indio_dev->active_scan_mask))
			dev->buffer[HTS221_SENSOR_H] = data;
	}

	mutex_unlock(&dev->lock);

	return 0;
}

static int hts221_update_odr(struct hts221_dev *dev, u8 odr)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(hts221_odr_table); i++)
		if (hts221_odr_table[i].hz == odr)
			break;

	if (i == ARRAY_SIZE(hts221_odr_table))
		return -EINVAL;

	err = hts221_write_with_mask(dev, REG_CNTRL1_ADDR, ODR_MASK,
				     ENABLE_SENSOR | BDU_MASK |
				     hts221_odr_table[i].val);
	if (err < 0)
		return err;

	dev->odr = odr;

	return 0;
}

static int hts221_update_avg(struct hts221_dev *dev,
			     enum hts221_sensor_type type, u16 val)
{
	int i, err;
	const struct hts221_avg *avg = &hts221_avg_list[type];

	for (i = 0; i < HTS221_AVG_DEPTH; i++)
		if (hts221_avg_list[type].avg_avl[i].avg == val)
			break;

	if (i == HTS221_AVG_DEPTH)
		return -EINVAL;

	err = hts221_write_with_mask(dev, avg->addr, avg->mask,
				     avg->avg_avl[i].val);
	if (err < 0)
		return err;

	dev->sensors[type].cur_avg_idx = i;

	return 0;
}

static ssize_t
hts221_sysfs_get_h_avg_val(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	struct hts221_dev *dev = iio_priv(dev_get_drvdata(device));
	u8 idx = dev->sensors[HTS221_SENSOR_H].cur_avg_idx;
	u16 val = hts221_avg_list[HTS221_SENSOR_H].avg_avl[idx].avg;

	return sprintf(buf, "%d\n", val);
}

static ssize_t
hts221_sysfs_set_h_avg_val(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	int err;
	unsigned int val;
	struct iio_dev *indio_dev = dev_get_drvdata(device);
	struct hts221_dev *dev = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&dev->lock);
	err = hts221_update_avg(dev, HTS221_SENSOR_H, (u16)val);
	mutex_unlock(&dev->lock);

	return err < 0 ? err : size;
}

static ssize_t
hts221_sysfs_get_t_avg_val(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	struct hts221_dev *dev = iio_priv(dev_get_drvdata(device));
	u8 idx = dev->sensors[HTS221_SENSOR_T].cur_avg_idx;
	u16 val = hts221_avg_list[HTS221_SENSOR_T].avg_avl[idx].avg;

	return sprintf(buf, "%d\n", val);
}

static ssize_t
hts221_sysfs_set_t_avg_val(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	int err;
	unsigned int val;
	struct iio_dev *indio_dev = dev_get_drvdata(device);
	struct hts221_dev *dev = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&dev->lock);
	err = hts221_update_avg(dev, HTS221_SENSOR_T, (u16)val);
	mutex_unlock(&dev->lock);

	return err < 0 ? err : size;
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
hts221_sysfs_get_sampling_frequency(struct device *device,
				    struct device_attribute *attr, char *buf)
{
	struct hts221_dev *dev = iio_priv(dev_get_drvdata(device));

	return sprintf(buf, "%d\n", dev->odr);
}

static ssize_t
hts221_sysfs_set_sampling_frequency(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int err;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(device);
	struct hts221_dev *dev = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&dev->lock);
	err = hts221_update_odr(dev, odr);
	mutex_unlock(&dev->lock);

	return err < 0 ? err : size;
}

int hts221_power_on(struct hts221_dev *dev)
{
	u8 idx;
	u16 val;
	int err;

	idx = dev->sensors[HTS221_SENSOR_H].cur_avg_idx;
	val = hts221_avg_list[HTS221_SENSOR_H].avg_avl[idx].avg;
	err = hts221_update_avg(dev, HTS221_SENSOR_H, val);
	if (err < 0)
		return err;

	idx = dev->sensors[HTS221_SENSOR_T].cur_avg_idx;
	val = hts221_avg_list[HTS221_SENSOR_T].avg_avl[idx].avg;
	err = hts221_update_avg(dev, HTS221_SENSOR_T, val);
	if (err < 0)
		return err;

	return hts221_update_odr(dev, dev->odr);
}

int hts221_power_off(struct hts221_dev *dev)
{
	int err;
	u8 data = 0;

	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to write %02x register\n",
			REG_CNTRL1_ADDR);
		return err;
	}

	err = dev->tf->write(dev->dev, REG_CNTRL2_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to write %02x register\n",
			REG_CNTRL2_ADDR);
		return err;
	}

	return 0;
}

static int hts221_get_caldata(struct hts221_dev *dev,
			      enum hts221_sensor_type type)
{
	int err, *slope, *b_gen;
	u8 addr_x0, addr_x1;
	s16 cal_x0, cal_x1, cal_y0, cal_y1;

	switch (type) {
	case HTS221_SENSOR_H:
		addr_x0 = REG_0RH_CAL_X_H;
		addr_x1 = REG_1RH_CAL_X_H;

		cal_y1 = 0;
		cal_y0 = 0;
		err = dev->tf->read(dev->dev, REG_0RH_CAL_Y_H, 1,
				    (u8 *)&cal_y0);
		if (err < 0)
			return err;

		err = dev->tf->read(dev->dev, REG_1RH_CAL_Y_H, 1,
				    (u8 *)&cal_y1);
		if (err < 0)
			return err;
		break;
	case HTS221_SENSOR_T: {
		u8 cal0, cal1;

		addr_x0 = REG_0T_CAL_X_L;
		addr_x1 = REG_1T_CAL_X_L;

		err = dev->tf->read(dev->dev, REG_0T_CAL_Y_H, 1, &cal0);
		if (err < 0)
			return err;

		err = dev->tf->read(dev->dev, REG_T1_T0_CAL_Y_H, 1, &cal1);
		if (err < 0)
			return err;
		cal_y0 = (le16_to_cpu(cal1 & 0x3) << 8) | cal0;

		err = dev->tf->read(dev->dev, REG_1T_CAL_Y_H, 1, &cal0);
		if (err < 0)
			return err;

		err = dev->tf->read(dev->dev, REG_T1_T0_CAL_Y_H, 1, &cal1);
		if (err < 0)
			return err;
		cal_y1 = (le16_to_cpu((cal1 & 0xc) >> 2) << 8) | cal0;
		break;
	}
	default:
		return -ENODEV;
	}

	err = dev->tf->read(dev->dev, addr_x0, 2, (u8 *)&cal_x0);
	if (err < 0)
		return err;
	cal_x0 = le32_to_cpu(cal_x0);

	err = dev->tf->read(dev->dev, addr_x1, 2, (u8 *)&cal_x1);
	if (err < 0)
		return err;
	cal_x1 = le32_to_cpu(cal_x1);

	slope = &dev->sensors[type].slope;
	b_gen = &dev->sensors[type].b_gen;

	*slope = ((cal_y1 - cal_y0) * 8000) / (cal_x1 - cal_x0);
	*b_gen = (((s32)cal_x1 * cal_y0 - (s32)cal_x0 * cal_y1) * 1000) /
		 (cal_x1 - cal_x0);
	*b_gen *= 8;

	return 0;
}

static int hts221_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *ch,
			   int *val, int *val2, long mask)
{
	int ret;
	struct hts221_dev *dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		u8 data[2];
		enum hts221_sensor_type type;

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED)
			return -EBUSY;

		type = (ch->type == IIO_TEMP) ? HTS221_SENSOR_T
					      : HTS221_SENSOR_H;
		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE:
			type = HTS221_SENSOR_H;
			break;
		case IIO_TEMP:
			type = HTS221_SENSOR_T;
			break;
		default:
			return -EINVAL;
		}

		mutex_lock(&dev->lock);

		ret = hts221_power_on(dev);
		if (ret < 0) {
			mutex_unlock(&dev->lock);
			return ret;
		}

		msleep(50);
		ret = dev->tf->read(dev->dev, ch->address, 2, data);
		if (ret < 0) {
			mutex_unlock(&dev->lock);
			return ret;
		}

		ret = hts221_power_off(dev);
		if (ret < 0) {
			mutex_unlock(&dev->lock);
			return ret;
		}

		*val = (s16)get_unaligned_le16(data);
		ret = IIO_VAL_INT;

		mutex_unlock(&dev->lock);
		break;
	}
	case IIO_CHAN_INFO_CALIBSCALE: {
		s64 tmp;
		s32 rem, div, data;

		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE: {
			data = dev->sensors[HTS221_SENSOR_H].slope;
			div = (1 << 4);
			break;
		}
		case IIO_TEMP: {
			data = dev->sensors[HTS221_SENSOR_T].slope;
			div = (1 << 6);
			break;
		}
		default:
			return -EINVAL;
		}

		tmp = div_s64(data * 1000000000LL, div);
		tmp = div_s64_rem(tmp, 1000000000LL, &rem);

		*val = tmp;
		*val2 = abs(rem);
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	}
	case IIO_CHAN_INFO_CALIBBIAS: {
		s64 tmp;
		s32 rem, div, data;

		switch (ch->type) {
		case IIO_HUMIDITYRELATIVE: {
			data = dev->sensors[HTS221_SENSOR_H].b_gen;
			div = (1 << 4);
			break;
		}
		case IIO_TEMP: {
			data = dev->sensors[HTS221_SENSOR_T].b_gen;
			div = (1 << 6);
			break;
		}
		default:
			return -EINVAL;
		}

		tmp = div_s64(data * 1000000000LL, div);
		tmp = div_s64_rem(tmp, 1000000000LL, &rem);

		*val = tmp;
		*val2 = abs(rem);
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static IIO_CONST_ATTR(humidityrelative_avg_sample_available,
		      "4 8 16 32 64 128 256 512");
static IIO_DEVICE_ATTR(humidityrelative_avg_sample, S_IWUSR | S_IRUGO,
		       hts221_sysfs_get_h_avg_val,
		       hts221_sysfs_set_h_avg_val, 0);
static IIO_CONST_ATTR(temp_avg_sample_available,
		      "2 4 8 16 32 64 128 256");
static IIO_DEVICE_ATTR(temp_avg_sample, S_IWUSR | S_IRUGO,
		       hts221_sysfs_get_t_avg_val,
		       hts221_sysfs_set_t_avg_val, 0);

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(hts221_sysfs_sampling_freq);
static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
			      hts221_sysfs_get_sampling_frequency,
			      hts221_sysfs_set_sampling_frequency);

static struct attribute *hts221_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_humidityrelative_avg_sample_available.dev_attr.attr,
	&iio_dev_attr_humidityrelative_avg_sample.dev_attr.attr,
	&iio_const_attr_temp_avg_sample_available.dev_attr.attr,
	&iio_dev_attr_temp_avg_sample.dev_attr.attr,
	NULL,
};

static const struct attribute_group hts221_attribute_group = {
	.attrs = hts221_attributes,
};

static const struct iio_info hts221_info = {
	.driver_module = THIS_MODULE,
	.attrs = &hts221_attribute_group,
	.read_raw = &hts221_read_raw,
};

int hts221_probe(struct iio_dev *indio_dev)
{
	int err;
	struct hts221_dev *dev = iio_priv(indio_dev);

	mutex_init(&dev->lock);

	mutex_lock(&dev->lock);

	err = hts221_check_whoami(dev);
	if (err < 0)
		goto unlock;

	err = hts221_update_odr(dev, 1);
	if (err < 0)
		goto unlock;

	err = hts221_update_avg(dev, HTS221_SENSOR_H, 4);
	if (err < 0)
		goto power_off;

	err = hts221_update_avg(dev, HTS221_SENSOR_T, 2);
	if (err < 0)
		goto power_off;

	/* get calibration data */
	if ((hts221_get_caldata(dev, HTS221_SENSOR_H) < 0) ||
	    (hts221_get_caldata(dev, HTS221_SENSOR_T) < 0))
		goto power_off;

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &hts221_info;

	indio_dev->channels = hts221_channels;
	indio_dev->num_channels = ARRAY_SIZE(hts221_channels);

	if (dev->irq > 0) {
		err = hts221_allocate_buffer(indio_dev);
		if (err < 0)
			goto power_off;

		err = hts221_allocate_trigger(indio_dev);
		if (err)
			goto trigger_error;
	}

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto power_off;

	mutex_unlock(&dev->lock);

	return 0;

trigger_error:
	if (dev->irq > 0) {
		hts221_deallocate_trigger(indio_dev);
		hts221_deallocate_buffer(indio_dev);
	}
power_off:
	hts221_power_off(dev);
unlock:
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(hts221_probe);

int hts221_remove(struct iio_dev *indio_dev)
{
	int err;
	struct hts221_dev *dev = iio_priv(indio_dev);

	err = hts221_power_off(dev);
	if (err < 0)
		return err;

	if (dev->irq > 0) {
		hts221_deallocate_trigger(indio_dev);
		hts221_deallocate_buffer(indio_dev);
	}
	iio_device_unregister(indio_dev);

	return 0;
}
EXPORT_SYMBOL(hts221_remove);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics hts221 sensor driver");
MODULE_LICENSE("GPL v2");
