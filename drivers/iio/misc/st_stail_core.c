/*
 * STMicroelectronics st_stail sensor driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "st_stail.h"

#define ST_STAIL_CHANNEL(chan_type, addr, mod, scan_idx)		\
{									\
	.type = chan_type,						\
	.address = addr,						\
	.modified = 1,							\
	.channel2 = mod,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
				   BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = scan_idx,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_BE,					\
	},								\
}

static const struct iio_chan_spec st_stail_acc_channels[] = {
	ST_STAIL_CHANNEL(IIO_ACCEL, 0xff, IIO_MOD_X, 0),
	ST_STAIL_CHANNEL(IIO_ACCEL, 0xff, IIO_MOD_Y, 1),
	ST_STAIL_CHANNEL(IIO_ACCEL, 0xff, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_stail_gyro_channels[] = {
	ST_STAIL_CHANNEL(IIO_ANGL_VEL, 0xff, IIO_MOD_X, 0),
	ST_STAIL_CHANNEL(IIO_ANGL_VEL, 0xff, IIO_MOD_Y, 1),
	ST_STAIL_CHANNEL(IIO_ANGL_VEL, 0xff, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_stail_magn_channels[] = {
	ST_STAIL_CHANNEL(IIO_MAGN, 0xff, IIO_MOD_X, 0),
	ST_STAIL_CHANNEL(IIO_MAGN, 0xff, IIO_MOD_Y, 1),
	ST_STAIL_CHANNEL(IIO_MAGN, 0xff, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int st_stail_read_oneshot(struct st_stail_sensor *sensor,
				 u8 addr, int *val)
{
	*val = 0;

	return IIO_VAL_INT;
}

static int st_stail_read_raw(struct iio_dev *iio_dev,
			     struct iio_chan_spec const *ch,
			     int *val, int *val2, long mask)
{
	struct st_stail_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			break;
		ret = st_stail_read_oneshot(sensor, ch->address, val);
		iio_device_release_direct_mode(iio_dev);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_stail_write_raw(struct iio_dev *iio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	int ret;

	switch (mask) {
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct attribute *st_stail_acc_attributes[] = {
	NULL,
};

static const struct attribute_group st_stail_acc_attribute_group = {
	.attrs = st_stail_acc_attributes,
};

static const struct iio_info st_stail_acc_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_stail_acc_attribute_group,
	.read_raw = st_stail_read_raw,
	.write_raw = st_stail_write_raw,
};

static struct attribute *st_stail_gyro_attributes[] = {
	NULL,
};

static const struct attribute_group st_stail_gyro_attribute_group = {
	.attrs = st_stail_gyro_attributes,
};

static const struct iio_info st_stail_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_stail_gyro_attribute_group,
	.read_raw = st_stail_read_raw,
	.write_raw = st_stail_write_raw,
};

static struct attribute *st_stail_magn_attributes[] = {
	NULL,
};

static const struct attribute_group st_stail_magn_attribute_group = {
	.attrs = st_stail_magn_attributes,
};

static const struct iio_info st_stail_magn_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_stail_magn_attribute_group,
	.read_raw = st_stail_read_raw,
	.write_raw = st_stail_write_raw,
};

static const unsigned long st_stail_available_scan_masks[] = {0x7, 0x0};

static struct iio_dev *st_stail_alloc_iiodev(struct st_stail_hw *hw,
					     enum st_stail_sensor_id id)
{
	struct st_stail_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;
	iio_dev->available_scan_masks = st_stail_available_scan_masks;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;

	switch (id) {
	case ST_STAIL_ID_ACC:
		iio_dev->channels = st_stail_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_stail_acc_channels);
		iio_dev->name = "stail_accel";
		iio_dev->info = &st_stail_acc_info;
		break;
	case ST_STAIL_ID_GYRO:
		iio_dev->channels = st_stail_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_stail_gyro_channels);
		iio_dev->name = "stail_gyro";
		iio_dev->info = &st_stail_gyro_info;
		break;
	case ST_STAIL_ID_MAG:
		iio_dev->channels = st_stail_magn_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_stail_magn_channels);
		iio_dev->name = "stail_magn";
		iio_dev->info = &st_stail_magn_info;
		break;
	default:
		return NULL;
	}

	return iio_dev;
}

int st_stail_probe(struct st_stail_hw *hw)
{
	int i, err;

	for (i = 0; i < ST_STAIL_ID_MAX; i++) {
		hw->iio_devs[i] = st_stail_alloc_iiodev(hw, i);
		if (!hw->iio_devs[i])
			return -ENOMEM;

		err = st_stail_allocate_buffer(hw->iio_devs[i]);
		if (err < 0)
			return err;
	}

	st_stail_allocate_trigger(hw);

	for (i = 0; i < ST_STAIL_ID_MAX; i++) {
		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_stail_probe);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_stail driver");
MODULE_LICENSE("GPL v2");
