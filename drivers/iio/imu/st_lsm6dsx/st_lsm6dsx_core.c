/*
 * STMicroelectronics st_lsm6dsx sensor driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/unaligned.h>
#include "st_lsm6dsx.h"

#define REG_ACC_DRDY_IRQ_MASK	0x01
#define REG_GYRO_DRDY_IRQ_MASK	0x02
#define REG_WHOAMI_ADDR		0x0f
#define REG_RESET_ADDR		0x12
#define REG_RESET_MASK		0x01
#define REG_BDU_ADDR		0x12
#define REG_BDU_MASK		0x40
#define REG_INT2_ON_INT1_ADDR	0x13
#define REG_INT2_ON_INT1_MASK	0x20
#define REG_ROUNDING_ADDR	0x16
#define REG_ROUNDING_MASK	0x04
#define REG_LIR_ADDR		0x58
#define REG_LIR_MASK		0x01

#define REG_ACC_ODR_ADDR	0x10
#define REG_ACC_ODR_MASK	0xf0
#define REG_ACC_FS_ADDR		0x10
#define REG_ACC_FS_MASK		0x0c
#define REG_ACC_OUT_X_L_ADDR	0x28
#define REG_ACC_OUT_Y_L_ADDR	0x2a
#define REG_ACC_OUT_Z_L_ADDR	0x2c

#define REG_GYRO_ODR_ADDR	0x11
#define REG_GYRO_ODR_MASK	0xf0
#define REG_GYRO_FS_ADDR	0x11
#define REG_GYRO_FS_MASK	0x0c
#define REG_GYRO_OUT_X_L_ADDR	0x22
#define REG_GYRO_OUT_Y_L_ADDR	0x24
#define REG_GYRO_OUT_Z_L_ADDR	0x26

#define ST_LSM6DS3_WHOAMI	0x69
#define ST_LSM6DSM_WHOAMI	0x6a

#define ST_LSM6DSX_ACC_FS_2G_GAIN	IIO_G_TO_M_S_2(61)
#define ST_LSM6DSX_ACC_FS_4G_GAIN	IIO_G_TO_M_S_2(122)
#define ST_LSM6DSX_ACC_FS_8G_GAIN	IIO_G_TO_M_S_2(244)
#define ST_LSM6DSX_ACC_FS_16G_GAIN	IIO_G_TO_M_S_2(488)

#define ST_LSM6DSX_DATA_ACC_AVL_MASK	0x01

#define ST_LSM6DSX_GYRO_FS_245_GAIN	IIO_DEGREE_TO_RAD(4375)
#define ST_LSM6DSX_GYRO_FS_500_GAIN	IIO_DEGREE_TO_RAD(8750)
#define ST_LSM6DSX_GYRO_FS_1000_GAIN	IIO_DEGREE_TO_RAD(17500)
#define ST_LSM6DSX_GYRO_FS_2000_GAIN	IIO_DEGREE_TO_RAD(70000)

#define ST_LSM6DSX_DATA_GYRO_AVL_MASK	0x02

struct st_lsm6dsx_odr {
	u32 hz;
	u8 val;
};

#define ST_LSM6DSX_ODR_LIST_SIZE	6
struct st_lsm6dsx_odr_table_entry {
	u8 addr;
	u8 mask;
	struct st_lsm6dsx_odr odr_avl[ST_LSM6DSX_ODR_LIST_SIZE];
};

static const struct st_lsm6dsx_odr_table_entry st_lsm6dsx_odr_table[] = {
	[ST_LSM6DSX_ID_ACC] = {
		.addr = REG_ACC_ODR_ADDR,
		.mask = REG_ACC_ODR_MASK,
		.odr_avl[0] = {  13, 0x01 },
		.odr_avl[1] = {  26, 0x02 },
		.odr_avl[2] = {  52, 0x03 },
		.odr_avl[3] = { 104, 0x04 },
		.odr_avl[4] = { 208, 0x05 },
		.odr_avl[5] = { 416, 0x06 },
	},
	[ST_LSM6DSX_ID_GYRO] = {
		.addr = REG_GYRO_ODR_ADDR,
		.mask = REG_GYRO_ODR_MASK,
		.odr_avl[0] = {  13, 0x01 },
		.odr_avl[1] = {  26, 0x02 },
		.odr_avl[2] = {  52, 0x03 },
		.odr_avl[3] = { 104, 0x04 },
		.odr_avl[4] = { 208, 0x05 },
		.odr_avl[5] = { 416, 0x06 },
	}
};

struct st_lsm6dsx_fs {
	u32 gain;
	u8 val;
};

#define ST_LSM6DSX_FS_LIST_SIZE		4
struct st_lsm6dsx_fs_table_entry {
	u8 addr;
	u8 mask;
	struct st_lsm6dsx_fs fs_avl[ST_LSM6DSX_FS_LIST_SIZE];
};

static const struct st_lsm6dsx_fs_table_entry st_lsm6dsx_fs_table[] = {
	[ST_LSM6DSX_ID_ACC] = {
		.addr = REG_ACC_FS_ADDR,
		.mask = REG_ACC_FS_MASK,
		.fs_avl[0] = {  ST_LSM6DSX_ACC_FS_2G_GAIN, 0x0 },
		.fs_avl[1] = {  ST_LSM6DSX_ACC_FS_4G_GAIN, 0x2 },
		.fs_avl[2] = {  ST_LSM6DSX_ACC_FS_8G_GAIN, 0x3 },
		.fs_avl[3] = { ST_LSM6DSX_ACC_FS_16G_GAIN, 0x1 },
	},
	[ST_LSM6DSX_ID_GYRO] = {
		.addr = REG_GYRO_FS_ADDR,
		.mask = REG_GYRO_FS_MASK,
		.fs_avl[0] = {  ST_LSM6DSX_GYRO_FS_245_GAIN, 0x0 },
		.fs_avl[1] = {  ST_LSM6DSX_GYRO_FS_500_GAIN, 0x1 },
		.fs_avl[2] = { ST_LSM6DSX_GYRO_FS_1000_GAIN, 0x2 },
		.fs_avl[3] = { ST_LSM6DSX_GYRO_FS_2000_GAIN, 0x3 },
	}
};

static const struct iio_chan_spec st_lsm6dsx_acc_channels[] = {
	{
		.type = IIO_ACCEL,
		.address = REG_ACC_OUT_X_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ACCEL,
		.address = REG_ACC_OUT_Y_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ACCEL,
		.address = REG_ACC_OUT_Z_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 2,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dsx_gyro_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.address = REG_GYRO_OUT_X_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ANGL_VEL,
		.address = REG_GYRO_OUT_Y_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ANGL_VEL,
		.address = REG_GYRO_OUT_Z_L_ADDR,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 2,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

int st_lsm6dsx_write_with_mask(struct st_lsm6dsx_dev *dev, u8 addr, u8 mask,
			       u8 val)
{
	u8 data;
	int err;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, addr, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read %02x register\n", addr);
		mutex_unlock(&dev->lock);

		return err;
	}

	data = (data & ~mask) | ((val << __ffs(mask)) & mask);

	err = dev->tf->write(dev->dev, addr, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to write %02x register\n", addr);
		mutex_unlock(&dev->lock);

		return err;
	}

	mutex_unlock(&dev->lock);

	return 0;
}

static int st_lsm6dsx_check_whoami(struct st_lsm6dsx_dev *dev)
{
	u8 data;
	int err;

	err = dev->tf->read(dev->dev, REG_WHOAMI_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_LSM6DS3_WHOAMI &&
	    data != ST_LSM6DSM_WHOAMI) {
		dev_err(dev->dev, "wrong whoami [%02x]\n", data);
		return -ENODEV;
	}

	return 0;
}

static int st_lsm6dsx_set_fs(struct st_lsm6dsx_sensor *sensor, u32 gain)
{
	u8 val;
	int i, err;
	enum st_lsm6dsx_sensor_id id = sensor->id;

	for (i = 0; i < ST_LSM6DSX_FS_LIST_SIZE; i++)
		if (st_lsm6dsx_fs_table[id].fs_avl[i].gain == gain)
			break;

	if (i == ST_LSM6DSX_FS_LIST_SIZE)
		return -EINVAL;

	val = st_lsm6dsx_fs_table[id].fs_avl[i].val;
	err = st_lsm6dsx_write_with_mask(sensor->dev,
					 st_lsm6dsx_fs_table[id].addr,
					 st_lsm6dsx_fs_table[id].mask, val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

static int st_lsm6dsx_set_odr(struct st_lsm6dsx_sensor *sensor, u16 odr)
{
	u8 val;
	int i, err;
	enum st_lsm6dsx_sensor_id id = sensor->id;
	struct st_lsm6dsx_dev *dev = sensor->dev;

	for (i = 0; i < ST_LSM6DSX_ODR_LIST_SIZE; i++)
		if (st_lsm6dsx_odr_table[id].odr_avl[i].hz == odr)
			break;

	if (i == ST_LSM6DSX_ODR_LIST_SIZE)
		return -EINVAL;

	disable_irq(dev->irq);

	val = st_lsm6dsx_odr_table[id].odr_avl[i].val;
	err = st_lsm6dsx_write_with_mask(sensor->dev,
					 st_lsm6dsx_odr_table[id].addr,
					 st_lsm6dsx_odr_table[id].mask, val);
	if (err < 0) {
		enable_irq(dev->irq);
		return err;
	}

	sensor->odr = odr;
	enable_irq(dev->irq);

	return 0;
}

int st_lsm6dsx_set_enable(struct st_lsm6dsx_sensor *sensor, bool enable)
{
	enum st_lsm6dsx_sensor_id id = sensor->id;

	if (enable)
		return st_lsm6dsx_set_odr(sensor, sensor->odr);
	else
		return st_lsm6dsx_write_with_mask(sensor->dev,
					st_lsm6dsx_odr_table[id].addr,
					st_lsm6dsx_odr_table[id].mask, 0);
}

static int st_lsm6dsx_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		int err;
		u8 data[2];
		struct st_lsm6dsx_dev *dev;

		mutex_lock(&iio_dev->mlock);

		if (iio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&iio_dev->mlock);
			return -EBUSY;
		}

		dev = sensor->dev;
		err = st_lsm6dsx_set_enable(sensor, true);
		if (err < 0) {
			mutex_unlock(&iio_dev->mlock);
			return err;
		}

		msleep(120);

		err = dev->tf->read(dev->dev, ch->address, 2, data);
		if (err < 0) {
			mutex_unlock(&iio_dev->mlock);
			return err;
		}

		st_lsm6dsx_set_enable(sensor, false);

		*val = (s16)get_unaligned_le16(data);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&iio_dev->mlock);

		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sensor->gain;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int st_lsm6dsx_write_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	int err;
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&iio_dev->mlock);

		if (iio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&iio_dev->mlock);
			return -EBUSY;
		}
		err = st_lsm6dsx_set_fs(sensor, val2);

		mutex_unlock(&iio_dev->mlock);
		break;
	default:
		return -EINVAL;
	}

	return err < 0 ? err : 0;
}

static int st_lsm6dsx_validate_trigger(struct iio_dev *iio_dev,
				       struct iio_trigger *trig)
{
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	return sensor->trig == trig ? 0 : -EINVAL;
}

static ssize_t
st_lsm6dsx_sysfs_get_sampling_frequency(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	struct st_lsm6dsx_sensor *sensor = iio_priv(dev_get_drvdata(device));

	return sprintf(buf, "%d\n", sensor->odr);
}

static ssize_t
st_lsm6dsx_sysfs_set_sampling_frequency(struct device *device,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int err;
	u32 odr;
	struct st_lsm6dsx_sensor *sensor = iio_priv(dev_get_drvdata(device));

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	err = st_lsm6dsx_set_odr(sensor, odr);

	return err < 0 ? err : size;
}

static ssize_t
st_lsm6dsx_sysfs_sampling_frequency_avl(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int i, len = 0;
	struct st_lsm6dsx_sensor *sensor = iio_priv(dev_get_drvdata(device));
	enum st_lsm6dsx_sensor_id id = sensor->id;

	for (i = 0; i < ST_LSM6DSX_ODR_LIST_SIZE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 st_lsm6dsx_odr_table[id].odr_avl[i].hz);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6dsx_sysfs_scale_avail(struct device *device,
					    struct device_attribute *attr,
					    char *buf)
{
	int i, len = 0;
	struct st_lsm6dsx_sensor *sensor = iio_priv(dev_get_drvdata(device));
	enum st_lsm6dsx_sensor_id id = sensor->id;

	for (i = 0; i < ST_LSM6DSX_FS_LIST_SIZE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_lsm6dsx_fs_table[id].fs_avl[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
			      st_lsm6dsx_sysfs_get_sampling_frequency,
			      st_lsm6dsx_sysfs_set_sampling_frequency);
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lsm6dsx_sysfs_sampling_frequency_avl);
static IIO_DEVICE_ATTR(in_accel_scale_available, S_IRUGO,
		       st_lsm6dsx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, S_IRUGO,
		       st_lsm6dsx_sysfs_scale_avail, NULL, 0);

static struct attribute *st_lsm6dsx_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsx_acc_attribute_group = {
	.attrs = st_lsm6dsx_acc_attributes,
};

static const struct iio_info st_lsm6dsx_acc_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dsx_acc_attribute_group,
	.read_raw = st_lsm6dsx_read_raw,
	.write_raw = st_lsm6dsx_write_raw,
	.validate_trigger = st_lsm6dsx_validate_trigger,
};

static struct attribute *st_lsm6dsx_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsx_gyro_attribute_group = {
	.attrs = st_lsm6dsx_gyro_attributes,
};

static const struct iio_info st_lsm6dsx_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dsx_gyro_attribute_group,
	.read_raw = st_lsm6dsx_read_raw,
	.write_raw = st_lsm6dsx_write_raw,
};

static const unsigned long st_lsm6dsx_available_scan_masks[] = {0x7, 0x0};

static int st_lsm6dsx_init_device(struct st_lsm6dsx_dev *dev)
{
	int err;
	u8 data;

	data = REG_RESET_MASK;
	err = dev->tf->write(dev->dev, REG_RESET_ADDR, 1, &data);
	if (err < 0)
		return err;

	msleep(200);

	/* latch interrupts */
	err = st_lsm6dsx_write_with_mask(dev, REG_LIR_ADDR, REG_LIR_MASK, 1);
	if (err < 0)
		return err;

	/* enable BDU */
	err = st_lsm6dsx_write_with_mask(dev, REG_BDU_ADDR, REG_BDU_MASK, 1);
	if (err < 0)
		return err;

	err = st_lsm6dsx_write_with_mask(dev, REG_ROUNDING_ADDR,
					 REG_ROUNDING_MASK, 1);
	if (err < 0)
		return err;

	/* redirect INT2 on INT1 */
	err = st_lsm6dsx_write_with_mask(dev, REG_INT2_ON_INT1_ADDR,
					 REG_INT2_ON_INT1_MASK, 1);
	if (err < 0)
		return err;

	return 0;
}

static struct iio_dev *st_lsm6dsx_alloc_iiodev(struct st_lsm6dsx_dev *dev,
					       enum st_lsm6dsx_sensor_id id)
{
	struct iio_dev *iio_dev;
	struct st_lsm6dsx_sensor *sensor;

	iio_dev = devm_iio_device_alloc(dev->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = dev->dev;
	iio_dev->available_scan_masks = st_lsm6dsx_available_scan_masks;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->dev = dev;
	sensor->odr = st_lsm6dsx_odr_table[id].odr_avl[0].hz;
	sensor->gain = st_lsm6dsx_fs_table[id].fs_avl[0].gain;

	switch (id) {
	case ST_LSM6DSX_ID_ACC:
		iio_dev->channels = st_lsm6dsx_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dsx_acc_channels);
		iio_dev->name = "lsm6dsx_accel";
		iio_dev->info = &st_lsm6dsx_acc_info;

		sensor->drdy_data_mask = ST_LSM6DSX_DATA_ACC_AVL_MASK;
		sensor->drdy_irq_mask = REG_ACC_DRDY_IRQ_MASK;
		break;
	case ST_LSM6DSX_ID_GYRO:
		iio_dev->channels = st_lsm6dsx_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dsx_gyro_channels);
		iio_dev->name = "lsm6dsx_gyro";
		iio_dev->info = &st_lsm6dsx_gyro_info;

		sensor->drdy_data_mask = ST_LSM6DSX_DATA_GYRO_AVL_MASK;
		sensor->drdy_irq_mask = REG_GYRO_DRDY_IRQ_MASK;
		break;
	default:
		return NULL;
	}

	return iio_dev;
}

int st_lsm6dsx_probe(struct st_lsm6dsx_dev *dev)
{
	int i, err;

	mutex_init(&dev->lock);

	err = st_lsm6dsx_check_whoami(dev);
	if (err < 0)
		return err;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		dev->iio_devs[i] = st_lsm6dsx_alloc_iiodev(dev, i);
		if (!dev->iio_devs[i])
			return -ENOMEM;
	}

	err = st_lsm6dsx_init_device(dev);
	if (err < 0)
		return err;

	if (dev->irq > 0) {
		err = st_lsm6dsx_allocate_buffers(dev);
		if (err < 0)
			return err;

		err = st_lsm6dsx_allocate_triggers(dev);
		if (err < 0)
			return err;
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		err = devm_iio_device_register(dev->dev, dev->iio_devs[i]);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsx_probe);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsx driver");
MODULE_LICENSE("GPL v2");
