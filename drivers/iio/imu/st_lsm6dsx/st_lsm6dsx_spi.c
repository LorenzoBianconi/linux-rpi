/*
 * STMicroelectronics st_lsm6dsx spi driver
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
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>
#include "st_lsm6dsx.h"

#define SENSORS_SPI_READ	0x80

static int st_lsm6dsx_spi_read(struct device *device, u8 addr, int len,
			       u8 *data)
{
	int err;
	struct spi_device *spi = to_spi_device(device);
	struct st_lsm6dsx_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = dev->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},{
			.rx_buf = dev->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	dev->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	err = spi_sync_transfer(spi, xfers,  ARRAY_SIZE(xfers));
	if (err < 0)
		return err;

	memcpy(data, dev->tb.rx_buf, len * sizeof(u8));

	return len;
}

static int st_lsm6dsx_spi_write(struct device *device, u8 addr, int len,
				u8 *data)
{
	struct spi_device *spi = to_spi_device(device);
	struct st_lsm6dsx_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= ST_LSM6DSX_TX_MAX_LENGTH)
		return -ENOMEM;

	dev->tb.tx_buf[0] = addr;
	memcpy(&dev->tb.tx_buf[1], data, len);

	return spi_sync_transfer(spi, &xfers, 1);
}

static const struct st_lsm6dsx_transfer_function st_lsm6dsx_transfer_fn = {
	.read = st_lsm6dsx_spi_read,
	.write = st_lsm6dsx_spi_write,
};

static int st_lsm6dsx_spi_probe(struct spi_device *spi)
{
	int err;
	struct st_lsm6dsx_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spi_set_drvdata(spi, dev);
	dev->name = spi->modalias;
	dev->dev = &spi->dev;
	dev->irq = spi->irq;
	dev->tf = &st_lsm6dsx_transfer_fn;

	err = st_lsm6dsx_probe(dev);
	if (err < 0)
		kfree(dev);

	return err;
}

static const struct of_device_id st_lsm6dsx_spi_of_match[] = {
	{
		.compatible = "st,lsm6ds3",
		.data = ST_LSM6DS3_DEV_NAME,
	},
	{
		.compatible = "st,lsm6dsm",
		.data = ST_LSM6DSM_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_lsm6dsx_spi_of_match);

static const struct spi_device_id st_lsm6dsx_spi_id_table[] = {
	{ ST_LSM6DS3_DEV_NAME },
	{ ST_LSM6DSM_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_lsm6dsx_spi_id_table);

static struct spi_driver st_lsm6dsx_driver = {
	.driver = {
		.name = "st_lsm6dsx_spi",
		.of_match_table = of_match_ptr(st_lsm6dsx_spi_of_match),
	},
	.probe = st_lsm6dsx_spi_probe,
	.id_table = st_lsm6dsx_spi_id_table,
};
module_spi_driver(st_lsm6dsx_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsx spi driver");
MODULE_LICENSE("GPL v2");
