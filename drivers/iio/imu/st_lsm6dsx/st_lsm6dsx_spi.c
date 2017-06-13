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

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_lsm6dsx.h"

#define ST_LSM6DSX_REG_SIM_ADDR		0x12
#define ST_LSM6DSX_REG_SIM_MASK		BIT(3)
#define ST_LSM6DSX_REG_IFINC_MASK	BIT(2)

#define SENSORS_SPI_READ		BIT(7)

static int st_lsm6dsx_spi_read(struct device *dev, u8 addr, int len,
			       u8 *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct st_lsm6dsx_hw *hw = spi_get_drvdata(spi);
	int err;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = hw->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = hw->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	hw->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	err = spi_sync_transfer(spi, xfers,  ARRAY_SIZE(xfers));
	if (err < 0)
		return err;

	memcpy(data, hw->tb.rx_buf, len * sizeof(u8));

	return len;
}

static int st_lsm6dsx_spi_write(struct device *dev, u8 addr, int len,
				u8 *data)
{
	struct st_lsm6dsx_hw *hw;
	struct spi_device *spi;

	if (len >= ST_LSM6DSX_TX_MAX_LENGTH)
		return -ENOMEM;

	spi = to_spi_device(dev);
	hw = spi_get_drvdata(spi);

	hw->tb.tx_buf[0] = addr;
	memcpy(&hw->tb.tx_buf[1], data, len);

	return spi_write(spi, hw->tb.tx_buf, len + 1);
}

static const struct st_lsm6dsx_transfer_function st_lsm6dsx_transfer_fn = {
	.read = st_lsm6dsx_spi_read,
	.write = st_lsm6dsx_spi_write,
};

static int st_lsm6dsx_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct device_node *np = spi->dev.of_node;
	struct st_sensors_platform_data *pdata;

	/*
	 * Enable spi-3wire if device requires 3-wire mode. Enable IF_INC
	 * since it will be overwritten by spi configuration
	 */
	pdata = (struct st_sensors_platform_data *)spi->dev.platform_data;
	if ((np && of_find_property(np, "spi-3wire", NULL)) ||
	    (pdata && pdata->spi_3wire)) {
		u8 data = ST_LSM6DSX_REG_SIM_MASK | ST_LSM6DSX_REG_IFINC_MASK;
		int err;

		err = st_lsm6dsx_spi_write(&spi->dev, ST_LSM6DSX_REG_SIM_ADDR,
					   sizeof(data), &data);
		if (err < 0)
			return err;
	}

	return st_lsm6dsx_probe(&spi->dev, spi->irq,
				(int)id->driver_data, id->name,
				&st_lsm6dsx_transfer_fn);
}

static const struct of_device_id st_lsm6dsx_spi_of_match[] = {
	{
		.compatible = "st,lsm6ds3",
		.data = (void *)ST_LSM6DS3_ID,
	},
	{
		.compatible = "st,lsm6ds3h",
		.data = (void *)ST_LSM6DS3H_ID,
	},
	{
		.compatible = "st,lsm6dsl",
		.data = (void *)ST_LSM6DSL_ID,
	},
	{
		.compatible = "st,lsm6dsm",
		.data = (void *)ST_LSM6DSM_ID,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_lsm6dsx_spi_of_match);

static const struct spi_device_id st_lsm6dsx_spi_id_table[] = {
	{ ST_LSM6DS3_DEV_NAME, ST_LSM6DS3_ID },
	{ ST_LSM6DS3H_DEV_NAME, ST_LSM6DS3H_ID },
	{ ST_LSM6DSL_DEV_NAME, ST_LSM6DSL_ID },
	{ ST_LSM6DSM_DEV_NAME, ST_LSM6DSM_ID },
	{},
};
MODULE_DEVICE_TABLE(spi, st_lsm6dsx_spi_id_table);

static struct spi_driver st_lsm6dsx_driver = {
	.driver = {
		.name = "st_lsm6dsx_spi",
		.pm = &st_lsm6dsx_pm_ops,
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
