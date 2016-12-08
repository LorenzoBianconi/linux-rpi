/*
 * STMicroelectronics st_lsm6dsx i2c driver
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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "st_lsm6dsx.h"

static int st_lsm6dsx_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
{
	return i2c_smbus_read_i2c_block_data_or_emulated(to_i2c_client(dev),
							 addr, len, data);
}

static int st_lsm6dsx_i2c_write(struct device *dev, u8 addr, int len, u8 *data)
{
	return i2c_smbus_write_i2c_block_data(to_i2c_client(dev), addr,
					      len, data);
}

static const struct st_lsm6dsx_transfer_function st_lsm6dsx_transfer_fn = {
	.read = st_lsm6dsx_i2c_read,
	.write = st_lsm6dsx_i2c_write,
};

static int st_lsm6dsx_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct st_lsm6dsx_hw *hw;

	hw = devm_kzalloc(&client->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	i2c_set_clientdata(client, hw);
	hw->name = client->name;
	hw->dev = &client->dev;
	hw->irq = client->irq;
	hw->tf = &st_lsm6dsx_transfer_fn;

	return st_lsm6dsx_probe(hw);
}

static const struct of_device_id st_lsm6dsx_i2c_of_match[] = {
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
MODULE_DEVICE_TABLE(of, st_lsm6dsx_i2c_of_match);

static const struct i2c_device_id st_lsm6dsx_i2c_id_table[] = {
	{ ST_LSM6DS3_DEV_NAME },
	{ ST_LSM6DSM_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_lsm6dsx_i2c_id_table);

static struct i2c_driver st_lsm6dsx_driver = {
	.driver = {
		.name = "st_lsm6dsx_i2c",
		.of_match_table = of_match_ptr(st_lsm6dsx_i2c_of_match),
	},
	.probe = st_lsm6dsx_i2c_probe,
	.id_table = st_lsm6dsx_i2c_id_table,
};
module_i2c_driver(st_lsm6dsx_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsx i2c driver");
MODULE_LICENSE("GPL v2");
