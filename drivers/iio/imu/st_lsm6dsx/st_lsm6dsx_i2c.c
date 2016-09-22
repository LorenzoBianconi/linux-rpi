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
#include "st_lsm6dsx.h"

static int st_lsm6dsx_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	return i2c_transfer(client->adapter, msg, 2);
}

static int st_lsm6dsx_i2c_write(struct device *dev, u8 addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len + 1;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static const struct st_lsm6dsx_transfer_function st_lsm6dsx_transfer_fn = {
	.read = st_lsm6dsx_i2c_read,
	.write = st_lsm6dsx_i2c_write,
};

static int st_lsm6dsx_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct st_lsm6dsx_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	i2c_set_clientdata(client, dev);
	dev->name = client->name;
	dev->dev = &client->dev;
	dev->irq = client->irq;
	dev->tf = &st_lsm6dsx_transfer_fn;

	err = st_lsm6dsx_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	dev_info(&client->dev, "sensor probed\n");

	return 0;
}

static int st_lsm6dsx_i2c_remove(struct i2c_client *client)
{
	int err;
	struct st_lsm6dsx_dev *dev = i2c_get_clientdata(client);

	err = st_lsm6dsx_remove(dev);
	if (err < 0)
		return err;

	dev_info(&client->dev, "sensor removed\n");

	return 0;
}

#ifdef CONFIG_OF
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
#endif /* CONFIG_OF */

static const struct i2c_device_id st_lsm6dsx_i2c_id_table[] = {
	{ ST_LSM6DS3_DEV_NAME },
	{ ST_LSM6DSM_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_lsm6dsx_i2c_id_table);

static struct i2c_driver st_lsm6dsx_driver = {
	.driver = {
		.name = "st_lsm6dsx_i2c",
#ifdef CONFIG_OF
		.of_match_table = st_lsm6dsx_i2c_of_match,
#endif /* CONFIG_OF */
	},
	.probe = st_lsm6dsx_i2c_probe,
	.remove = st_lsm6dsx_i2c_remove,
	.id_table = st_lsm6dsx_i2c_id_table,
};
module_i2c_driver(st_lsm6dsx_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsx i2c driver");
MODULE_LICENSE("GPL v2");
