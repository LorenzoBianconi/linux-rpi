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

#ifndef ST_LSM6DSX_H
#define ST_LSM6DSX_H

#include <linux/device.h>

#define ST_LSM6DS3_DEV_NAME	"lsm6ds3"
#define ST_LSM6DSM_DEV_NAME	"lsm6dsm"

#define ST_LSM6DSX_SAMPLE_SIZE	6

#if defined(CONFIG_IIO_ST_LSM6DSX_SPI) || \
	defined(CONFIG_IIO_ST_LSM6DSX_SPI_MODULE)
#define ST_LSM6DSX_RX_MAX_LENGTH	8
#define ST_LSM6DSX_TX_MAX_LENGTH	8

struct st_lsm6dsx_transfer_buffer {
	u8 rx_buf[ST_LSM6DSX_RX_MAX_LENGTH];
	u8 tx_buf[ST_LSM6DSX_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_IIO_ST_LSM6DSX_SPI */

struct st_lsm6dsx_transfer_function {
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
};

enum st_lsm6dsx_sensor_id {
	ST_LSM6DSX_ID_ACC,
	ST_LSM6DSX_ID_GYRO,
	ST_LSM6DSX_ID_MAX,
};

struct st_lsm6dsx_sensor {
	enum st_lsm6dsx_sensor_id id;
	struct st_lsm6dsx_dev *dev;
	struct iio_trigger *trig;

	u16 odr;
	u32 gain;

	u8 drdy_data_mask;
	u8 drdy_irq_mask;
};

struct st_lsm6dsx_dev {
	const char *name;
	struct device *dev;
	int irq;
	struct mutex lock;

	struct iio_dev *iio_devs[ST_LSM6DSX_ID_MAX];

	const struct st_lsm6dsx_transfer_function *tf;
#if defined(CONFIG_IIO_ST_LSM6DSX_SPI) || \
	defined(CONFIG_IIO_ST_LSM6DSX_SPI_MODULE)
	struct st_lsm6dsx_transfer_buffer tb;
#endif /* CONFIG_IIO_ST_LSM6DSX_SPI */
};

int st_lsm6dsx_probe(struct st_lsm6dsx_dev *dev);
int st_lsm6dsx_set_enable(struct st_lsm6dsx_sensor *sensor, bool enable);
int st_lsm6dsx_write_with_mask(struct st_lsm6dsx_dev *dev, u8 addr, u8 mask,
			       u8 val);
int st_lsm6dsx_allocate_triggers(struct st_lsm6dsx_dev *dev);
int st_lsm6dsx_allocate_buffers(struct st_lsm6dsx_dev *dev);

#endif /* ST_LSM6DSX_H */

