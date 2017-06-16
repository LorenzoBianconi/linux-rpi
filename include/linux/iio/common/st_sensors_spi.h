/*
 * STMicroelectronics sensors spi library driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_SENSORS_SPI_H
#define ST_SENSORS_SPI_H

#include <linux/spi/spi.h>
#include <linux/iio/common/st_sensors.h>

#define ST_SENSORS_MAX_SIM	8
struct st_sensor_sim {
	char ids[ST_SENSORS_MAX_SIM][ST_SENSORS_MAX_NAME];
	u8 addr;
	u8 val;
};

int st_sensors_spi_configure(struct iio_dev *indio_dev, struct spi_device *spi,
			     struct st_sensor_data *sdata,
			     const struct st_sensor_sim *sensor_sim_list,
			     int sensor_sim_len)

#endif /* ST_SENSORS_SPI_H */
