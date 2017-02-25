/*
 * STMicroelectronics st_stail sensor driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_STAIL_H
#define ST_STAIL_H

#include <linux/device.h>
#include <linux/module.h>

struct st_stail_frm {
	u32 start_mark;
	u32 idx;

	u32 acc_data[3];
	u32 gyro_data[3];
	u32 mag_data[3];
	u32 rv_data[4];

	u32 ts_sec;
	u32 ts_nsec;

	u32 stop_mark;
} __packed __aligned(4);

struct st_stail_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

enum st_stail_sensor_id {
	ST_STAIL_ID_ACC,
	ST_STAIL_ID_GYRO,
	ST_STAIL_ID_MAG,
	ST_STAIL_ID_MAX,
};

struct st_stail_sensor {
	enum st_stail_sensor_id id;
	struct st_stail_hw *hw;
};

struct st_stail_hw {
	struct device *dev;

	struct iio_dev *iio_devs[ST_STAIL_ID_MAX];

	const struct st_stail_transfer_function *tf;
};

int st_stail_probe(struct device *dev, void *data,
		   const struct st_stail_transfer_function *ops);

#endif /* ST_STAIL_H */
