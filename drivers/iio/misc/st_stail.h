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

struct st_stail_transfer_function {
	int (*enable)(struct device *dev, bool state);
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

struct st_stail_usb {
	struct usb_device *udev;
	struct urb *urb;
	bool resched;

	unsigned char *buff;
	dma_addr_t dma_buff;

	u8 out_addr;
	u8 in_addr;
};

#define ST_STAIL_BUFF_SIZE	128
struct st_stail_hw {
	struct device *dev;

	struct iio_dev *iio_devs[ST_STAIL_ID_MAX];
	struct iio_trigger *trig;
	u8 data[ST_STAIL_BUFF_SIZE];

	const struct st_stail_transfer_function *tf;
	struct st_stail_usb usb;
};

void st_stail_trigger_handler(struct st_stail_hw *hw, u8 *data);
int st_stail_allocate_buffer(struct iio_dev *iio_dev);
int st_stail_allocate_trigger(struct st_stail_hw *hw);
int st_stail_probe(struct st_stail_hw *hw);

#endif /* ST_STAIL_H */
