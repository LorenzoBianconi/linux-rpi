/*
 * STMicroelectronics st_stile sensor driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_STILE_H
#define ST_STILE_H

#include <linux/device.h>
#include <linux/module.h>

struct st_stile_transfer_function {
	int (*enable)(struct device *dev, bool state);
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

enum st_stile_sensor_id {
	ST_STILE_ID_ACC,
	ST_STILE_ID_GYRO,
	ST_STILE_ID_MAG,
	ST_STILE_ID_MAX,
};

struct st_stile_sensor {
	enum st_stile_sensor_id id;
	struct st_stile_hw *hw;
};

struct st_stile_usb {
	struct usb_device *udev;
	struct urb *urb;
	bool resched;

	unsigned char *buff;
	dma_addr_t dma_buff;

	u8 out_addr;
	u8 in_addr;
};

#define ST_STILE_BUFF_SIZE	128
struct st_stile_hw {
	struct device *dev;

	struct iio_dev *iio_devs[ST_STILE_ID_MAX];
	struct iio_trigger *trig;
	u8 data[ST_STILE_BUFF_SIZE];

	const struct st_stile_transfer_function *tf;
	struct st_stile_usb usb;
};

void st_stile_trigger_handler(struct st_stile_hw *hw, u8 *data);
int st_stile_allocate_buffer(struct iio_dev *iio_dev);
int st_stile_allocate_trigger(struct st_stile_hw *hw);
int st_stile_probe(struct st_stile_hw *hw);

#endif /* ST_STILE_H */
