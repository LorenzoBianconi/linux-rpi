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

enum {
	ST_STILE_CMD_ENABLE = 1,
	ST_STILE_CMD_SET_ODR = 2,
};

struct st_stile_transfer_function {
	int (*enable)(struct device *dev, bool state);
	int (*write)(struct device *dev, u8 *data, int len);
	int (*read)(struct device *dev, u8 *data, int len);
};

enum st_stile_sensor_id {
	ST_STILE_ID_ACC,
	ST_STILE_ID_GYRO,
	ST_STILE_ID_MAG,
	ST_STILE_ID_MAX,
};

#define ST_STILE_SAMPLE_SIZE	12
#define ST_STILE_BUFF_SIZE	128

struct st_stile_sensor {
	enum st_stile_sensor_id id;
	struct iio_trigger *trig;
	struct st_stile_hw *hw;
	u16 odr;

	u8 data[ALIGN(ST_STILE_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
};

struct st_stile_usb {
	struct usb_device *udev;
	struct urb *urb;

	unsigned char *buff;
	dma_addr_t dma_buff;

	u8 out_addr;
	u8 in_addr;
};

struct st_stile_hw {
	struct device *dev;

	struct iio_dev *iio_devs[ST_STILE_ID_MAX];
	u8 enable_mask;

	const struct st_stile_transfer_function *tf;
	struct st_stile_usb usb;
};

void st_stile_trigger_handler(struct st_stile_hw *hw, u8 *data);
int st_stile_allocate_buffer(struct iio_dev *iio_dev);
int st_stile_allocate_trigger(struct iio_dev *iio_dev);
int st_stile_probe(struct st_stile_hw *hw);

#endif /* ST_STILE_H */
