/*
 * STMicroelectronics hts221 sensor driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef HTS221_H
#define HTS221_H

#define HTS221_DEV_NAME		"hts221"

#include <linux/iio/iio.h>

#if defined(CONFIG_IIO_HTS221_SPI) || \
    defined(CONFIG_IIO_HTS221_SPI_MODULE)
#define HTS221_RX_MAX_LENGTH	500
#define HTS221_TX_MAX_LENGTH	500

struct hts221_transfer_buffer {
	u8 rx_buf[HTS221_RX_MAX_LENGTH];
	u8 tx_buf[HTS221_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_IIO_HTS221_SPI */

struct hts221_transfer_function {
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
};

#define HTS221_RES_DEPTH	8
struct hts221_res_avl {
	u16 res;
	u8 val;
};

struct hts221_res {
	u8 addr;
	u8 mask;
	struct hts221_res_avl res_avl[HTS221_RES_DEPTH];
};

struct hts221_settings {
	struct hts221_res res;
};

struct hts221_sensor {
	u8 cur_res_idx;
	int slope, b_gen;
};

enum hts221_sensor_type {
	HTS221_SENSOR_H,
	HTS221_SENSOR_T,
	HTS221_SENSOR_MAX,
};

struct hts221_dev {
	const char *name;
	struct device *dev;

	struct mutex lock;

	u8 odr;
	struct hts221_sensor sensors[HTS221_SENSOR_MAX];

	const struct hts221_transfer_function *tf;
#if defined(CONFIG_IIO_HTS221_SPI) || \
    defined(CONFIG_IIO_HTS221_SPI_MODULE)
	struct hts221_transfer_buffer tb;
#endif /* CONFIG_IIO_HTS221_SPI */
};

/**
 * hts221_convert - linear interpolation
 * (x0,y0) (x1,y1) y = mx + b
 *
 * m = (y1 - y0) / (x1 - x0)
 * b = (x0 * y1 - x1 * y0) / (x1 - x0)
 *
 * Humidity
 * {x1,y1} = {H1_T0_OUT,H1_RH}
 * {x0,y0} = {H0_T0_OUT,H0_RH}
 * x = H_OUT
 *
 * Temperature
 * {x1,y1} = {T1_OUT,T1_DegC}
 * {x0,y0} = {T0_OUT,T0_DegC}
 * x = T_OUT
 */
static inline int hts221_convert(int slope, int b_gen, s16 x,
				 enum hts221_sensor_type type)
{
	int res = (slope * x) + b_gen;

	switch (type) {
	case HTS221_SENSOR_H:
		res >>= 4;
		break;
	case HTS221_SENSOR_T:
		res >>= 6;
		break;
	default:
		break;
	}

	return res;
}

int hts221_probe(struct iio_dev *indio_dev);
int hts221_remove(struct iio_dev *indio_dev);

#endif /* HTS221_H */
