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

#define HTS221_AVG_DEPTH	8
struct hts221_avg_avl {
	u16 avg;
	u8 val;
};

struct hts221_sensor {
	u8 cur_avg_idx;
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
	int irq;
	struct iio_trigger *trig;
	struct mutex lock;

	u8 odr;
	s16 buffer[HTS221_SENSOR_MAX];
	struct hts221_sensor sensors[HTS221_SENSOR_MAX];

	const struct hts221_transfer_function *tf;
#if defined(CONFIG_IIO_HTS221_SPI) || \
	defined(CONFIG_IIO_HTS221_SPI_MODULE)
	struct hts221_transfer_buffer tb;
#endif /* CONFIG_IIO_HTS221_SPI */
};

int hts221_config_drdy(struct hts221_dev *dev, bool enable);
int hts221_push_data(struct iio_dev *indio_dev);
int hts221_probe(struct iio_dev *indio_dev);
int hts221_remove(struct iio_dev *indio_dev);
int hts221_power_on(struct hts221_dev *dev);
int hts221_power_off(struct hts221_dev *dev);
#ifdef CONFIG_IIO_BUFFER
int hts221_allocate_buffer(struct iio_dev *indio_dev);
void hts221_deallocate_buffer(struct iio_dev *indio_dev);
int hts221_allocate_trigger(struct iio_dev *indio_dev);
void hts221_deallocate_trigger(struct iio_dev *indio_dev);
#else
static inline int hts221_allocate_buffer(struct iio_dev *indio_dev)
{
	return 0;
}

static inline int hts221_allocate_trigger(struct iio_dev *indio_dev)
{
	return 0;
}
static inline void hts221_deallocate_trigger(struct iio_dev *indio_dev)
{
}
#endif /* CONFIG_IIO_BUFFER */

#endif /* HTS221_H */
