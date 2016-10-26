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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iio/kfifo_buf.h>

#include "st_lsm6dsx.h"

#define ST_LSM6DX_REG_DATA_AVL_ADDR	0x1e

static irqreturn_t st_lsm6dsx_ring_handler_thread(int irq, void *private)
{
	u8 buffer[ALIGN(ST_LSM6DSX_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	struct st_lsm6dsx_hw *hw = (struct st_lsm6dsx_hw *)private;
	struct st_lsm6dsx_sensor *sensor;
	struct iio_chan_spec const *ch;
	int i, err, count = 0;
	u8 avl_data;

	err = hw->tf->read(hw->dev, ST_LSM6DX_REG_DATA_AVL_ADDR,
			   sizeof(avl_data), &avl_data);
	if (err < 0)
		return IRQ_HANDLED;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);

		if (avl_data & sensor->drdy_data_mask) {
			ch = hw->iio_devs[i]->channels;
			err = hw->tf->read(hw->dev, ch[0].address,
					   ST_LSM6DSX_SAMPLE_SIZE, buffer);
			if (err < 0)
				return IRQ_HANDLED;

			iio_push_to_buffers_with_timestamp(hw->iio_devs[i],
							   buffer,
							   iio_get_time_ns());
			count++;
		}
	}

	return count > 0 ? IRQ_HANDLED : IRQ_NONE;
}

static int st_lsm6dsx_buffer_preenable(struct iio_dev *iio_dev)
{
	return st_lsm6dsx_set_enable(iio_priv(iio_dev), true);
}

static int st_lsm6dsx_buffer_postdisable(struct iio_dev *iio_dev)
{
	return st_lsm6dsx_set_enable(iio_priv(iio_dev), false);
}

static const struct iio_buffer_setup_ops st_lsm6dsx_buffer_ops = {
	.preenable = st_lsm6dsx_buffer_preenable,
	.postdisable = st_lsm6dsx_buffer_postdisable,
};

int st_lsm6dsx_allocate_buffers(struct st_lsm6dsx_hw *hw)
{
	struct iio_buffer *buffer;
	unsigned long irq_type;
	int i, err;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		break;
	default:
		dev_info(hw->dev,
			 "mode %lx unsupported, using IRQF_TRIGGER_HIGH\n",
			 irq_type);
		irq_type = IRQF_TRIGGER_HIGH;
		break;
	}

	err = devm_request_threaded_irq(hw->dev, hw->irq, NULL,
					st_lsm6dsx_ring_handler_thread,
					irq_type | IRQF_ONESHOT,
					hw->name, hw);
	if (err) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return err;
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		buffer = devm_iio_kfifo_allocate(hw->dev);
		if (!buffer)
			return -ENOMEM;

		iio_device_attach_buffer(hw->iio_devs[i], buffer);
		hw->iio_devs[i]->modes |= INDIO_BUFFER_SOFTWARE;
		hw->iio_devs[i]->setup_ops = &st_lsm6dsx_buffer_ops;
	}

	return 0;
}

