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

#define ST_LSM6DSX_REG_FIFO_THL_ADDR	0x06
#define ST_LSM6DSX_REG_FIFO_THH_ADDR	0x07
#define ST_LSM6DSX_FIFO_TH_MASK		0x0fff
#define ST_LSM6DSX_REG_FIFO_DEC_ADDR	0x08
#define ST_LSM6DSX_REG_FIFO_MODE_ADDR	0x0a
#define ST_LSM6DX_REG_DATA_AVL_ADDR	0x1e

#define ST_LSM6DSX_FIFO_MAX_ODR		0x40

static int st_lsm6dsx_set_fifo_mode(struct st_lsm6dsx_hw *hw,
				    enum st_lsm6dsx_fifo_mode fifo_mode)
{
	u8 data;
	int err;

	switch (fifo_mode) {
	case ST_LSM6DSX_FIFO_BYPASS:
		data = fifo_mode;
		break;
	case ST_LSM6DSX_FIFO_CONT:
		data = fifo_mode | ST_LSM6DSX_FIFO_MAX_ODR;
		break;
	default:
		return -EINVAL;
	}

	err = hw->tf->write(hw->dev, ST_LSM6DSX_REG_FIFO_MODE_ADDR,
			    sizeof(data), &data);
	if (err < 0)
		return err;

	hw->fifo_mode = fifo_mode;

	return 0;
}

static int st_lsm6dsx_update_decimators(struct st_lsm6dsx_hw *hw)
{
	int err, i, max_odr = 0, min_odr = ~0;
	struct st_lsm6dsx_sensor *sensor;
	u8 decimator;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);

		if (hw->enable_mask & BIT(sensor->id)) {
			if (sensor->odr > max_odr)
				max_odr = sensor->odr;
			else if (sensor->odr < min_odr)
				min_odr = sensor->odr;
		}
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		/* update fifo decimators */
		decimator = max_odr / sensor->odr;
		err = st_lsm6dsx_write_with_mask(hw,
						 ST_LSM6DSX_REG_FIFO_DEC_ADDR,
						 sensor->decimator_mask,
						 decimator);
		if (err < 0)
			return err;

		/* compute sensor samples in pattern */
		sensor->sip = sensor->odr / min_odr;
	}

	return 0;
}

int st_lsm6dsx_flush_fifo(struct st_lsm6dsx_hw *hw)
{
	/* XXX read the fifo */
	return st_lsm6dsx_set_fifo_mode(hw, ST_LSM6DSX_FIFO_BYPASS);
}

int st_lsm6dsx_update_watermark(struct st_lsm6dsx_sensor *sensor,
				u16 watermark)
{
	u16 fifo_watermark, cur_watermark, sip = 0, min_pattern = ~0;
	struct st_lsm6dsx_hw *hw = sensor->hw;
	struct st_lsm6dsx_sensor *cur_sensor;
	int i, err;
	u8 data;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		cur_sensor = iio_priv(hw->iio_devs[i]);

		cur_watermark = (cur_sensor == sensor) ? watermark
						       : sensor->watermark;
		sip += cur_sensor->sip;
		if (cur_sensor->sip > 0)
			min_pattern = min_t(u16, min_pattern,
					    cur_watermark / cur_sensor->sip);
	}

	if (!sip)
		return -EINVAL;

	min_pattern = min_t(u16, min_pattern, ST_LSM6DSX_MAX_FIFO_TH / sip);
	fifo_watermark = min_pattern * sip * ST_LSM6DSX_SAMPLE_SIZE;

	mutex_lock(&hw->lock);

	err = hw->tf->read(hw->dev, ST_LSM6DSX_REG_FIFO_THH_ADDR,
			   sizeof(data), &data);
	if (err < 0)
		goto out;

	fifo_watermark = (data & ~ST_LSM6DSX_FIFO_TH_MASK) |
			 (fifo_watermark & ST_LSM6DSX_FIFO_TH_MASK);

	err = hw->tf->write(hw->dev, ST_LSM6DSX_REG_FIFO_THL_ADDR,
			   sizeof(fifo_watermark), (u8 *)&fifo_watermark);
out:
	mutex_unlock(&hw->lock);

	return err < 0 ? err : 0;
}

static int st_lsm6dsx_update_fifo(struct st_lsm6dsx_sensor *sensor,
				  bool enable)
{
	struct st_lsm6dsx_hw *hw = sensor->hw;
	int err;

	if (hw->fifo_mode != ST_LSM6DSX_FIFO_BYPASS) {
		err = st_lsm6dsx_flush_fifo(hw);
		if (err < 0)
			return err;
	}

	err = enable ? st_lsm6dsx_sensor_enable(sensor)
		     : st_lsm6dsx_sensor_disable(sensor);
	if (err < 0)
		return err;

	err = st_lsm6dsx_update_decimators(hw);
	if (err < 0)
		return err;

	err = st_lsm6dsx_update_watermark(sensor, sensor->watermark);
	if (err < 0)
		return err;

	if (hw->enable_mask)
		err = st_lsm6dsx_set_fifo_mode(hw, ST_LSM6DSX_FIFO_CONT);

	return err;
}

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
	return st_lsm6dsx_update_fifo(iio_priv(iio_dev), true);
}

static int st_lsm6dsx_buffer_postdisable(struct iio_dev *iio_dev)
{
	return st_lsm6dsx_update_fifo(iio_priv(iio_dev), false);
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

