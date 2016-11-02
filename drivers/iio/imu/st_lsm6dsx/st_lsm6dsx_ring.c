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
#include <asm/unaligned.h>

#include "st_lsm6dsx.h"

#define ST_LSM6DSX_REG_FIFO_THL_ADDR		0x06
#define ST_LSM6DSX_REG_FIFO_THH_ADDR		0x07
#define ST_LSM6DSX_FIFO_TH_MASK			0x0fff
#define ST_LSM6DSX_REG_FIFO_DEC_ADDR		0x08
#define ST_LSM6DSX_REG_FIFO_MODE_ADDR		0x0a
#define ST_LSM6DSX_REG_FIFO_DIFFL_ADDR		0x3a
#define ST_LSM6DSX_FIFO_DIFF_MASK		0x0f
#define ST_LSM6DSX_FIFO_EMPTY_MASK		0x10
#define ST_LSM6DSX_REG_DATA_AVL_ADDR		0x1e
#define ST_LSM6DSX_REG_FIFO_OUTL_ADDR		0x3e

#define ST_LSM6DSX_FIFO_MAX_ODR			0xa0

struct st_lsm6dsx_dec_entry {
	u8 decimator;
	u8 val;
};

static const struct st_lsm6dsx_dec_entry st_lsm6dsx_dec_table[] = {
	{  0, 0x0 },
	{  1, 0x1 },
	{  2, 0x2 },
	{  3, 0x3 },
	{  4, 0x4 },
	{  8, 0x5 },
	{ 16, 0x6 },
	{ 32, 0x7 },
};

static int st_lsm6dsx_get_decimator(u8 val)
{
	int i, max_size = ARRAY_SIZE(st_lsm6dsx_dec_table);

	for (i = 0; i < max_size; i++)
		if (st_lsm6dsx_dec_table[i].decimator == val)
			break;

	return i == max_size ? 0 : st_lsm6dsx_dec_table[i].val;
}

static int st_lsm6dsx_update_decimators(struct st_lsm6dsx_hw *hw)
{
	u32 sip = 0, max_odr = 0, min_odr = ~0;
	struct st_lsm6dsx_sensor *sensor;
	int err, i;
	u8 data;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		if (sensor->odr > max_odr)
			max_odr = sensor->odr;
		if (sensor->odr < min_odr)
			min_odr = sensor->odr;
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);

		/* update fifo decimators and sample in pattern */
		if (hw->enable_mask & BIT(sensor->id)) {
			u8 decimator;

			decimator = max_odr / sensor->odr;
			data = st_lsm6dsx_get_decimator(decimator);

			sensor->sip = sensor->odr / min_odr;
			sip += sensor->sip;
		} else {
			data = 0;
			sensor->sip = 0;
		}

		err = st_lsm6dsx_write_with_mask(hw,
						 ST_LSM6DSX_REG_FIFO_DEC_ADDR,
						 sensor->decimator_mask, data);
		if (err < 0)
			return err;
	}
	hw->sip = sip;

	return 0;
}

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


int st_lsm6dsx_update_watermark(struct st_lsm6dsx_sensor *sensor,
				u16 watermark)
{
	u16 fifo_watermark, sip = 0, cur_watermark;
	u16 cur_pattern, min_pattern = ~0;
	struct st_lsm6dsx_hw *hw = sensor->hw;
	struct st_lsm6dsx_sensor *cur_sensor;
	int i, err;
	u8 data;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		cur_sensor = iio_priv(hw->iio_devs[i]);

		if (!cur_sensor->sip)
			continue;

		sip += cur_sensor->sip;
		cur_watermark = (cur_sensor == sensor) ? watermark
						       : sensor->watermark;
		cur_pattern = cur_watermark / cur_sensor->sip;
		if (!cur_pattern)
			cur_pattern = 1;
		min_pattern = min_t(u16, cur_pattern, min_pattern);
	}

	fifo_watermark = min_pattern * sip * ST_LSM6DSX_SAMPLE_SIZE;

	mutex_lock(&hw->lock);

	err = hw->tf->read(hw->dev, ST_LSM6DSX_REG_FIFO_THH_ADDR,
			   sizeof(data), &data);
	if (err < 0)
		goto out;

	fifo_watermark = ((data & ~ST_LSM6DSX_FIFO_TH_MASK) << 8) |
			 (fifo_watermark & ST_LSM6DSX_FIFO_TH_MASK);

	err = hw->tf->write(hw->dev, ST_LSM6DSX_REG_FIFO_THL_ADDR,
			    sizeof(fifo_watermark), (u8 *)&fifo_watermark);
out:
	mutex_unlock(&hw->lock);

	return err < 0 ? err : 0;
}

static int st_lsm6dsx_read_fifo(struct st_lsm6dsx_hw *hw)
{
	u8 buffer[ALIGN(ST_LSM6DSX_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	struct st_lsm6dsx_sensor *acc_sensor, *gyro_sensor;
	int err, i = 0, acc_sip, gyro_sip;
	u16 fifo_depth, fifo_len;
	u8 fifo_status[2];

	if (WARN_ON_ONCE(!hw->sip))
		return 0;

	err = hw->tf->read(hw->dev, ST_LSM6DSX_REG_FIFO_DIFFL_ADDR,
			   sizeof(fifo_status), fifo_status);
	if (err < 0)
		return err;

	if (fifo_status[1] & ST_LSM6DSX_FIFO_EMPTY_MASK)
		return 0;

	/* leave last sample in the FIFO to guarantee proper alignment */
	fifo_status[1] &= ST_LSM6DSX_FIFO_DIFF_MASK;
	fifo_depth = (u16)get_unaligned_le16(fifo_status);

	fifo_len = (fifo_depth / hw->sip) * hw->sip;

	acc_sensor = iio_priv(hw->iio_devs[ST_LSM6DSX_ID_ACC]);
	gyro_sensor = iio_priv(hw->iio_devs[ST_LSM6DSX_ID_GYRO]);

	while (i < fifo_len) {
		acc_sip = acc_sensor->sip;
		gyro_sip = gyro_sensor->sip;

		while (acc_sip > 0 || gyro_sip > 0) {
			if (gyro_sip > 0) {
				err =  hw->tf->read(hw->dev,
						ST_LSM6DSX_REG_FIFO_OUTL_ADDR,
						ST_LSM6DSX_SAMPLE_SIZE,
						buffer);
				if (err < 0)
					return err;

				iio_push_to_buffers_with_timestamp(
						hw->iio_devs[ST_LSM6DSX_ID_GYRO],
						buffer, iio_get_time_ns());
				gyro_sip--;
				i += ST_LSM6DSX_SAMPLE_SIZE;
			}

			if (acc_sip > 0) {
				err =  hw->tf->read(hw->dev,
						ST_LSM6DSX_REG_FIFO_OUTL_ADDR,
						ST_LSM6DSX_SAMPLE_SIZE,
						buffer);
				if (err < 0)
					return err;

				iio_push_to_buffers_with_timestamp(
						hw->iio_devs[ST_LSM6DSX_ID_ACC],
						buffer, iio_get_time_ns());
				acc_sip--;
				i += ST_LSM6DSX_SAMPLE_SIZE;
			}
		}
	}

	return fifo_len;
}

static int st_lsm6dsx_flush_fifo(struct st_lsm6dsx_hw *hw)
{
	int err;

	disable_irq(hw->irq);

	st_lsm6dsx_read_fifo(hw);
	err = st_lsm6dsx_set_fifo_mode(hw, ST_LSM6DSX_FIFO_BYPASS);

	enable_irq(hw->irq);

	return err;
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
	struct st_lsm6dsx_hw *hw = (struct st_lsm6dsx_hw *)private;
	int count;

	count = st_lsm6dsx_read_fifo(hw);

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

