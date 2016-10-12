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
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irqreturn.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>

#include "st_lsm6dsx.h"

#define ST_LSM6DX_REG_INT1_ADDR		0x0d
#define ST_LSM6DX_REG_DATA_AVL_ADDR	0x1e

static int st_lsm6dsx_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *iio_dev = iio_trigger_get_drvdata(trig);
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);
	u8 val = !!state;

	return st_lsm6dsx_write_with_mask(sensor->hw, ST_LSM6DX_REG_INT1_ADDR,
					  sensor->drdy_irq_mask, val);
}

static int st_lsm6dsx_validate_device(struct iio_trigger *trig,
				      struct iio_dev *iio_dev)
{
	struct iio_dev *indio = iio_trigger_get_drvdata(trig);

	return indio == iio_dev ? 0 : -EINVAL;
}

static const struct iio_trigger_ops st_lsm6dsx_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = st_lsm6dsx_trig_set_state,
	.validate_device = st_lsm6dsx_validate_device,
};

static irqreturn_t st_lsm6dsx_trigger_handler_thread(int irq, void *private)
{
	struct st_lsm6dsx_hw *hw = (struct st_lsm6dsx_hw *)private;
	struct st_lsm6dsx_sensor *sensor;
	int i, err, count = 0;
	u8 avl_data;

	err = hw->tf->read(hw->dev, ST_LSM6DX_REG_DATA_AVL_ADDR,
			   sizeof(avl_data), &avl_data);
	if (err < 0)
		return IRQ_HANDLED;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);

		if (avl_data & sensor->drdy_data_mask) {
			iio_trigger_poll_chained(sensor->trig);
			count++;
		}
	}

	return count > 0 ? IRQ_HANDLED : IRQ_NONE;
}

int st_lsm6dsx_allocate_triggers(struct st_lsm6dsx_hw *hw)
{
	int i, err;
	struct st_lsm6dsx_sensor *sensor;
	unsigned long irq_type;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_LOW:
	case IRQF_TRIGGER_RISING:
		break;
	default:
		dev_info(hw->dev,
			 "mode %lx unsupported, using IRQF_TRIGGER_RISING\n",
			 irq_type);
		irq_type = IRQF_TRIGGER_RISING;
		break;
	}

	err = devm_request_threaded_irq(hw->dev, hw->irq, NULL,
					st_lsm6dsx_trigger_handler_thread,
					irq_type | IRQF_ONESHOT,
					hw->name, hw);
	if (err) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return err;
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);
		sensor->trig = devm_iio_trigger_alloc(hw->dev, "%s-trigger",
						      hw->iio_devs[i]->name);
		if (!sensor->trig)
			return -ENOMEM;

		iio_trigger_set_drvdata(sensor->trig, hw->iio_devs[i]);
		sensor->trig->ops = &st_lsm6dsx_trigger_ops;
		sensor->trig->dev.parent = hw->dev;

		err = devm_iio_trigger_register(hw->dev, sensor->trig);
		if (err < 0) {
			dev_err(hw->dev, "failed to register iio trigger\n");
			return err;
		}
		hw->iio_devs[i]->trig = iio_trigger_get(sensor->trig);
	}

	return 0;
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
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
	.postdisable = st_lsm6dsx_buffer_postdisable,
};

static irqreturn_t st_lsm6dsx_buffer_handler_thread(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct iio_chan_spec const *ch = iio_dev->channels;
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsx_hw *hw = sensor->hw;
	u8 buffer[ALIGN(ST_LSM6DSX_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	int err;

	err = hw->tf->read(hw->dev, ch[0].address, ST_LSM6DSX_SAMPLE_SIZE,
			   buffer);
	if (err < 0)
		goto out;

	iio_push_to_buffers_with_timestamp(iio_dev, buffer, iio_get_time_ns());

out:
	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int st_lsm6dsx_allocate_buffers(struct st_lsm6dsx_hw *hw)
{
	int i, err;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		err = devm_iio_triggered_buffer_setup(hw->dev,
						hw->iio_devs[i], NULL,
						st_lsm6dsx_buffer_handler_thread,
						&st_lsm6dsx_buffer_ops);
		if (err < 0)
			return err;
	}

	return 0;
}

