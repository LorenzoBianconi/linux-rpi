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

#define REG_INT1_ADDR		0x0d
#define REG_DATA_AVL_ADDR	0x1e

static int st_lsm6dsx_trig_set_state(struct iio_trigger *trig, bool state)
{
	u8 val = (state) ? 1 : 0;
	struct iio_dev *iio_dev = iio_trigger_get_drvdata(trig);
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	return st_lsm6dsx_write_with_mask(sensor->dev, REG_INT1_ADDR,
					  sensor->drdy_irq_mask, val);
}

static const struct iio_trigger_ops st_lsm6dsx_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = st_lsm6dsx_trig_set_state,
};

static irqreturn_t st_lsm6dsx_trigger_handler_th(int irq, void *private)
{
	struct st_lsm6dsx_dev *dev = (struct st_lsm6dsx_dev *)private;

	dev->hw_timestamp = iio_get_time_ns();

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lsm6dsx_trigger_handler_bh(int irq, void *private)
{
	int i, err;
	u8 avl_data;
	struct st_lsm6dsx_sensor *sensor;
	struct st_lsm6dsx_dev *dev = (struct st_lsm6dsx_dev *)private;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, REG_DATA_AVL_ADDR, 1, &avl_data);
	if (err < 0)
		goto unlock;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);

		if (avl_data & sensor->drdy_data_mask)
			iio_trigger_poll_chained(sensor->trig);
	}

unlock:
	mutex_unlock(&dev->lock);

	return IRQ_HANDLED;
}

int st_lsm6dsx_allocate_triggers(struct st_lsm6dsx_dev *dev)
{
	int i, err, count = 0;
	struct st_lsm6dsx_sensor *sensor;
	unsigned long irq_type;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(dev->irq));
	err = devm_request_threaded_irq(dev->dev, dev->irq,
					st_lsm6dsx_trigger_handler_th,
					st_lsm6dsx_trigger_handler_bh,
					irq_type | IRQF_ONESHOT,
					dev->name, dev);
	if (err) {
		dev_err(dev->dev, "failed to request trigger irq %d\n",
			dev->irq);
		return err;
	}

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		sensor->trig = iio_trigger_alloc("%s-trigger",
						 dev->iio_devs[i]->name);
		if (!sensor->trig) {
			err = -ENOMEM;
			goto iio_trigger_error;
		}

		iio_trigger_set_drvdata(sensor->trig, dev->iio_devs[i]);
		sensor->trig->ops = &st_lsm6dsx_trigger_ops;
		sensor->trig->dev.parent = dev->dev;

		err = iio_trigger_register(sensor->trig);
		if (err < 0) {
			dev_err(dev->dev, "failed to register iio trigger\n");
			goto iio_trigger_error;
		}
		dev->iio_devs[i]->trig = iio_trigger_get(sensor->trig);

		count++;
	}

	return 0;

iio_trigger_error:
	for (i = count - 1; i >= 0; i--) {
		sensor = iio_priv(dev->iio_devs[i]);
		iio_trigger_unregister(sensor->trig);
	}
	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		if (sensor->trig)
			iio_trigger_free(sensor->trig);
	}

	return err;
}

int st_lsm6dsx_deallocate_triggers(struct st_lsm6dsx_dev *dev)
{
	int i;
	struct st_lsm6dsx_sensor *sensor;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		iio_trigger_unregister(sensor->trig);
		iio_trigger_free(sensor->trig);
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

static irqreturn_t st_lsm6dsx_buffer_handler_bh(int irq, void *p)
{
	int i, err, chan_byte, in = 0, out = 0;
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct iio_chan_spec const *ch = iio_dev->channels;
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsx_dev *dev = sensor->dev;
	u8 out_data[iio_dev->scan_bytes], buffer[ST_LSM6DSX_SAMPLE_SIZE];

	err = dev->tf->read(dev->dev, ch[0].address, ST_LSM6DSX_SAMPLE_SIZE,
			    buffer);
	if (err < 0)
		goto out;

	for (i = 0; i < iio_dev->num_channels; i++) {
		chan_byte = ch[i].scan_type.storagebits >> 3;

		if (test_bit(i, iio_dev->active_scan_mask)) {
			memcpy(&out_data[out], &buffer[in], chan_byte);
			out += chan_byte;
		}
		in += chan_byte;
	}

	iio_push_to_buffers_with_timestamp(iio_dev, out_data,
					   dev->hw_timestamp);

out:
	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int st_lsm6dsx_allocate_buffers(struct st_lsm6dsx_dev *dev)
{
	int i, err, count = 0;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		err = iio_triggered_buffer_setup(dev->iio_devs[i], NULL,
						 st_lsm6dsx_buffer_handler_bh,
						 &st_lsm6dsx_buffer_ops);
		if (err < 0)
			goto iio_buffer_error;
		count++;
	}

	return 0;

iio_buffer_error:
	for (i = count - 1; i >= 0; i--)
		iio_triggered_buffer_cleanup(dev->iio_devs[i]);

	return err;
}

int st_lsm6dsx_deallocate_buffers(struct st_lsm6dsx_dev *dev)
{
	int i;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++)
		iio_triggered_buffer_cleanup(dev->iio_devs[i]);

	return 0;
}

