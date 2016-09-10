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
#include <linux/irqreturn.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>
#include "st_lsm6dsx.h"

static int st_lsm6dsx_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *iio_dev = iio_trigger_get_drvdata(trig);
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	return st_lsm6dsx_set_drdy_irq(sensor, state);
}

static const struct iio_trigger_ops st_lsm6dsx_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &st_lsm6dsx_trig_set_state,
};

static irqreturn_t st_lsm6dsx_trigger_handler_bh(int irq, void *private)
{
	struct st_lsm6dsx_dev *dev = (struct st_lsm6dsx_dev *)private;

	st_lsm6dsx_get_outdata(dev);

	return IRQ_HANDLED;
}

int st_lsm6dsx_allocate_triggers(struct st_lsm6dsx_dev *dev)
{
	int i, err, count = 0;
	struct st_lsm6dsx_sensor *sensor;

	err = devm_request_threaded_irq(dev->dev, dev->irq, NULL,
					st_lsm6dsx_trigger_handler_bh,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
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
	u8 *ptr;
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_lsm6dsx_sensor *sensor = iio_priv(iio_dev);

	mutex_lock(&sensor->lock);
	while (sensor->rdata.h_rb != sensor->rdata.t_rb) {
		ptr = sensor->rdata.data[sensor->rdata.h_rb].sample;
		iio_push_to_buffers_with_timestamp(iio_dev, ptr,
						   pf->timestamp);
		INCR(sensor->rdata.h_rb, ST_LSM6DSX_RING_SIZE);
	}
	mutex_unlock(&sensor->lock);

	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int st_lsm6dsx_allocate_buffers(struct st_lsm6dsx_dev *dev)
{
	int i, err, count = 0;

	for (i = 0; i < ST_LSM6DSX_ID_MAX; i++) {
		err = iio_triggered_buffer_setup(dev->iio_devs[i],
						 iio_pollfunc_store_time,
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

