/*
 * STMicroelectronics hts221 sensor driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/irqreturn.h>

#include "hts221.h"

int hts221_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct hts221_dev *dev = iio_priv(indio_dev);

	return hts221_config_drdy(dev, state);
}

static const struct iio_trigger_ops hts221_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &hts221_trig_set_state,
};

static irqreturn_t hts221_data_rdy_trig_poll(int irq, void *private)
{
	struct hts221_dev *dev = (struct hts221_dev *)private;

	iio_trigger_poll(dev->trig);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hts221_event_handler(int irq, void *private)
{
	return IRQ_HANDLED;
}

int hts221_allocate_trigger(struct iio_dev *indio_dev)
{
	int err;
	struct hts221_dev *dev = iio_priv(indio_dev);

	dev->trig = devm_iio_trigger_alloc(dev->dev, "%s-trigger",
					   indio_dev->name,
					   indio_dev->id);
	if (!dev->trig)
		return -ENOMEM;

	err = devm_request_threaded_irq(dev->dev, dev->irq,
					hts221_data_rdy_trig_poll,
					hts221_event_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev->trig->name, dev);
	if (err) {
		dev_err(dev->dev, "failed to request trigger irq %d\n",
			dev->irq);
		goto request_irq_error;
	}

	iio_trigger_set_drvdata(dev->trig, indio_dev);
	dev->trig->ops = &hts221_trigger_ops;
	dev->trig->dev.parent = dev->dev;

	err = iio_trigger_register(dev->trig);
	if (err < 0) {
		dev_err(dev->dev, "failed to register iio trigger\n");
		goto iio_trigger_register_error;
	}
	indio_dev->trig = iio_trigger_get(dev->trig);

	return 0;

iio_trigger_register_error:
	devm_free_irq(dev->dev, dev->irq, dev->trig);
request_irq_error:
	devm_iio_trigger_free(dev->dev, dev->trig);

	return err;
}

void hts221_deallocate_trigger(struct iio_dev *indio_dev)
{
	struct hts221_dev *dev = iio_priv(indio_dev);

	iio_trigger_unregister(dev->trig);
	devm_free_irq(dev->dev, dev->irq, dev->trig);
	devm_iio_trigger_free(dev->dev, dev->trig);
}

static int hts221_buffer_preenable(struct iio_dev *indio_dev)
{
	struct hts221_dev *dev = iio_priv(indio_dev);

	return hts221_power_on(dev);
}

static int hts221_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct hts221_dev *dev = iio_priv(indio_dev);

	return hts221_power_off(dev);
}

static const struct iio_buffer_setup_ops hts221_buffer_ops = {
	.preenable = hts221_buffer_preenable,
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
	.postdisable = hts221_buffer_postdisable,
};

static irqreturn_t hts221_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct hts221_dev *dev = iio_priv(indio_dev);

	if (!hts221_push_data(indio_dev))
		iio_push_to_buffers_with_timestamp(indio_dev, dev->buffer,
						   pf->timestamp);

	iio_trigger_notify_done(dev->trig);

	return IRQ_HANDLED;
}

int hts221_allocate_buffer(struct iio_dev *indio_dev)
{
	return iio_triggered_buffer_setup(indio_dev, iio_pollfunc_store_time,
					  hts221_trigger_handler,
					  &hts221_buffer_ops);
}

void hts221_deallocate_buffer(struct iio_dev *indio_dev)
{
	iio_triggered_buffer_cleanup(indio_dev);
}

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics hts221 buffer driver");
MODULE_LICENSE("GPL v2");
