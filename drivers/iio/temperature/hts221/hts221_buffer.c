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
#include <linux/interrupt.h>
#include <linux/irqreturn.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>

#include "hts221.h"

int hts221_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct hts221_sensor *sensor = iio_priv(indio_dev);

	return hts221_config_drdy(sensor->dev, state);
}

static const struct iio_trigger_ops hts221_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &hts221_trig_set_state,
};

static irqreturn_t hts221_trigger_handler_bh(int irq, void *private)
{
	struct hts221_dev *dev = (struct hts221_dev *)private;

	if (!hts221_parse_data(dev)) {
		int i;

		for (i = 0; i < HTS221_SENSOR_MAX; i++) {
			struct iio_dev *iio_dev = dev->iio_devs[i];
			struct hts221_sensor *sensor = iio_priv(iio_dev);

			if (iio_dev->active_scan_mask &&
			    test_bit(0, iio_dev->active_scan_mask))
				iio_trigger_poll(sensor->trig);
		}
	}

	return IRQ_HANDLED;
}

int hts221_allocate_triggers(struct hts221_dev *dev)
{
	int i, err, count = 0;
	struct hts221_sensor *sensor;

	err = devm_request_threaded_irq(dev->dev, dev->irq, NULL,
					hts221_trigger_handler_bh,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					dev->name, dev);
	if (err) {
		dev_err(dev->dev, "failed to request trigger irq %d\n",
			dev->irq);
		return err;
	}

	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		sensor->trig = iio_trigger_alloc("%s-trigger",
						 dev->iio_devs[i]->name);
		if (!sensor->trig) {
			err = -ENOMEM;
			goto iio_trigger_error;
		}

		iio_trigger_set_drvdata(sensor->trig, dev->iio_devs[i]);
		sensor->trig->ops = &hts221_trigger_ops;
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
	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		if (sensor->trig)
			iio_trigger_free(sensor->trig);
	}
	devm_free_irq(dev->dev, dev->irq, dev);

	return err;
}

void hts221_deallocate_triggers(struct hts221_dev *dev)
{
	int i;
	struct hts221_sensor *sensor;

	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		sensor = iio_priv(dev->iio_devs[i]);
		iio_trigger_unregister(sensor->trig);
		iio_trigger_free(sensor->trig);
	}

	devm_free_irq(dev->dev, dev->irq, dev);
}

static int hts221_buffer_preenable(struct iio_dev *indio_dev)
{
	return hts221_sensor_power_on(iio_priv(indio_dev));
}

static int hts221_buffer_postdisable(struct iio_dev *indio_dev)
{
	return hts221_sensor_power_off(iio_priv(indio_dev));
}

static const struct iio_buffer_setup_ops hts221_buffer_ops = {
	.preenable = hts221_buffer_preenable,
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
	.postdisable = hts221_buffer_postdisable,
};

static irqreturn_t hts221_buffer_handler_bh(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct hts221_sensor *sensor = iio_priv(indio_dev);

	iio_push_to_buffers_with_timestamp(indio_dev, sensor->buffer,
					   pf->timestamp);
	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int hts221_allocate_buffers(struct hts221_dev *dev)
{
	int i, err, count = 0;

	for (i = 0; i < HTS221_SENSOR_MAX; i++) {
		err = iio_triggered_buffer_setup(dev->iio_devs[i],
						 iio_pollfunc_store_time,
						 hts221_buffer_handler_bh,
						 &hts221_buffer_ops);
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

void hts221_deallocate_buffers(struct hts221_dev *dev)
{
	int i;

	for (i = 0; i < HTS221_SENSOR_MAX; i++)
		iio_triggered_buffer_cleanup(dev->iio_devs[i]);
}

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics hts221 buffer driver");
MODULE_LICENSE("GPL v2");
