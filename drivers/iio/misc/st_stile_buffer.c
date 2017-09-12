/*
 * STMicroelectronics st_stile sensor driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/iio/trigger.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>

#include "st_stile.h"

static const struct iio_trigger_ops st_stile_trigger_ops = {
	.owner = THIS_MODULE,
};

int st_stile_allocate_trigger(struct iio_dev *iio_dev)
{
	struct st_stile_sensor *sensor = iio_priv(iio_dev);
	struct st_stile_hw *hw = sensor->hw;

	sensor->trig = devm_iio_trigger_alloc(hw->dev, "%s-trigger",
					      iio_dev->name);
	if (!sensor->trig)
		return -ENOMEM;

	iio_trigger_set_drvdata(sensor->trig, sensor);
	sensor->trig->ops = &st_stile_trigger_ops;
	sensor->trig->dev.parent = hw->dev;
	iio_dev->trig = iio_trigger_get(sensor->trig);

	return devm_iio_trigger_register(hw->dev, sensor->trig);
}

void st_stile_trigger_handler(struct st_stile_hw *hw, u8 *data)
{
	int i;

	for (i = 0; i < ST_STILE_ID_MAX; i++) {
		struct st_stile_sensor *sensor = iio_priv(hw->iio_devs[i]);

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		memcpy(sensor->data, data + i * ST_STILE_SAMPLE_SIZE,
		       ST_STILE_SAMPLE_SIZE);
		iio_trigger_poll(sensor->trig);
	}
}
EXPORT_SYMBOL(st_stile_trigger_handler);

static int st_stile_set_enable(struct st_stile_sensor *sensor, bool enable)
{
	struct {
		u8 cmd;
		u8 index;
		u8 value;
	} cmd = {
		.cmd = ST_STILE_CMD_ENABLE,
		.index = sensor->id,
		.value = enable,
	};
	struct st_stile_hw *hw = sensor->hw;

	return hw->tf->write(hw->dev, (u8 *)&cmd, sizeof(cmd));
}

static int st_stile_buffer_preenable(struct iio_dev *iio_dev)
{
	struct st_stile_sensor *sensor = iio_priv(iio_dev);
	struct st_stile_hw *hw = sensor->hw;
	bool enabled = hw->enable_mask;
	int err;

	err = st_stile_set_enable(sensor, true);
	if (err < 0)
		return err;

	hw->enable_mask |= BIT(sensor->id);
	return enabled ? 0 : hw->tf->enable(hw->dev, true);
}

static int st_stile_buffer_postdisable(struct iio_dev *iio_dev)
{
	struct st_stile_sensor *sensor = iio_priv(iio_dev);
	struct st_stile_hw *hw = sensor->hw;
	int err;

	err = st_stile_set_enable(sensor, false);
	if (err < 0)
		return err;

	hw->enable_mask &= ~BIT(sensor->id);
	return !hw->enable_mask ? hw->tf->enable(hw->dev, false) : 0;
}

static const struct iio_buffer_setup_ops st_stile_buffer_ops = {
	.preenable = st_stile_buffer_preenable,
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
	.postdisable = st_stile_buffer_postdisable,
};

static irqreturn_t st_stile_buffer_handler_thread(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_stile_sensor *sensor = iio_priv(iio_dev);

	iio_push_to_buffers_with_timestamp(iio_dev, sensor->data,
					   iio_get_time_ns(iio_dev));

	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int st_stile_allocate_buffer(struct iio_dev *iio_dev)
{
	struct st_stile_sensor *sensor = iio_priv(iio_dev);
	struct st_stile_hw *hw = sensor->hw;

	return devm_iio_triggered_buffer_setup(hw->dev, iio_dev, NULL,
					       st_stile_buffer_handler_thread,
					       &st_stile_buffer_ops);
}

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_stile driver");
MODULE_LICENSE("GPL v2");
