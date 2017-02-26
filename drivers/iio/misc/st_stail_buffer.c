/*
 * STMicroelectronics st_stail sensor driver
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

#include "st_stail.h"

#define ST_STAIL_SAMPLE_SIZE	12

static const struct iio_trigger_ops st_stail_trigger_ops = {
	.owner = THIS_MODULE,
};

int st_stail_allocate_trigger(struct st_stail_hw *hw)
{
	int i;

	hw->trig = devm_iio_trigger_alloc(hw->dev, "stail-trigger");
	if (!hw->trig)
		return -ENOMEM;

	iio_trigger_set_drvdata(hw->trig, hw);
	hw->trig->ops = &st_stail_trigger_ops;
	hw->trig->dev.parent = hw->dev;
	for (i = 0; i < ST_STAIL_ID_MAX; i++)
		hw->iio_devs[i]->trig = iio_trigger_get(hw->trig);

	return devm_iio_trigger_register(hw->dev, hw->trig);
}

void st_stail_trigger_handler(struct st_stail_hw *hw, u8 *data)
{
	memcpy(&hw->data, data, sizeof(hw->data));
	iio_trigger_poll(hw->trig);
}
EXPORT_SYMBOL(st_stail_trigger_handler);

static int st_stail_buffer_preenable(struct iio_dev *iio_dev)
{
	struct st_stail_sensor *sensor = iio_priv(iio_dev);
	struct st_stail_hw *hw = sensor->hw;
	
	return hw->tf->enable(hw->dev, true);
}

static int st_stail_buffer_postdisable(struct iio_dev *iio_dev)
{
	struct st_stail_sensor *sensor = iio_priv(iio_dev);
	struct st_stail_hw *hw = sensor->hw;
	
	return hw->tf->enable(hw->dev, false);
}

static const struct iio_buffer_setup_ops st_stail_buffer_ops = {
	.preenable = st_stail_buffer_preenable,
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
	.postdisable = st_stail_buffer_postdisable,
};

static irqreturn_t st_stail_buffer_handler_thread(int irq, void *p)
{
	u8 buff[ALIGN(ST_STAIL_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_stail_sensor *sensor = iio_priv(iio_dev);
	struct st_stail_hw *hw = sensor->hw;
	int offset = -1;

	switch (sensor->id) {
	case ST_STAIL_ID_ACC:
		offset = 8;
		break;
	case ST_STAIL_ID_GYRO:
		offset = 20;
		break;
	case ST_STAIL_ID_MAG:
		offset = 32;
		break;
	default:
		break;
	}

	if (offset >= 0) {
		memcpy(buff, hw->data + offset, ST_STAIL_SAMPLE_SIZE);
		iio_push_to_buffers_with_timestamp(iio_dev, buff,
						   iio_get_time_ns());
	}

out:
	iio_trigger_notify_done(hw->trig);

	return IRQ_HANDLED;
}

int st_stail_allocate_buffer(struct iio_dev *iio_dev)
{
	struct st_stail_sensor *sensor = iio_priv(iio_dev);
	struct st_stail_hw *hw = sensor->hw;

	return devm_iio_triggered_buffer_setup(hw->dev, iio_dev, NULL,
					       st_stail_buffer_handler_thread,
					       &st_stail_buffer_ops);
}

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_stail driver");
MODULE_LICENSE("GPL v2");
