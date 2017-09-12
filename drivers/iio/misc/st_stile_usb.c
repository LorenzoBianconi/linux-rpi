/*
 * STMicroelectronics st_stile usb driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/usb.h>

#include "st_stile.h"

static int st_stile_usb_read(struct device *dev, u8 *data, int len)
{
	struct st_stile_usb *udata;
	int count, err;

	udata = dev_get_drvdata(dev);
	if (!udata)
		return -EINVAL;

	err = usb_bulk_msg(udata->udev,
			   usb_rcvbulkpipe(udata->udev, udata->in_addr),
			   data, len, &count, 10 * HZ);

	return err < 0 ? err : count;
}

static int st_stile_usb_write(struct device *dev, u8 *data, int len)
{
	struct st_stile_usb *udata;
	int count;

	udata = dev_get_drvdata(dev);
	if (!udata)
		return -EINVAL;

	return usb_bulk_msg(udata->udev,
			    usb_sndbulkpipe(udata->udev, udata->out_addr),
			    data, len, &count, 10 * HZ);
}

static int st_stile_usb_set_enable(struct device *dev, bool state)
{
	struct st_stile_usb *udata = dev_get_drvdata(dev);

	if (!udata)
		return -EINVAL;

	if (state) {
		return usb_submit_urb(udata->urb, GFP_KERNEL);
	} else {
		usb_kill_urb(udata->urb);
		return 0;
	}
}

static const struct st_stile_transfer_function st_stile_usb_tf = {
	.enable = st_stile_usb_set_enable,
	.write = st_stile_usb_write,
	.read = st_stile_usb_read,
};

static const struct usb_device_id st_stile_usb_id_table[] = {
	{ USB_DEVICE(0x0483, 0x5740) },
	{}
};
MODULE_DEVICE_TABLE(usb, st_stile_usb_id_table);

static void st_stile_usb_complete(struct urb *urb)
{
	switch (urb->status) {
	case 0: {
		struct st_stile_usb *udata = urb->context;
		struct st_stile_hw *hw;

		hw = container_of(udata, struct st_stile_hw, usb);
		st_stile_trigger_handler(hw, udata->buff);
	default:
		usb_submit_urb(urb, GFP_ATOMIC);
		break;
	}
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -EPERM:
		break;
	}
}

static int st_stile_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_host_interface *cur_setting = interface->cur_altsetting;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *ep;
	struct st_stile_hw *hw;
	int i, err = 0;

	hw = devm_kzalloc(&interface->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	for (i = 0; i < cur_setting->desc.bNumEndpoints; i++) {
		ep = &cur_setting->endpoint[i].desc;

		if (!hw->usb.in_addr && usb_endpoint_is_bulk_in(ep))
			hw->usb.in_addr = ep->bEndpointAddress;
		else if (!hw->usb.out_addr && usb_endpoint_is_bulk_out(ep))
			hw->usb.out_addr = ep->bEndpointAddress;

		if (hw->usb.in_addr && hw->usb.out_addr)
			break;
	}

	if (!hw->usb.in_addr || !hw->usb.out_addr)
		return -ENODEV;

	usb_set_intfdata(interface, &hw->usb);
	hw->tf = &st_stile_usb_tf;
	hw->dev = &interface->dev;
	hw->usb.udev = udev;

	hw->usb.buff = usb_alloc_coherent(udev, ST_STILE_BUFF_SIZE,
					  GFP_KERNEL, &hw->usb.dma_buff);
	if (!hw->usb.buff)
		return -ENOMEM;

	/* allocate RX urb */
	hw->usb.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!hw->usb.urb)
		goto free_dma_buff;

	usb_fill_bulk_urb(hw->usb.urb, udev,
			  usb_rcvbulkpipe(udev, hw->usb.in_addr),
			  hw->usb.buff, ST_STILE_BUFF_SIZE,
			  st_stile_usb_complete, &hw->usb);
	/* enable DMA transfer */
	hw->usb.urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	hw->usb.urb->transfer_dma = hw->usb.dma_buff;

	err = st_stile_probe(hw);
	if (err < 0)
		goto free_urb;

	return 0;

free_urb:
	usb_free_urb(hw->usb.urb);
free_dma_buff:
	usb_free_coherent(udev, ST_STILE_BUFF_SIZE, hw->usb.buff,
			  hw->usb.dma_buff);

	return err;
}

static void st_stile_usb_disconnect(struct usb_interface *interface)
{
	struct st_stile_usb *udata = usb_get_intfdata(interface);

	usb_kill_urb(udata->urb);
	usb_free_urb(udata->urb);
	usb_free_coherent(udata->udev, ST_STILE_BUFF_SIZE, udata->buff,
			  udata->dma_buff);
}

static struct usb_driver st_stile_usb_driver = {
	.name = "st_stile",
	.id_table = st_stile_usb_id_table,
	.probe = st_stile_usb_probe,
	.disconnect = st_stile_usb_disconnect,
};
module_usb_driver(st_stile_usb_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_stile usb driver");
MODULE_LICENSE("GPL v2");
