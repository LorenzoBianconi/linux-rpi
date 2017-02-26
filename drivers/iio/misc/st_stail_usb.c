/*
 * STMicroelectronics st_stail usb driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/usb.h>

#include "st_stail.h"

static int st_stail_usb_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct st_stail_usb *udata = dev_get_drvdata(dev);
	struct usb_device *udev = udata->udev;
	int count, err;

	err = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, udata->in_addr),
			   data, len, &count, 10 * HZ);

	return err < 0 ? err : count;
}

static int st_stail_usb_write(struct device *dev, u8 addr, int len, u8 *data)
{
	struct st_stail_usb *udata = dev_get_drvdata(dev);
	struct usb_device *udev = udata->udev;
	int count;

	return usb_bulk_msg(udev, usb_sndbulkpipe(udev, udata->out_addr),
			    data, len, &count, 10 * HZ);
}

static int st_stail_usb_set_enable(struct device *dev, bool state)
{
	struct st_stail_usb *udata = dev_get_drvdata(dev);
	int err = 0;

	if (state)
		err = usb_submit_urb(udata->urb, GFP_KERNEL);
	udata->resched = state;

	return err;
}

static const struct st_stail_transfer_function st_stail_usb_tf = {
	.enable = st_stail_usb_set_enable,
	.write = st_stail_usb_write,
	.read = st_stail_usb_read,
};

static const struct usb_device_id st_stail_usb_id_table[] = {
	{ USB_DEVICE(0x0483, 0x5740) },
	{}
};
MODULE_DEVICE_TABLE(usb, st_stail_usb_id_table);

static void st_stail_usb_complete(struct urb *urb)
{
	struct st_stail_usb *udata = urb->context;

	if (!urb->status) {
		struct st_stail_hw *hw;

		hw = container_of(udata, struct st_stail_hw, usb);
		st_stail_trigger_handler(hw, udata->buff);
	}
	
	if (udata->resched)
		usb_submit_urb(urb, GFP_KERNEL);
}

static int st_stail_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_host_interface *cur_setting = interface->cur_altsetting;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *ep;
	struct st_stail_hw *hw;
	int i;

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
	hw->tf = &st_stail_usb_tf;
	hw->dev = &interface->dev;
	hw->usb.udev = udev;

	hw->usb.buff = usb_alloc_coherent(udev, ST_STAIL_BUFF_SIZE,
					  GFP_KERNEL, &hw->usb.dma_buff);
	if (!hw->usb.buff)
		return -ENOMEM;

	/* allocate RX urb */
	hw->usb.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!hw->usb.urb)
		return -ENOMEM;

	usb_fill_bulk_urb(hw->usb.urb, udev,
			  usb_rcvbulkpipe(udev, hw->usb.in_addr),
			  hw->usb.buff, ST_STAIL_BUFF_SIZE,
			  st_stail_usb_complete, &hw->usb);
	/* enable DMA transfer */
	hw->usb.urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	hw->usb.urb->transfer_dma = hw->usb.dma_buff;

	return st_stail_probe(hw);
}

static void st_stail_usb_disconnect(struct usb_interface *interface)
{
	struct st_stail_usb *udata = usb_get_intfdata(interface);

	usb_kill_urb(udata->urb);
	usb_free_urb(udata->urb);
}

static struct usb_driver st_stail_usb_driver = {
	.name = "st_stail",
	.id_table = st_stail_usb_id_table,
	.probe = st_stail_usb_probe,
	.disconnect = st_stail_usb_disconnect,
};
module_usb_driver(st_stail_usb_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_stail usb driver");
MODULE_LICENSE("GPL v2");
