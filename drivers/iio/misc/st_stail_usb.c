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

#define ST_STAIL_BUFF_SIZE	128
struct st_stail_usb {
	struct usb_device *udev;
	struct urb *urb;

	u8 buff[ST_STAIL_BUFF_SIZE];
	u8 out_addr;
	u8 in_addr;
};

static int st_stail_usb_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct st_stail_usb *udata = dev_get_drvdata(dev);
	struct usb_device *udev = udata->udev;
	int count;

	return usb_bulk_msg(udev, usb_rcvbulkpipe(udev, udata->in_addr),
			    data, len, &count, 10 * HZ);
}

static int st_stail_usb_write(struct device *dev, u8 addr, int len, u8 *data)
{
	struct st_stail_usb *udata = dev_get_drvdata(dev);
	struct usb_device *udev = udata->udev;
	int count;

	return usb_bulk_msg(udev, usb_sndbulkpipe(udev, udata->out_addr),
			    data, len, &count, 10 * HZ);
}

static const struct st_stail_transfer_function st_stail_usb_tf = {
	.write = st_stail_usb_write,
	.read = st_stail_usb_read,
};

static const struct usb_device_id st_stail_usb_id_table[] = {
	{ USB_DEVICE(0x0483, 0x5740) },
	{}
};
MODULE_DEVICE_TABLE(usb, st_stail_usb_id_table);

static void st_stail_usb_irq(struct urb *urb)
{
}

static int st_stail_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_host_interface *cur_setting = interface->cur_altsetting;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *ep;
	struct st_stail_usb *udata;
	int i;

	udata = devm_kzalloc(&interface->dev, sizeof(*udata), GFP_KERNEL);
	if (!udata)
		return -ENOMEM;

	for (i = 0; i < cur_setting->desc.bNumEndpoints; i++) {
		ep = &cur_setting->endpoint[i].desc;

		if (!udata->in_addr && usb_endpoint_is_bulk_in(ep))
			udata->in_addr = ep->bEndpointAddress;
		else if (!udata->out_addr && usb_endpoint_is_bulk_out(ep))
			udata->out_addr = ep->bEndpointAddress;

		if (udata->in_addr && udata->out_addr)
			break;
	}

	if (!udata->in_addr || !udata->out_addr)
		return -ENODEV;

	/* allocate RX urb */
	udata->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!udata->urb)
		return -ENOMEM;

	usb_fill_bulk_urb(udata->urb, udev,
			  usb_rcvbulkpipe(udev, udata->in_addr),
			  udata->buff, sizeof(udata->buff),
			  st_stail_usb_irq, udata);

	udata->udev = udev;

	return st_stail_probe(&interface->dev, (void *)udata,
			      &st_stail_usb_tf);
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
