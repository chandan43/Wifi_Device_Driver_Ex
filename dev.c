/*
 * Linux device driver for RTL8187
 *
 *
 * Based on the r8187 driver, which is:
 * Copyright 2005 Andrea Merello <andrea.merello@gmail.com>, et al.
 *
 */
#define pr_fmt(fmt) "RTL8187: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>

enum {
	DEVICE_RTL8187,
	DEVICE_RTL8187B
};

static const struct usb_device_id rtl8187_table[] = {
	{USB_DEVICE(0x0bda, 0x8187), .driver_info = DEVICE_RTL8187}, //Added only for Realtek 8187 chip
	{}
};
MODULE_DEVICE_TABLE(usb, rtl8187_table);

static int rtl8187_probe(struct usb_interface *intf,
				   const struct usb_device_id *id)
{
	pr_info("%s: Probed Drver, XXXXXXX \n",__func__); // Update soon
	return 0;	
}

static void rtl8187_disconnect(struct usb_interface *intf)
{
	pr_info("%s: Bye Bye driver \n",__func__); // Update soon
	
}
static struct usb_driver rtl8187_driver = {
	.name 		= KBUILD_MODNAME,
	.id_table	= rtl8187_table,
	.probe		= rtl8187_probe,
	.disconnect	= rtl8187_disconnect,
};

module_usb_driver(rtl8187_driver);
MODULE_LICENSE("GPL");
