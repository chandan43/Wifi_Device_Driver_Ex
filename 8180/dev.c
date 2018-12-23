/*
 * Linux device driver for RTL8180
 *
 *
 * Based on the r8180 driver, which is:
 * Copyright 2005 Andrea Merello <andrea.merello@gmail.com>, et al.
 *
 */
#define pr_fmt(fmt) "RTL8180: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>


static const struct pci_device_id rtl8180_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8199) },
	{}
};
MODULE_DEVICE_TABLE(pci, rtl8180_table);

static int rtl8180_probe(struct pci_dev *pdev,
				   const struct pci_device_id *id)
{
	pr_info("%s: Probed Drver, XXXXXXX \n",__func__); // Update soon
	return 0;	
}

static void rtl8180_remove(struct pci_dev *pdev)
{
	pr_info("%s: Bye Bye driver \n",__func__); // Update soon
	
}
static struct pci_driver rtl8180_driver = {
	.name 		= KBUILD_MODNAME,
	.id_table	= rtl8180_table,
	.probe		= rtl8180_probe,
	.remove	       = rtl8180_remove,
};

module_pci_driver(rtl8180_driver);
MODULE_LICENSE("GPL");
