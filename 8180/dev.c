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
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/eeprom_93cx6.h>
#include <net/mac80211.h>

#include "rtl8180.h"


static const struct pci_device_id rtl8180_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8199) },
	/* rtl8180 */
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8180) },
	{}
};
MODULE_DEVICE_TABLE(pci, rtl8180_table);
/**
 * struct ieee80211_hw - hardware information and state
 *
 * This structure contains the configuration and hardware
 * information for an 802.11 PHY.
 */
/**
 * ieee80211_alloc_hw -  Allocate a new hardware device
 *
 * This must be called once for each hardware device. The returned pointer
 * must be used to refer to this device when calling other functions.
 * mac80211 allocates a private data area for the driver pointed to by
 * @priv in &struct ieee80211_hw, the size of this area is given as
 * @priv_data_len.
 *
 * @priv_data_len: length of private data
 * @ops: callbacks for this device
 *
 * Return: A pointer to the new hardware device, or %NULL on error.
 */
/*
 * @max_rates: maximum number of alternate rate retry stages the hw
 *	can handle.
 */	
/**
 * SET_IEEE80211_DEV - set device for 802.11 hardware
 *
 * @hw: the &struct ieee80211_hw to set the device for
 * @dev: the &struct device of this 802.11 device
 */
/*  @max_signal: Maximum value for signal (rssi) in RX information, used
 *	only when @IEEE80211_HW_SIGNAL_UNSPEC or @IEEE80211_HW_SIGNAL_DB
 */

/**
 * wiphy_ext_feature_set - set the extended feature flag
 *
 * @wiphy: the wiphy to modify.
 * @ftidx: extended feature bit index.
 *
 * The extended features are flagged in multiple bytes (see
 * &struct wiphy.@ext_features)
 */
/**
 * is_valid_ether_addr - Determine if the given Ethernet address is valid
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Check that the Ethernet address (MAC) is not 00:00:00:00:00:00, is not
 * a multicast address, and is not FF:FF:FF:FF:FF:FF.
 *
 * Return true if the address is valid.
 */
/**
 * eth_random_addr - Generate software assigned random Ethernet address
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Generate a random Ethernet address (MAC) that is not multicast
 * and has the local assigned bit set.
 */
static int rtl8180_probe(struct pci_dev *pdev,
				   const struct pci_device_id *id)
{
	pr_info("%s: Probed Drver, XXXXXXX \n",__func__); // Update soon
	struct ieee80211_hw *dev;
	struct rtl8180_priv *priv; /*Private Structre*/
	unsigned long mem_addr, mem_len; 
	unsigned int io_addr, io_len;
	int err;
	const char *chip_name, *rf_name = NULL;
	u32 reg;
	err = pci_enable_device(pdev);
	if (err) {
		pr_err("%s (rtl8180): Cannot enable new PCI device\n",
		       pci_name(pdev));
		return err;
	}
	/*Request memory region*/
	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		pr_err("%s (rtl8180): Cannot obtain PCI resources\n",
		       pci_name(pdev));
		goto err_disable_dev;
	}
	/*PCI Configuration: page 48*/
	io_addr = pci_resource_start(pdev, 0);
	io_len = pci_resource_len(pdev, 0);
	mem_addr = pci_resource_start(pdev, 1);
	mem_len = pci_resource_len(pdev, 1);
	
        if (mem_len < sizeof(struct rtl818x_csr) ||
	    io_len < sizeof(struct rtl818x_csr)) {
		pr_err("%s (rtl8180): Too short PCI resources\n",
		       pci_name(pdev));
		err = -ENOMEM;
		goto err_free_reg;
	}
	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) ||
	    (err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32)))) {
		pr_err("%s (rtl8180): No suitable DMA available\n",
		       pci_name(pdev));
		goto err_free_reg;
	}

	pci_set_master(pdev);

	dev = ieee80211_alloc_hw(sizeof(*priv), &rtl8180_ops);
	if (!dev) {
		printk(KERN_ERR "%s (rtl8180): ieee80211 alloc failed\n",
		       pci_name(pdev));
		err = -ENOMEM;
		goto err_free_reg;
	}
	
	priv = dev->priv;
	priv->pdev = pdev;

	dev->max_rates = 1;
	SET_IEEE80211_DEV(dev, &pdev->dev);
	pci_set_drvdata(pdev, dev);

	priv->map_pio = false;
	priv->map = pci_iomap(pdev, 1, mem_len);
	if (!priv->map) {
		priv->map = pci_iomap(pdev, 0, io_len);
		priv->map_pio = true;
	}

	if (!priv->map) {
		dev_err(&pdev->dev, "Can't map device memory/PIO\n");
		err = -ENOMEM;
		goto err_free_dev;
	}
	
	BUILD_BUG_ON(sizeof(priv->channels) != sizeof(rtl818x_channels));
	BUILD_BUG_ON(sizeof(priv->rates) != sizeof(rtl818x_rates));

	memcpy(priv->channels, rtl818x_channels, sizeof(rtl818x_channels));
	memcpy(priv->rates, rtl818x_rates, sizeof(rtl818x_rates));
	priv->band.band = NL80211_BAND_2GHZ;
	priv->band.channels = priv->channels;
	priv->band.n_channels = ARRAY_SIZE(rtl818x_channels);
	priv->band.bitrates = priv->rates;
	priv->band.n_bitrates = 4;
	dev->wiphy->bands[NL80211_BAND_2GHZ] = &priv->band;

	ieee80211_hw_set(dev, HOST_BROADCAST_PS_BUFFERING);
	ieee80211_hw_set(dev, RX_INCLUDES_FCS);

	dev->vif_data_size = sizeof(struct rtl8180_vif);
	dev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
					BIT(NL80211_IFTYPE_ADHOC);

	dev->max_signal = 65;

	reg = rtl818x_ioread32(priv, &priv->map->TX_CONF);
	reg &= RTL818X_TX_CONF_HWVER_MASK;
	switch (reg) {
	case RTL818X_TX_CONF_R8180_ABCD:
		chip_name = "RTL8180";
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8180;
		break;

	case RTL818X_TX_CONF_R8180_F:
		chip_name = "RTL8180vF";
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8180;
		break;

	case RTL818X_TX_CONF_R8185_ABC:
		chip_name = "RTL8185";
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8185;
		break;

	case RTL818X_TX_CONF_R8185_D:
		chip_name = "RTL8185vD";
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8185;
		break;

	case RTL818X_TX_CONF_RTL8187SE:
		chip_name = "RTL8187SE";
		if (priv->map_pio) {
			dev_err(&pdev->dev,
				"MMIO failed. PIO not supported on RTL8187SE\n");
			err = -ENOMEM;
			goto err_iounmap;
		}
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8187SE;
		break;

	default:
		pr_info("%s (rtl8180): Unknown chip! (0x%x)\n",
		       pci_name(pdev), reg >> 25);
		err = -ENODEV;
		goto err_iounmap;
	}
	
	/* we declare to MAC80211 all the queues except for beacon queue
	 * that will be eventually handled by DRV.
	 * TX rings are arranged in such a way that lower is the IDX,
	 * higher is the priority, in order to achieve direct mapping
	 * with mac80211, however the beacon queue is an exception and it
	 * is mapped on the highst tx ring IDX.
	 */
	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
		dev->queues = RTL8187SE_NR_TX_QUEUES - 1;
	else
		dev->queues = RTL8180_NR_TX_QUEUES - 1;	
	
	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8180) {
		priv->band.n_bitrates = ARRAY_SIZE(rtl818x_rates);
		pci_try_set_mwi(pdev);
	}

	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8180)
		ieee80211_hw_set(dev, SIGNAL_DBM);
	else
		ieee80211_hw_set(dev, SIGNAL_UNSPEC);
	wiphy_ext_feature_set(dev->wiphy, NL80211_EXT_FEATURE_CQM_RSSI_LIST);
	rtl8180_eeprom_read(priv); //TODO:
	
	/*RTL8180L: page No. 29 : TODO: */
	switch (priv->rf_type) {
	case 1:	rf_name = "Intersil";
		break;
	case 2:	rf_name = "RFMD";
		break;
	case 3:	priv->rf = &sa2400_rf_ops;
		break;
	case 4:	priv->rf = &max2820_rf_ops;
		break;
	case 5:	priv->rf = &grf5101_rf_ops;
		break;
	case 9:
		if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
			priv->rf = rtl8187se_detect_rf(dev);
		else
			priv->rf = rtl8180_detect_rf(dev);
		break;
	case 10:
		rf_name = "RTL8255";
		break;
	default:
		pr_err("%s (rtl8180): Unknown RF! (0x%x)\n",
		       pci_name(pdev), priv->rf_type);
		err = -ENODEV;
		goto err_iounmap;
	}

	if (!priv->rf) {
		pr_err("%s (rtl8180): %s RF frontend not supported!\n",
		       pci_name(pdev), rf_name);
		err = -ENODEV;
		goto err_iounmap;
	}
	
	if (!is_valid_ether_addr(priv->mac_addr)) {
		pr_warn("%s (rtl8180): Invalid hwaddr! Using"
		       " randomly generated MAC addr\n", pci_name(pdev));
		eth_random_addr(priv->mac_addr);
	}
	
 	/* SET_IEEE80211_PERM_ADDR - set the permanent MAC address 
	 * for 802.11 hardware 
         */
	SET_IEEE80211_PERM_ADDR(dev, priv->mac_addr);

	spin_lock_init(&priv->lock);

	/**
 	 * ieee80211_register_hw - Register hardware device
 	 */
	err = ieee80211_register_hw(dev);
	if (err) {
		pr_err("%s (rtl8180): Cannot register device\n",
		       pci_name(pdev));
		goto err_iounmap;
	}

	wiphy_info(dev->wiphy, "hwaddr %pm, %s + %s\n",
		   priv->mac_addr, chip_name, priv->rf->name);

	return 0;
err_iounmap:
	pci_iounmap(pdev, priv->map);

err_free_dev:
	ieee80211_free_hw(dev);

err_free_reg:
	pci_release_regions(pdev);
	
err_disable_dev:
	pci_disable_device(pdev);
	return err;
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
MODULE_AUTHOR("<Chandan Jha> beingchandanjha@gmail.com ")
/*Driver is based on RTL8180 driver and Rewritten for learning*/
MODULE_DESCRIPTION("RTL8180 PCI wireless driver, Rewritten for knowledge/understanding");
