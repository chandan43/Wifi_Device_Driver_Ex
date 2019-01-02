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

static int rtl8180_conf_tx(struct ieee80211_hw *dev,
			    struct ieee80211_vif *vif, u16 queue,
			    const struct ieee80211_tx_queue_params *params)
{
	struct rtl8180_priv *priv = dev->priv;
	u8 cw_min, cw_max;
	if (priv->chip_name == RTL818X_CHIP_FAMILY_RTL8180)
		return;

	/* find last set bit in word */
	cw_min = fls(params->cw_min);
	cw_max = fls(params->cw_max);

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE) {
		priv->queue_param[queue] = *params;
		rtl8187se_conf_ac_parm(dev, queue);
	} else 
		rtl818x_iowrite8(priv, &priv->map->CW_VAL,
						 (cw_max << 4) | cw_min);
	return 0;
}


static void rtl8180_conf_erp(struct ieee80211_hw *dev,
			    struct ieee80211_bss_conf *info)
{
	struct rtl8180_priv *priv = dev->priv;
	if (priv->chip_name == RTL818X_CHIP_FAMILY_RTL8180)
		return;
}		    	
/**
 * ieee80211_generic_frame_duration - Calculate the duration field for a frame
 * @hw: pointer obtained from ieee80211_alloc_hw().
 * @vif: &struct ieee80211_vif pointer from the add_interface callback.
 * @band: the band to calculate the frame duration on
 * @frame_len: the length of the frame.
 * @rate: the rate at which the frame is going to be transmitted.
 *
 * Calculate the duration field of some generic frame, given its
 * length and transmission rate (in 100kbps).
 *
 * Return: The duration.
 */
static void rtl8180_bss_info_changed(struct ieee80211_hw *dev,
				     struct ieee80211_vif *vif,
				     struct ieee80211_bss_conf *info,
				     u32 changed)
{
	struct rtl8180_priv *priv = dev->priv;
	struct rtl8180_vif *vif_priv;
	int i;
	u8 reg;

	vif_priv = (struct rtl8180_vif *)&vif->drv_priv;

	/* @BSS_CHANGED_BSSID: BSSID changed, for whatever
 	 * reason (IBSS and managed mode)
 	 */
	
	if (changed & BSS_CHANGED_BSSID) {
		rtl818x_iowrite16(priv, (__le16 __iomem *)&priv->map->BSSID[0],
				  le16_to_cpu(*(__le16 *)info->bssid));
		rtl818x_iowrite32(priv, (__le32 __iomem *)&priv->map->BSSID[2],
				  le32_to_cpu(*(__le32 *)(info->bssid + 2)));
		
		if (is_valid_ether_addr(info->bssid)) {
			if (vif->type == NL80211_IFTYPE_ADHOC) //independent BSS member
				reg = RTL818X_MSR_ADHOC;
			else
				reg = RTL818X_MSR_INFRA;
		} else
			reg = RTL818X_MSR_NO_LINK;

		if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
			reg |= RTL818X_MSR_ENEDCA;

		rtl818x_iowrite8(priv, &priv->map->MSR, reg);
	}

	if (changed & BSS_CHANGED_BASIC_RATES)  //Basic rateset changed
		rtl8180_conf_basic_rates(dev, info->basic_rates);
	/* Slot timing changed $ preamble changed */
	if (changed & (BSS_CHANGED_ERP_SLOT | BSS_CHANGED_ERP_PREAMBLE)) {  

		/* when preamble changes, acktime duration changes, and erp must
		 * be recalculated. ACK time is calculated at lowest rate.
		 * Since mac80211 include SIFS time we remove it (-10)
		 */
		priv->ack_time =
			le16_to_cpu(ieee80211_generic_frame_duration(dev,
					priv->vif,
					NL80211_BAND_2GHZ, 10,
					&priv->rates[0])) - 10;

		rtl8180_conf_erp(dev, info); //TODO:

		/* mac80211 supplies aifs_n to driver and calls
		 * conf_tx callback whether aifs_n changes, NOT
		 * when aifs changes.
		 * Aifs should be recalculated if slot changes.
		 */
		if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE) {
			for (i = 0; i < 4; i++)
				rtl8187se_conf_ac_parm(dev, i); //TODO
		}
	}
	/* Beaconing should be ENABLED/Desabled */	
	if (changed & BSS_CHANGED_BEACON_ENABLED)
		vif_priv->enable_beacon = info->enable_beacon;

	/* Beacon data changed, retrieve */
	if (changed & (BSS_CHANGED_BEACON_ENABLED | BSS_CHANGED_BEACON)) {
		/* Cancel a delayed work and wait for it to finish : ?? TODO*/
		cancel_delayed_work_sync(&vif_priv->beacon_work);
		if (vif_priv->enable_beacon)
			schedule_work(&vif_priv->beacon_work.work);
	}
			
}

static u64 rtl8180_prepare_multicast(struct ieee80211_hw *dev,
				     struct netdev_hw_addr_list *mc_list)
{
	return netdev_hw_addr_list_count(mc_list);
}

static void rtl8180_configure_filter(struct ieee80211_hw *dev,
				     unsigned int changed_flags,
				     unsigned int *total_flags,
				     u64 multicast)
{
	struct rtl8180_priv *priv = dev->priv;
	
	if (changed_flags & FIF_FCSFAIL)
		priv->rx_conf ^= RTL818X_RX_CONF_FCS;	
	if (changed_flags & FIF_CONTROL)
		priv->rx_conf ^= RTL818X_RX_CONF_CTRL;
	if (changed_flags & FIF_OTHER_BSS)
		priv->rx_conf ^= RTL818X_RX_CONF_MONITOR;
	if (*total_flags & FIF_ALLMULTI || multicast > 0)
		priv->rx_conf |= RTL818X_RX_CONF_MULTICAST;
	else
		priv->rx_conf &= ~RTL818X_RX_CONF_MULTICAST;
	
	if (priv->rx_conf & RTL818X_RX_CONF_FCS)
		*total_flags |= FIF_FCSFAIL;
	if (priv->rx_conf & RTL818X_RX_CONF_CTRL)
		*total_flags |= FIF_CONTROL;
	if (priv->rx_conf & RTL818X_RX_CONF_MONITOR)
		*total_flags |= FIF_OTHER_BSS;
	if (priv->rx_conf & RTL818X_RX_CONF_MULTICAST)
		*total_flags |= FIF_ALLMULTI;
	rtl818x_iowrite32(priv, &priv->map->RX_CONF, priv->rx_conf);
}
/**
 * struct ieee80211_ops - callbacks from mac80211 to the driver
 *
 * This structure contains various callbacks that the driver may
 * handle or, in some cases, must handle, for example to configure
 * the hardware to a new channel or to transmit a frame.
 *
 * @tx: Handler that 802.11 module calls for each transmitted frame.
 *	skb contains the buffer starting from the IEEE 802.11 header.
 *	The low-level driver should send the frame out based on
 *	configuration in the TX control data. This handler should,
 *	preferably, never fail and stop queues appropriately.
 *	Must be atomic.
 *
 * @start: Called before the first netdevice attached to the hardware
 *	is enabled. This should turn on the hardware and must turn on
 *	frame reception (for possibly enabled monitor interfaces.)
 *	Returns negative error codes, these may be seen in userspace,
 *	or zero.
 *	When the device is started it should not have a MAC address
 *	to avoid acknowledging frames before a non-monitor device
 *	is added.
 *	Must be implemented and can sleep.
 *
 * @stop: Called after last netdevice attached to the hardware
 *	is disabled. This should turn off the hardware (at least
 *	it must turn off frame reception.)
 *	May be called right after add_interface if that rejects
 *	an interface. If you added any work onto the mac80211 workqueue
 *	you should ensure to cancel it on this callback.
 *	Must be implemented and can sleep.
 * 
 * @add_interface: Called when a netdevice attached to the hardware is
 *	enabled. Because it is not called for monitor mode devices, @start
 *	and @stop must be implemented.
 *	The driver should perform any initialization it needs before
 *	the device can be enabled. The initial configuration for the
 *	interface is given in the conf parameter.
 *	The callback may refuse to add an interface by returning a
 *	negative error code (which will be seen in userspace.)
 *	Must be implemented and can sleep.
 *
 * @remove_interface: Notifies a driver that an interface is going down.
 *	The @stop callback is called after this if it is the last interface
 *	and no monitor interfaces are present.
 *	When all interfaces are removed, the MAC address in the hardware
 *	must be cleared so the device no longer acknowledges packets,
 *	the mac_addr member of the conf structure is, however, set to the
 *	MAC address of the device going away.
 *	Hence, this callback must be implemented. It can sleep.
 *
 * @config: Handler for configuration requests. IEEE 802.11 code calls this
 *	function to change hardware configuration, e.g., channel.
 *	This function should never fail but returns a negative error code
 *	if it does. The callback can sleep.
 *
 * @bss_info_changed: Handler for configuration requests related to BSS
 *	parameters that may vary during BSS's lifespan, and may affect low
 *	level driver (e.g. assoc/disassoc status, erp parameters).
 *	This function should not be used if no BSS has been set, unless
 *	for association indication. The @changed parameter indicates which
 *	of the bss parameters has changed when a call is made. The callback
 *	can sleep.
 *
 * @conf_tx: Configure TX queue parameters (EDCF (aifs, cw_min, cw_max),
 *	bursting) for a hardware TX queue.
 *	Returns a negative error code on failure.
 *	The callback can sleep.
 *
 * @prepare_multicast: Prepare for multicast filter configuration.
 *	This callback is optional, and its return value is passed
 *	to configure_filter(). This callback must be atomic.
 *
 * @configure_filter: Configure the device's RX filter.
 *	See the section "Frame filtering" for more information.
 *	This callback must be implemented and can sleep.
 *
 * @get_tsf: Get the current TSF timer value from firmware/hardware. Currently,
 *	this is only used for IBSS mode BSSID merging and debugging. Is not a
 *	required function.
 *	The callback can sleep.
 */
static const struct ieee80211_ops rtl8180_ops = {
	.tx			= rtl8180_tx,
	.start			= rtl8180_start,
	.stop			= rtl8180_stop,
	.add_interface		= rtl8180_add_interface,
	.remove_interface	= rtl8180_remove_interface,
	.config			= rtl8180_config,
	.bss_info_changed	= rtl8180_bss_info_changed,
	.conf_tx		= rtl8180_conf_tx,
	.prepare_multicast	= rtl8180_prepare_multicast,
	.configure_filter	= rtl8180_configure_filter,
	.get_tsf		= rtl8180_get_tsf,	
};
static void rtl8180_eeprom_register_read(struct eeprom_93cx6 *eeprom)
{
	struct rtl8180_priv *priv = eeprom->data;
	u8 reg = rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);

	eeprom->reg_data_in = reg & RTL818X_EEPROM_CMD_WRITE;
	eeprom->reg_data_out = reg & RTL818X_EEPROM_CMD_READ;
	eeprom->reg_data_clock = reg & RTL818X_EEPROM_CMD_CK;
	eeprom->reg_chip_select = reg & RTL818X_EEPROM_CMD_CS;
}

static void rtl8180_eeprom_register_write(struct eeprom_93cx6 *eeprom)
{
	struct rtl8180_priv *priv = eeprom->data;
	u8 reg = 2 << 6;

	if (eeprom->reg_data_in)
		reg |= RTL818X_EEPROM_CMD_WRITE;
	if (eeprom->reg_data_out)
		reg |= RTL818X_EEPROM_CMD_READ;
	if (eeprom->reg_data_clock)
		reg |= RTL818X_EEPROM_CMD_CK;
	if (eeprom->reg_chip_select)
		reg |= RTL818X_EEPROM_CMD_CS;

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, reg);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(10);
}

/**
 * eeprom_93cx6_read - Read a word from eeprom
 * @eeprom: Pointer to eeprom structure
 * @word: Word index from where we should start reading
 * @data: target pointer where the information will have to be stored
 *
 * This function will read the eeprom data as host-endian word
 * into the given data pointer.
 */
/**
 * eeprom_93cx6_multiread - Read multiple words from eeprom
 * @eeprom: Pointer to eeprom structure
 * @word: Word index from where we should start reading
 * @data: target pointer where the information will have to be stored
 * @words: Number of words that should be read.
 *
 * This function will read all requested words from the eeprom,
 * this is done by calling eeprom_93cx6_read() multiple times.
 * But with the additional change that while the eeprom_93cx6_read
 * will return host ordered bytes, this method will return little
 * endian words.
 */
static void rtl8180_eeprom_read(struct rtl8180_priv *priv)
{
	struct eeprom_93cx6 eeprom;
	int eeprom_cck_table_adr;
	u16 eeprom_val;
	int i;

	eeprom.data = priv;
	eeprom.register_read = rtl8180_eeprom_register_read; //TODO : Done
	eeprom.register_write = rtl8180_eeprom_register_write; //TODO : Done
	if (rtl818x_ioread32(priv, &priv->map->RX_CONF) & (1 << 6)) // page 23
		eeprom.width = PCI_EEPROM_WIDTH_93C66;
	else 
		eeprom.width = PCI_EEPROM_WIDTH_93C46;

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD,
			RTL818X_EEPROM_CMD_PROGRAM);

	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(10);
	
	eeprom_93cx6_read(&eeprom, 0x06, &eeprom_val);
	eeprom_val &= 0xFF;
	priv->rf_type = eeprom_val;
	pr_err("%s: RF type value is %04x\n",__func__,eeprom_val);	
	eeprom_93cx6_read(&eeprom, 0x17, &eeprom_val);
	priv->csthreshold = eeprom_val >> 8;
	
	eeprom_93cx6_multiread(&eeprom, 0x7, (__le16 *)priv->mac_addr, 3);
	
	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
		eeprom_cck_table_adr = 0x30;
	else
		eeprom_cck_table_adr = 0x10;
	
	/* CCK TX power */
	for (i = 0; i < 14; i += 2) {
		u16 txpwr;
		eeprom_93cx6_read(&eeprom, eeprom_cck_table_adr + (i >> 1),
				&txpwr);
		priv->channels[i].hw_value = txpwr & 0xFF;
		priv->channels[i + 1].hw_value = txpwr >> 8;
	}

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8180) {
		__le32 anaparam;
		eeprom_93cx6_multiread(&eeprom, 0xD, (__le16 *)&anaparam, 2);
		priv->anaparam = le32_to_cpu(anaparam);
		eeprom_93cx6_read(&eeprom, 0x19, &priv->rfparam);
	}

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE) {
		eeprom_93cx6_read(&eeprom, 0x3F, &eeprom_val);
		priv->antenna_diversity_en = !!(eeprom_val & 0x100);
		priv->antenna_diversity_default = (eeprom_val & 0xC00) == 0x400;

		eeprom_93cx6_read(&eeprom, 0x7C, &eeprom_val);
		priv->xtal_out = eeprom_val & 0xF;
		priv->xtal_in = (eeprom_val & 0xF0) >> 4;
		priv->xtal_cal = !!(eeprom_val & 0x1000);
		priv->thermal_meter_val = (eeprom_val & 0xF00) >> 8;
		priv->thermal_meter_en = !!(eeprom_val & 0x2000);
	}

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD,
			RTL818X_EEPROM_CMD_NORMAL);
}
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

	dev = ieee80211_alloc_hw(sizeof(*priv), &rtl8180_ops);  //TODO: 
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
