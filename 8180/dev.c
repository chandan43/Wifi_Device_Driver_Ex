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

static int rtl8180_init_rx_ring(struct ieee80211_hw *dev)
{
	struct rtl8180_priv *priv = dev->priv;
	struct rtl818x_rx_cmd_desc *entry;
	int i;
	
	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
		priv->rx_ring_sz = sizeof(struct rtl8187se_rx_desc);
	else
		priv->rx_ring_sz = sizeof(struct rtl8180_rx_desc);

	priv->rx_ring = pci_zalloc_consistent(priv->pdev, priv->rx_ring_sz * 32,
					      &priv->rx_ring_dma);

	if (!priv->rx_ring || (unsigned long)priv->rx_ring & 0xFF) {
		wiphy_err(dev->wiphy, "Cannot allocate RX ring\n");
		return -ENOMEM;
	}

	priv->rx_idx = 0;	
	/* legacy helper around netdev_alloc_skb() */
	for (i = 0; i < 32; i++) {
		struct sk_buff *skb = dev_alloc_skb(MAX_RX_SIZE);
		dma_addr_t *mapping;
		entry = priv->rx_ring + priv->rx_ring_sz*i;
		if (!skb) {
			pci_free_consistent(priv->pdev, priv->rx_ring_sz * 32,
					priv->rx_ring, priv->rx_ring_dma);
			wiphy_err(dev->wiphy, "Cannot allocate RX skb\n");
			return -ENOMEM;
		}
		priv->rx_buf[i] = skb;
		mapping = (dma_addr_t *)skb->cb;
		*mapping = pci_map_single(priv->pdev, skb_tail_pointer(skb),
					  MAX_RX_SIZE, PCI_DMA_FROMDEVICE);

		if (pci_dma_mapping_error(priv->pdev, *mapping)) {
			kfree_skb(skb);
			pci_free_consistent(priv->pdev, priv->rx_ring_sz * 32,
					priv->rx_ring, priv->rx_ring_dma);
			wiphy_err(dev->wiphy, "Cannot map DMA for RX skb\n");
			return -ENOMEM;
		}

		entry->rx_buf = cpu_to_le32(*mapping);
		entry->flags = cpu_to_le32(RTL818X_RX_DESC_FLAG_OWN |
					   MAX_RX_SIZE);
	}
	entry->flags |= cpu_to_le32(RTL818X_RX_DESC_FLAG_EOR);
	return 0;	
}
static void rtl8180_free_rx_ring(struct ieee80211_hw *dev)
{
	struct rtl8180_priv *priv = dev->priv;
	int i;

	for (i = 0; i < 32; i++) {
		struct sk_buff *skb = priv->rx_buf[i];
		if (!skb)
			continue;

		pci_unmap_single(priv->pdev,
				 *((dma_addr_t *)skb->cb),
				 MAX_RX_SIZE, PCI_DMA_FROMDEVICE);
		kfree_skb(skb);
	}

	pci_free_consistent(priv->pdev, priv->rx_ring_sz * 32,
			    priv->rx_ring, priv->rx_ring_dma);
	priv->rx_ring = NULL;
}

/*
 * This function creates a split out lock class for each invocation;
 * this is needed for now since a whole lot of users of the skb-queue
 * infrastructure in drivers have different locking usage (in hardirq)
 * than the networking core (in softirq only). In the long run either the
 * network layer or drivers should need annotation to consolidate the
 * main types of usage into 3 classes.
 */
static int rtl8180_init_tx_ring(struct ieee80211_hw *dev,
				unsigned int prio, unsigned int entries)
{
	struct rtl8180_priv *priv = dev->priv;
	struct rtl8180_tx_desc *ring;
	dma_addr_t dma;
	int i;

	/* DMA cohrent buffer allocation */
	ring = pci_zalloc_consistent(priv->pdev, sizeof(*ring) * entries,
				     &dma);
	if (!ring || (unsigned long)ring & 0xFF) {
		wiphy_err(dev->wiphy, "Cannot allocate TX ring (prio = %d)\n",
			  prio);
		return -ENOMEM;
	}

	priv->tx_ring[prio].desc = ring;
	priv->tx_ring[prio].dma = dma;
	priv->tx_ring[prio].idx = 0;
	priv->tx_ring[prio].entries = entries;
	skb_queue_head_init(&priv->tx_ring[prio].queue);

	for (i = 0; i < entries; i++)
		ring[i].next_tx_desc =
			cpu_to_le32((u32)dma + ((i + 1) % entries) * sizeof(*ring));

	return 0;
}

/**
 *	__skb_dequeue - remove from the head of the queue
 *	@list: list to dequeue from
 *
 *	Remove the head of the list. This function does not take any locks
 *	so must be used with appropriate locks held only. The head item is
 *	returned or %NULL if the list is empty.
 */
static void rtl8180_free_tx_ring(struct ieee80211_hw *dev, unsigned int prio)
{
	struct rtl8180_priv *priv = dev->priv;
	struct rtl8180_tx_ring *ring = &priv->tx_ring[prio];

	while (skb_queue_len(&ring->queue)) {
		struct rtl8180_tx_desc *entry = &ring->desc[ring->idx];
		struct sk_buff *skb = __skb_dequeue(&ring->queue);

		pci_unmap_single(priv->pdev, le32_to_cpu(entry->tx_buf),
				 skb->len, PCI_DMA_TODEVICE);
		kfree_skb(skb);
		ring->idx = (ring->idx + 1) % ring->entries;
	}

	pci_free_consistent(priv->pdev, sizeof(*ring->desc)*ring->entries,
			    ring->desc, ring->dma);
	ring->desc = NULL;
}
/*
 * This function creates a split out lock class for each invocation;
 * this is needed for now since a whole lot of users of the skb-queue
 * infrastructure in drivers have different locking usage (in hardirq)
 * than the networking core (in softirq only). In the long run either the
 * network layer or drivers should need annotation to consolidate the
 * main types of usage into 3 classes.
 */
static int rtl8180_start(struct ieee80211_hw *dev)
{
	struct rtl8180_priv *priv = dev->priv;
	int ret, i;
	u32 reg;
	
	ret = rtl8180_init_rx_ring(dev); //TODO:
	if (ret)
		return ret;
	for (i = 0; i < (dev->queues + 1); i++)
		if ((ret = rtl8180_init_tx_ring(dev, i, 16))) //TODO
			goto err_free_rings;

	ret = rtl8180_init_hw(dev);
	if (ret)
		goto err_free_rings;
	
	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE) {
		ret = request_irq(priv->pdev->irq, rtl8187se_interrupt,
			  IRQF_SHARED, KBUILD_MODNAME, dev);
	} else {
		ret = request_irq(priv->pdev->irq, rtl8180_interrupt,
			  IRQF_SHARED, KBUILD_MODNAME, dev);
	}
	
	if (ret) {
		wiphy_err(dev->wiphy, "failed to register IRQ handler\n");
		goto err_free_rings;
	}

	rtl8180_int_enable(dev); //TODO
	/* in rtl8187se at MAR regs offset there is the management
	 * TX descriptor DMA addres..
	 */
	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8187SE) {
		rtl818x_iowrite32(priv, &priv->map->MAR[0], ~0);
		rtl818x_iowrite32(priv, &priv->map->MAR[1], ~0);
	}
	
	reg = RTL818X_RX_CONF_ONLYERLPKT |
	      RTL818X_RX_CONF_RX_AUTORESETPHY |
	      RTL818X_RX_CONF_MGMT |
	      RTL818X_RX_CONF_DATA |
	      (7 << 8 /* MAX RX DMA */) |
	      RTL818X_RX_CONF_BROADCAST |
	      RTL818X_RX_CONF_NICMAC;

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8185)
		reg |= RTL818X_RX_CONF_CSDM1 | RTL818X_RX_CONF_CSDM2;
	else if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8180) {
		reg |= (priv->rfparam & RF_PARAM_CARRIERSENSE1)
			? RTL818X_RX_CONF_CSDM1 : 0;
		reg |= (priv->rfparam & RF_PARAM_CARRIERSENSE2)
			? RTL818X_RX_CONF_CSDM2 : 0;
	} else {
		reg &= ~(RTL818X_RX_CONF_CSDM1 | RTL818X_RX_CONF_CSDM2);
	}

	priv->rx_conf = reg;
	rtl818x_iowrite32(priv, &priv->map->RX_CONF, reg);
	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8180) {
		reg = rtl818x_ioread8(priv, &priv->map->CW_CONF);

		/* CW is not on per-packet basis.
		 * in rtl8185 the CW_VALUE reg is used.
		 * in rtl8187se the AC param regs are used.
		 */
		reg &= ~RTL818X_CW_CONF_PERPACKET_CW;
		/* retry limit IS on per-packet basis.
		 * the short and long retry limit in TX_CONF
		 * reg are ignored
		 */
		reg |= RTL818X_CW_CONF_PERPACKET_RETRY;
		rtl818x_iowrite8(priv, &priv->map->CW_CONF, reg);

		reg = rtl818x_ioread8(priv, &priv->map->TX_AGC_CTL);
		reg |= (priv->rfparam & RF_PARAM_CARRIERSENSE2)
			? RTL818X_RX_CONF_CSDM2 : 0;
	} else {
		reg &= ~(RTL818X_RX_CONF_CSDM1 | RTL818X_RX_CONF_CSDM2);
	}

	priv->rx_conf = reg;
	rtl818x_iowrite32(priv, &priv->map->RX_CONF, reg);
	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8180) {
		reg = rtl818x_ioread8(priv, &priv->map->CW_CONF);

		/* CW is not on per-packet basis.
		 * in rtl8185 the CW_VALUE reg is used.
		 * in rtl8187se the AC param regs are used.
		 */
		reg &= ~RTL818X_CW_CONF_PERPACKET_CW;
		/* retry limit IS on per-packet basis.
		 * the short and long retry limit in TX_CONF
		 * reg are ignored
		 */
		reg |= RTL818X_CW_CONF_PERPACKET_RETRY;
		rtl818x_iowrite8(priv, &priv->map->CW_CONF, reg);

		reg = rtl818x_ioread8(priv, &priv->map->TX_AGC_CTL);
		/* TX antenna and TX gain are not on per-packet basis.
		 * TX Antenna is selected by ANTSEL reg (RX in BB regs).
		 * TX gain is selected with CCK_TX_AGC and OFDM_TX_AGC regs
		 */
		reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_GAIN;
		reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_ANTSEL;
		reg |=  RTL818X_TX_AGC_CTL_FEEDBACK_ANT;
		rtl818x_iowrite8(priv, &priv->map->TX_AGC_CTL, reg);

		/* disable early TX */
		rtl818x_iowrite8(priv, (u8 __iomem *)priv->map + 0xec, 0x3f);
	}
	reg = rtl818x_ioread32(priv, &priv->map->TX_CONF);
	reg |= (6 << 21 /* MAX TX DMA */) |
	       RTL818X_TX_CONF_NO_ICV;

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
		reg |= 1<<30;  /*  "duration procedure mode" */

	if (priv->chip_family != RTL818X_CHIP_FAMILY_RTL8180)
		reg &= ~RTL818X_TX_CONF_PROBE_DTS;
	else
		reg &= ~RTL818X_TX_CONF_HW_SEQNUM;

	reg &= ~RTL818X_TX_CONF_DISCW;
	
	/* different meaning, same value on both rtl8185 and rtl8180 */
	reg &= ~RTL818X_TX_CONF_SAT_HWPLCP;

	rtl818x_iowrite32(priv, &priv->map->TX_CONF, reg);

	reg = rtl818x_ioread8(priv, &priv->map->CMD);
	reg |= RTL818X_CMD_RX_ENABLE;
	reg |= RTL818X_CMD_TX_ENABLE;
	rtl818x_iowrite8(priv, &priv->map->CMD, reg);

	return 0;

err_free_rings:
	rtl8180_free_rx_ring(dev); //TODO
	for (i = 0; i < (dev->queues + 1); i++)
		if (priv->tx_ring[i].desc)
			rtl8180_free_tx_ring(dev, i);

	return ret;	
}

static void rtl8180_stop(struct ieee80211_hw *dev)
{
	struct rtl8180_priv *priv = dev->priv;
	u8 reg;
	int i;
	
	rtl8180_int_disable(dev); //TODO
	
	reg = rtl818x_ioread8(priv, &priv->map->CMD);
	reg &= ~RTL818X_CMD_TX_ENABLE;
	reg &= ~RTL818X_CMD_RX_ENABLE;
	rtl818x_iowrite8(priv, &priv->map->CMD, reg);

	priv->rf->stop(dev);

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG4);
	rtl818x_iowrite8(priv, &priv->map->CONFIG4, reg | RTL818X_CONFIG4_VCOOFF);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	free_irq(priv->pdev->irq, dev);

	rtl8180_free_rx_ring(dev); //TODO
	for (i = 0; i < (dev->queues + 1); i++)
		rtl8180_free_tx_ring(dev, i); //TODO		
}

static u64 rtl8180_get_tsf(struct ieee80211_hw *dev,
			   struct ieee80211_vif *vif)
{
	struct rtl8180_priv *priv = dev->priv;

	return rtl818x_ioread32(priv, &priv->map->TSFT[0]) |
	       (u64)(rtl818x_ioread32(priv, &priv->map->TSFT[1])) << 32;
}

/**
 * container_of - cast a member of a structure out to the containing structure
 *
 * @ptr:	the pointer to the member.
 * @type:       the type of the container struct this is embedded in.
 * @member:     the name of the member within the struct.
 *
 */
/**
 * ieee80211_queue_stopped - test status of the queue
 * @hw: pointer as obtained from ieee80211_alloc_hw().
 * @queue: queue number (counted from zero).
 *
 * Drivers should use this function instead of netif_stop_queue.
 *
 * Return: %true if the queue is stopped. %false otherwise.
 */
//TODO
static void rtl8180_beacon_work(struct work_struct *work)
{
	struct rtl8180_vif *vif_priv =
		container_of(work, struct rtl8180_vif, beacon_work.work);
	struct ieee80211_vif *vif =
		container_of((void *)vif_priv, struct ieee80211_vif, drv_priv);
	struct ieee80211_hw *dev = vif_priv->dev;
	struct ieee80211_mgmt *mgmt;
	struct sk_buff *skb;
	
	/* don't overflow the tx ring */
	if (ieee80211_queue_stopped(dev, 0)) //TODO
		goto resched;

	/* grab a fresh beacon */
	skb = ieee80211_beacon_get(dev, vif);
	if (!skb)
		goto resched;

	/*
	 * update beacon timestamp w/ TSF value
	 * TODO: make hardware update beacon timestamp
	 */
	mgmt = (struct ieee80211_mgmt *)skb->data;
	mgmt->u.beacon.timestamp = cpu_to_le64(rtl8180_get_tsf(dev, vif));

	/* TODO: use actual beacon queue */
	skb_set_queue_mapping(skb, 0);

	rtl8180_tx(dev, NULL, skb);

resched:
	/*
	 * schedule next beacon
	 * TODO: use hardware support for beacon timing
	 */
	schedule_delayed_work(&vif_priv->beacon_work,
			usecs_to_jiffies(1024 * vif->bss_conf.beacon_int));

}
static int rtl8180_add_interface(struct ieee80211_hw *dev,
				 struct ieee80211_vif *vif)
{
	struct rtl8180_priv *priv = dev->priv;
	struct rtl8180_vif *vif_priv;

	/*
	 * We only support one active interface at a time.
	 */
	if (priv->vif)
		return -EBUSY;
	
	switch (vif->type) {
	/*Managed BSS member*/
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_ADHOC:
		break;
	default:
		return -EOPNOTSUPP;
	}		
	priv->vif = vif;
	/* Initialize driver private area */
	vif_priv = (struct rtl8180_vif *)&vif->drv_priv;
	vif_priv->dev = dev;
	INIT_DELAYED_WORK(&vif_priv->beacon_work, rtl8180_beacon_work); //TODO
	vif_priv->enable_beacon = false;
	
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	rtl818x_iowrite32(priv, (__le32 __iomem *)&priv->map->MAC[0],
			  le32_to_cpu(*(__le32 *)vif->addr));
	rtl818x_iowrite16(priv, (__le16 __iomem *)&priv->map->MAC[4],
			  le16_to_cpu(*(__le16 *)(vif->addr + 4)));
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	return 0;
}

static void rtl8180_remove_interface(struct ieee80211_hw *dev,
				     struct ieee80211_vif *vif)
{
	struct rtl8180_priv *priv = dev->priv;
	priv->vif = NULL;
}

static int rtl8180_config(struct ieee80211_hw *dev, u32 changed)
{
	struct rtl8180_priv *priv = dev->priv;
	struct ieee80211_conf *conf = &dev->conf;

	priv->rf->set_chan(dev, conf);

	return 0;
}


/**
 * enum ieee80211_ac_numbers - AC numbers as used in mac80211
 * @IEEE80211_AC_VO: voice
 * @IEEE80211_AC_VI: video
 * @IEEE80211_AC_BE: best effort
 * @IEEE80211_AC_BK: background
 */
static void rtl8187se_conf_ac_parm(struct ieee80211_hw *dev, u8 queue)
{
	const struct ieee80211_tx_queue_params *params;
	struct rtl8180_priv *priv = dev->priv;

	/* hw value */
	u32 ac_param;
	u8 aifs;
	u8 txop;
	u8 cw_min, cw_max;

	params = &priv->queue_param[queue];
	cw_min = fls(params->cw_min);
	cw_max = fls(params->cw_max);

	aifs = 10 + params->aifs * priv->slot_time;
	/* TODO: check if txop HW is in us (mult by 32) */
	txop = params->txop;
	ac_param = txop << AC_PARAM_TXOP_LIMIT_SHIFT |
		cw_max << AC_PARAM_ECW_MAX_SHIFT |
		cw_min << AC_PARAM_ECW_MIN_SHIFT |
		aifs << AC_PARAM_AIFS_SHIFT;
	switch (queue) {
	case IEEE80211_AC_BK:
		rtl818x_iowrite32(priv, &priv->map->AC_BK_PARAM, ac_param);
		break;
	case IEEE80211_AC_BE:
		rtl818x_iowrite32(priv, &priv->map->AC_BE_PARAM, ac_param);
		break;
	case IEEE80211_AC_VI:
		rtl818x_iowrite32(priv, &priv->map->AC_VI_PARAM, ac_param);
		break;
	case IEEE80211_AC_VO:
		rtl818x_iowrite32(priv, &priv->map->AC_VO_PARAM, ac_param);
		break;
	}	
}	

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
	/* I _hope_ this means 10uS for the HW.
	 * In reference code it is 0x22 for
	 * both rtl8187L and rtl8187SE
	 */
	sifs = 0x22;
	if (info->use_short_slot)
		priv->slot_time = 9;
	else
		priv->slot_time = 20;

	/* from reference code. set ack timeout reg = eifs reg */
	rtl818x_iowrite8(priv, &priv->map->CARRIER_SENSE_COUNTER, hw_eifs);

	if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8187SE)
		rtl818x_iowrite8(priv, &priv->map->EIFS_8187SE, hw_eifs);
	else if (priv->chip_family == RTL818X_CHIP_FAMILY_RTL8185) {
		/* rtl8187/rtl8185 HW bug. After EIFS is elapsed,
		 * the HW still wait for DIFS.
		 * HW uses 4uS units for EIFS.
		 */
		hw_eifs = DIV_ROUND_UP(eifs - difs, 4);

		rtl818x_iowrite8(priv, &priv->map->EIFS, hw_eifs);
	}
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
