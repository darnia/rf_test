/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <net_private.h>

#include <device.h>

#include <net/buf.h>
#include <net/ieee802154_radio.h>

#include "my_zigbee.h"

#include <zephyr.h>
#include <board.h>
#include <gpio.h>

#include "usb_acm.h"

/* Change this if you have an LED connected to a custom port */
#ifndef LED0_GPIO_CONTROLLER
#define LED0_GPIO_CONTROLLER 	LED0_GPIO_PORT
#endif

#define LED_PORT LED0_GPIO_CONTROLLER

/* Change this if you have an LED connected to a custom pin */
#define LED	LED0_GPIO_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000



/* Max Bluetooth command data size */
#define BLE_ZIGBEE_CLASS_MAX_DATA_SIZE	100

static struct ieee802154_radio_api *radio_api;
static struct device *ieee802154_dev;

static struct k_fifo tx_queue;

/**
 * Stack for the tx thread.
 */
static K_THREAD_STACK_DEFINE(tx_stack, 1024);
static struct k_thread tx_thread_data;

static int set_channel(void *data, int len)
{
	struct set_channel *req = data;

	SEGGER_SYSVIEW_PrintfHost("page %u channel %u", req->page, req->channel);

	return radio_api->set_channel(ieee802154_dev, req->channel);
}

static int set_ieee_addr(void *data, int len)
{
	struct set_ieee_addr *req = data;

	SEGGER_SYSVIEW_PrintfHost("len %u", len);

	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (u8_t *)&req->ieee_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(void *data, int len)
{
	struct set_short_addr *req = data;

	SEGGER_SYSVIEW_PrintfHost("len %u", len);


	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.short_addr = req->short_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_SHORT_ADDR,
					 &filter);
	}

	return 0;
}

static int set_pan_id(void *data, int len)
{
	struct set_pan_id *req = data;

	SEGGER_SYSVIEW_PrintfHost("len %u", len);

	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.pan_id = req->pan_id;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_PAN_ID,
					 &filter);
	}

	return 0;
}

static int start(void)
{
	SEGGER_SYSVIEW_PrintfHost("Start IEEE 802.15.4 device");

	return radio_api->start(ieee802154_dev);
}

static int stop(void)
{
	SEGGER_SYSVIEW_PrintfHost("Stop IEEE 802.15.4 device");

	return radio_api->stop(ieee802154_dev);
}

static int tx(struct net_pkt *pkt)
{
	struct net_buf *buf = net_buf_frag_last(pkt->frags);
	u8_t seq = net_buf_pull_u8(buf);
	int retries = 3;
	int ret;

	SEGGER_SYSVIEW_PrintfHost("len %d seq %u", buf->len, seq);

	do {
		ret = radio_api->tx(ieee802154_dev, pkt, buf);
	} while (ret && retries--);

	if (ret) {
		/* Send seq = 0 for unsuccessful send */
		seq = 0;
	}

	/*try_write(WPANUSB_ENDP_BULK_IN, &seq, sizeof(seq)); */

	return ret;
}


static void tx_thread(void)
{
	SEGGER_SYSVIEW_PrintfHost("Tx thread started");

	while (1) {
		u8_t cmd;
		struct net_pkt *pkt;
		struct net_buf *buf;

		pkt = k_fifo_get(&tx_queue, K_FOREVER);
		buf = net_buf_frag_last(pkt->frags);
		cmd = net_buf_pull_u8(buf);

		net_hexdump(">", buf->data, buf->len);

		switch (cmd) {
		case RESET:
			SEGGER_SYSVIEW_PrintfHost("Reset device");
			break;
		case TX:
			tx(pkt);
			break;
		case START:
			start();
			break;
		case STOP:
			stop();
			break;
		case SET_CHANNEL:
			set_channel(buf->data, buf->len);
			break;
		case SET_IEEE_ADDR:
			set_ieee_addr(buf->data, buf->len);
			break;
		case SET_SHORT_ADDR:
			set_short_addr(buf->data, buf->len);
			break;
		case SET_PAN_ID:
			set_pan_id(buf->data, buf->len);
			break;
		default:
			SEGGER_SYSVIEW_PrintfHost("%x: Not handled for now", cmd);
			break;
		}

		net_pkt_unref(pkt);

		k_yield();
	}
}



static void init_tx_queue(void)
{
	/* Transmit queue init */
	k_fifo_init(&tx_queue);

	k_thread_create(&tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	struct net_buf *frag;

	frag = net_buf_frag_last(pkt->frags);

	/**
	 * Add length 1 byte, do not forget to reserve it
	 */
	net_buf_push_u8(frag, net_pkt_get_len(pkt));

	/**
	 * Add LQI at the end of the packet
	 */
	net_buf_add_u8(frag, net_pkt_ieee802154_lqi(pkt));

	net_hexdump("<", frag->data, net_pkt_get_len(pkt));

	/*try_write(WPANUSB_ENDP_BULK_IN, frag->data, net_pkt_get_len(pkt));*/

	net_pkt_unref(pkt);

	return 0;
}

void main(void)
{
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();	

	ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	if (!ieee802154_dev) {
		SEGGER_SYSVIEW_PrintfHost("Cannot get IEEE802.15.4 device");
		return;
	}

	radio_api = (struct ieee802154_radio_api *)ieee802154_dev->driver_api;
	
	SEGGER_SYSVIEW_PrintfHost("IEEE802.15.4 radio initialized!");
	/* Initialize net_pkt */
	net_pkt_init();

	/* Initialize transmit queue */
	init_tx_queue();
	radio_api = (struct ieee802154_radio_api *)ieee802154_dev->driver_api;

	/* TODO: Initialize more */

	SEGGER_SYSVIEW_PrintfHost("radio_api %p initialized", radio_api);

	int cnt = 0;
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);

	struct device* usb_acm_dev;
	usb_acm_setup(&usb_acm_dev);

	while (1) {
		write_data(usb_acm_dev, "a", 1);
		/* Set pin to HIGH/LOW every 1 second */
		gpio_pin_write(dev, LED, cnt % 2);
		cnt++;
		k_sleep(SLEEP_TIME);
	}
}
