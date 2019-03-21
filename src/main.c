/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample app for CDC ACM class driver
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <stdio.h>
#include <string.h>
#include <device.h>
#include <uart.h>
#include <zephyr.h>

#include <gpio.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <misc/printk.h>
#include <misc/util.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16,
              0xaa, 0xfe, /* Eddystone UUID */
              0x10, /* Eddystone-URL frame type */
              0x00, /* Calibrated Tx power at 0m */
              0x00, /* URL Scheme Prefix http://www. */
              'z', 'e', 'p', 'h', 'y', 'r',
              'p', 'r', 'o', 'j', 'e', 'c', 't',
              0x08) /* .org */
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};


static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
                  sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Beacon started\n");
}


static const char *banner1 = "All your base are belong to us\n";

static volatile bool data_transmitted;
static volatile bool data_arrived;
static	char data_buf[64];

static void interrupt_handler(struct device *dev)
{
	uart_irq_update(dev);

	if (uart_irq_tx_ready(dev)) {
		data_transmitted = true;
	}

	if (uart_irq_rx_ready(dev)) {
		data_arrived = true;
	}
}

static void write_data(struct device *dev, const char *buf, int len)
{
	uart_irq_tx_enable(dev);

	while (len) {
		int written;

		data_transmitted = false;
		written = uart_fifo_fill(dev, (const u8_t *)buf, len);
		while (data_transmitted == false) {
			k_yield();
		}

		len -= written;
		buf += written;
	}

	uart_irq_tx_disable(dev);
}

static void read_and_echo_data(struct device *dev, int *bytes_read)
{
    int t = 100;
	while (t && data_arrived == false)
    {
        t--;
    }

	data_arrived = false;

	/* Read all data and echo it back */
	while ((*bytes_read = uart_fifo_read(dev,
	    (u8_t *)data_buf, sizeof(data_buf)))) {
		write_data(dev, data_buf, *bytes_read);
	}
}

void main(void)
{

	struct device *dev;
	u32_t baudrate, bytes_read, dtr = 0U;
	int ret;
	dev = device_get_binding(CONFIG_CDC_ACM_PORT_NAME_0);
	if (!dev) {
		printf("CDC ACM device not found\n");
		return;
	}

    int t = 100;
	printf("Wait for DTR\n");
	while (t) {
		uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
		if (dtr)
			break;
        t--;
	}
	printf("DTR set, start test\n");

	ret = uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
	if (ret)
		printf("Failed to set DCD, ret code %d\n", ret);
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);
	if (ret)
		printf("Failed to set DSR, ret code %d\n", ret);

	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
		printf("Failed to get baudrate, ret code %d\n", ret);
	else
		printf("Baudrate detected: %d\n", baudrate);

	uart_irq_callback_set(dev, interrupt_handler);
	write_data(dev, banner1, strlen(banner1));

	uart_irq_rx_enable(dev);
    int err;

    printk("Starting Beacon Demo\n");

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    struct device *dev_led;
    int cnt = 1;
    dev_led = device_get_binding(LED0_GPIO_CONTROLLER);
    gpio_pin_configure(dev_led, LED0_GPIO_PIN, GPIO_DIR_OUT); 

	/* Echo the received data */
	while (1) {
        gpio_pin_write(dev_led, LED0_GPIO_PIN, cnt % 2);
        cnt++;
        k_sleep(1000);
		read_and_echo_data(dev, (int *) &bytes_read);
	}
}
