#include <stdio.h>
#include <string.h>

#include <device.h>
#include <uart.h>
#include <zephyr.h>
#include <uart.h>
#include "usb_acm.h"

static const char *banner1 = "Send characters to the UART device\r\n";
static const char *banner2 = "Characters read:\r\n";

static volatile bool data_transmitted;
static volatile bool data_arrived;
static	char data_buf[64];

#define CONFIG_CDC_ACM_PORT_NAME_O "CDC_ACM"

int usb_acm_setup(struct device** dev)
{
	u32_t baudrate, dtr = 0;
	int ret;
	(*dev) = device_get_binding("CDC_ACM");
	if (!dev) {
		/*printf("CDC ACM device not found\n");*/
		return -1;
	}
	while (1) {
		uart_line_ctrl_get((*dev), LINE_CTRL_DTR, &dtr);
		if (dtr)
			break;
	}
	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set((*dev), LINE_CTRL_DCD, 1);
	if (ret)
	{
		return -2;
		/*printf("Failed to set DCD, ret code %d\n", ret);*/
	}

	ret = uart_line_ctrl_set((*dev), LINE_CTRL_DSR, 1);
	if (ret)
	{
		return -3;
		/*printf("Failed to set DSR, ret code %d\n", ret);*/
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get((*dev), LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
	{
		return -4;
		/*printf("Failed to get baudrate, ret code %d\n", ret);*/
	}
	
	/*printf("Baudrate detected: %d\n", baudrate);*/

	uart_irq_callback_set((*dev), interrupt_handler);
	write_data((*dev), banner1, strlen(banner1));
	write_data((*dev), banner2, strlen(banner2));

	/* Enable rx interrupts */
	uart_irq_rx_enable(*dev);
	return (int) 0;
}

void interrupt_handler(struct device *dev)
{
	uart_irq_update(dev);

	if (uart_irq_tx_ready(dev)) {
		data_transmitted = true;
	}

	if (uart_irq_rx_ready(dev)) {
		data_arrived = true;
	}
}
 
void write_data(struct device *dev, const char *buf, int len)
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

void read_and_echo_data(struct device *dev, int *bytes_read)
{
	char c = '\n';
	
	while (data_arrived == false)
		;

	data_arrived = false;

	/* Read all data and echo it back */
	while ((*bytes_read = uart_fifo_read(dev,
	    (u8_t *)data_buf, sizeof(data_buf)))) {
		write_data(dev, data_buf, *bytes_read);
	}
	write_data(dev, &c, 1);
}

