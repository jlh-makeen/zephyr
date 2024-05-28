/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_NODELABEL(uart0)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

static uint16_t tx_buf[MSG_SIZE];
static uint8_t tx_len;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint16_t c;

	printk("In IRQ\n");

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		uart_fifo_read_u16(uart_dev, &c, 1);
		rx_buf[rx_buf_pos++] = c;
		return;
	}

	if (uart_irq_tx_ready(uart_dev)) {
		/* send next char */
		if (tx_len > 0) {
			tx_len--;
			uart_fifo_fill_u16(uart_dev, &tx_buf[tx_len], 1);
		} else {
			uart_irq_tx_disable(uart_dev);
		}
		return;
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

/*#define UART_9BIT_RECEIVE*/

void main(void)
{
	if (!uart_dev) {
		printk("UART device not found!\n");
		return;
	}

	/* Configure UART for 9N1 format */
	struct uart_config uart_cfg = {
		.baudrate = 9600,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_9,
	};

	if (uart_configure(uart_dev, &uart_cfg)) {
		printk("UART configuration failed!\n");
		return;
	}

	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
	}
	/* uart_irq_rx_enable(uart_dev);*/


	uint16_t tx_data = 0x123;
	uint16_t rx_data;

	while (1) {
#ifdef UART_9BIT_RECEIVE
		uart_poll_out_u16(uart_dev, tx_data);
		if (uart_poll_in_u16(uart_dev, &rx_data) != 0) {
			printk("Failed to receive data!\n");
		} else {
			printk("Recv 1 %d: %x -> %x\n", tx_data == rx_data, tx_data, rx_data);
		}
#endif
		/*uart_poll_out_u16(uart_dev, tx_data);*/
		tx_data ^= 0x100;


		tx_len = 0;
		for (uint16_t i = 0; i < 8; i++) {
			tx_data = (tx_data & 0xff00) + i;
			tx_buf[tx_len++] = tx_data;
		}

		for (uint16_t i = 0; i < tx_len; i++) {
			printk("TX: %d %x\n", i, tx_buf[i]);
		}
		/* uart_irq_rx_enable(uart_dev);*/
		/* uart_irq_tx_enable(uart_dev);*/

		while (tx_len > 0) {
			k_sleep(K_MSEC(1));
		}


		k_sleep(K_MSEC(1));

		for (uint16_t i = 0; i < rx_buf_pos; i++) {
			printk("RX: %d %x\n", i, rx_buf[i]);
		}

		rx_buf_pos = 0;

	}
}
