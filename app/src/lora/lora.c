#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include <string.h>
#include <stdio.h>

#include "lora.h"

LOG_MODULE_REGISTER(lora);

#define LORA_UART_MSG_SIZE 128
static char rx_buf_lora_msg_uart[LORA_UART_MSG_SIZE];
static char tx_buf_lora_msg_uart[LORA_UART_MSG_SIZE];
static int rx_buf_lora_msg_uart_pos;
static const struct device *lora_uart_dev;

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(lora_uart_msgq, LORA_UART_MSG_SIZE, 10, 4);

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void lora_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(dev))
    {
        return;
    }

    while (uart_irq_rx_ready(dev))
    {

        uart_fifo_read(dev, &c, 1);

        if (rx_buf_lora_msg_uart_pos > 1 && (c == '\n' && rx_buf_lora_msg_uart[rx_buf_lora_msg_uart_pos - 1] == '\r'))
        {
            /* terminate string */
            rx_buf_lora_msg_uart[rx_buf_lora_msg_uart_pos] = '\n';
            rx_buf_lora_msg_uart[rx_buf_lora_msg_uart_pos + 1] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&lora_uart_msgq, &rx_buf_lora_msg_uart, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_lora_msg_uart_pos = 0;

            LOG_INF("Answer: %s", rx_buf_lora_msg_uart);
        }
        else if (rx_buf_lora_msg_uart_pos < (sizeof(rx_buf_lora_msg_uart) - 1))
        {
            rx_buf_lora_msg_uart[rx_buf_lora_msg_uart_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

int lora_init(const struct device *dev)
{
    if (!device_is_ready(dev))
    {
        LOG_ERR("lora sensor: device not ready.");
        return -ENODEV;
    }

    /* configure interrupt and callback to receive data */
    uart_irq_callback_user_data_set(dev, lora_uart_cb, NULL);
    uart_irq_rx_enable(dev);

    lora_uart_dev = dev;

    return 0;
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_lora_uart(char *buf)
{
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(lora_uart_dev, buf[i]);
    }

    LOG_INF("Request: %s", buf);
}

int parse_lora_result(const char *expected, k_timeout_t timeout)
{
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, timeout) != -EAGAIN)
    {
        if (strcmp(expected, tx_buf_lora_msg_uart) == 0)
        {
            return 0;
        }
    }

    LOG_ERR("LoRa parse result timeout!");
    return -EAGAIN;
}

int lora_join()
{
    print_lora_uart("AT\r\n");
    if (parse_lora_result("+AT: OK\r\n", K_MSEC(100)) != 0)
    {
        LOG_ERR("Can't access LoRa device over UART!");
        return -EIO;
    }

    print_lora_uart("AT+ID=DevEui\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    print_lora_uart("AT+ID=AppEui\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    //TODO: Don't put keys in code! :(
    print_lora_uart("AT+KEY=APPKEY,\"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\"\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    print_lora_uart("AT+DR=EU868\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    print_lora_uart("AT+CH=NUM,0-2\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    print_lora_uart("AT+MODE=LWOTAA\r\n");
    while (k_msgq_get(&lora_uart_msgq, &tx_buf_lora_msg_uart, K_MSEC(100)) == 0)
    {
    }

    print_lora_uart("AT+JOIN\r\n");
    if (parse_lora_result("+JOIN: Done\r\n", K_SECONDS(30)) != 0)
    {
        // TODO: +JOIN: Join failed
        LOG_ERR("LoRa join error!");
        return -EIO;
    }

    LOG_INF("LoRa join complete!");
    return 0;
}

int lora_senddata(const uint8_t *data, size_t size)
{
    char buffer[256];
    int n = sprintf(buffer, "AT+MSGHEX=\"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\"\r\n",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    data[4],
                    data[5],
                    data[6],
                    data[7],
                    data[8],
                    data[9],
                    data[10],
                    data[11],
                    data[12],
                    data[13],
                    data[14],
                    data[15],
                    data[16],
                    data[17],
                    data[18],
                    data[19],
                    data[20],
                    data[21],
                    data[22],
                    data[23],
                    data[24],
                    data[25],
                    data[26],
                    data[27],
                    data[28],
                    data[29]);
    buffer[n+1] = '\0';
    
    print_lora_uart(buffer);
    if (parse_lora_result("+MSGHEX: Done\r\n", K_SECONDS(30)) != 0)
    {
        LOG_ERR("LoRa send error!");
        return -EIO;
    }
    
    LOG_INF("LoRa send complete!");
    return 0;
}