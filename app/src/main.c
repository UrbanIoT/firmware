/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "app_version.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

// const struct device *particle_sensor_dev = DEVICE_DT_GET_ONE(seeed_hm3x00);
// struct sensor_value pm1_0, pm2_5, pm10;

// #define CONSOLE_DEVICE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
// #define USB_CDC_ACM_CHECK DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)

#define STATUS_LED_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(STATUS_LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(STATUS_LED_NODE, gpios);

#define LORA_UART_NODE DT_ALIAS(lora_uart)

#if !DT_NODE_HAS_STATUS(LORA_UART_NODE, okay)
#error "Unsupported board: lora_uart devicetree alias is not defined"
#endif

static const struct device *const lora_uart_dev = DEVICE_DT_GET(LORA_UART_NODE);

int main(void)
{
// #if (IS_ENABLED(USB_CDC_ACM_CHECK) && IS_ENABLED(CONFIG_LOG))
// 	const struct device *const console_dev = CONSOLE_DEVICE;
// 	uint32_t dtr_line = 0;

// 	while (!dtr_line)
// 	{
// 		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr_line);
// 		k_sleep(K_MSEC(100));
// 	}
// #endif

	int ret;
	// const struct device *sensor;

	LOG_INF("UrbanIoT firmware %s", APP_VERSION_STR);

	LOG_DBG("Init led0 device!");
	if (!device_is_ready(led.port))
	{
		LOG_ERR("Led0 %s is not ready!", led.port->name);
		return -ENOTSUP;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR("Can't init status led %i!", ret);
		return -ENODEV;
	};

	// if (!device_is_ready(sensor_dev))
	// {
	// 	LOG_ERR("sensor: device not ready.");
	// 	return -ENODEV;
	// }

	if (!device_is_ready(particle_sensor_dev))
	{
		LOG_ERR("particle sensor: device not ready.");
		return -ENODEV;
	}

	while (1)
	{
		LOG_INF("Hello World, from Zephyr! %s", CONFIG_BOARD);
		ret = gpio_pin_toggle_dt(&led);

		// if (sensor_sample_fetch(particle_sensor_dev) == 0)
		// {
		// 	sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_1_0, &pm1_0);
		// 	sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_2_5, &pm2_5);
		// 	sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_10, &pm10);

		// 	LOG_INF("PM 1.0: %d.%02dug/m3 PM 2.5: %d.%02dug/m3 PM 10: %d.%02dug/m3",
		// 			pm1_0.val1, pm1_0.val2,
		// 			pm2_5.val1, pm2_5.val2,
		// 			pm10.val1, pm10.val2);
		// }
		// else
		// {
		// 	LOG_ERR("Cant get data from particle_sensor_dev");
		// }

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
