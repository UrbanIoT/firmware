/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <minmea/minmea.h>

#include "lora.h"
#include "gps.h"
#include "app_version.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static struct gps_data gps_data = {};
static uint8_t lora_data[30];

const struct device *particle_sensor_dev = DEVICE_DT_GET_ONE(seeed_hm3x00);
struct sensor_value pm1_0, pm2_5, pm10;

const struct device *env_sensor_dev = DEVICE_DT_GET_ONE(bosch_bme680);
struct sensor_value temp, press, humidity, gas_res;

#define CONSOLE_DEVICE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
#define USB_CDC_ACM_CHECK DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)

// Status LED
#define STATUS_LED_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(STATUS_LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(STATUS_LED_NODE, gpios);

// LoRa device
#define LORA_UART_NODE DT_ALIAS(lora_uart)

#if !DT_NODE_HAS_STATUS(LORA_UART_NODE, okay)
#error "Unsupported board: lora_uart devicetree alias is not defined"
#endif

static const struct device *const lora_uart_dev = DEVICE_DT_GET(LORA_UART_NODE);

// GPS device
#define GPS_UART_NODE DT_ALIAS(gps_uart)

#if !DT_NODE_HAS_STATUS(GPS_UART_NODE, okay)
#error "Unsupported board: gps_uart devicetree alias is not defined"
#endif

static const struct device *const gps_uart_dev = DEVICE_DT_GET(GPS_UART_NODE);

int main(void)
{
#if (IS_ENABLED(USB_CDC_ACM_CHECK) && IS_ENABLED(CONFIG_LOG))
	const struct device *const console_dev = CONSOLE_DEVICE;
	uint32_t dtr_line = 0;

	while (!dtr_line)
	{
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr_line);
		k_sleep(K_MSEC(100));
	}
#endif

	int ret;

	LOG_INF("UrbanIoT firmware %s", APP_VERSION_STR);

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

	if (!device_is_ready(env_sensor_dev))
	{
		LOG_ERR("sensor: device not ready.");
		return -ENODEV;
	}

	if (!device_is_ready(particle_sensor_dev))
	{
		LOG_ERR("sensor: device not ready.");
		return -ENODEV;
	}

	if (lora_init(lora_uart_dev) < 0)
	{
		LOG_ERR("Can't init lora device!");
		return -ENODEV;
	}

	lora_join();

	if (gps_init(gps_uart_dev) < 0)
	{
		LOG_ERR("Can't init gps device!");
		return -ENODEV;
	}

	// if (!device_is_ready(particle_sensor_dev))
	// {
	// 	LOG_ERR("particle sensor: device not ready.");
	// 	return -ENODEV;
	// }

	while (1)
	{
		LOG_INF("Hello World, from Zephyr! %s", CONFIG_BOARD);
		ret = gpio_pin_toggle_dt(&led);

		if (sensor_sample_fetch(env_sensor_dev) == 0)
		{
			sensor_channel_get(env_sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(env_sensor_dev, SENSOR_CHAN_PRESS, &press);
			sensor_channel_get(env_sensor_dev, SENSOR_CHAN_HUMIDITY, &humidity);
			sensor_channel_get(env_sensor_dev, SENSOR_CHAN_GAS_RES, &gas_res);

			LOG_INF("T: %d.%06d°C P: %d.%06dkPa H: %d.%06d% G: %d.%06dohm",
					temp.val1, temp.val2,
					press.val1, press.val2,
					humidity.val1, humidity.val2,
					gas_res.val1, gas_res.val2);
		}
		else
		{
			LOG_ERR("Cant get data from env_sensor_dev");
		}

		if (sensor_sample_fetch(particle_sensor_dev) == 0)
		{
			sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_1_0, &pm1_0);
			sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_2_5, &pm2_5);
			sensor_channel_get(particle_sensor_dev, SENSOR_CHAN_PM_10, &pm10);

			LOG_INF("PM 1.0: %d.%06dug/m3 PM 2.5: %d.%06dug/m3 PM 10: %d.%06d%ug/m3",
					pm1_0.val1, pm1_0.val2,
					pm2_5.val1, pm2_5.val2,
					pm10.val1, pm10.val2);
		}
		else
		{
			LOG_ERR("Cant get data from particle_sensor_dev");
		}

		gps_get_data(&gps_data);

		lora_data[0] = 0x00;
		// lora_data[0] |= 1 << 0; // UV index sensor
		lora_data[0] |= 1 << 1; // Temperature sensor
		lora_data[0] |= 0 << 2; // Pressure sensor
		lora_data[0] |= 1 << 3; // Humidity sensor
		// lora_data[0] |= 0 << 4; // IAQ sensor
		lora_data[0] |= 1 << 5; // dust/particle sensor
		// lora_data[0] |= 0 << 6; // not defined
		// lora_data[0] |= 0 << 7; // not defined

		lora_data[1] = 0x00; // not defined
		lora_data[2] = 0x00; // not defined

		// Battery
		lora_data[3] = 0x00;

		uint32_t latitudeBinary = ((gps_data.lat + 90) / 180) * 16777215;
		lora_data[4] = (latitudeBinary >> 16) & 0xFF;
		lora_data[5] = (latitudeBinary >> 8) & 0xFF;
		lora_data[6] = latitudeBinary & 0xFF;

		uint32_t longitudeBinary = ((gps_data.lng + 180) / 360) * 16777215;
		lora_data[7] = (longitudeBinary >> 16) & 0xFF;
		lora_data[8] = (longitudeBinary >> 8) & 0xFF;
		lora_data[9] = longitudeBinary & 0xFF;

		uint16_t speedKmhGps = gps_data.speed;
		lora_data[14] = (speedKmhGps >> 8) & 0xFF;
		lora_data[15] = speedKmhGps & 0xFF;

		lora_data[18] = temp.val1 & 0xFF;
		lora_data[19] = (press.val1 >> 8) & 0xFF;
		lora_data[20] = press.val1 & 0xFF;
		lora_data[21] = humidity.val1 & 0xFF;

		lora_data[24] = (pm1_0.val1 >> 8) & 0xFF;
		lora_data[25] = pm1_0.val1 & 0xFF;
		lora_data[26] = (pm2_5.val1 >> 8) & 0xFF;
		lora_data[27] = pm2_5.val1 & 0xFF;
		lora_data[28] = (pm10.val1 >> 8) & 0xFF;
		lora_data[29] = pm10.val1 & 0xFF;

		lora_senddata(&lora_data[0], sizeof(lora_data));

		k_sleep(K_SECONDS(10));
	}

	return 0;
}
