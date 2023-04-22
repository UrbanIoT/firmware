#define DT_DRV_COMPAT seeed_hm3x00

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(HM3X00, CONFIG_SENSOR_LOG_LEVEL);

struct hm3x00_registers
{
	uint16_t reserved;		 // reserved
	uint16_t sensorNo;		 // Sensor number
	uint16_t pm1_0_cf1;		 // PM1.0 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm2_5_cf1;		 // PM2.5 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm10_0_cf1;	 // PM10 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm1_0_ae;		 // PM1.0 concentration （Atmospheric environment）unit μg/ m3
	uint16_t pm2_5_ae;		 // PM2.5 concentration （Atmospheric environment）unit μg/ m3
	uint16_t pm10_0_ae;		 // PM10 concentration （Atmospheric environment）unit μg/ m3
	uint16_t particles_0_3;	 // the number of particles with diameter 0.3um or above in 1 liter of air
	uint16_t particles_0_5;	 // the number of particles with diameter 0.5um or above in 1 liter of air
	uint16_t particles_1_0;	 // the number of particles with diameter 1.0um or above in 1 liter of air
	uint16_t particles_2_5;	 // the number of particles with diameter 2.5um or above in 1 liter of air
	uint16_t particles_5_0;	 // the number of particles with diameter 5.0um or above in 1 liter of air
	uint16_t particles_10_0; // the number of particles with diameter 10um or above in 1 liter of air
	uint8_t checksum;		 // checksum
} __packed;

struct hm3x00_config
{
	struct i2c_dt_spec i2c;
};

struct hm3x00_data
{
	uint16_t pm1_0_cf1;	   // PM1.0 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm2_5_cf1;	   // PM2.5 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm10_0_cf1;   // PM10 concentration（CF=1 ，Standard particulate）unit μg/ m3
	uint16_t pm1_0_ae;	   // PM1.0 concentration （Atmospheric environment）unit μg/ m3
	uint16_t pm2_5_ae;	   // PM2.5 concentration （Atmospheric environment）unit μg/ m3
	uint16_t pm10_0_ae;	   // PM10 concentration （Atmospheric environment）unit μg/ m3
	bool checksum_correct; // Checksum is correct
};

static int hm3x00_sample_fetch(const struct device *dev,
							   enum sensor_channel chan)
{
	struct hm3x00_data *drv_data = dev->data;
	const struct hm3x00_config *config = dev->config;
	struct hm3x00_registers buf;
	// struct i2c_msg msg;
	int ret;
	uint8_t checksum = 0;

	// __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	// msg.buf = (uint8_t *)&buf;
	// msg.len = sizeof(struct hm3x00_registers);
	// msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_write_dt(&config->i2c, (uint8_t *)&buf, sizeof(uint8_t));

	if (ret < 0)
	{
		LOG_DBG("Can't set device to i2c communication mode (%d)!", ret);
		// retry++;
		// k_sleep(K_MSEC(10));
		return -EIO;
	}

	ret = i2c_burst_read_dt(&config->i2c, 0x1D, (uint8_t *)&buf, sizeof(struct hm3x00_registers));
	// ret = i2c_transfer_dt(&config->i2c, &msg, 1);
	if (ret < 0)
	{
		LOG_ERR("Failed to read registers data [%d].", ret);
		return -EIO;
	}

	// Calculate checksum
	for (uint8_t i = 0; i < 28; i++)
	{
		checksum += *(((uint8_t *)&buf) + i);
	}

	if (checksum != buf.checksum)
	{
		LOG_ERR("Checksum not correct!");
		drv_data->checksum_correct = false;
		return -EIO;
	}

	drv_data->pm1_0_cf1 = sys_be16_to_cpu(buf.pm1_0_cf1);
	drv_data->pm2_5_cf1 = sys_be16_to_cpu(buf.pm2_5_cf1);
	drv_data->pm10_0_cf1 = sys_be16_to_cpu(buf.pm10_0_cf1);

	drv_data->pm1_0_ae = sys_be16_to_cpu(buf.pm1_0_ae);
	drv_data->pm2_5_ae = sys_be16_to_cpu(buf.pm2_5_ae);
	drv_data->pm10_0_ae = sys_be16_to_cpu(buf.pm10_0_ae);

	drv_data->checksum_correct = true;

	return 0;
}

static int hm3x00_channel_get(const struct device *dev,
							  enum sensor_channel chan,
							  struct sensor_value *val)
{
	struct hm3x00_data *drv_data = dev->data;

	if (!drv_data->checksum_correct)
	{
		LOG_ERR("Checksum not correct! Can't get sample!");
		return -EIO;
	}

#if HM3X00_RETURN_VALUES == 0 // CF=1，Standard particulate
	switch (chan)
	{
	case SENSOR_CHAN_PM_1_0:
		val->val1 = drv_data->pm1_0_cf1;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_2_5:
		val->val1 = drv_data->pm2_5_cf1;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_10:
		val->val1 = drv_data->pm10_0_cf1;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}
#elif HM3X00_RETURN_VALUES == 1 // Atmospheric environment
	switch (chan)
	{
	case SENSOR_CHAN_PM_1_0:
		val->val1 = drv_data->pm1_0_ae;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_2_5:
		val->val1 = drv_data->pm2_5_ae;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_PM_10:
		val->val1 = drv_data->pm10_0_ae;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}
#else
#error "Unknown HM3X00_RETURN_VALUES defined! Please check your configuration!"
#endif

	return 0;
}

static const struct sensor_driver_api hm3x00_driver_api = {
	.sample_fetch = &hm3x00_sample_fetch,
	.channel_get = &hm3x00_channel_get,
};

static int hm3x00_init(const struct device *dev)
{
	const struct hm3x00_config *config = dev->config;
	int ret, retry;
	uint8_t buf = 0x88;

	if (!device_is_ready(config->i2c.bus))
	{
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	while (retry < 5 && ret < 0)
	{
		ret = i2c_write_dt(&config->i2c, (uint8_t *)&buf, sizeof(uint8_t));

		if (ret < 0)
		{
			LOG_DBG("Can't set device to i2c communication mode (%d)! Retry no %d", ret, retry);
			retry++;
			k_sleep(K_MSEC(10));
		}
	}

	if (ret < 0)
	{
		LOG_ERR("Can't set device to i2c communication mode (%d)!", ret);
		return ret;
	}

	return 0;
}

#define HM3X00_DEFINE(inst)                                    \
	static struct hm3x00_data hm3x00_data_##inst;              \
                                                               \
	static const struct hm3x00_config hm3x00_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                     \
	};                                                         \
                                                               \
	DEVICE_DT_INST_DEFINE(inst, hm3x00_init, NULL,             \
						  &hm3x00_data_##inst,                 \
						  &hm3x00_config_##inst, POST_KERNEL,  \
						  CONFIG_SENSOR_INIT_PRIORITY, &hm3x00_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HM3X00_DEFINE)