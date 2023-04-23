#ifndef GPS_H
#define GPS_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct gps_data
{
    float lat;
    float lng;
    float alt;
    float speed;
};

int gps_init(const struct device *dev);
int gps_get_data(struct gps_data *data);

#endif