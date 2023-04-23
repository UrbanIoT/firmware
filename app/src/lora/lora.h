#ifndef LORA_H
#define LORA_H

#include <zephyr/device.h>

int lora_init(const struct device *dev);
int lora_join();
int lora_senddata(const uint8_t* data, size_t size);

#endif