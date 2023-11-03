/** @file app_accel.c
* 
* @brief Accelerometer interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl. All rights reserved.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/i2c.h>

#include "app_accel.h"
#include "app_types.h"

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};

int app_accel_service(accel_data_t *accel_data)
{
    const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(accel));
    if (!device_is_ready(accel_dev)) {
		printf("Device %s is not ready\n", accel_dev->name);
		return ACCEL_ERROR;
	}
    printf("I2C Accel Success!\n");

    int ret;
	struct sensor_value accel[3];
    
	ret = sensor_sample_fetch(accel_dev);
	if (ret < 0) {
		printk("%s: sensor_sample_fetch() failed: %d\n", accel_dev->name, ret);
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		ret = sensor_channel_get(accel_dev, channels[i], &accel[i]);
		if (ret < 0) {
			printk("%s: sensor_channel_get(%c) failed: %d\n", accel_dev->name, 'X' + i, ret);
			return ret;
		}
	}

    printk("[m/s^2]: (%12.6f, %12.6f, %12.6f)\n", sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));

    float val_x = sensor_value_to_double(&accel[0]);
    float val_y = sensor_value_to_double(&accel[1]);
    float val_z = sensor_value_to_double(&accel[2]);
    int16_t val_x_cast = (int16_t)(val_x*1000);
    int16_t val_y_cast = (int16_t)(val_y*1000);
    int16_t val_z_cast = (int16_t)(val_z*1000);

    printk("x: %i, y: %i, z: %i\n", val_x_cast, val_y_cast, val_z_cast);

    accel_data->x_accel = val_x_cast;
    accel_data->y_accel = val_y_cast;
    accel_data->z_accel = val_z_cast;

    return ACCEL_SUCCESS;
}
