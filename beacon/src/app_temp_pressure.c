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

#include "app_temp_pressure.h"
#include "app_types.h"

static void lps22hh_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	//process_sample(dev);
}

//#define TESTING
#ifdef TESTING

static void process_sample(const struct device *dev)
{
	static unsigned int obs;
	struct sensor_value pressure, temp;

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printf("Cannot read LPS22HH pressure channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read LPS22HH temperature channel\n");
		return;
	}

	++obs;
	printf("Observation: %u\n", obs);

	/* display pressure */
	printf("Pressure: %.3f kPa\n", sensor_value_to_double(&pressure));

	/* display temperature */
	printf("Temperature: %.2f C\n", sensor_value_to_double(&temp));

}

void testing(void)
{
    const struct device *const dev = DEVICE_DT_GET_ONE(st_lps22hh);

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return;
	}

	if (IS_ENABLED(CONFIG_LPS22HH_TRIGGER)) {
		struct sensor_value attr = {
			.val1 = 1,
		};
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};

		if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
				    SENSOR_ATTR_SAMPLING_FREQUENCY, &attr) < 0) {
			printf("Cannot configure sampling rate\n");
			return;
		}
		if (sensor_trigger_set(dev, &trig, lps22hh_handler) < 0) {
			printf("Cannot configure trigger\n");
			return;
		}
		printk("Configured for triggered collection at %u Hz\n",
		       attr.val1);
	}

    process_sample(dev);

	while (!IS_ENABLED(CONFIG_LPS22HH_TRIGGER)) {
		process_sample(dev);
		k_sleep(K_MSEC(2000));
	}
    k_sleep(K_FOREVER);

}

#endif

int app_temp_pressure_service(temp_pressure_data_t *temp_pressure_data)
{
#ifdef TESTING
    testing();
#else 
	const struct device *const dev = DEVICE_DT_GET_ONE(st_lps22hh);
	struct sensor_value pressure, temp;

    if (IS_ENABLED(CONFIG_LPS22HH_TRIGGER)) {
		struct sensor_value attr = {
			.val1 = 1,
		};
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};

		if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
				    SENSOR_ATTR_SAMPLING_FREQUENCY, &attr) < 0) {
			printf("Cannot configure sampling rate\n");
			return TEMP_PRESSURE_ERROR;
		}
		if (sensor_trigger_set(dev, &trig, lps22hh_handler) < 0) {
			printf("Cannot configure trigger\n");
			return TEMP_PRESSURE_ERROR;
		}
		printk("Configured for triggered collection at %u Hz\n",
		       attr.val1);
	}

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return TEMP_PRESSURE_ERROR;
	}

    printk("Temp & Pressure Init Success!\n");

    k_msleep(10);

	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return TEMP_PRESSURE_ERROR;
	}

    //k_msleep(10);

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printk("Cannot read LPS22HH pressure channel\n");
		return TEMP_PRESSURE_ERROR;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printk("Cannot read LPS22HH temperature channel\n");
		return TEMP_PRESSURE_ERROR;
	}

	/* display pressure */
	printk("Pressure: %.3f kPa\n", sensor_value_to_double(&pressure));
	printk("Temperature: %.2f C\n", sensor_value_to_double(&temp));

    float pressure_f =  sensor_value_to_double(&pressure);
    float temp_f = sensor_value_to_double(&temp);
    int16_t pressure_cast = (int16_t)(pressure_f*100);
    int16_t temp_cast = (int16_t)(temp_f*100);
    temp_pressure_data->pressure = pressure_cast;
    temp_pressure_data->temp = temp_cast;

    printk("Pressure Casted: %i kPa\n",pressure_cast);
	printk("Temperature Casted: %i C\n", temp_cast);
#endif
    return TEMP_PRESSURE_SUCCESS;

}