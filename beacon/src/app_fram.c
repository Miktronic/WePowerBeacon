/** @file fram.c
* 
* @brief FRAM interface
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
#include <zephyr/drivers/i2c.h>

#include "app_fram.h"
#include "app_types.h"

// FRAM Defines
#define FRAM_COUNTER_ADDR 0x00
#define FRAM_COUNTER_NUM_BYTES 4
#define FRAM_I2C_ADDR 0x50

int i2c_write_bytes(const struct device *i2c_dev, uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;
    
	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}

int i2c_read_bytes(const struct device *i2c_dev, uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}

int app_fram_service(uint32_t *counter)
{
    const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    uint32_t dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
    int res = i2c_configure(i2c_dev, dev_config);
    if(res != 0)
    {
        return FRAM_ERROR;
    }
	int ret;
    u32_u8_t fram_data;
    fram_data.u32 = 0;

	if (!device_is_ready(i2c_dev)) {
		printk("FRAM: I2C Device is not ready.\n");
		return FRAM_ERROR;
	}
    printk("FRAM I2C Init Success!\n");

    // THIS WILL CLEAR THE DEVICE FRAM //
    // ret = i2c_write_bytes(i2c_dev, FRAM_COUNTER_ADDR, &fram_data.u8[0], FRAM_COUNTER_NUM_BYTES);
	// if (ret) {
	// 	printk("Error writing to FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Wrote %u to address 0x00.\n", fram_data.u32);
	// }
    // END RESET //

    // Read the 4 data bytes at this address
	ret = i2c_read_bytes(i2c_dev, FRAM_COUNTER_ADDR, &fram_data.u8[0], FRAM_COUNTER_NUM_BYTES);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk("Read %u (%08X)from address %u.\n", fram_data.u32, fram_data.u32, FRAM_COUNTER_ADDR);
	}

    // Increment count
    fram_data.u32++;

	ret = i2c_write_bytes(i2c_dev, FRAM_COUNTER_ADDR, &fram_data.u8[0], FRAM_COUNTER_NUM_BYTES);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk("Wrote %u (%08X) to address %u.\n", fram_data.u32, fram_data.u32, FRAM_COUNTER_ADDR);
	}

    *counter = fram_data.u32;

    return FRAM_SUCCESS;
}
