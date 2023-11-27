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

int app_fram_read_field(uint8_t field, uint8_t *dat)
{
	int ret;
	uint16_t addr = 0;
	uint32_t length = 0;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	if(field == EV_CTR){
		addr = FRAM_COUNTER_ADDR;
		length = FRAM_COUNTER_NUM_BYTES;
	}
	else if(field == SER_NUM){
		addr = SER_NUM_ADDR;
		length = SER_NUM_BYTES;
	}
	else if (field == TYPE){
		addr = TYPE_ADDR;
		length = TYPE_NUM_BYTES;
	}
	else if (field == EV_INT){
		addr = EV_INT_ADDR;
		length = EV_INT_NUM_BYTES;
	}
	else if (field == EV_MAX){
		addr = EV_MAX_ADDR;
		length = EV_MAX_NUM_BYTES;
	}
	else if(field == EV_SLP){
		addr = EV_SLP_ADDR;
		length = EV_SLP_NUM_BYTES;
	}
	else if(field == IN_SLP){
		addr = IN_SLP_ADDR;
		length = IN_SLP_NUM_BYTES;
	}
	else if (field == ISL9122){
		addr = ISL9122_ADDR;
		length = ISL9122_NUM_BYTES;
	}
	else if (field == POL_MET){
		addr = POL_MET_ADDR;
		length = POL_MET_NUM_BYTES;
	}
	else if (field == ENCRYPTED_KEY){
		addr = ENCRYPTED_KEY_ADDR;
		length = ENCRYPTED_KEY_NUM_BYTES;
	}
	else if (field == TX_DBM){
		addr = TX_DBM_ADDR;
		length = TX_DBM_NUM_BYTES;
	}
	else if (field == NAME){
		addr = NAME_ADDR;
		length = NAME_NUM_BYTES;
	}

	ret = i2c_read_bytes(i2c_dev, addr, dat, length);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk(">>Read the data successfully\n");
	}

	return FRAM_SUCCESS;
}

int app_fram_write_field(uint8_t field, uint8_t *dat)
{
	int ret;
	uint16_t addr = 0;
	uint32_t num_bytes = 0;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	switch(field)
	{
		case EV_CTR:
			addr = FRAM_COUNTER_ADDR;
			num_bytes = FRAM_COUNTER_NUM_BYTES;
			break;
		case SER_NUM:
			addr = SER_NUM_ADDR;
			num_bytes = SER_NUM_BYTES;
			break;
		case TYPE:
			addr = TYPE_ADDR;
			num_bytes = TYPE_NUM_BYTES;
			break;
		case EV_INT:
			addr = EV_INT_ADDR;
			num_bytes = EV_INT_NUM_BYTES;
			break;
		case EV_MAX:
			addr = EV_MAX_ADDR;
			num_bytes = EV_MAX_NUM_BYTES;
			break;
		case EV_SLP:
			addr = EV_SLP_ADDR;
			num_bytes = EV_SLP_NUM_BYTES;
			break;
		case IN_SLP:
			addr = IN_SLP_ADDR;
			num_bytes = IN_SLP_NUM_BYTES;
			break;
		case ISL9122:
			addr = ISL9122_ADDR;
			num_bytes = ISL9122_NUM_BYTES;
			break;
		case POL_MET:
			addr = POL_MET_ADDR;
			num_bytes = POL_MET_NUM_BYTES;
			break;
		case ENCRYPTED_KEY:
			addr = ENCRYPTED_KEY_ADDR;
			num_bytes = ENCRYPTED_KEY_NUM_BYTES;
			break;
		case TX_DBM:
			addr = TX_DBM_ADDR;
			num_bytes = TX_DBM_NUM_BYTES;
			break;
		case NAME:
			addr = NAME_ADDR;
			num_bytes = NAME_NUM_BYTES;
			break;
	}

	ret = i2c_write_bytes(i2c_dev, addr, dat, num_bytes);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk(">>Write the data successfully\n");
	}

	return FRAM_SUCCESS;
}


int app_fram_read_data(fram_data_t *dat)
{
	int ret;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	ret = i2c_read_bytes(i2c_dev, FRAM_COUNTER_ADDR, dat, sizeof(fram_data_t));
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} 
	return FRAM_SUCCESS;

}

int app_fram_write_data(fram_data_t *dat)
{
	int ret;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	ret = i2c_write_bytes(i2c_dev, FRAM_COUNTER_ADDR, dat, sizeof(fram_data_t));
	if (ret) {
		printk("Error writing from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk(">> ------- Writing FRAM Data -------\n");
		printk(">>[FRAM INFO]->Event Counter: 0x%08X\n", dat->event_counter);
		printk(">>[FRAM INFO]->Serial Number: 0x%08X\n", dat->serial_number);
		printk(">>[FRAM INFO]->Device Type: %d\n", dat->type);
		printk(">>[FRAM INFO]->Frame Interval: %d ms\n", dat->event_inteval);
		printk(">>[FRAM INFO]->Frame Maximum Number: %d\n", dat->event_max_limits);
		printk(">>[FRAM INFO]->Minimum Sleeping Interval: %d\n", dat->sleep_min_interval);
		printk(">>[FRAM INFO]->Sleep time After Wake up: %d\n", dat->sleep_after_wake);
		printk(">>[FRAM INFO]->Voltage of ISL9122: %d\n", dat->u8_voltsISL9122);
		printk(">>[FRAM INFO]->POL Method: %d\n", dat->u8_POLmethod);
		printk(">>[FRAM INFO]->Encrypted Key: ");
		for (uint8_t i = 0; i < ENCRYPTED_KEY_NUM_BYTES; i++){
			printk("0x%02X,", dat->encrypted_key[i]);
		}
		printk("\n");
		printk(">>[FRAM INFO]->TX dBM 10: %d\n", dat->tx_dbm_10);
		printk(">>[FRAM INFO]->cName: %s\n", dat->cName);
	}

	return FRAM_SUCCESS;

}

int app_fram_read_counter(fram_data_t *dat)
{
	int ret;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	ret = i2c_read_bytes(i2c_dev, FRAM_COUNTER_ADDR, dat, FRAM_COUNTER_NUM_BYTES);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk(">>[FRAM INFO]->Frame Counter: 0x%08X\n", dat->event_counter);
	}

	return FRAM_SUCCESS;
}

int app_fram_write_counter(fram_data_t *dat)
{
	int ret;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

	ret = i2c_write_bytes(i2c_dev, FRAM_COUNTER_ADDR, dat, FRAM_COUNTER_NUM_BYTES);
	if (ret) {
		printk("Error writing from FRAM! error code (%d)\n", ret);
		return FRAM_ERROR;
	} else {
		printk(">>[FRAM INFO]->Frame Counter: 0x%08X\n", dat->event_counter);
	}

	return FRAM_SUCCESS;
}

int app_fram_service(uint32_t *counter)
{
	int ret;
    u32_u8_t fram_data;
    fram_data.u32 = 0;

	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C Device is not ready.\n");
		return FRAM_ERROR;
	}

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
