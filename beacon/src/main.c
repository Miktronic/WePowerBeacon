/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <stdlib.h>
#include <math.h>

#include "app_types.h"
#include "app_fram.h"
#include "app_encryption.h"
#include "app_accel.h"
#include "app_temp_pressure.h"


#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define MANUF_DATA_CUSTOM_START_INDEX   2
#define PAYLOAD_PACKET_COUNTER_INDEX    18
#define PAYLOAD_DEVICE_ID_INDEX         19
#define PAYLOAD_STATUS_BYTE_INDEX       21
#define PAYLOAD_RFU_BYTE_INDEX          22
#define PAYLOAD_ENCRYPTION_STATUS_ENC   0
#define PAYLOAD_ENCRYPTION_STATUS_CLEAR 1

#define BLE_ADV_INTERVAL_MIN            (32) // N * 0.625. corresponds to dec. 32; 32*0.625 = 20ms
#define BLE_ADV_INTERVAL_MAX            (36) // N * 0.625. corresponds to dec. 36; 36*0.625 = 25ms
#define BLE_ADV_TIMEOUT                 (1)  // N * 10ms for advertiser timeout
#define BLE_ADV_EVENTS                  (1)

void advertising_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(advertising_timer, advertising_timer_handler, NULL);

static we_power_data_ble_adv_t we_power_data;
static struct bt_data we_power_adv_data;

#define DEVICE_ID 3
static u16_u8_t device_id;

static struct bt_le_ext_adv *adv;

//                                           |--------- ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ------------ | cnt    id    id  status RFU
static uint8_t manf_data[23] = { 0x50, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F};
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manf_data, 23),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x50, 0x57)
};

static void adv_sent(struct bt_le_ext_adv *instance,
		     struct bt_le_ext_adv_sent_info *info)
{	
	printk("Advertising stopped. num_sent: %d ", info->num_sent);
	manf_data[PAYLOAD_PACKET_COUNTER_INDEX] = manf_data[PAYLOAD_PACKET_COUNTER_INDEX]+1;
	//k_work_submit(&start_advertising_worker);
}

static int create_advertising(void)
{
	int err;
	struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(
				     BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_USE_NAME,
				     BLE_ADV_INTERVAL_MIN,
				     BLE_ADV_INTERVAL_MAX,
				     NULL);

    static const struct bt_le_ext_adv_cb adv_cb = {
		.sent = adv_sent,
	};

	err = bt_le_ext_adv_create(&param, &adv_cb, &adv);
	if (err) {
		printk("Failed to create advertiser set (%d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	return 0;
}

void advertising_timer_handler(struct k_timer *timer)
{
	int err;

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (%d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_PARAM(BLE_ADV_TIMEOUT, BLE_ADV_EVENTS));
	if (err) {
		printk("Failed to start advertising set (%d)\n", err);
		return;
	}

	printk("Advertiser %p set started\n", adv);
}

static void bt_ready(void)
{
	int err = 0;

	printk("Bluetooth initialized\n");

	err = create_advertising();
	if (err) {
		printk("Advertising failed to create (err %d)\n", err);
		return;
	}
    k_timer_start(&advertising_timer, K_MSEC(20), K_MSEC(20));	
}
void main(void)
{
    int err;
    printk("Starting WePower gemns BLE Beacon Demo!\n"); 
    /*
    // Set the ID, we do this in the clear AND encrypted, for verification
    device_id.u16 = DEVICE_ID;
    we_power_data.data_fields.id = device_id;

    // Get the counter out of fram and increment it
    uint32_t fram_counter = 0;
    if(app_fram_service(&fram_counter) == FRAM_SUCCESS)
    {
        // Success! Add it to the payload
        we_power_data.data_fields.fram_counter.u32 = fram_counter;
    }
    else
    {
         we_power_data.data_fields.fram_counter.u32 = 0xFFFFFFFF;
    }
    
    // Get the accelerometer data
    static accel_data_t accel_data;
    if(app_accel_service(&accel_data) == ACCEL_SUCCESS)
    {
        // Success!
        we_power_data.data_fields.accel_x.i16 = accel_data.x_accel;
        we_power_data.data_fields.accel_y.i16 = accel_data.y_accel;
        we_power_data.data_fields.accel_z.i16 = accel_data.z_accel;
    }
    else
    {
        we_power_data.data_fields.accel_x.i16 = 0xFFFF;
        we_power_data.data_fields.accel_y.i16 = 0xFFFF;
        we_power_data.data_fields.accel_z.i16 = 0xFFFF;
    }

    // Get temp and pressure
    temp_pressure_data_t temp_pressure_data;
    if(app_temp_pressure_service(&temp_pressure_data) == TEMP_PRESSURE_SUCCESS)
    {
        we_power_data.data_fields.pressure.i16 = temp_pressure_data.pressure;
        we_power_data.data_fields.temp.i16 = temp_pressure_data.temp;
    }
    else
    {
        we_power_data.data_fields.pressure.i16 = 0xFFFF;
        we_power_data.data_fields.temp.i16 = 0xFFFF;
    }

    // Handle encryption
    static uint8_t cipher_text[DATA_SIZE_BYTES];

#define ENCRYPT
#ifdef ENCRYPT
    // Not we want to encrypt the data
    if(app_encrypt_payload(we_power_data.data_bytes, DATA_SIZE_BYTES, cipher_text, DATA_SIZE_BYTES) == ENCRYPTION_ERROR)
    {
        memcpy(cipher_text, we_power_data.data_bytes, 16);
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
    }
    else
    {
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_ENC;
    }
#else
    memcpy(cipher_text, we_power_data.data_bytes, 16);
    manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
#endif

    printk("Payload - Cleartext: ");
    for(int i = 0; i < DATA_SIZE_BYTES; i++)
    {
        printk("%02X ", we_power_data.data_bytes[i]);
    }
    printk("\n");

    printk("Payload - Encrypted: ");
    for(int i = 0; i < DATA_SIZE_BYTES; i++)
    {
        printk("%02X ", cipher_text[i]);
    }
    printk("\n");

    // Build the BLE adv data
    memcpy(&manf_data[MANUF_DATA_CUSTOM_START_INDEX], cipher_text, DATA_SIZE_BYTES);
    memcpy(&manf_data[PAYLOAD_DEVICE_ID_INDEX], device_id.u8, 2);

    we_power_adv_data.type = BT_DATA_SVC_DATA16;
    we_power_adv_data.data = manf_data;
    we_power_adv_data.data_len = sizeof(manf_data);
    */
    // Init and run the BLE
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	
	bt_ready();
}
