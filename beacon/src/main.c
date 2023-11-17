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
#include <hal/nrf_gpio.h>

#include "app_types.h"
#include "app_fram.h"
#include "app_encryption.h"
#include "app_accel.h"
#include "app_temp_pressure.h"
#include "zephyr/drivers/gpio.h"
#include <zephyr/init.h>


#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define DEVICE_ID 3

#define MANUF_DATA_CUSTOM_START_INDEX   2
#define PAYLOAD_TX_REPEAT_COUNTER_INDEX 18
#define PAYLOAD_DEVICE_ID_INDEX         19
#define PAYLOAD_STATUS_BYTE_INDEX       21
#define PAYLOAD_RFU_BYTE_INDEX          22
#define PAYLOAD_ENCRYPTION_STATUS_ENC   0
#define PAYLOAD_ENCRYPTION_STATUS_CLEAR 1
#define FRAME_LENGTH                    23

#define BLE_ADV_INTERVAL_MIN            (32) // N * 0.625. corresponds to dec. 32; 32*0.625 = 20ms
#define BLE_ADV_INTERVAL_MAX            (36) // N * 0.625. corresponds to dec. 36; 36*0.625 = 22.5ms
#define BLE_ADV_TIMEOUT                 (0)  // N * 10ms for advertiser timeout
#define BLE_ADV_EVENTS                  (1)


#define PACKET_INTERVAL_MIN (10) // ms repeat rate
#define PACKET_INTERVAL_MAX (50) // ms repeat rate

#define LOGS (1) // if printk doesn't handle it use if(LOGS) printk();

#define POL_GPIO_PIN                    13
//static const struct gpio_dt_spec pol_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(pol), gpios);

static void timer_event_handler(struct k_timer *dummy);
K_TIMER_DEFINE(timer_event, timer_event_handler, NULL);

static struct k_work start_advertising_worker;
static struct k_work_delayable update_frame_work;

static we_power_data_ble_adv_t we_power_data;
static struct bt_data we_power_adv_data;
static struct bt_le_ext_adv *adv;

// variable to store the FRAM data
static fram_data_t fram_data = {0};
static u16_u8_t device_id;
static uint8_t TX_Repeat_Counter;

uint32_t serial_number = DEVICE_ID;
uint8_t  type = 1;
uint8_t  event_inteval = 20;
uint16_t event_max_limits = 240;
uint16_t sleep_min_interval = 200;
uint16_t sleep_after_wake = 0; // assumes sleep(0) is just a yield()

//                                                    |--------- ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ------------ |  cnt    id    id  status RFU
static uint8_t manf_data[FRAME_LENGTH] = { 0x50, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F};
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manf_data, FRAME_LENGTH),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x50, 0x57)
};

static void init_gpio_fn(void)
{
    nrf_gpio_cfg_output(POL_GPIO_PIN);
    nrf_gpio_pin_toggle(POL_GPIO_PIN);
}

SYS_INIT(init_gpio_fn, POST_KERNEL, 0);

static void adv_sent(struct bt_le_ext_adv *instance,
		     struct bt_le_ext_adv_sent_info *info)
{	
    nrf_gpio_pin_toggle(POL_GPIO_PIN);
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

void start_advertising(struct k_work *work)
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
    
}

static void timer_event_handler(struct k_timer *dummy)
{
    if(LOGS){
        printk("Manufacturer Data: ");
        for (uint8_t i = 0; i < FRAME_LENGTH; i++){
            printk("%02X ", manf_data[i]);
        }
        printk("\n");
    }

    TX_Repeat_Counter++;

    if (TX_Repeat_Counter < fram_data.event_max_limits) // starts at 0, we send 1 if max is 1 by incrementing after the test.
    {
        
        manf_data[PAYLOAD_TX_REPEAT_COUNTER_INDEX] = TX_Repeat_Counter;
        k_work_submit(&start_advertising_worker);
    }
    else {
        printk(">>> Sent maximum packets.\n");
        TX_Repeat_Counter = 0;
        k_timer_stop(&timer_event);
        k_work_schedule(&update_frame_work, K_MSEC(fram_data.sleep_min_interval));
    }
}

static void measure_sensors(we_power_data_ble_adv_t *we_power_data)
{
     // Get the accelerometer data
    static accel_data_t accel_data;
    if(app_accel_service(&accel_data) == ACCEL_SUCCESS)
    {
        // Success!
        we_power_data->data_fields.accel_x.i16 = accel_data.x_accel;
        we_power_data->data_fields.accel_y.i16 = accel_data.y_accel;
        we_power_data->data_fields.accel_z.i16 = accel_data.z_accel;
    }
    else
    {
        we_power_data->data_fields.accel_x.i16 = 0xFFFF;
        we_power_data->data_fields.accel_y.i16 = 0xFFFF;
        we_power_data->data_fields.accel_z.i16 = 0xFFFF;
    }

    // Get temp and pressure
    temp_pressure_data_t temp_pressure_data;
    if(app_temp_pressure_service(&temp_pressure_data) == TEMP_PRESSURE_SUCCESS)
    {
        we_power_data->data_fields.pressure.i16 = temp_pressure_data.pressure;
        we_power_data->data_fields.temp.i16 = temp_pressure_data.temp;
    }
    else
    {
        we_power_data->data_fields.pressure.i16 = 0xFFFF;
        we_power_data->data_fields.temp.i16 = 0xFFFF;
    }

}

void encryptedData(uint8_t* clear_text_buf, uint8_t* encrypted_text_buf, uint8_t len )
{
#define ENCRYPT
#ifdef ENCRYPT
    // Not we want to encrypt the data
    if(app_encrypt_payload(clear_text_buf, len, encrypted_text_buf, len) == ENCRYPTION_ERROR)
    {
        memcpy(encrypted_text_buf, clear_text_buf, 16);
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
    }
    else
    {
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_ENC;
    }
#else
    memcpy(cipher_text, encrypted_text_buf, 16);
    manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
#endif
    if (LOGS){
        printk("Payload - Cleartext: ");
        for(int i = 0; i < len; i++)
        {
            printk("%02X ", clear_text_buf[i]);
        }
        printk("\n");

        printk("Payload - Encrypted: ");
        for(int i = 0; i < len; i++)
        {
            printk("%02X ", encrypted_text_buf[i]);
        }
        printk("\n");
    }
}
// only call this ONCE per EventCounter (FRAM[0:3])
void updateManufacturerData(struct k_work *work){
    printk(">>> Updating the Manufacturer Data\n");
    //Get sensor data
    measure_sensors(&we_power_data);

    // increase the FRAM Event counter
	we_power_data.data_fields.event_counter.u32++;

    	// initialize the TX counter
	TX_Repeat_Counter = 0;
	manf_data[PAYLOAD_TX_REPEAT_COUNTER_INDEX] = TX_Repeat_Counter;
   
    // Handle encryption
    static uint8_t cipher_text[DATA_SIZE_BYTES];

    encryptedData(we_power_data.data_bytes, cipher_text, DATA_SIZE_BYTES);
    // Build the BLE adv data
    memcpy(&manf_data[MANUF_DATA_CUSTOM_START_INDEX], cipher_text, DATA_SIZE_BYTES);
    memcpy(&manf_data[PAYLOAD_DEVICE_ID_INDEX], device_id.u8, 2);

    we_power_adv_data.type = BT_DATA_SVC_DATA16;
    we_power_adv_data.data = manf_data;
    we_power_adv_data.data_len = sizeof(manf_data);

    k_timer_start(&timer_event, K_MSEC(fram_data.event_inteval), K_MSEC(fram_data.event_inteval)); // Start 20ms timer event

    fram_data.event_counter = we_power_data.data_fields.event_counter.u32;
    app_fram_write_counter(&fram_data);
}

// Main Application
void main(void)
{
    int err;

    // read VEXT10 ADC, decide to run this or the configuration app.

	// if VEXT10 > 0.165V we have external power

    // Reading FRAM.
    
    if(app_fram_read_data(&fram_data) == FRAM_SUCCESS)
    {
        // Success! Add it to the payload
		serial_number = (fram_data.serial_number?(fram_data.serial_number<0xFFFF? fram_data.serial_number:DEVICE_ID):DEVICE_ID);
		type = (fram_data.type ? (fram_data.type < 0xFF ? fram_data.type : 1) : 1);
		event_inteval = (fram_data.event_inteval >= PACKET_INTERVAL_MIN ? (fram_data.event_inteval <= PACKET_INTERVAL_MAX ? fram_data.event_inteval : PACKET_INTERVAL_MAX) : PACKET_INTERVAL_MIN); //
		event_max_limits = (fram_data.event_max_limits >= 3 ? (fram_data.event_max_limits < 0xF0 ? fram_data.event_max_limits : 3) : 0xF0);
		sleep_min_interval = (fram_data.sleep_min_interval >= 50 ? (fram_data.sleep_min_interval < 0xFFFF ? fram_data.sleep_min_interval : 0xFFF0) : 50);
		sleep_after_wake = (fram_data.sleep_after_wake ? (fram_data.sleep_after_wake < 0xFFFF ? fram_data.sleep_after_wake : 20) : 0);
		we_power_data.data_fields.event_counter.u32 = fram_data.event_counter;
        //manf_data[PAYLOAD_TX_REPEAT_COUNTER_INDEX] = fram_data.event_counter + 1; do it when we read sensors
    }
    else
    {
         we_power_data.data_fields.event_counter.u32 = 0; // assume we are new, will increment and store 1
		 fram_data.event_counter = 0;
    }
    
    // Init and run the BLE
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	
	bt_ready();

    // Init works for advertising and measuring data
    k_work_init(&start_advertising_worker, start_advertising);
    k_work_init_delayable(&update_frame_work, updateManufacturerData);

    // Run the measuring task
    k_work_schedule(&update_frame_work, K_MSEC(fram_data.sleep_min_interval));

}
